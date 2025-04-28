import logging
import threading
import time
from dataclasses import dataclass
from enum import Enum
from math import floor
from types import TracebackType
from typing import Any, Dict, List, Optional, Tuple, Type, Union

import serial

logging.basicConfig(level=logging.INFO)


class SMC100Command(Enum):
    """
    Enumeration of valid SMC100 commands with their parameters and descriptions.
    Format: (command, has_query, has_setter, parameter_type, min_value, max_value, description)
    """

    # Motion Commands
    MOVE_ABSOLUTE = ("PA", True, True, float, None, None, "Move to absolute position")
    MOVE_RELATIVE = (
        "PR",
        True,
        True,
        float,
        None,
        None,
        "Move relative to current position",
    )
    GET_POSITION = ("TP", True, False, None, None, None, "Get current position")
    STOP_MOTION = ("ST", False, False, None, None, None, "Stop motion")
    HOME = ("OR", False, False, None, None, None, "Home the stage")
    HOME_SEARCH_TYPE = ("HT", True, True, int, 0, 4, "Set/Get HOME search type")
    # Configuration Commands
    SET_VELOCITY = ("VA", True, True, float, 0, 20, "Set velocity in mm/s")
    GET_VELOCITY = ("VA", True, False, None, None, None, "Get current velocity")
    SET_ACCELERATION = ("AC", True, True, float, 0, 400, "Set acceleration in mm/s²")
    GET_ACCELERATION = ("AC", True, False, None, None, None, "Get current acceleration")

    # Status Commands
    GET_STATUS = ("TS", True, False, None, None, None, "Get controller status")
    GET_ERROR = ("TE", True, False, None, None, None, "Get error status")

    # Stage Limit Commands
    SET_NEGATIVE_SOFTWARE_LIMIT = (
        "SL",
        True,
        True,
        float,
        -1e12,
        0,
        "Set/Get negative software limit",
    )
    SET_POSITIVE_SOFTWARE_LIMIT = (
        "SR",
        True,
        True,
        float,
        0,
        1e12,
        "Set/Get positive software limit",
    )

    # Configuration Commands
    ENTER_CONFIGURATION = (
        "PW",
        False,
        True,
        int,
        0,
        1,
        "Enter/exit configuration mode",
    )
    STAGE_PARAMETERS = ("ZX", False, True, int, 1, 2, "Load/enable stage parameters")
    GET_IDENTIFIER = ("ID", True, False, None, None, None, "Get stage identifier")
    RESET = ("RS", False, False, None, None, None, "Reset the controller")
    GET_ALL_PARAMETERS = (
        "ZT",
        True,
        False,
        None,
        None,
        None,
        "Get all configuration parameters",
    )

    def __init__(
        self,
        cmd: str,
        has_query: bool,
        has_setter: bool,
        param_type: Optional[Type],
        min_val: Optional[float],
        max_val: Optional[float],
        description: str,
    ):
        self.cmd = cmd
        self.has_query = has_query
        self.has_setter = has_setter
        self.param_type = param_type
        self.min_val = min_val
        self.max_val = max_val
        self.description = description


@dataclass
class SMC100State:
    """Represents the state of the SMC100 controller."""

    code: str
    description: str
    is_error: bool = False


class SMC100States:
    """All possible states of the SMC100 controller."""

    STATES: Dict[str, SMC100State] = {
        "0A": SMC100State("0A", "NOT REFERENCED from reset"),
        "0B": SMC100State("0B", "NOT REFERENCED from homing"),
        "0C": SMC100State("0C", "NOT REFERENCED from configuration"),
        "0D": SMC100State("0D", "NOT REFERENCED from disable"),
        "0E": SMC100State("0E", "NOT REFERENCED from ready"),
        "0F": SMC100State("0F", "NOT REFERENCED from moving"),
        "10": SMC100State("10", "NOT REFERENCED ESP stage error", True),
        "14": SMC100State("14", "CONFIGURATION"),
        "1E": SMC100State("1E", "HOMING commanded from RS-232-C"),
        "1F": SMC100State("1F", "HOMING commanded by SMC-RC"),
        "28": SMC100State("28", "MOVING"),
        "32": SMC100State("32", "READY from HOMING"),
        "33": SMC100State("33", "READY from MOVING"),
        "34": SMC100State("34", "READY from DISABLE"),
        "3C": SMC100State("3C", "DISABLE from READY"),
        "3D": SMC100State("3D", "DISABLE from MOVING"),
        "3E": SMC100State("3E", "DISABLE from JOGGING"),
    }

    @classmethod
    def get_state(cls, code: str) -> SMC100State:
        """Get state object from state code."""
        if code not in cls.STATES:
            raise ValueError(f"Unknown state code: {code}")
        return cls.STATES[code]


class SMC100Error(Exception):
    """Base exception for SMC100 errors."""

    pass


class SMC100CommandError(SMC100Error):
    """Raised when a command is invalid or has invalid parameters."""

    pass


class SMC100CommunicationError(SMC100Error):
    """Raised when communication with the controller fails."""

    pass


class SMC100TimeoutError(SMC100Error):
    """Raised when a command or operation times out."""

    pass


class SMC100StateError(SMC100Error):
    """Raised when the controller enters an error state."""

    pass


MAX_WAIT_TIME_SEC = 60
COMMAND_WAIT_TIME_SEC = 0.9


class SMC100:
    """
    SMC100 controller interface with robust command handling and validation.

    Features:
    - Type-safe command handling with parameter validation
    - Comprehensive state tracking and error handling
    - Automatic resource management through context manager
    - Detailed logging and error reporting
    """

    def __init__(
        self,
        smcID: int,
        port: str = "/dev/ttyUSB0",
        backlash_compensation: bool = True,
        silent: bool = True,
        sleepfunc=None,
        command_timeout: float = 1.0,
        max_retries: int = 3,
    ):
        """
        Initialize SMC100 controller.

        Args:
            smcID: Controller ID number
            port: Serial port device path
            backlash_compensation: Enable/disable backlash compensation
            silent: Suppress debug output if True
            sleepfunc: Alternative sleep function for GUI integration
            command_timeout: Timeout for command responses in seconds
            max_retries: Maximum number of command retries
        """
        self._smcID = str(smcID)
        self._silent = silent
        self._sleepfunc = sleepfunc if sleepfunc is not None else time.sleep
        self._command_timeout = command_timeout
        self._max_retries = max_retries
        self._last_command_time = 0
        self._current_state: Optional[SMC100State] = None
        self._lock = threading.Lock()
        self._port = serial.Serial(
            port=port,
            baudrate=57600,
            bytesize=8,
            stopbits=1,
            parity="N",
            xonxoff=True,
            timeout=command_timeout,
        )

        self._logger = logging.getLogger(f"SMC100_{smcID}")
        if not silent:
            self._logger.setLevel(logging.DEBUG)

        self._logger.info(f"Connected to SMC100 controller on {port}")

    def __enter__(self) -> "SMC100":
        """Context manager entry point.

        Returns:
            self: The SMC100 instance
        """
        return self

    def __exit__(
        self,
        exc_type: Optional[Type[BaseException]],
        exc_val: Optional[BaseException],
        exc_tb: Optional[TracebackType],
    ) -> None:
        """Context manager exit point.

        Ensures the serial port is properly closed when exiting the context.
        """
        self.close()

    def get_status(self, silent: bool = False) -> Tuple[int, str]:
        """
        Get the controller status, matching original implementation.

        Returns:
            Tuple of (error_code, state_code)
        """
        resp = self.execute_command(SMC100Command.GET_STATUS, query=True)
        if not resp:
            raise SMC100CommunicationError("No response to status query")

        errors = int(resp[0:4], 16)
        state = resp[4:6]

        if not silent:
            self._logger.info("status:")
            state_descriptions = {
                "0A": "  state: NOT REFERENCED from reset",
                "0B": "  state: NOT REFERENCED from HOMING",
                "0C": "  state: NOT REFERENCED from CONFIGURATION",
                "0D": "  state: NOT REFERENCED from DISABLE",
                "0E": "  state: NOT REFERENCED from READY",
                "0F": "  state: NOT REFERENCED from MOVING",
                "10": "  state: NOT REFERENCED ESP stage error",
                "14": "  state: CONFIGURATION",
                "1E": "  state: HOMING commanded from RS-232-C",
                "1F": "  state: HOMING commanded by SMC-RC",
                "28": "  state: MOVING",
                "32": "  state: READY from HOMING",
                "33": "  state: READY from MOVING",
                "34": "  state: READY from DISABLE",
                "35": "  state: READY from JOGGING",
                "3C": "  state: DISABLE from READY",
                "3D": "  state: DISABLE from MOVING",
                "3E": "  state: DISABLE from JOGGING",
                "46": "  state: JOGGING from READY",
                "47": "  state: JOGGING from DISABLE",
            }
            if state in state_descriptions:
                self._logger.info(state_descriptions[state])

        return errors, state

    def execute_command(
        self,
        command: SMC100Command,
        parameter: Any = None,
        query: bool = False,
        retry: bool = True,
    ) -> Optional[str]:
        """
        Execute a controller command with parameter validation and improved error handling.

        Args:
            command: Command to execute from SMC100Command enum
            parameter: Optional parameter for the command
            query: True to execute as query (append ?)
            retry: Allow command retry on failure

        Returns:
            Optional response string from controller

        Raises:
            SMC100CommandError: If command or parameters are invalid
            SMC100CommunicationError: If communication fails
            SMC100TimeoutError: If command times out
        """
        with self._lock:
            if query and not command.has_query:
                raise SMC100CommandError(
                    f"Command {command.cmd} does not support queries"
                )

            if parameter is not None:
                if not command.has_setter:
                    raise SMC100CommandError(
                        f"Command {command.cmd} does not accept parameters"
                    )

                if command.param_type is not None:
                    try:
                        parameter = command.param_type(parameter)
                    except (ValueError, TypeError):
                        raise SMC100CommandError(
                            f"Parameter {parameter} is not of type {command.param_type.__name__}"
                        )

                if command.min_val is not None and parameter < command.min_val:
                    raise SMC100CommandError(
                        f"Parameter {parameter} is below minimum value {command.min_val}"
                    )

                if command.max_val is not None and parameter > command.max_val:
                    raise SMC100CommandError(
                        f"Parameter {parameter} is above maximum value {command.max_val}"
                    )

            cmd_str = f"{self._smcID}{command.cmd}"
            if query:
                cmd_str += "?"
            elif parameter is not None:
                cmd_str += str(parameter)

            retries = self._max_retries if retry else 0
            last_error = None
            expect_response = query
            while True:
                try:
                    self._logger.info(f"[SMC100] executing command: {command}")
                    return self._send_command(cmd_str, expect_response=expect_response)
                except (SMC100CommunicationError, SMC100TimeoutError) as e:
                    last_error = e
                    if retries <= 0:
                        self._logger.error(
                            f"Command failed after all retries: {cmd_str}"
                        )
                        raise last_error
                    retries -= 1
                    self._logger.warning(
                        f"Command failed, retrying ({retries} attempts left): {str(e)}"
                    )
                    self._sleepfunc(0.5)

    def _send_command(self, command: str, expect_response: bool) -> Optional[str]:
        """
        Low-level send. If expect_response is False, just send and return.
        Otherwise read and parse response more flexibly.
        """
        now = time.time()
        delta = now - self._last_command_time
        if delta < COMMAND_WAIT_TIME_SEC:
            self._sleepfunc(COMMAND_WAIT_TIME_SEC - delta)

        formatted = f"0{command}" if command[0].isdigit() else command
        cmd_bytes = formatted.encode("utf-8")

        self._port.flushInput() if expect_response else None
        self._port.flushOutput()
        self._port.write(cmd_bytes)
        self._logger.info(f"[SMC100 sent] {cmd_bytes}")
        self._sleepfunc(0.09)
        self._port.write(b"\r\n")
        self._port.flush()

        self._last_command_time = time.time()

        if not expect_response:
            return None

        response = self._port.readline().decode("utf-8").strip()
        self._logger.info(f"Response: {response!r}")

        if not response:
            raise SMC100CommunicationError("No response received")

        cmd_part = formatted[2:] if formatted.startswith("01") else formatted
        cmd_part = cmd_part.rstrip("?")

        idx = response.find(cmd_part)
        if idx < 0:
            raise SMC100CommunicationError(
                f"Unexpected response. Looking for '{cmd_part}' in '{response}'"
            )

        return response[idx + len(cmd_part) :]

    def get_position_mm(self) -> float:
        """Get current position in millimeters."""
        response = self.execute_command(SMC100Command.GET_POSITION, query=True)
        return float(response)

    def get_position_um(self) -> int:
        """Get current position in micrometers."""
        return int(self.get_position_mm() * 1000)

    def move_absolute_mm(self, position_mm: float, wait: bool = True) -> None:
        """
        Move to absolute position in millimeters.

        Args:
            position_mm: Target position in mm
            wait: Wait for movement to complete if True
        """
        self.execute_command(SMC100Command.MOVE_ABSOLUTE, position_mm)
        if wait:
            self._wait_for_states(
                [
                    SMC100States.STATES["32"],  # READY from HOMING
                    SMC100States.STATES["33"],  # READY from MOVING
                ]
            )

    def move_absolute_um(self, position_um: int, wait: bool = True) -> None:
        """Move to absolute position in micrometers."""
        self.move_absolute_mm(floor(position_um) / 1000, wait)

    def move_relative_mm(self, dist_mm, waitStop=True):
        """Move relatively by given distance in mm, matching original implementation."""
        self.execute_command(SMC100Command.MOVE_RELATIVE, dist_mm)
        if waitStop:
            self.wait_states(("33", "32"))

    def wait_states(self, targetstates, ignore_disabled_states=False):
        """
        Wait for controller to reach target state, matching original implementation.
        """
        start_time = time.time()
        self._logger.info(f"[SMC100] waiting for states {targetstates}")

        while True:
            if time.time() - start_time > MAX_WAIT_TIME_SEC:
                raise SMC100TimeoutError(f"Timeout waiting for states: {targetstates}")

            try:
                errors, state = self.get_status()
                if state in targetstates:
                    return state

                if (not ignore_disabled_states) and state in ["3C", "3D", "3E"]:
                    raise SMC100StateError(state)

            except SMC100TimeoutError:
                self._logger.info("[SMC100] Read timed out, retrying...")
                self._sleepfunc(0.1)
                continue

    def home(self, wait: bool = False) -> None:
        """
        Home the controller and optionally wait for completion.

        Args:
            wait: Wait for homing to complete if True
        """
        cmd = f"{self._smcID}{SMC100Command.HOME.cmd}"
        self._send_command(cmd, expect_response=False)

        if wait:
            ready_states = [
                SMC100States.STATES["32"],  # READY from HOMING
                SMC100States.STATES["33"],  # READY from MOVING
            ]
        self._wait_for_states(ready_states)
        self.move_absolute_um(0, wait=True)

    def _wait_for_states(
        self,
        target_states: List[SMC100State],
        timeout: Optional[float] = None,
        ignore_disabled: bool = False,
    ) -> SMC100State:
        """
        Wait for controller to enter one of the target states.

        Args:
            target_states: List of acceptable target states
            timeout: Maximum time to wait in seconds, or None for default
            ignore_disabled: Ignore disabled states if True

        Returns:
            Reached state

        Raises:
            SMC100TimeoutError: If timeout is exceeded
            SMC100StateError: If error state is encountered
        """
        if timeout is None:
            timeout = MAX_WAIT_TIME_SEC

        start_time = time.time()
        target_codes = [state.code for state in target_states]

        while True:
            if time.time() - start_time > timeout:
                raise SMC100TimeoutError(
                    f"Timeout waiting for states: {', '.join(target_codes)}"
                )

            try:
                current_state = self._get_state()

                if current_state in target_states:
                    return current_state

                if not ignore_disabled:
                    if current_state.code in ["3C", "3D", "3E"]:  # Disabled states
                        raise SMC100StateError(
                            f"Entered disabled state: {current_state.description}"
                        )

                if current_state.is_error:
                    raise SMC100StateError(
                        f"Entered error state: {current_state.description}"
                    )

            except SMC100CommunicationError:
                self._logger.debug("Communication error during state wait, retrying...")
                self._sleepfunc(0.5)
                continue

            self._sleepfunc(0.1)

    def _get_state(self) -> SMC100State:
        """Get current controller state."""
        response = self.execute_command(SMC100Command.GET_STATUS, query=True)
        error_code = int(response[0:4], 16)
        state_code = response[4:6]

        state = SMC100States.get_state(state_code)
        if error_code != 0:
            self._logger.warning(f"Controller reported error: {state.description}")
        return state

    def close(self) -> None:
        """Close the serial port connection."""
        if self._port:
            self._port.close()
            self._port = None

    def __del__(self):
        """Destructor to ensure port is closed."""
        self.close()


if __name__ == "__main__":
    with SMC100(1, silent=False) as controller:
        controller._logger.setLevel(logging.DEBUG)
        # get ID
        controller.execute_command(SMC100Command.GET_IDENTIFIER, query=True)
        # center configuration mode
        controller.execute_command(SMC100Command.ENTER_CONFIGURATION, 1)
        # load stage parameters
        controller.execute_command(SMC100Command.STAGE_PARAMETERS, 1)
        # exit configuration mode
        controller.execute_command(SMC100Command.ENTER_CONFIGURATION, 0)

        controller.execute_command(SMC100Command.GET_STATUS)
        # controller.home()
        controller.move_absolute_mm(10)
        print(f"Current position: {controller.get_position_mm()} mm")
