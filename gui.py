import json
import os
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import serial
from smc100.controller import (
    SMC100,
    SMC100Command,
    SMC100CommunicationError,
    SMC100StateError,
    SMC100TimeoutError,
)


class LimitFinderApp:

    def __init__(self, root: tk.Tk):
        """Initialize the application."""
        self.root = root
        self.root.title("SMC100 Stage Control")
        self.root.geometry("850x650")

        self.controller = None
        self.connected = False
        self.port_var = tk.StringVar(value="/dev/ttyUSB0")
        self.smc_id_var = tk.StringVar(value="1")

        self.position_var = tk.StringVar(value="N/A")
        self.status_var = tk.StringVar(value="Not Connected")
        self.update_thread = None
        self.stop_thread = False

        self.move_speed_var = tk.DoubleVar(value=1.0)
        self.move_distance_var = tk.DoubleVar(value=0.1)

        self.neg_limit_var = tk.StringVar(value="N/A")
        self.pos_limit_var = tk.StringVar(value="N/A")

        self.virtual_config_file = "virtual_limits_config.json"
        self.virtual_center_var = tk.DoubleVar(value=12.0)
        self.virtual_range_var = tk.DoubleVar(value=2.5)
        self.virtual_center_abs = None
        self.virtual_range = None
        self.virtual_neg_limit_abs = None
        self.virtual_pos_limit_abs = None
        self.software_limits_active = False

        self.home_type_var = tk.StringVar(value="1")

        self.create_widgets()
        self.load_virtual_limits()
        self._configure_resizing()

    def _configure_resizing(self):
        """Configure row/column weights for main window resizing."""
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=3)
        self.root.rowconfigure(0, weight=1)

        self.left_frame.columnconfigure(0, weight=1)
        self.left_frame.rowconfigure(0, weight=0)
        self.left_frame.rowconfigure(1, weight=0)
        self.left_frame.rowconfigure(2, weight=1)
        self.left_frame.rowconfigure(3, weight=0)
        self.left_frame.rowconfigure(4, weight=0)

        self.right_frame.columnconfigure(0, weight=1)
        self.right_frame.rowconfigure(0, weight=1)

    def create_widgets(self):
        """Create and layout GUI widgets."""
        self.left_frame = ttk.Frame(self.root)
        self.left_frame.grid(row=0, column=0, padx=(10, 5), pady=10, sticky="nsew")

        self.right_frame = ttk.Frame(self.root)
        self.right_frame.grid(row=0, column=1, padx=(5, 10), pady=10, sticky="nsew")

        current_row = 0

        conn_frame = ttk.LabelFrame(self.left_frame, text="Connection")
        conn_frame.grid(row=current_row, column=0, sticky="ew", padx=5, pady=5)

        conn_frame.columnconfigure(1, weight=1)
        conn_frame.grid()
        current_row += 1

        ttk.Label(conn_frame, text="Port:").grid(
            row=0, column=0, padx=5, pady=5, sticky="w"
        )
        self.port_entry = ttk.Entry(conn_frame, textvariable=self.port_var)
        self.port_entry.grid(row=0, column=1, padx=5, pady=5, sticky="ew")

        ttk.Label(conn_frame, text="SMC ID:").grid(
            row=1, column=0, padx=5, pady=5, sticky="w"
        )
        self.smc_id_entry = ttk.Entry(conn_frame, textvariable=self.smc_id_var, width=5)
        self.smc_id_entry.grid(row=1, column=1, padx=5, pady=5, sticky="w")

        self.move_to_origin_var = tk.BooleanVar(value=False)
        self.move_to_origin_cb = ttk.Checkbutton(
            conn_frame,
            text="Move to Origin on Connect",
            variable=self.move_to_origin_var,
        )
        self.move_to_origin_cb.grid(
            row=2, column=0, columnspan=2, padx=5, pady=5, sticky="w"
        )

        self.connect_btn = ttk.Button(
            conn_frame, text="Connect", command=self.toggle_connection
        )
        self.connect_btn.grid(
            row=3, column=0, columnspan=2, padx=5, pady=5, sticky="ew"
        )

        status_frame = ttk.LabelFrame(self.left_frame, text="Status")
        status_frame.grid(row=current_row, column=0, sticky="ew", padx=5, pady=5)
        status_frame.columnconfigure(1, weight=1)
        current_row += 1

        ttk.Label(
            status_frame, text="Position:", font=("TkDefaultFont", 10, "bold")
        ).grid(row=0, column=0, padx=5, pady=5, sticky="w")
        ttk.Label(status_frame, textvariable=self.position_var).grid(
            row=0, column=1, padx=5, pady=5, sticky="w"
        )

        ttk.Label(
            status_frame, text="Status:", font=("TkDefaultFont", 10, "bold")
        ).grid(row=1, column=0, padx=5, pady=5, sticky="w")
        ttk.Label(status_frame, textvariable=self.status_var).grid(
            row=1, column=1, padx=5, pady=5, sticky="w"
        )

        limits_frame = ttk.LabelFrame(
            self.left_frame, text="Controller & Virtual Limits"
        )
        limits_frame.grid(row=current_row, column=0, sticky="nsew", padx=5, pady=5)
        limits_frame.columnconfigure(1, weight=1)
        current_row += 1

        ttk.Label(limits_frame, text="Ctrl Neg Lim:").grid(
            row=0, column=0, padx=5, pady=2, sticky="w"
        )
        ttk.Label(limits_frame, textvariable=self.neg_limit_var).grid(
            row=0, column=1, padx=5, pady=2, sticky="ew"
        )

        ttk.Label(limits_frame, text="Ctrl Pos Lim:").grid(
            row=1, column=0, padx=5, pady=2, sticky="w"
        )
        ttk.Label(limits_frame, textvariable=self.pos_limit_var).grid(
            row=1, column=1, padx=5, pady=2, sticky="ew"
        )

        ttk.Separator(limits_frame, orient="horizontal").grid(
            row=2, column=0, columnspan=2, sticky="ew", pady=5
        )

        ttk.Label(limits_frame, text="Virtual Center (mm):").grid(
            row=3, column=0, padx=5, pady=2, sticky="w"
        )
        self.virtual_center_entry = ttk.Entry(
            limits_frame,
            textvariable=self.virtual_center_var,
            width=10,
            state="disabled",
        )
        self.virtual_center_entry.grid(row=3, column=1, padx=5, pady=2, sticky="w")

        ttk.Label(limits_frame, text="Virtual Range (+/- mm):").grid(
            row=4, column=0, padx=5, pady=2, sticky="w"
        )
        self.virtual_range_entry = ttk.Entry(
            limits_frame,
            textvariable=self.virtual_range_var,
            width=10,
            state="disabled",
        )
        self.virtual_range_entry.grid(row=4, column=1, padx=5, pady=2, sticky="w")

        self.set_virtual_limits_btn = ttk.Button(
            limits_frame,
            text="Activate & Save Virtual Limits",
            command=self.set_and_save_virtual_limits,
            state="disabled",  # Enable on connect
        )
        self.set_virtual_limits_btn.grid(
            row=5, column=0, columnspan=2, padx=5, pady=5, sticky="ew"
        )

        config_frame = ttk.LabelFrame(self.left_frame, text="Controller Configuration")
        config_frame.grid(row=current_row, column=0, sticky="ew", padx=5, pady=5)
        config_frame.columnconfigure(0, weight=1)
        current_row += 1

        self.reset_config_btn = ttk.Button(
            config_frame,
            text="Reset Controller to Defaults",
            command=self.reset_to_defaults,
            state="disabled",
        )
        self.reset_config_btn.grid(row=0, column=0, padx=5, pady=5, sticky="ew")

        save_load_frame = ttk.Frame(config_frame)
        save_load_frame.grid(row=1, column=0, sticky="ew")
        save_load_frame.columnconfigure(0, weight=1)
        save_load_frame.columnconfigure(1, weight=1)

        self.save_config_btn = ttk.Button(
            save_load_frame,
            text="Save Ctrlr Config",
            command=self.save_configuration,
            state="disabled",
        )
        self.save_config_btn.grid(row=0, column=0, padx=(5, 2), pady=5, sticky="ew")

        self.load_config_btn = ttk.Button(
            save_load_frame,
            text="Load Ctrlr Config",
            command=self.load_configuration,
            state="disabled",
        )
        self.load_config_btn.grid(row=0, column=1, padx=(2, 5), pady=5, sticky="ew")

        home_frame = ttk.LabelFrame(self.left_frame, text="Home Position")
        home_frame.grid(row=current_row, column=0, sticky="ew", padx=5, pady=5)
        home_frame.columnconfigure(1, weight=1)
        current_row += 1

        ttk.Label(home_frame, text="Home Type:").grid(
            row=0, column=0, padx=5, pady=5, sticky="w"
        )
        self.home_type_combo = ttk.Combobox(
            home_frame,
            textvariable=self.home_type_var,
            width=30,
            state="readonly",
            values=(
                "0 - Use MZ switch and encoder index",
                "1 - Use current position as HOME",  # Not reliable for zeroing!
                "2 - Use MZ switch only",
                "3 - Use EoR- switch and encoder index",
                "4 - Use EoR- switch only",
            ),
        )
        # Find index for HT=3 (default recommended hardware home)
        default_ht_index = 3  # Default to HT=3
        for i, val in enumerate(self.home_type_combo["values"]):
            if val.startswith("3"):
                default_ht_index = i
                break
        self.home_type_combo.current(default_ht_index)
        self.home_type_combo.grid(
            row=0, column=1, columnspan=2, padx=5, pady=5, sticky="ew"
        )

        self.set_home_type_btn = ttk.Button(
            home_frame,
            text="Set Ctrlr Home Type",
            command=self.set_home_type,
            state="disabled",
        )
        self.set_home_type_btn.grid(row=1, column=0, padx=5, pady=5, sticky="ew")

        self.execute_home_btn = ttk.Button(
            home_frame,
            text="Execute Ctrlr Home",
            command=self.execute_home,
            state="disabled",
        )
        self.execute_home_btn.grid(row=1, column=1, padx=5, pady=5, sticky="ew")

        move_frame = ttk.LabelFrame(self.right_frame, text="Motion Control")
        move_frame.pack(fill="both", expand=True)
        move_frame.columnconfigure(0, weight=1)

        params_frame = ttk.Frame(move_frame)
        params_frame.grid(row=0, column=0, sticky="ew", padx=5, pady=5)
        params_frame.columnconfigure(1, weight=1)

        ttk.Label(params_frame, text="Speed (mm/s):").grid(
            row=0, column=0, padx=5, pady=5, sticky="w"
        )
        ttk.Spinbox(
            params_frame,
            from_=0.01,
            to=20,
            increment=0.01,
            textvariable=self.move_speed_var,
            width=8,
        ).grid(row=0, column=1, padx=5, pady=5, sticky="w")

        ttk.Label(params_frame, text="Step (mm):").grid(
            row=1, column=0, padx=5, pady=5, sticky="w"
        )
        ttk.Spinbox(
            params_frame,
            from_=0.001,
            to=10,
            increment=0.001,
            textvariable=self.move_distance_var,
            width=8,
        ).grid(row=1, column=1, padx=5, pady=5, sticky="w")

        step_frame = ttk.LabelFrame(move_frame, text="Step Movement")
        step_frame.grid(row=1, column=0, sticky="ew", padx=5, pady=10)
        step_frame.columnconfigure(0, weight=1)
        step_frame.columnconfigure(1, weight=1)

        self.step_neg_btn = ttk.Button(
            step_frame,
            text="← Step",
            command=lambda: self.step_stage(-1),
            state="disabled",
        )
        self.step_neg_btn.grid(row=0, column=0, padx=5, pady=5, sticky="ew")

        self.step_pos_btn = ttk.Button(
            step_frame,
            text="Step →",
            command=lambda: self.step_stage(1),
            state="disabled",
        )
        self.step_pos_btn.grid(row=0, column=1, padx=5, pady=5, sticky="ew")

        abs_move_frame = ttk.LabelFrame(move_frame, text="Absolute Move")
        abs_move_frame.grid(row=2, column=0, sticky="ew", padx=5, pady=10)
        abs_move_frame.columnconfigure(1, weight=1)

        ttk.Label(abs_move_frame, text="Target (mm):").grid(
            row=0, column=0, padx=5, pady=5, sticky="w"
        )
        self.abs_target_entry = ttk.Entry(abs_move_frame, width=10)
        self.abs_target_entry.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        self.move_abs_btn = ttk.Button(
            abs_move_frame,
            text="Move Absolute",
            command=self.move_absolute,
            state="disabled",
        )
        self.move_abs_btn.grid(row=0, column=2, padx=5, pady=5, sticky="e")

        vacuum_frame = ttk.LabelFrame(move_frame, text="⚠️ Vacuum Chamber Setup ⚠️")
        vacuum_frame.grid(row=3, column=0, sticky="ew", padx=5, pady=10)
        vacuum_frame.columnconfigure(0, weight=1)

        ttk.Label(
            vacuum_frame,
            text="Center stage at 12.5mm and activate +/- 1.0mm virtual limits.",
            justify=tk.LEFT,
        ).grid(row=0, column=0, columnspan=2, sticky="w", padx=5, pady=(5, 0))
        ttk.Label(
            vacuum_frame,
            text="(Requires hardware home first!)",
            font=("TkDefaultFont", 8),
        ).grid(row=1, column=0, columnspan=2, sticky="w", padx=5, pady=(0, 5))

        self.vacuum_config_btn = ttk.Button(
            vacuum_frame,
            text="Run Vacuum Center & Limit Setup",
            command=self.configure_for_vacuum_chamber,
            state="disabled",
        )
        self.vacuum_config_btn.grid(
            row=2, column=0, columnspan=2, padx=5, pady=5, sticky="ew"
        )

        stop_frame = ttk.Frame(move_frame)
        stop_frame.grid(row=4, column=0, sticky="ew", padx=5, pady=10)
        stop_frame.columnconfigure(0, weight=1)

        style = ttk.Style()
        style.configure(
            "Stop.TButton", font=("TkDefaultFont", 12, "bold"), foreground="red"
        )
        self.stop_btn = ttk.Button(
            stop_frame,
            text="STOP MOTION",
            command=self.stop_motion,
            state="disabled",
            style="Stop.TButton",
        )
        self.stop_btn.pack(fill="x", expand=True, ipady=5)

    def _set_ui_state(self, connected: bool):
        """Enable/disable UI elements based on connection status."""
        state = tk.NORMAL if connected else tk.DISABLED
        widget_groups = [
            self.reset_config_btn,
            self.save_config_btn,
            self.load_config_btn,
            self.virtual_center_entry,
            self.virtual_range_entry,
            self.set_virtual_limits_btn,
            self.set_home_type_btn,
            self.execute_home_btn,
            self.step_neg_btn,
            self.step_pos_btn,
            self.stop_btn,
            self.abs_target_entry,
            self.move_abs_btn,
            self.vacuum_config_btn,
        ]
        for widget in widget_groups:
            widget.config(state=state)

        self.virtual_center_entry.config(state=tk.NORMAL if connected else tk.DISABLED)
        self.virtual_range_entry.config(state=tk.NORMAL if connected else tk.DISABLED)
        self.abs_target_entry.config(state=tk.NORMAL if connected else tk.DISABLED)

        self.connect_btn.config(text="Disconnect" if connected else "Connect")

    def save_virtual_limits(self, filename=None):
        """Saves current virtual limits (from internal vars) to a JSON file."""
        if filename is None:
            filename = self.virtual_config_file
        if self.virtual_center_abs is not None and self.virtual_range is not None:
            config = {
                "virtual_center_abs": self.virtual_center_abs,
                "virtual_range": self.virtual_range,
            }
            try:
                with open(filename, "w") as f:
                    json.dump(config, f, indent=4)
                print(f"Virtual limits saved to {filename}")
            except Exception as e:
                messagebox.showerror(
                    "Save Error", f"Failed to save virtual limits: {str(e)}"
                )
                print(f"Failed to save virtual limits: {e}")
        else:
            print("No virtual limits set to save.")

    def load_virtual_limits(self, filename=None):
        """Loads virtual limits from a JSON file and updates state."""
        if filename is None:
            filename = self.virtual_config_file

        limits_loaded = False
        if os.path.exists(filename):
            try:
                with open(filename, "r") as f:
                    config = json.load(f)

                center = config.get("virtual_center_abs")
                range_val = config.get("virtual_range")

                if (
                    isinstance(center, (int, float))
                    and isinstance(range_val, (int, float))
                    and range_val > 0
                ):
                    self.virtual_center_abs = float(center)
                    self.virtual_range = float(range_val)
                    self.virtual_neg_limit_abs = (
                        self.virtual_center_abs - self.virtual_range
                    )
                    self.virtual_pos_limit_abs = (
                        self.virtual_center_abs + self.virtual_range
                    )
                    self.software_limits_active = True
                    self.virtual_center_var.set(self.virtual_center_abs)
                    self.virtual_range_var.set(self.virtual_range)
                    limits_loaded = True
                    print(
                        f"Loaded virtual limits from {filename}: Center={self.virtual_center_abs}, Range={self.virtual_range}"
                    )
                else:
                    print(f"Invalid data found in {filename}")
            except Exception as e:
                messagebox.showerror(
                    "Load Error",
                    f"Failed to load virtual limits from {filename}: {str(e)}",
                )
                print(f"Failed to load virtual limits: {e}")

        if not limits_loaded:
            print(
                f"Virtual limits file ({filename}) not found or invalid. Limits inactive."
            )
            self.software_limits_active = False
            self.virtual_center_abs = None
            self.virtual_range = None
            self.virtual_neg_limit_abs = None
            self.virtual_pos_limit_abs = None

        self.update_limit_labels()

    def update_limit_labels(self):
        """Updates the Neg/Pos Limit labels based on whether virtual limits are active."""
        if self.software_limits_active and self.virtual_neg_limit_abs is not None:
            self.neg_limit_var.set(f"{self.virtual_neg_limit_abs:.3f} (Virtual)")
            self.pos_limit_var.set(f"{self.virtual_pos_limit_abs:.3f} (Virtual)")
        elif self.controller:
            try:
                neg_limit = float(
                    self.controller.execute_command(
                        SMC100Command.SET_NEGATIVE_SOFTWARE_LIMIT, query=True
                    )
                )
                pos_limit = float(
                    self.controller.execute_command(
                        SMC100Command.SET_POSITIVE_SOFTWARE_LIMIT, query=True
                    )
                )
                self.neg_limit_var.set(f"{neg_limit:.3f} (Ctrlr)")
                self.pos_limit_var.set(f"{pos_limit:.3f} (Ctrlr)")
            except Exception:
                self.neg_limit_var.set("N/A (Ctrlr)")
                self.pos_limit_var.set("N/A (Ctrlr)")
        else:
            self.neg_limit_var.set("N/A")
            self.pos_limit_var.set("N/A")

    def set_and_save_virtual_limits(self):
        """Callback function to set internal limits from GUI vars and save."""
        try:
            center = self.virtual_center_var.get()
            range_val = self.virtual_range_var.get()

            if range_val <= 0:
                messagebox.showerror("Input Error", "Virtual range must be positive.")
                return

            # Update internal state
            self.virtual_center_abs = center
            self.virtual_range = range_val
            self.virtual_neg_limit_abs = self.virtual_center_abs - self.virtual_range
            self.virtual_pos_limit_abs = self.virtual_center_abs + self.virtual_range
            self.software_limits_active = True  # Activate them when set manually

            self.update_limit_labels()
            self.save_virtual_limits()  # Save the newly set limits
            messagebox.showinfo("Success", "Virtual limits set and saved.")
            print(
                f"Virtual limits activated: {self.virtual_neg_limit_abs:.3f} to {self.virtual_pos_limit_abs:.3f}"
            )

        except tk.TclError:
            messagebox.showerror(
                "Input Error", "Invalid numeric value for virtual center or range."
            )
        except Exception as e:
            messagebox.showerror("Error", f"Failed to set virtual limits: {str(e)}")
            print(f"Failed to set virtual limits: {e}")

    def is_within_virtual_limits(self, target_abs_position: float) -> bool:
        """Checks if a target absolute position is within defined virtual limits."""
        if not self.software_limits_active or self.virtual_neg_limit_abs is None:
            return True 

        allowed = (
            self.virtual_neg_limit_abs
            <= target_abs_position
            <= self.virtual_pos_limit_abs
        )
        if not allowed:
            print(
                f"Move denied: Target {target_abs_position:.3f}mm outside virtual limits [{self.virtual_neg_limit_abs:.3f}, {self.virtual_pos_limit_abs:.3f}]"
            )
            messagebox.showwarning(
                "Limit Warning",
                f"Target position {target_abs_position:.3f}mm is outside the virtual limits [{self.virtual_neg_limit_abs:.3f}, {self.virtual_pos_limit_abs:.3f}]",
            )
        return allowed

    def toggle_connection(self):
        """Connect or disconnect the controller."""
        if not self.connected:
            threading.Thread(target=self.connect, daemon=True).start()
        else:
            self.disconnect()

    def connect(self):
        """Establish connection and initialize the controller."""
        try:
            port = self.port_var.get()
            smc_id = int(self.smc_id_var.get())
            print(f"Attempting connection to SMC ID {smc_id} on {port}...")
            self.status_var.set(f"Connecting to {port}...")
            self.root.update()
            if self.controller:
                self.controller.close()
                self.controller = None

            self.controller = SMC100(smcID=smc_id, port=port, silent=False)
            print("Controller object created.")

            self.status_var.set("Performing initial hardware home...")
            self.root.update()
            move_to_origin = self.move_to_origin_var.get()
            home_mode = self.home_type_var.get().split(" ")[0]
            if move_to_origin:
                home_mode = 4
                self.status_var.set("Moving to origin...")

            else:
                home_mode = 1  
                print("Move to origin not requested or already connected.")
            self.initialize_stage(
                move_to_origin, home_mode
            ) 

            pos = self.controller.get_position_mm()
            self.position_var.set(f"{pos:.3f} mm (Abs)") 
            print("Initial homing complete.")

            self.connected = True
            self._set_ui_state(True)
            self.status_var.set("Connected & Ready")

            self.load_virtual_limits()


            self.stop_thread = False
            self.update_thread = threading.Thread(
                target=self.update_position_loop, daemon=True
            )
            self.update_thread.start()
            print("Connection successful. Status updates started.")

        except (
            SMC100CommunicationError,
            SMC100StateError,
            SMC100TimeoutError,
            serial.SerialException,
            ValueError,
            RuntimeError,
        ) as e:
            error_msg = f"Failed to connect/init: {str(e)}"
            print(f"ERROR: {error_msg}")
            messagebox.showerror("Connection Error", error_msg)
            if self.controller:
                self.controller.close()
                self.controller = None
            self.status_var.set("Connection Failed")
            self._set_ui_state(False)  # Ensure UI is disabled
            self.update_limit_labels()  
        except Exception as e:
            error_msg = f"An unexpected error occurred: {str(e)}"
            print(f"ERROR: {error_msg}")
            messagebox.showerror("Error", error_msg)
            if self.controller:
                self.controller.close()
                self.controller = None
            self.status_var.set("Error")
            self._set_ui_state(False)
            self.update_limit_labels()

    def disconnect(self):
        """Disconnect the controller and clean up."""
        print("Disconnecting...")
        self.stop_thread = True
        if self.update_thread and self.update_thread.is_alive():
            try:
                self.update_thread.join(timeout=1.0)
            except Exception as e:
                print(f"Error joining update thread: {e}")
        self.update_thread = None

        if self.controller:
            try:
                # Attempt to stop motion before closing, ignore errors
                self.controller.execute_command(SMC100Command.STOP_MOTION)
            except Exception:
                pass
            self.controller.close()
            self.controller = None
            print("Controller closed.")

        self.connected = False
        self._set_ui_state(False)  # Disable UI elements

        self.position_var.set("N/A")
        self.status_var.set("Not Connected")
        self.software_limits_active = False  # Deactivate virtual limits
        self.update_limit_labels()  # Update labels to show N/A
        print("Disconnected.")

    def update_position_loop(self):
        """Periodically query position and status in a background thread."""
        polling_interval = 0.2 
        print(f"Starting status polling loop (Interval: {polling_interval}s)")
        while not self.stop_thread:
            start_time = time.monotonic()
            try:
                if self.controller and self.connected:
                    pos = self.controller.get_position_mm()
                    errors, state = self.controller.get_status(
                        silent=True
                    )  
                    self.root.after(
                        0, lambda p=pos, s=state, err=errors: self.update_ui(p, s, err)
                    )
                else:
                    break
            except (
                SMC100CommunicationError,
                SMC100StateError,
                SMC100TimeoutError,
                serial.SerialException,
            ) as e:
                error_msg = f"Error in update thread: {type(e).__name__}"
                print(f"ERROR: {error_msg}")
                
                self.root.after(0, lambda msg=error_msg: self.status_var.set(msg))
                
                time.sleep(1.0)
                
            except Exception as e:
                error_msg = f"Unexpected error in update thread: {e}"
                print(f"ERROR: {error_msg}")
                self.root.after(
                    0, lambda msg=error_msg: self.status_var.set("Update Error")
                )
                time.sleep(1.0)  # Pause longer

            elapsed = time.monotonic() - start_time
            sleep_time = max(0, polling_interval - elapsed)
            if self.stop_thread:
                break  
            time.sleep(sleep_time)
        print("Status polling loop stopped.")

    def update_ui(self, position, state_code, error_code):
        """Update GUI labels from the main thread."""
        if not self.connected:  
            return

        pos_unit = (
            "mm (Abs)" if not self.software_limits_active else "mm"
        ) 
        self.position_var.set(f"{position:.3f} {pos_unit}")

        state_descriptions = {
            "0A": "NOT REFERENCED from reset",
            "0B": "NOT REFERENCED from HOMING",
            "0C": "NOT REFERENCED from CONFIG",
            "0D": "NOT REFERENCED from DISABLE",
            "0E": "NOT REFERENCED from READY",
            "0F": "NOT REFERENCED from MOVING",
            "10": "NOT REFERENCED ESP error",
            "11": "NOT REFERENCED from JOGGING",
            "14": "CONFIGURATION",
            "1E": "HOMING (CMD)",
            "1F": "HOMING (Keypad)",
            "28": "MOVING",
            "32": "READY from HOMING",
            "33": "READY from MOVING",
            "34": "READY from DISABLE",
            "35": "READY from JOGGING",
            "3C": "DISABLE from READY",
            "3D": "DISABLE from MOVING",
            "3E": "DISABLE from JOGGING",
            "46": "JOGGING from READY",
            "47": "JOGGING from DISABLE",
        }
        state_text = state_descriptions.get(state_code, f"Unknown ({state_code})")

        if error_code != 0:
            state_text += f" ERR:0x{error_code:04X}"

        self.status_var.set(state_text)

    def execute_home(self):
        """Execute the currently configured controller homing sequence."""
        if not self.controller:
            return
        print("Executing hardware home...")
        try:
            errors, state = self.controller.get_status(silent=True)

            if not state.startswith("0"):
                messagebox.showwarning(
                    "Homing", "Stage must be in a 'NOT REFERENCED' state to home."
                )
                print(f"Homing denied. Current state: {state}")

                return

            ht = self.home_type_var.get().split(" ")[0]
            print(
                f"Sending OR command (Controller should use its configured HT={ht})..."
            )
            self.status_var.set(f"Homing (HT={ht})...")

            self.controller.home(wait=True)

            pos = self.controller.get_position_mm()
            self.position_var.set(f"{pos:.3f} mm (Abs)")
            self.status_var.set("Homing Complete")
            print("Homing complete.")
            messagebox.showinfo("Home", "Controller Homing Sequence Executed.")

        except Exception as e:
            messagebox.showerror("Homing Error", f"Failed to execute home: {str(e)}")
            print(f"Homing error: {e}")
            self.status_var.set("Homing Failed")

    def set_home_type(self):
        """Set the controller's homing type (HT command). Requires Config mode."""
        if not self.controller:
            return
        try:
            home_type = int(self.home_type_var.get().split(" ")[0]) 
            print(f"Setting controller Home Type (HT) to {home_type}...")

            self.status_var.set(f"Setting HT={home_type}...")
            self.controller.execute_command(SMC100Command.ENTER_CONFIGURATION, 1)
            self.controller.execute_command(SMC100Command.HOME_SEARCH_TYPE, home_type)
            self.controller.execute_command(SMC100Command.ENTER_CONFIGURATION, 0)
            time.sleep(1.0)  # Small delay

            # current_ht = int(self.controller.execute_command(SMC100Command.HOME_SEARCH_TYPE, query=True))
            # print(f"Verified HT is now {current_ht}")

            self.status_var.set("Ready")
            messagebox.showinfo(
                "Success", f"Controller Home Type (HT) set to {home_type}"
            )

        except Exception as e:
            messagebox.showerror("Error", f"Failed to set home type: {str(e)}")
            print(f"Set Home Type error: {e}")
            self.status_var.set("Error")

    def stop_motion(self):
        """Send the STOP command to the controller."""
        if not self.controller or not self.connected:
            print("Cannot stop: Not connected.")
            return
        print("Sending STOP command...")
        try:
            self.controller.execute_command(SMC100Command.STOP_MOTION)
            print("STOP command sent.")
        except Exception as e:
            messagebox.showerror("Stop Error", f"Failed to send stop command: {str(e)}")
            print(f"Stop command error: {e}")

    def step_stage(self, direction: int) -> None:
        """Move the stage by the defined step distance, checking virtual limits."""
        if not self.controller or not self.connected:
            return
        print(f"Initiating step: Direction={direction}")
        try:
            current_pos_abs = self.controller.get_position_mm()
            step_dist = direction * self.move_distance_var.get()
            target_pos_abs = current_pos_abs + step_dist

            if self.is_within_virtual_limits(target_pos_abs):
                print(
                    f"Commanding relative move by {step_dist:.3f}mm from {current_pos_abs:.3f}mm"
                )
                self.status_var.set("Stepping...")
                self.controller.execute_command(
                    SMC100Command.SET_VELOCITY, self.move_speed_var.get()
                )
                self.controller.move_relative_mm(step_dist, waitStop=True)
                print("Step move complete.")
                self.status_var.set("Ready")

        except Exception as e:
            messagebox.showerror("Step Error", f"Failed to step: {str(e)}")
            print(f"Step error: {e}")
            try:
                _, state = self.controller.get_status(silent=True)
                self.update_ui(self.controller.get_position_mm(), state, 0)
            except Exception as e:
                self.status_var.set(f"Error during step {e}")

    def move_absolute(self):
        """Move stage to the absolute position specified in the entry field."""
        if not self.controller or not self.connected:
            return

        try:
            target_abs = float(self.abs_target_entry.get())
            print(f"Initiating absolute move to {target_abs:.3f}mm")

            if self.is_within_virtual_limits(target_abs):
                self.status_var.set(f"Moving to {target_abs:.3f}...")
                self.controller.execute_command(
                    SMC100Command.SET_VELOCITY, self.move_speed_var.get()
                )
                self.controller.move_absolute_mm(target_abs, wait=False)
                print(f"Absolute move to {target_abs:.3f} commanded.")

        except ValueError:
            messagebox.showerror("Input Error", "Invalid target position entered.")
            print("Invalid absolute target value.")
        except Exception as e:
            messagebox.showerror("Move Error", f"Failed absolute move: {str(e)}")
            print(f"Absolute move error: {e}")
            self.status_var.set("Move Error")

    def reset_to_defaults(self):
        """Reset the controller to factory defaults and reload stage parameters."""
        if not self.controller:
            return
        print("Resetting controller to defaults...")
        try:
            if messagebox.askyesno(
                "Reset Configuration",
                "This will reset all controller settings to factory defaults.\n\n"
                "CAUTION: Stage will move during initialization.\n\n"
                "Continue?",
                icon=messagebox.WARNING,
            ):
                self.status_var.set("Resetting controller...")
                self.root.update()
                self.controller.execute_command(SMC100Command.RESET)
                print("Reset command sent. Waiting...")
                time.sleep(3)

                print("Re-initializing stage after reset...")
                self.initialize_stage()

                pos = self.controller.get_position_mm()
                self.position_var.set(f"{pos:.3f} mm (Abs)")
                print("Reset and initialization complete.")

                self.software_limits_active = False
                self.load_virtual_limits()
                self.update_limit_labels()

                messagebox.showinfo(
                    "Reset Complete",
                    "Controller has been reset and stage re-initialized.",
                )
            else:
                print("Reset cancelled by user.")

        except Exception as e:
            messagebox.showerror("Reset Error", f"Failed to reset controller: {str(e)}")
            print(f"Reset error: {e}")
            self.status_var.set("Reset Failed")

    def save_configuration(self):
        """Save current CONTROLLER configuration (SL, SR, VA, AC, HT) to a JSON file."""
        if not self.controller:
            return
        print("Saving controller configuration...")
        try:
            config_dict = {}
            commands_to_query = [
                (SMC100Command.SET_NEGATIVE_SOFTWARE_LIMIT, "SL", float),
                (SMC100Command.SET_POSITIVE_SOFTWARE_LIMIT, "SR", float),
                (SMC100Command.SET_VELOCITY, "VA", float),
                (SMC100Command.SET_ACCELERATION, "AC", float),
                (SMC100Command.HOME_SEARCH_TYPE, "HT", int),
            ]
            print("Querying controller parameters...")
            for cmd_enum, cmd_key, value_type in commands_to_query:
                try:
                    value_str = self.controller.execute_command(cmd_enum, query=True)
                    if value_str is not None:
                        config_dict[cmd_key] = value_type(value_str)
                        print(f"  {cmd_key}: {config_dict[cmd_key]}")
                    else:
                        print(f"  {cmd_key}: Query returned None")
                except Exception as e:
                    print(f"  Error querying {cmd_key}: {str(e)}")

            try:
                id_val = self.controller.execute_command(
                    SMC100Command.GET_IDENTIFIER, query=True
                )
                config_dict["ID"] = id_val
                print(f"  ID: {id_val}")
            except Exception as e:
                print(f"  Error querying ID: {str(e)}")

            file_path = filedialog.asksaveasfilename(
                defaultextension=".json",
                filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
                title="Save Controller Configuration",
                initialfile="smc100_config.json",
            )

            if not file_path:
                print("Save configuration cancelled.")
                return

            with open(file_path, "w") as f:
                json.dump(config_dict, f, indent=4)

            print(f"Controller configuration saved to {file_path}")
            messagebox.showinfo(
                "Success",
                f"Controller configuration saved to {os.path.basename(file_path)}",
            )

        except Exception as e:
            messagebox.showerror(
                "Save Error", f"Failed to save controller configuration: {str(e)}"
            )
            print(f"Save configuration error: {e}")

    def load_configuration(self):
        """Load CONTROLLER configuration (SL, SR, VA, AC, HT) from JSON and apply."""
        if not self.controller:
            return
        print("Loading controller configuration...")
        try:
            file_path = filedialog.askopenfilename(
                defaultextension=".json",
                filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
                title="Load Controller Configuration",
            )

            if not file_path:
                print("Load configuration cancelled.")
                return

            with open(file_path, "r") as f:
                config_dict = json.load(f)

            if not isinstance(config_dict, dict) or not config_dict:
                messagebox.showerror("Error", "Configuration file is empty or invalid.")
                return

            if not messagebox.askyesno(
                "Confirm Load",
                "This will overwrite current controller settings.\n\n"
                "These settings will be saved permanently.\n\n"
                "Continue?",
                icon=messagebox.WARNING,
            ):
                print("Load configuration aborted by user.")
                return

            command_map = {
                "SL": SMC100Command.SET_NEGATIVE_SOFTWARE_LIMIT,
                "SR": SMC100Command.SET_POSITIVE_SOFTWARE_LIMIT,
                "VA": SMC100Command.SET_VELOCITY,
                "AC": SMC100Command.SET_ACCELERATION,
                "HT": SMC100Command.HOME_SEARCH_TYPE,
            }

            self.status_var.set("Loading configuration...")
            self.root.update()
            print("Applying configuration to controller...")

            self.controller.execute_command(SMC100Command.ENTER_CONFIGURATION, 1)
            print("  Entered configuration mode.")

            config_mode_keys = ["SL", "SR", "HT"]
            for key in config_mode_keys:
                if key in config_dict:
                    try:
                        value = config_dict[key]
                        self.controller.execute_command(command_map[key], value)
                        print(f"  Set {key} = {value}")
                    except Exception as e:
                        print(f"  Error setting {key}: {str(e)}")

            self.controller.execute_command(SMC100Command.ENTER_CONFIGURATION, 0)
            print("  Exited configuration mode (Saved SL, SR, HT).")

            non_config_keys = ["VA", "AC"]
            for key in non_config_keys:
                if key in config_dict:
                    try:
                        value = config_dict[key]
                        self.controller.execute_command(command_map[key], value)
                        print(f"  Set {key} = {value} (temporary)")
                        # Note: VA/AC set outside config mode are temporary unless saved again later
                    except Exception as e:
                        print(f"  Error setting {key}: {str(e)}")

            self.status_var.set("Configuration Loaded")
            print("Configuration loaded successfully.")
            messagebox.showinfo(
                "Success", "Configuration loaded and applied to controller."
            )

            self.update_limit_labels()

        except FileNotFoundError:
            messagebox.showerror("Load Error", f"File not found: {file_path}")
            print(f"Load error: File not found {file_path}")
        except json.JSONDecodeError:
            messagebox.showerror(
                "Load Error",
                f"Invalid JSON format in file: {os.path.basename(file_path)}",
            )
            print(f"Load error: Invalid JSON in {file_path}")
        except Exception as e:
            messagebox.showerror(
                "Load Error", f"Failed to load configuration: {str(e)}"
            )
            print(f"Load configuration error: {e}")
            self.status_var.set("Load Failed")
            # Attempt to exit config mode if stuck
            try:
                self.controller.execute_command(SMC100Command.ENTER_CONFIGURATION, 0)
            except Exception:
                pass

    def configure_for_vacuum_chamber(self):
        """Configure stage for vacuum use: Hardware home, center at 12.5mm, set virtual limits."""
        if not self.controller or not self.connected:
            messagebox.showerror("Error", "Controller not connected.")
            return

        print("Starting vacuum chamber configuration process...")
        if not messagebox.askyesno(
            "⚠️ Vacuum Setup Confirmation ⚠️",
            "This process will:\n\n"
            "1. Perform a hardware home (stage will move).\n"
            "2. Move the stage to absolute 12.5 mm.\n"
            "3. Activate virtual limits +/- 1.0 mm around 12.5 mm.\n"
            "4. Save these virtual limits to 'virtual_limits_config.json'.\n"
            "5. Set wide controller limits (e.g., 0-25mm) permanently.\n\n"
            "Ensure stage path is clear. Continue?",
            icon=messagebox.WARNING,
        ):
            print("Vacuum configuration cancelled by user.")
            return

        try:
            self.status_var.set("Starting Vacuum Config...")
            self.root.update()

            print("Step 1: Performing hardware home...")

            current_ht = int(
                self.controller.execute_command(
                    SMC100Command.HOME_SEARCH_TYPE, query=True
                )
            )
            print(f"  Using controller's current Home Type: HT={current_ht}")
            if current_ht not in [0, 2, 3, 4]:
                messagebox.showerror(
                    "Error",
                    f"Unsupported Home Type {current_ht} set on controller. Please set HT 0, 2, 3, or 4 first.",
                )
                return

            self.status_var.set(f"Hardware Homing (HT={current_ht})...")
            self.controller.home(wait=True)
            initial_pos = self.controller.get_position_mm()
            self.status_var.set(f"Home complete (Pos: {initial_pos:.3f})")
            print(f"  Hardware home complete. Position is ~{initial_pos:.3f} mm.")
            if abs(initial_pos) > 0.01:
                print(
                    f"WARNING: Position after home is not zero ({initial_pos:.3f}). Proceeding cautiously."
                )

            target_center_abs = 12.5
            print(f"Step 2: Moving to absolute center {target_center_abs} mm...")
            self.status_var.set(f"Moving to {target_center_abs} mm...")
            self.controller.execute_command(
                SMC100Command.SET_VELOCITY, self.move_speed_var.get()
            )
            self.controller.move_absolute_mm(target_center_abs, wait=True)

            pos = self.controller.get_position_mm()
            print(f"  Position after move: {pos:.5f} mm")
            if abs(pos - target_center_abs) > 0.01:
                raise RuntimeError(
                    f"Failed verification: Stage did not reach {target_center_abs}mm (at {pos:.3f}mm)"
                )
            self.status_var.set(f"Centered at {pos:.3f} mm")
            print(f"  Successfully centered at {pos:.3f} mm.")

            print("Step 3: Setting and saving virtual limits...")
            self.virtual_center_abs = pos  # Use actual reached position
            self.virtual_range = 1.0  # Desired range
            self.virtual_neg_limit_abs = self.virtual_center_abs - self.virtual_range
            self.virtual_pos_limit_abs = self.virtual_center_abs + self.virtual_range
            self.software_limits_active = True
            self.virtual_center_var.set(self.virtual_center_abs)
            self.virtual_range_var.set(self.virtual_range)
            self.update_limit_labels()
            self.save_virtual_limits()
            print(
                f"  Virtual limits active and saved: {self.virtual_neg_limit_abs:.3f} to {self.virtual_pos_limit_abs:.3f}"
            )

            wide_neg_limit = 0.0

            wide_pos_limit = 25.0

            print(
                f"Step 4: Setting wide controller limits SL={wide_neg_limit}, SR={wide_pos_limit} permanently..."
            )
            self.status_var.set("Saving final config...")
            self.controller.execute_command(SMC100Command.ENTER_CONFIGURATION, 1)
            self.controller.execute_command(
                SMC100Command.SET_NEGATIVE_SOFTWARE_LIMIT, wide_neg_limit
            )
            self.controller.execute_command(
                SMC100Command.SET_POSITIVE_SOFTWARE_LIMIT, wide_pos_limit
            )
            self.controller.execute_command(SMC100Command.HOME_SEARCH_TYPE, current_ht)
            self.controller.execute_command(SMC100Command.ENTER_CONFIGURATION, 0)
            print("  Wide controller limits and default HT saved.")

            current_pos = self.controller.get_position_mm()
            self.position_var.set(f"{current_pos:.3f} mm (Abs)")
            self.update_limit_labels()
            self.status_var.set("Vacuum Config Complete (Virtual Limits Active)")
            print("Vacuum configuration process finished.")

            messagebox.showinfo(
                "✓ Vacuum Setup Complete",
                f"Stage configured for vacuum use:\n\n"
                f"• Stage centered at absolute {current_pos:.3f}mm.\n"
                f"• Virtual limits activated: {self.virtual_neg_limit_abs:.3f} mm to {self.virtual_pos_limit_abs:.3f} mm.\n"
                f"• Virtual limits saved to '{self.virtual_config_file}'.\n"
                f"• Controller limits set wide ({wide_neg_limit:.1f} to {wide_pos_limit:.1f} mm).\n"
                f"• Default Home Type set to HT={current_ht}.\n\n"
                f"⚠️ Movement restricted by SOFTWARE.",
                parent=self.root,
            )

        except Exception as e:
            error_msg = f"Vacuum configuration failed: {str(e)}"
            messagebox.showerror("Configuration Error", error_msg)
            print(f"ERROR: {error_msg}")
            self.status_var.set("Vacuum Config Failed")
            self.software_limits_active = False
            self.update_limit_labels()

    def initialize_stage(
        self, move_to_origin: bool = False, home_type: int = 3
    ) -> None:
        """Initialize stage on connection: Set default HT, Home."""
        if not self.controller:
            return
        print("Initializing stage...")
        try:
            for _ in range(2):  # Try twice
                errors, state = self.controller.get_status(silent=True)
                if state == "14":
                    print("  Controller was in config mode, exiting...")
                    self.controller.execute_command(
                        SMC100Command.ENTER_CONFIGURATION, 0
                    )
                    time.sleep(1)  # Give time to exit
                else:
                    break
            else:
                raise RuntimeError("Controller stuck in config mode.")

            print(f"  Setting default Home Type to HT={home_type}...")
            self.controller.execute_command(SMC100Command.ENTER_CONFIGURATION, 1)
            self.controller.execute_command(SMC100Command.HOME_SEARCH_TYPE, home_type)
            self.controller.execute_command(SMC100Command.ENTER_CONFIGURATION, 0)
            self.home_type_var.set(
                f"{home_type} - {dict(enumerate(self.home_type_combo['values']))[home_type].split(' - ')[1]}"
            )
            print("  Default Home Type set.")
            time.sleep(2.0)  # Short delay after config

            print("  Performing initial hardware home...")
            self.status_var.set(f"Homing (HT={home_type})...")
            self.controller.home(
                wait=True, move_to_origin=move_to_origin
            )  # Uses the HT set above

            pos = self.controller.get_position_mm()
            print(f"  Initial homing complete. Position: {pos:.3f}")
            self.position_var.set(f"{pos:.3f} mm (Abs)")
            self.status_var.set("Ready")

        except Exception as e:
            raise RuntimeError(f"Stage initialization failed: {e}") from e


if __name__ == "__main__":
    root = tk.Tk()

    app = LimitFinderApp(root)

    try:
        root.mainloop()
    finally:
        if hasattr(app, "controller") and app.controller:
            print("\nClosing controller connection...")
            app.disconnect()
