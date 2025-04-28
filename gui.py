import json
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk


from smc100.controller import SMC100, SMC100Command


class LimitFinderApp:
    def __init__(self, root):
        self.init(root)

    def initialize_stage(self) -> None:
        """
        Full power-on setup:
         1) Enter CONFIGURATION (PW1)
         2) Load & save stage EEPROM parameters (ZX2)
         3) Exit CONFIGURATION (PW0)
         4) Home the stage (OR), waiting for it to finish
        """
        try:
            self.status_var.set("Initializing…")
            # 1) enter config
            self.controller.execute_command(SMC100Command.ENTER_CONFIGURATION, 1)
            # 2) load stage params
            self.controller.execute_command(SMC100Command.STAGE_PARAMETERS, 1)
            # 3) exit config (can take →10 s)
            self.controller.execute_command(SMC100Command.ENTER_CONFIGURATION, 0)

            # wait for PW0 to finish
            start = time.time()
            while True:
                _, st = self.controller.get_status(silent=True)
                if st != "14":
                    break
                if time.time() - start > 12:
                    raise RuntimeError("Configuration exit timeout")
                time.sleep(0.1)

            # 4) set home search to “current position” (HT=1)
            self.controller.execute_command(SMC100Command.HOME_SEARCH_TYPE, 1)

            # 5) home and wait for READY
            self.controller.home(wait=True)

            pos = self.controller.get_position_mm()
            self.position_var.set(f"{pos:.3f} mm")
            self.status_var.set("Ready")
        except Exception as e:
            messagebox.showerror("Initialization Error", f"Stage init failed:\n{e}")
            self.status_var.set("Init failed")

    def init(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("SMC100 Limit Finder")
        self.root.geometry("800x600")
        self.controller = None
        self.position_var = tk.StringVar(value="Not connected")
        self.jog_speed = tk.DoubleVar(value=1.0)
        self.jog_distance = tk.DoubleVar(value=0.1)
        self.status_var = tk.StringVar(value="Not connected")
        self.neg_limit_var = tk.StringVar(value="Not set")
        self.pos_limit_var = tk.StringVar(value="Not set")
        self.connected = False
        self.stop_thread = False
        self.update_thread = None
        self.create_widgets()

    def create_widgets(self):
        left_frame = ttk.Frame(self.root)
        left_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)

        right_frame = ttk.Frame(self.root)
        right_frame.pack(side="right", fill="both", expand=True, padx=10, pady=10)

        conn_frame = ttk.LabelFrame(left_frame, text="Connection")
        conn_frame.pack(padx=5, pady=5, fill="x")

        conn_grid = ttk.Frame(conn_frame)
        conn_grid.pack(padx=5, pady=5, fill="x")

        ttk.Label(conn_grid, text="Port:").grid(
            row=0, column=0, padx=5, pady=5, sticky="w"
        )
        self.port_entry = ttk.Entry(conn_grid)
        self.port_entry.insert(0, "/dev/ttyUSB0")
        self.port_entry.grid(row=0, column=1, padx=5, pady=5, sticky="ew")

        ttk.Label(conn_grid, text="SMC ID:").grid(
            row=1, column=0, padx=5, pady=5, sticky="w"
        )
        self.smc_id_entry = ttk.Entry(conn_grid, width=5)
        self.smc_id_entry.insert(0, "1")
        self.smc_id_entry.grid(row=1, column=1, padx=5, pady=5, sticky="w")

        self.connect_btn = ttk.Button(
            conn_grid, text="Connect", command=self.toggle_connection
        )
        self.connect_btn.grid(
            row=2, column=0, columnspan=2, padx=5, pady=5, sticky="ew"
        )

        status_frame = ttk.LabelFrame(left_frame, text="Status")
        status_frame.pack(padx=5, pady=5, fill="x")

        status_grid = ttk.Frame(status_frame)
        status_grid.pack(padx=5, pady=5, fill="x")

        ttk.Label(
            status_grid, text="Position:", font=("TkDefaultFont", 10, "bold")
        ).grid(row=0, column=0, padx=5, pady=5, sticky="w")
        ttk.Label(status_grid, textvariable=self.position_var, width=15).grid(
            row=0, column=1, padx=5, pady=5, sticky="w"
        )

        ttk.Label(status_grid, text="Status:", font=("TkDefaultFont", 10, "bold")).grid(
            row=1, column=0, padx=5, pady=5, sticky="w"
        )
        ttk.Label(status_grid, textvariable=self.status_var, width=25).grid(
            row=1, column=1, padx=5, pady=5, sticky="w"
        )

        config_frame = ttk.LabelFrame(left_frame, text="Configuration")
        config_frame.pack(padx=5, pady=5, fill="x")
        config_grid = ttk.Frame(config_frame)
        config_grid.pack(padx=5, pady=5, fill="x")
        self.reset_config_btn = ttk.Button(
            config_grid,
            text="Reset to Defaults",
            command=self.reset_to_defaults,
            state="disabled",
        )
        self.reset_config_btn.grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        self.save_config_btn = ttk.Button(
            config_grid,
            text="Save Configuration",
            command=self.save_configuration,
            state="disabled",
        )
        self.save_config_btn.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        self.load_config_btn = ttk.Button(
            config_grid,
            text="Load Configuration",
            command=self.load_configuration,
            state="disabled",
        )
        self.load_config_btn.grid(row=2, column=0, padx=5, pady=5, sticky="ew")

        limits_frame = ttk.LabelFrame(left_frame, text="Limits")
        limits_frame.pack(padx=5, pady=5, fill="both", expand=True)

        limits_grid = ttk.Frame(limits_frame)
        limits_grid.pack(padx=5, pady=5, fill="both", expand=True)

        ttk.Label(
            limits_grid, text="Negative Limit:", font=("TkDefaultFont", 10, "bold")
        ).grid(row=0, column=0, padx=5, pady=5, sticky="w")
        ttk.Label(limits_grid, textvariable=self.neg_limit_var, width=15).grid(
            row=0, column=1, padx=5, pady=5, sticky="w"
        )

        self.set_neg_btn = ttk.Button(
            limits_grid,
            text="Set Current as Negative Limit",
            command=self.set_negative_limit,
            state="disabled",
        )
        self.set_neg_btn.grid(
            row=1, column=0, columnspan=2, padx=5, pady=5, sticky="ew"
        )

        ttk.Separator(limits_grid, orient="horizontal").grid(
            row=2, column=0, columnspan=2, sticky="ew", pady=10
        )

        vacuum_frame = ttk.LabelFrame(
            self.root, text="⚠️ VACUUM CHAMBER CONFIGURATION ⚠️"
        )
        vacuum_frame.pack(padx=10, pady=10, fill="x")
        ttk.Label(
            vacuum_frame,
            text="Configure BEFORE placing in vacuum chamber:",
            font=("TkDefaultFont", 10, "bold"),
        ).pack(padx=5, pady=2)

        self.vacuum_config_btn = ttk.Button(
            vacuum_frame,
            text="Set 12.5mm as Home with Travel Limits",
            command=self.configure_for_vacuum_chamber,
            state="disabled",
        )
        self.vacuum_config_btn.pack(padx=5, pady=5, fill="x")

        ttk.Label(
            vacuum_frame,
            text="⚠️ After configuration, HOME will be DISABLED",
            foreground="red",
        ).pack(padx=5, pady=2)

        ttk.Label(
            limits_grid, text="Positive Limit:", font=("TkDefaultFont", 10, "bold")
        ).grid(row=3, column=0, padx=5, pady=5, sticky="w")
        ttk.Label(limits_grid, textvariable=self.pos_limit_var, width=15).grid(
            row=3, column=1, padx=5, pady=5, sticky="w"
        )

        self.set_pos_btn = ttk.Button(
            limits_grid,
            text="Set Current as Positive Limit",
            command=self.set_positive_limit,
            state="disabled",
        )
        self.set_pos_btn.grid(
            row=4, column=0, columnspan=2, padx=5, pady=5, sticky="ew"
        )

        ttk.Separator(limits_grid, orient="horizontal").grid(
            row=5, column=0, columnspan=2, sticky="ew", pady=10
        )

        self.apply_temp_btn = ttk.Button(
            limits_grid,
            text="Apply Limits Temporarily",
            command=self.apply_limits_temp,
            state="disabled",
        )
        self.apply_temp_btn.grid(
            row=6, column=0, columnspan=2, padx=5, pady=5, sticky="ew"
        )

        self.apply_perm_btn = ttk.Button(
            limits_grid,
            text="Save Limits Permanently",
            command=self.apply_limits_perm,
            state="disabled",
        )
        self.apply_perm_btn.grid(
            row=7, column=0, columnspan=2, padx=5, pady=5, sticky="ew"
        )

        home_frame = ttk.LabelFrame(left_frame, text="Home Position")
        home_frame.pack(padx=5, pady=5, fill="x")

        # home_frame = ttk.LabelFrame(self.root, text="Home Position")
        # home_frame.pack(padx=10, pady=5, fill="x")

        self.home_type_var = tk.StringVar(value="1")
        ttk.Label(home_frame, text="Home Type:").grid(row=0, column=0, padx=5, pady=5)
        home_type_combo = ttk.Combobox(
            home_frame, textvariable=self.home_type_var, width=30, state="readonly"
        )
        home_type_combo["values"] = (
            "0 - Use MZ switch and encoder index",
            "1 - Use current position as HOME",
            "2 - Use MZ switch only",
            "3 - Use EoR- switch and encoder index",
            "4 - Use EoR- switch only",
        )
        home_type_combo.current(1)
        home_type_combo.grid(row=0, column=1, columnspan=2, padx=5, pady=5, sticky="w")

        self.set_home_type_btn = ttk.Button(
            home_frame,
            text="Set Home Type",
            command=self.set_home_type,
            state="disabled",
        )
        self.set_home_type_btn.grid(row=1, column=0, padx=5, pady=5)

        self.execute_home_btn = ttk.Button(
            home_frame, text="Execute Home", command=self.execute_home, state="disabled"
        )
        self.execute_home_btn.grid(row=1, column=1, padx=5, pady=5)

        jog_frame = ttk.LabelFrame(right_frame, text="Motion Control")
        jog_frame.pack(padx=5, pady=5, fill="both", expand=True)

        params_frame = ttk.Frame(jog_frame)
        params_frame.pack(padx=5, pady=5, fill="x")

        ttk.Label(
            params_frame, text="Speed (mm/s):", font=("TkDefaultFont", 10, "bold")
        ).grid(row=0, column=0, padx=5, pady=5, sticky="w")
        ttk.Spinbox(
            params_frame,
            from_=0.1,
            to=20,
            increment=0.1,
            textvariable=self.jog_speed,
            width=8,
        ).grid(row=0, column=1, padx=5, pady=5, sticky="w")

        ttk.Label(
            params_frame, text="Step (mm):", font=("TkDefaultFont", 10, "bold")
        ).grid(row=1, column=0, padx=5, pady=5, sticky="w")
        ttk.Spinbox(
            params_frame,
            from_=0.001,
            to=10,
            increment=0.001,
            textvariable=self.jog_distance,
            width=8,
        ).grid(row=1, column=1, padx=5, pady=5, sticky="w")

        # Motion buttons
        motion_frame = ttk.Frame(jog_frame)
        motion_frame.pack(padx=5, pady=10, fill="both", expand=True)

        # Step buttons
        step_frame = ttk.LabelFrame(motion_frame, text="Step Movement")
        step_frame.pack(padx=5, pady=5, fill="x")

        step_btn_frame = ttk.Frame(step_frame)
        step_btn_frame.pack(padx=5, pady=5, fill="x")

        self.step_neg_btn = ttk.Button(
            step_btn_frame,
            text="← Step",
            command=lambda: self.step_stage(-1),
            state="disabled",
            width=15,
        )
        self.step_neg_btn.pack(side="left", padx=5, pady=10, expand=True)

        self.step_pos_btn = ttk.Button(
            step_btn_frame,
            text="Step →",
            command=lambda: self.step_stage(1),
            state="disabled",
            width=15,
        )
        self.step_pos_btn.pack(side="right", padx=5, pady=10, expand=True)

        jog_label_frame = ttk.LabelFrame(motion_frame, text="Continuous Movement")
        jog_label_frame.pack(padx=5, pady=5, fill="x")

        jog_btn_frame = ttk.Frame(jog_label_frame)
        jog_btn_frame.pack(padx=5, pady=5, fill="x")

        self.jog_neg_btn = ttk.Button(
            jog_btn_frame,
            text="← Jog",
            state="disabled",
            width=15,
        )
        self.jog_neg_btn.pack(side="left", padx=5, pady=10, expand=True)

        self.jog_pos_btn = ttk.Button(
            jog_btn_frame,
            text="Jog →",
            state="disabled",
            width=15,
        )
        self.jog_pos_btn.pack(side="right", padx=5, pady=10, expand=True)

        self.jog_neg_btn.bind("<ButtonPress-1>", lambda e: self.on_jog_press(-1))
        self.jog_neg_btn.bind("<ButtonRelease-1>", lambda e: self.on_jog_release())
        self.jog_pos_btn.bind("<ButtonPress-1>", lambda e: self.on_jog_press(1))
        self.jog_pos_btn.bind("<ButtonRelease-1>", lambda e: self.on_jog_release())

        stop_frame = ttk.Frame(motion_frame)
        stop_frame.pack(padx=5, pady=10, fill="x")

        self.stop_btn = ttk.Button(
            stop_frame,
            text="STOP",
            command=self.stop_motion,
            state="disabled",
            style="Stop.TButton",
        )
        self.stop_btn.pack(fill="x", padx=5, pady=5)

        style = ttk.Style()
        style.configure(
            "Stop.TButton", font=("TkDefaultFont", 12, "bold"), foreground="red"
        )

        self.root.geometry("800x600")

    def toggle_connection(self):
        if not self.controller:
            threading.Thread(target=self.connect, daemon=True).start()
        else:
            self.disconnect()

    def connect(self):
        try:
            port = self.port_entry.get()
            smc_id = int(self.smc_id_entry.get())
            self.controller = SMC100(smcID=smc_id, port=port, silent=False)

            self.initialize_stage()

            pos = self.controller.get_position_mm()
            self.position_var.set(f"{pos:.3f} mm")

            try:
                neg_limit = float(
                    self.controller.execute_command(
                        SMC100Command.SET_NEGATIVE_SOFTWARE_LIMIT, query=True
                    )
                )
                self.neg_limit_var.set(f"{neg_limit:.3f} mm")
            except:
                pass

            try:
                pos_limit = float(
                    self.controller.execute_command(
                        SMC100Command.SET_POSITIVE_SOFTWARE_LIMIT, query=True
                    )
                )
                self.pos_limit_var.set(f"{pos_limit:.3f} mm")
            except:
                pass

            self.connected = True
            self.connect_btn.config(text="Disconnect")
            self.jog_neg_btn.config(state="normal")
            self.jog_pos_btn.config(state="normal")
            self.stop_btn.config(state="normal")
            self.set_neg_btn.config(state="normal")
            self.set_pos_btn.config(state="normal")
            self.apply_temp_btn.config(state="normal")
            self.apply_perm_btn.config(state="normal")
            self.step_neg_btn.config(state="normal")
            self.step_pos_btn.config(state="normal")
            self.set_home_type_btn.config(state="normal")
            self.execute_home_btn.config(state="normal")
            self.reset_config_btn.config(state="normal")
            self.save_config_btn.config(state="normal")
            self.load_config_btn.config(state="normal")
            self.vacuum_config_btn.config(state="normal")
            self.stop_thread = False
            self.update_thread = threading.Thread(
                target=self.update_position_loop, daemon=True
            )
            self.update_thread.start()

        except Exception as e:
            messagebox.showerror("Connection Error", f"Failed to connect: {str(e)}")

    def disconnect(self):
        self.stop_thread = True
        if self.update_thread:
            self.update_thread.join(timeout=1.0)

        if self.controller:
            self.controller.close()
            self.controller = None

        self.connected = False
        self.connect_btn.config(text="Connect")
        self.position_var.set("Not connected")
        self.status_var.set("Not connected")
        self.jog_neg_btn.config(state="disabled")
        self.jog_pos_btn.config(state="disabled")
        self.stop_btn.config(state="disabled")
        self.set_neg_btn.config(state="disabled")
        self.set_pos_btn.config(state="disabled")
        self.apply_temp_btn.config(state="disabled")
        self.apply_perm_btn.config(state="disabled")
        self.step_neg_btn.config(state="disabled")
        self.step_pos_btn.config(state="disabled")
        self.set_home_type_btn.config(state="disabled")
        self.execute_home_btn.config(state="disabled")
        self.reset_config_btn.config(state="disabled")
        self.save_config_btn.config(state="disabled")
        self.load_config_btn.config(state="disabled")
        self.vacuum_config_btn.config(state="disabled")

    def update_position_loop(self):
        while not self.stop_thread:
            try:
                pos = self.controller.get_position_mm()
                errors, state = self.controller.get_status(silent=True)

                self.root.after(0, lambda p=pos, s=state: self.update_ui(p, s))

                time.sleep(0.1)
            except Exception as e:
                error = str(e)
                print(f"Error in update thread: {error}")
                self.root.after(0, lambda: self.status_var.set(f"Error: {error}"))
                time.sleep(1.0)

    def update_ui(self, position, state):
        self.position_var.set(f"{position:.3f} mm")

        state_descriptions = {
            "0A": "NOT REFERENCED from reset",
            "0B": "NOT REFERENCED from HOMING",
            "0C": "NOT REFERENCED from CONFIGURATION",
            "0D": "NOT REFERENCED from DISABLE",
            "0E": "NOT REFERENCED from READY",
            "0F": "NOT REFERENCED from MOVING",
            "14": "CONFIGURATION",
            "1E": "HOMING",
            "28": "MOVING",
            "32": "READY from HOMING",
            "33": "READY from MOVING",
            "34": "READY from DISABLE",
            "3C": "DISABLE from READY",
            "3D": "DISABLE from MOVING",
        }

        state_text = state_descriptions.get(state, f"Unknown state: {state}")
        self.status_var.set(state_text)

    def reset_to_defaults(self):
        """Reset the controller to factory defaults and reload stage parameters."""
        if not self.controller:
            return
        try:
            if messagebox.askyesno(
                "Reset Configuration",
                "This will reset all controller settings to factory defaults.\n\n"
                "CAUTION: Stage will move during initialization.\n\n"
                "Continue?",
            ):

                self.status_var.set("Resetting controller...")

                self.controller.execute_command(SMC100Command.RESET)

                time.sleep(3)

                self.initialize_stage()

                pos = self.controller.get_position_mm()
                self.position_var.set(f"{pos:.3f} mm")

                try:
                    neg_limit = float(
                        self.controller.execute_command(
                            SMC100Command.SET_NEGATIVE_SOFTWARE_LIMIT, query=True
                        )
                    )
                    self.neg_limit_var.set(f"{neg_limit:.3f} mm")

                    pos_limit = float(
                        self.controller.execute_command(
                            SMC100Command.SET_POSITIVE_SOFTWARE_LIMIT, query=True
                        )
                    )
                    self.pos_limit_var.set(f"{pos_limit:.3f} mm")
                except:
                    self.neg_limit_var.set("Not set")
                    self.pos_limit_var.set("Not set")

                messagebox.showinfo(
                    "Reset Complete",
                    "Controller has been reset and stage parameters reloaded.",
                )

        except Exception as e:
            messagebox.showerror("Reset Error", f"Failed to reset controller: {str(e)}")
            self.status_var.set("Reset failed")

    def save_configuration(self):
        """Save current controller configuration to a JSON file."""
        if not self.controller:
            return
        try:
            config_dict = {}

            commands_to_query = [
                (SMC100Command.SET_NEGATIVE_SOFTWARE_LIMIT, "SL", float),
                (SMC100Command.SET_POSITIVE_SOFTWARE_LIMIT, "SR", float),
                (SMC100Command.SET_VELOCITY, "VA", float),
                (SMC100Command.SET_ACCELERATION, "AC", float),
                (SMC100Command.HOME_SEARCH_TYPE, "HT", int),
            ]

            for cmd_enum, cmd_key, value_type in commands_to_query:
                try:
                    value = self.controller.execute_command(cmd_enum, query=True)
                    if value is not None:
                        config_dict[cmd_key] = value_type(value)
                except Exception as e:
                    print(f"Error querying {cmd_key}: {str(e)}")

            try:
                config_dict["ID"] = self.controller.execute_command(
                    SMC100Command.GET_IDENTIFIER, query=True
                )
            except:
                pass

            file_path = filedialog.asksaveasfilename(
                defaultextension=".json",
                filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
                title="Save Controller Configuration",
            )

            if not file_path:
                return

            with open(file_path, "w") as f:
                json.dump(config_dict, f, indent=4)

            messagebox.showinfo("Success", f"Configuration saved to {file_path}")

        except Exception as e:
            messagebox.showerror(
                "Save Error", f"Failed to save configuration: {str(e)}"
            )

    def load_configuration(self):
        """Load controller configuration from a JSON file."""
        if not self.controller:
            return

        try:
            file_path = filedialog.askopenfilename(
                defaultextension=".json",
                filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
                title="Load Controller Configuration",
            )

            if not file_path:
                return

            with open(file_path, "r") as f:
                config_dict = json.load(f)

            if not config_dict:
                messagebox.showerror("Error", "Configuration file is empty or invalid")
                return

            if not messagebox.askyesno(
                "Confirm Load",
                "This will overwrite current controller settings.\n\n" "Continue?",
            ):
                return

            command_map = {
                "SL": SMC100Command.SET_NEGATIVE_SOFTWARE_LIMIT,
                "SR": SMC100Command.SET_POSITIVE_SOFTWARE_LIMIT,
                "VA": SMC100Command.SET_VELOCITY,
                "AC": SMC100Command.SET_ACCELERATION,
                "HT": SMC100Command.HOME_SEARCH_TYPE,
            }

            self.status_var.set("Loading configuration...")
            need_config_mode = any(
                key in ["SL", "SR", "HT"] for key in config_dict.keys()
            )

            if need_config_mode:
                self.controller.execute_command(SMC100Command.ENTER_CONFIGURATION, 1)

            for cmd_key, value in config_dict.items():
                try:
                    if cmd_key in command_map:
                        self.controller.execute_command(command_map[cmd_key], value)
                except Exception as e:
                    print(f"Error setting {cmd_key}: {str(e)}")

            if need_config_mode:
                self.controller.execute_command(SMC100Command.ENTER_CONFIGURATION, 0)

            self.status_var.set("Configuration loaded")
            messagebox.showinfo(
                "Success", "Configuration loaded and applied to controller"
            )

            try:
                neg_limit = float(
                    self.controller.execute_command(
                        SMC100Command.SET_NEGATIVE_SOFTWARE_LIMIT, query=True
                    )
                )
                self.neg_limit_var.set(f"{neg_limit:.3f} mm")

                pos_limit = float(
                    self.controller.execute_command(
                        SMC100Command.SET_POSITIVE_SOFTWARE_LIMIT, query=True
                    )
                )
                self.pos_limit_var.set(f"{pos_limit:.3f} mm")
            except:
                pass

        except Exception as e:
            messagebox.showerror(
                "Load Error", f"Failed to load configuration: {str(e)}"
            )
            self.status_var.set("Load failed")
            try:
                self.controller.execute_command(SMC100Command.ENTER_CONFIGURATION, 0)
            except:
                pass

    def on_jog_press(self, direction):
        if not self.controller:
            return
        self.jogging = True
        threading.Thread(
            target=self._continuous_jog, args=(direction,), daemon=True
        ).start()

    def on_jog_release(self):
        self.jogging = False
        try:
            self.controller.execute_command(SMC100Command.STOP_MOTION)
        except:
            pass

    def _continuous_jog(self, direction):
        velocity = self.jog_speed.get()
        self.controller.execute_command(SMC100Command.SET_VELOCITY, velocity)

        interval = 0.05  # 50 ms per update
        step = direction * velocity * interval

        while self.jogging:
            try:
                self.controller.execute_command(SMC100Command.MOVE_RELATIVE, step)
            except Exception:
                break
            time.sleep(interval)

    def jog_stage(self, direction):
        if not self.controller:
            return

        try:
            self.controller.execute_command(
                SMC100Command.SET_VELOCITY, self.jog_speed.get()
            )

            distance = direction * self.jog_distance.get()

            self.controller.move_relative_mm(distance, waitStop=False)

        except Exception as e:
            messagebox.showerror("Movement Error", f"Failed to jog: {str(e)}")

    def step_stage(self, direction: int) -> None:
        """
        Move by one step (self.jog_distance) and wait for it to finish.
        direction: +1 or -1
        """
        if not self.controller:
            return
        try:
            self.controller.execute_command(
                SMC100Command.SET_VELOCITY, self.jog_speed.get()
            )
            self.controller.move_relative_mm(
                direction * self.jog_distance.get(), waitStop=True
            )
        except Exception as e:
            messagebox.showerror("Step Error", f"Failed to step: {e}")

    def stop_motion(self):
        if not self.controller:
            return

        try:
            self.controller.execute_command(SMC100Command.STOP_MOTION)
        except Exception as e:
            messagebox.showerror("Stop Error", f"Failed to stop motion: {str(e)}")

    def set_negative_limit(self):
        if not self.controller:
            return

        try:
            pos = self.controller.get_position_mm()
            self.neg_limit_var.set(f"{pos:.3f} mm")
        except Exception as e:
            messagebox.showerror(
                "Limit Error", f"Failed to set negative limit: {str(e)}"
            )

    def set_positive_limit(self):
        if not self.controller:
            return

        try:
            pos = self.controller.get_position_mm()
            self.pos_limit_var.set(f"{pos:.3f} mm")
        except Exception as e:
            messagebox.showerror(
                "Limit Error", f"Failed to set positive limit: {str(e)}"
            )

    def set_home_type(self):
        if not self.controller:
            return

        try:
            home_type = int(self.home_type_var.get()[0])

            errors, state = self.controller.get_status(silent=True)

            if state != "14":
                self.controller.execute_command(SMC100Command.ENTER_CONFIGURATION, 1)

            self.controller.execute_command(SMC100Command.HOME_SEARCH_TYPE, home_type)

            if state != "14":
                self.controller.execute_command(SMC100Command.ENTER_CONFIGURATION, 0)

            messagebox.showinfo("Success", f"Home type set to {home_type}")

        except Exception as e:
            messagebox.showerror("Error", f"Failed to set home type: {str(e)}")

    def execute_home(self):
        if not self.controller:
            return
        try:
            errors, state = self.controller.get_status(silent=True)

            if not state.startswith("0"):
                if messagebox.askyesno(
                    "Reset Required",
                    "Controller must be reset before homing. This will stop all motion. Proceed?",
                ):
                    self.controller.execute_command(SMC100Command.RESET)
                    time.sleep(3)
                else:
                    return

            self.controller.execute_command(SMC100Command.HOME)

            messagebox.showinfo("Home", "Home command executed")

        except Exception as e:
            messagebox.showerror("Error", f"Failed to execute home: {str(e)}")

    def apply_limits_temp(self):
        if not self.controller:
            return

        try:
            neg_limit = self.neg_limit_var.get()
            pos_limit = self.pos_limit_var.get()

            if neg_limit == "Not set" or pos_limit == "Not set":
                messagebox.showwarning(
                    "Limits Not Set", "Please set both limits first."
                )
                return

            neg_limit = float(neg_limit.split()[0])
            pos_limit = float(pos_limit.split()[0])

            self.controller.execute_command(
                SMC100Command.SET_NEGATIVE_SOFTWARE_LIMIT, neg_limit
            )
            self.controller.execute_command(
                SMC100Command.SET_POSITIVE_SOFTWARE_LIMIT, pos_limit
            )

            messagebox.showinfo(
                "Success",
                "Limits applied temporarily. These will be lost after controller reset.",
            )

        except Exception as e:
            messagebox.showerror("Limit Error", f"Failed to apply limits: {str(e)}")

    def apply_limits_perm(self):
        if not self.controller:
            return

        try:
            neg_limit = self.neg_limit_var.get()
            pos_limit = self.pos_limit_var.get()

            if neg_limit == "Not set" or pos_limit == "Not set":
                messagebox.showwarning(
                    "Limits Not Set", "Please set both limits first."
                )
                return

            neg_limit = float(neg_limit.split()[0])
            pos_limit = float(pos_limit.split()[0])

            self.controller.execute_command(SMC100Command.ENTER_CONFIGURATION, 1)

            self.controller.execute_command(
                SMC100Command.SET_NEGATIVE_SOFTWARE_LIMIT, neg_limit
            )
            self.controller.execute_command(
                SMC100Command.SET_POSITIVE_SOFTWARE_LIMIT, pos_limit
            )

            self.controller.execute_command(SMC100Command.ENTER_CONFIGURATION, 0)

            messagebox.showinfo(
                "Success", "Limits saved permanently to controller memory."
            )

        except Exception as e:
            messagebox.showerror("Limit Error", f"Failed to save limits: {str(e)}")

    def configure_for_vacuum_chamber(self):
        """
        Configure the stage for safe operation in a vacuum chamber with fragile samples.
        This implementation uses ONLY position-based homing and tight travel limits.
        """
        if not self.controller:
            return
        try:
            if not messagebox.askyesno(
                "⚠️ VACUUM CONFIGURATION ⚠️",
                "This function will:\n\n"
                "1. Find hardware home reference (requires movement)\n"
                "2. Move to exactly 12.5mm position\n"
                "3. Set this position as new zero (WILL EXECUTE HOME ONCE)\n"
                "4. Set limits to ±1.0mm (creating 11.5-13.5mm physical range)\n\n"
                "Continue?",
                icon=messagebox.WARNING,
            ):
                return

            self.status_var.set("Configuring for vacuum chamber...")

            # STEP 1: Hardware homing to establish absolute reference
            ref_type = messagebox.askyesno(
                "Hardware Reference",
                "Which hardware reference method do you want to use?\n\n"
                "YES = End-of-Run with encoder index (HT=3)\n"
                "NO = End-of-Run switch only (HT=4)",
            )

            home_type = 3 if ref_type else 4

            # Configure for hardware homing
            self.controller.execute_command(SMC100Command.ENTER_CONFIGURATION, 1)
            self.controller.execute_command(SMC100Command.HOME_SEARCH_TYPE, home_type)
            self.controller.execute_command(SMC100Command.ENTER_CONFIGURATION, 0)

            start = time.time()
            while True:
                _, st = self.controller.get_status(silent=True)
                if st != "14":
                    break
                if time.time() - start > 12:
                    raise RuntimeError("Configuration exit timeout")
                time.sleep(0.1)

            self.status_var.set(f"Finding hardware reference (HT={home_type})...")
            self.controller.home(wait=True)

            initial_pos = self.controller.get_position_mm()
            self.status_var.set(f"Hardware reference found at {initial_pos:.3f}mm")

            # STEP 2: Move to exactly 12.5mm position
            self.status_var.set("Moving to 12.5mm position...")
            self.controller.move_absolute_mm(12.5)

            # Verify position
            pos = self.controller.get_position_mm()
            if abs(pos - 12.5) > 0.01:
                raise RuntimeError(
                    f"Failed to reach 12.5mm position (current: {pos:.3f}mm)"
                )

            # STEP 3: Now we need to redefine this position as zero
            self.controller.execute_command(SMC100Command.ENTER_CONFIGURATION, 1)
            self.controller.execute_command(SMC100Command.HOME_SEARCH_TYPE, 1)
            self.controller.execute_command(SMC100Command.ENTER_CONFIGURATION, 0)

            # Wait for configuration to complete
            start = time.time()
            while True:
                _, st = self.controller.get_status(silent=True)
                if st != "14":
                    break
                if time.time() - start > 12:
                    raise RuntimeError("Configuration exit timeout")
                time.sleep(0.1)

            # Execute home command once (CRITICAL: this sets current position as zero)
            self.status_var.set("Setting 12.5mm as new zero reference...")
            self.controller.home(wait=True)

            # Verify new position is zero
            new_pos = self.controller.get_position_mm()
            if abs(new_pos) > 0.01:
                raise RuntimeError(
                    f"Failed to set zero reference (position: {new_pos:.3f}mm)"
                )

            # STEP 4: Now set software limits relative to this new zero position
            rel_neg_limit = -1.0  # 1mm in negative direction (11.5mm physical)
            rel_pos_limit = 1.0  # 1mm in positive direction (13.5mm physical)

            self.controller.execute_command(
                SMC100Command.SET_NEGATIVE_SOFTWARE_LIMIT, rel_neg_limit
            )
            self.controller.execute_command(
                SMC100Command.SET_POSITIVE_SOFTWARE_LIMIT, rel_pos_limit
            )

            # STEP 5: Save configuration permanently
            self.controller.execute_command(SMC100Command.ENTER_CONFIGURATION, 1)
            # HT=1 already set, just need to exit to save
            self.controller.execute_command(SMC100Command.ENTER_CONFIGURATION, 0)

            # STEP 6: Save configuration for future reference
            config = {
                "vacuum_config": True,
                "prevent_rehoming": True,
                "physical_position": 12.5,
                "physical_range": [11.5, 13.5],
                "HT": 1,
                "rel_limits": [rel_neg_limit, rel_pos_limit],
                "setup_date": time.strftime("%Y-%m-%d %H:%M:%S"),
            }

            try:
                with open("vacuum_safety_config.json", "w") as f:
                    json.dump(config, f, indent=4)
            except Exception as e:
                print(f"Failed to save config file: {e}")

            # Update UI
            pos = self.controller.get_position_mm()
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

            self.position_var.set(f"{pos:.3f} mm")
            self.neg_limit_var.set(f"{neg_limit:.3f} mm")
            self.pos_limit_var.set(f"{pos_limit:.3f} mm")

            self.status_var.set("Vacuum configuration complete - DO NOT REHOME AGAIN")

            messagebox.showinfo(
                "✓ VACUUM CONFIGURATION COMPLETE",
                f"The stage has been configured for vacuum chamber use:\n\n"
                f"• Physical position 12.5mm is now set as 0.0mm\n"
                f"• Travel limits: {neg_limit:.3f}mm to {pos_limit:.3f}mm\n"
                f"• This creates a physical range of 11.5mm to 13.5mm\n"
                f"• Position-based homing (HT=1) configured for future\n\n"
                f"⚠️ CRITICAL: DO NOT EXECUTE ANY MORE HOME COMMANDS\n"
                f"OR RESET THE CONTROLLER WHILE IN VACUUM CHAMBER!",
            )

        except Exception as e:
            messagebox.showerror("Configuration Error", f"Failed: {str(e)}")
            self.status_var.set("Vacuum configuration failed")

    def connect_with_rehome_prevention(self):
        import os

        """Modified connect method that prevents rehoming"""
        try:
            port = self.port_entry.get()
            smc_id = int(self.smc_id_entry.get())
            if self.controller:
                self.controller.close()

            self.controller = SMC100(smcID=smc_id, port=port, silent=False)

            if os.path.exists("vacuum_safety_config.json"):
                try:
                    with open("vacuum_safety_config.json", "r") as f:
                        config = json.load(f)

                    if config.get("vacuum_config", False) and config.get(
                        "prevent_rehoming", False
                    ):
                        self.status_var.set(
                            "Loading vacuum configuration (no rehoming)..."
                        )

                        self.controller.execute_command(
                            SMC100Command.ENTER_CONFIGURATION, 1
                        )
                        self.controller.execute_command(
                            SMC100Command.HOME_SEARCH_TYPE, 1
                        )
                        self.controller.execute_command(
                            SMC100Command.ENTER_CONFIGURATION, 0
                        )

                        start = time.time()
                        while True:
                            _, st = self.controller.get_status(silent=True)
                            if st != "14":
                                break
                            if time.time() - start > 12:
                                raise RuntimeError("Configuration timeout")
                            time.sleep(0.1)

                        neg_limit = config.get("SL", -1.0)
                        pos_limit = config.get("SR", 1.0)
                        self.controller.execute_command(
                            SMC100Command.SET_NEGATIVE_SOFTWARE_LIMIT, neg_limit
                        )
                        self.controller.execute_command(
                            SMC100Command.SET_POSITIVE_SOFTWARE_LIMIT, pos_limit
                        )

                        abs_pos = config.get("absolute_position", 12.5)
                        phys_range = config.get(
                            "physical_range", [abs_pos - 1.0, abs_pos + 1.0]
                        )

                        pos = self.controller.get_position_mm()
                        self.position_var.set(f"{pos:.3f} mm")
                        self.neg_limit_var.set(f"{neg_limit:.3f} mm")
                        self.pos_limit_var.set(f"{pos_limit:.3f} mm")

                        messagebox.showwarning(
                            "⚠️ VACUUM CONFIGURATION ACTIVE",
                            f"Stage is in vacuum-safe mode:\n\n"
                            f"• Current position: {pos:.3f}mm on relative scale\n"
                            f"• This is approximately {abs_pos+pos:.1f}mm in absolute position\n"
                            f"• Safe travel range: {neg_limit:.1f}mm to {pos_limit:.1f}mm\n"
                            f"  (Physical range: {phys_range[0]:.1f}mm to {phys_range[1]:.1f}mm)\n\n"
                            f"⚠️ HOME COMMANDS DISABLED to prevent damage\n"
                            f"⚠️ DO NOT EXECUTE HOME while in vacuum chamber",
                        )

                        self.disable_home_buttons()

                        self.status_var.set(
                            "Vacuum configuration loaded - HOME DISABLED"
                        )
                    else:
                        self.initialize_stage()
                except Exception as e:
                    self._logger.error(f"Error loading vacuum config: {e}")
                    self.initialize_stage()
            else:
                self.initialize_stage()

            self.connected = True
            self.enable_standard_buttons()

            self.stop_thread = False
            self.update_thread = threading.Thread(target=self.update_position_loop)
            self.update_thread.daemon = True
            self.update_thread.start()

        except Exception as e:
            messagebox.showerror("Connection Error", f"Failed to connect: {str(e)}")

    def disable_home_buttons(self):
        """Disable all home-related buttons to prevent accidental homing"""
        for button_name in dir(self):
            if isinstance(getattr(self, button_name, None), ttk.Button) and any(
                x in button_name.lower() for x in ["home", "init"]
            ):
                button = getattr(self, button_name)
                button.config(state="disabled")
        try:
            if not hasattr(self, "home_warning_label"):
                self.home_warning_label = ttk.Label(
                    self.root,
                    text="⚠️ HOME COMMANDS DISABLED - VACUUM SAFETY MODE ACTIVE ⚠️",
                    foreground="red",
                    font=("TkDefaultFont", 12, "bold"),
                )
                self.home_warning_label.pack(side="top", fill="x", padx=10, pady=5)
        except:
            pass


if __name__ == "__main__":
    root = tk.Tk()
    app = LimitFinderApp(root)
    root.mainloop()
    if app.controller:
        app.controller.close()
