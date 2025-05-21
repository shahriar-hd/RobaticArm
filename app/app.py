import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import pygame
import threading
import time
import json # For saving/loading presets

# --- Configuration ---
BAUD_RATE = 9600
NUM_SERVOS = 5

# Define initial servo angles and their limits
# Format: [current_angle, min_angle, max_angle]
SERVO_CONFIGS = [
    [90, 45, 135],  # Servo 0: Limited 45-135 degrees
    [90, 0, 180],   # Servo 1: Full range
    [90, 0, 90],    # Servo 2: Limited 0-90 degrees
    [90, 0, 180],   # Servo 3: Full range
    [90, 0, 180]    # Servo 4: Full range
]

# Gamepad sensitivity/speed control
GAMEPAD_SENSITIVITY = 1.0 # Adjust this to control how fast joysticks change angles
MOVEMENT_SMOOTHING_FACTOR = 0.1 # Controls how gradually angles change (0.0 - 1.0, lower is smoother)

PRESET_FILE = "servo_presets.json"

class RoboticArmController:
    def __init__(self, master):
        self.master = master
        master.title("Robotic Arm Controller")
        master.geometry("700x700") # Adjust window size as needed

        self.ser = None
        self.connected = False
        self.joystick = None
        self.gamepad_active = False

        self.servo_angles = [config[0] for config in SERVO_CONFIGS]
        self.min_angles = [config[1] for config in SERVO_CONFIGS]
        self.max_angles = [config[2] for config in SERVO_CONFIGS]

        self.presets = self.load_presets()

        self.setup_gui()
        self.setup_threads()

        # Periodically check for gamepad input and update servos
        self.master.after(50, self.update_gamepad_input)

    def setup_gui(self):
        # --- Connection Section ---
        conn_frame = ttk.LabelFrame(self.master, text="Serial Connection")
        conn_frame.pack(padx=10, pady=5, fill="x")

        ttk.Label(conn_frame, text="COM Port:").grid(row=0, column=0, padx=5, pady=5)
        self.port_selection = ttk.Combobox(conn_frame, state="readonly")
        self.port_selection.grid(row=0, column=1, padx=5, pady=5)
        self.refresh_ports()
        self.refresh_button = ttk.Button(conn_frame, text="Refresh", command=self.refresh_ports)
        self.refresh_button.grid(row=0, column=2, padx=5, pady=5)

        self.connect_button = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=3, padx=5, pady=5)

        self.connection_status = ttk.Label(conn_frame, text="Status: Disconnected", foreground="red")
        self.connection_status.grid(row=0, column=4, padx=5, pady=5)

        # --- Servo Control Section ---
        servo_frame = ttk.LabelFrame(self.master, text="Servo Control (Angles 0-180)")
        servo_frame.pack(padx=10, pady=5, fill="x")

        self.servo_sliders = []
        self.angle_labels = []
        for i in range(NUM_SERVOS):
            ttk.Label(servo_frame, text=f"Servo {i}:").grid(row=i, column=0, padx=5, pady=2, sticky="w")
            
            # Display min/max limits
            limits_text = f"({self.min_angles[i]} - {self.max_angles[i]})"
            ttk.Label(servo_frame, text=limits_text).grid(row=i, column=1, padx=2, pady=2, sticky="w")

            slider = ttk.Scale(servo_frame, from_=0, to=180, orient="horizontal",
                               command=lambda val, idx=i: self.set_servo_angle_from_gui(idx, int(float(val))))
            slider.set(self.servo_angles[i]) # Set initial position
            slider.grid(row=i, column=2, padx=5, pady=2, sticky="ew")
            self.servo_sliders.append(slider)

            angle_label = ttk.Label(servo_frame, text=f"{self.servo_angles[i]}°")
            angle_label.grid(row=i, column=3, padx=5, pady=2, sticky="w")
            self.angle_labels.append(angle_label)

        servo_frame.grid_columnconfigure(2, weight=1) # Make slider expand

        # --- Speed Control Section ---
        speed_frame = ttk.LabelFrame(self.master, text="Movement Speed Control")
        speed_frame.pack(padx=10, pady=5, fill="x")

        ttk.Label(speed_frame, text="Global Smoothing:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.smoothing_slider = ttk.Scale(speed_frame, from_=0.01, to=1.0, orient="horizontal",
                                         command=self.set_smoothing_factor)
        self.smoothing_slider.set(MOVEMENT_SMOOTHING_FACTOR)
        self.smoothing_slider.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        self.smoothing_label = ttk.Label(speed_frame, text=f"{MOVEMENT_SMOOTHING_FACTOR:.2f}")
        self.smoothing_label.grid(row=0, column=2, padx=5, pady=5, sticky="w")
        speed_frame.grid_columnconfigure(1, weight=1)

        # --- Position Presets Section ---
        preset_frame = ttk.LabelFrame(self.master, text="Position Presets")
        preset_frame.pack(padx=10, pady=5, fill="x")

        self.preset_name_entry = ttk.Entry(preset_frame)
        self.preset_name_entry.grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        self.save_button = ttk.Button(preset_frame, text="Save Current Pos", command=self.save_position)
        self.save_button.grid(row=0, column=1, padx=5, pady=5)

        self.preset_selection = ttk.Combobox(preset_frame, state="readonly")
        self.preset_selection.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        self.preset_selection.bind("<<ComboboxSelected>>", self.update_go_button_state)
        self.go_button = ttk.Button(preset_frame, text="Go to Saved Pos", command=self.go_to_position, state="disabled")
        self.go_button.grid(row=1, column=1, padx=5, pady=5)

        self.delete_button = ttk.Button(preset_frame, text="Delete Selected Pos", command=self.delete_position, state="disabled")
        self.delete_button.grid(row=1, column=2, padx=5, pady=5)
        
        preset_frame.grid_columnconfigure(0, weight=1)
        self.populate_presets_dropdown()

        # --- Gamepad Status ---
        gamepad_status_frame = ttk.LabelFrame(self.master, text="Gamepad Status")
        gamepad_status_frame.pack(padx=10, pady=5, fill="x")
        self.gamepad_status_label = ttk.Label(gamepad_status_frame, text="Gamepad: Not Detected", foreground="red")
        self.gamepad_status_label.pack(padx=5, pady=5)


    def refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]
        self.port_selection['values'] = port_list
        if port_list:
            self.port_selection.set(port_list[0]) # Select the first port by default

    def toggle_connection(self):
        if self.connected:
            self.disconnect_arduino()
        else:
            selected_port = self.port_selection.get()
            if selected_port:
                self.connect_arduino(selected_port)
            else:
                messagebox.showwarning("No Port Selected", "Please select a serial port.")

    def connect_arduino(self, port):
        try:
            self.ser = serial.Serial(port, BAUD_RATE, timeout=1)
            time.sleep(2) # Allow time for connection to establish
            self.connected = True
            self.connection_status.config(text=f"Status: Connected to {port}", foreground="green")
            self.connect_button.config(text="Disconnect")
            self.port_selection.config(state="disabled")
            self.refresh_button.config(state="disabled")
            messagebox.showinfo("Connected", f"Successfully connected to Arduino on {port}")
            self.master.after(100, self.send_initial_angles) # Send current angles after connection
        except serial.SerialException as e:
            messagebox.showerror("Connection Error", f"Could not connect to {port}: {e}")
            self.connected = False

    def disconnect_arduino(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.connected = False
            self.connection_status.config(text="Status: Disconnected", foreground="red")
            self.connect_button.config(text="Connect")
            self.port_selection.config(state="readonly")
            self.refresh_button.config(state="normal")
            messagebox.showinfo("Disconnected", "Disconnected from Arduino.")

    def send_angles_to_arduino(self):
        if self.connected and self.ser:
            # Apply angle limits before sending
            limited_angles = []
            for i in range(NUM_SERVOS):
                limited_angles.append(
                    int(max(self.min_angles[i], min(self.max_angles[i], self.servo_angles[i])))
                )
            data_to_send = ",".join(map(str, limited_angles)) + "\n"
            try:
                self.ser.write(data_to_send.encode())
            except serial.SerialException as e:
                print(f"Serial write error: {e}")
                self.disconnect_arduino() # Attempt to disconnect on write error

    def send_initial_angles(self):
        # Send current angles to ensure servos are in sync with GUI
        self.send_angles_to_arduino()

    def set_servo_angle_from_gui(self, servo_idx, angle):
        # Apply limits to angle set by GUI slider
        constrained_angle = int(max(self.min_angles[servo_idx], min(self.max_angles[servo_idx], angle)))
        self.servo_angles[servo_idx] = constrained_angle
        self.angle_labels[servo_idx].config(text=f"{constrained_angle}°")
        # Update slider in case it was outside limits (e.g., if we were to allow setting outside limits initially)
        self.servo_sliders[servo_idx].set(constrained_angle)
        self.send_angles_to_arduino()

    def update_gamepad_input(self):
        pygame.event.pump() # Process pygame events
        current_time = time.time()
        
        # Initialize joystick if not already
        if not self.joystick and pygame.joystick.get_count() > 0:
            try:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self.gamepad_status_label.config(text=f"Gamepad: {self.joystick.get_name()}", foreground="blue")
                self.gamepad_active = True
                print(f"Detected Joystick: {self.joystick.get_name()}")
            except Exception as e:
                self.gamepad_status_label.config(text="Gamepad: Error", foreground="red")
                print(f"Error initializing joystick: {e}")
                self.joystick = None # Reset
                self.gamepad_active = False

        if self.joystick and self.gamepad_active:
            # Handle joystick axis movements for incremental control
            for i in range(self.joystick.get_numaxes()):
                axis_value = self.joystick.get_axis(i)

                # Example mapping: Customize this based on your gamepad and desired control
                # We'll use a simple mapping here, assuming common joystick axes
                if i == 0:  # Left Stick X (e.g., for Servo 0)
                    if abs(axis_value) > 0.1: # Deadzone
                        # Incrementally change angle based on joystick value and sensitivity
                        target_angle = self.servo_angles[0] + (axis_value * GAMEPAD_SENSITIVITY)
                        self.servo_angles[0] = self.apply_smoothing(self.servo_angles[0], target_angle)
                        self.update_gui_angle(0)
                elif i == 1: # Left Stick Y (e.g., for Servo 1) - Inverted for typical robotics
                    if abs(axis_value) > 0.1:
                        target_angle = self.servo_angles[1] + (-axis_value * GAMEPAD_SENSITIVITY) # Inverted
                        self.servo_angles[1] = self.apply_smoothing(self.servo_angles[1], target_angle)
                        self.update_gui_angle(1)
                elif i == 2: # Right Stick X (e.g., for Servo 2)
                    if abs(axis_value) > 0.1:
                        target_angle = self.servo_angles[2] + (axis_value * GAMEPAD_SENSITIVITY)
                        self.servo_angles[2] = self.apply_smoothing(self.servo_angles[2], target_angle)
                        self.update_gui_angle(2)
                elif i == 3: # Right Stick Y (e.g., for Servo 3) - Inverted
                    if abs(axis_value) > 0.1:
                        target_angle = self.servo_angles[3] + (-axis_value * GAMEPAD_SENSITIVITY) # Inverted
                        self.servo_angles[3] = self.apply_smoothing(self.servo_angles[3], target_angle)
                        self.update_gui_angle(3)
                elif i == 4: # Left Trigger (e.g., for Servo 4, adjust mapping as needed)
                    # Triggers usually range from -1 (unpressed) to 1 (fully pressed) on Xbox
                    # If it's 0 to 1, then no need for abs(axis_value) > 0.1, simply check if > 0.01
                    if axis_value > -0.9: # assuming -1 to 1, small deadzone
                        # Map trigger to a range that affects servo 4
                        # For example, move servo 4 only when trigger is pressed
                        target_angle = self.servo_angles[4] + (axis_value * GAMEPAD_SENSITIVITY)
                        self.servo_angles[4] = self.apply_smoothing(self.servo_angles[4], target_angle)
                        self.update_gui_angle(4)


            # You can also map buttons for discrete movements or mode changes
            # for i in range(self.joystick.get_numbuttons()):
            #     if self.joystick.get_button(i):
            #         print(f"Button {i} pressed!")

        self.master.after(50, self.update_gamepad_input) # Schedule next update

    def apply_smoothing(self, current_value, target_value):
        """Gradually moves current_value towards target_value."""
        new_value = current_value + (target_value - current_value) * MOVEMENT_SMOOTHING_FACTOR
        return max(0, min(180, new_value)) # Ensure it stays within 0-180 for smoothing calc

    def update_gui_angle(self, servo_idx):
        # Apply limits before updating GUI, as gamepad might suggest out-of-bounds targets during smoothing
        constrained_angle = int(max(self.min_angles[servo_idx], min(self.max_angles[servo_idx], self.servo_angles[servo_idx])))
        self.servo_angles[servo_idx] = constrained_angle # Ensure internal angle is also constrained
        self.servo_sliders[servo_idx].set(constrained_angle)
        self.angle_labels[servo_idx].config(text=f"{constrained_angle}°")

    def set_smoothing_factor(self, value):
        global MOVEMENT_SMOOTHING_FACTOR
        MOVEMENT_SMOOTHING_FACTOR = float(value)
        self.smoothing_label.config(text=f"{MOVEMENT_SMOOTHING_FACTOR:.2f}")

    def serial_writer_thread(self):
        while True:
            if self.connected:
                self.send_angles_to_arduino()
            time.sleep(0.1) # Send updates every 100ms

    def setup_threads(self):
        # Thread for sending serial data to Arduino
        self.serial_thread = threading.Thread(target=self.serial_writer_thread, daemon=True)
        self.serial_thread.start()

        # Initialize Pygame in a separate thread/way that doesn't block Tkinter
        # Pygame event loop will run in the main thread's after loop.
        pygame.init()
        pygame.joystick.init()

    # --- Preset Management ---
    def load_presets(self):
        try:
            with open(PRESET_FILE, 'r') as f:
                return json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            return {}

    def save_presets(self):
        with open(PRESET_FILE, 'w') as f:
            json.dump(self.presets, f, indent=4)

    def populate_presets_dropdown(self):
        self.preset_selection['values'] = list(self.presets.keys())
        if self.presets:
            self.preset_selection.set(list(self.presets.keys())[0])
            self.update_go_button_state()
        else:
            self.preset_selection.set("")
            self.update_go_button_state()

    def save_position(self):
        preset_name = self.preset_name_entry.get().strip()
        if not preset_name:
            messagebox.showwarning("Input Error", "Please enter a name for the position preset.")
            return
        
        current_angles = [int(s.get()) for s in self.servo_sliders]
        self.presets[preset_name] = current_angles
        self.save_presets()
        self.populate_presets_dropdown()
        messagebox.showinfo("Saved", f"Position '{preset_name}' saved.")

    def go_to_position(self):
        selected_preset = self.preset_selection.get()
        if selected_preset and selected_preset in self.presets:
            target_angles = self.presets[selected_preset]
            for i in range(NUM_SERVOS):
                # Update internal angles and GUI sliders
                self.servo_angles[i] = target_angles[i]
                self.update_gui_angle(i) # This will also constrain and send to Arduino
            messagebox.showinfo("Go To Position", f"Moving to position '{selected_preset}'.")
        else:
            messagebox.showwarning("Selection Error", "Please select a valid preset to go to.")

    def delete_position(self):
        selected_preset = self.preset_selection.get()
        if selected_preset and selected_preset in self.presets:
            if messagebox.askyesno("Confirm Delete", f"Are you sure you want to delete '{selected_preset}'?"):
                del self.presets[selected_preset]
                self.save_presets()
                self.populate_presets_dropdown()
                messagebox.showinfo("Deleted", f"Position '{selected_preset}' deleted.")
        else:
            messagebox.showwarning("Selection Error", "Please select a preset to delete.")

    def update_go_button_state(self, event=None):
        if self.preset_selection.get() and self.preset_selection.get() in self.presets:
            self.go_button.config(state="normal")
            self.delete_button.config(state="normal")
        else:
            self.go_button.config(state="disabled")
            self.delete_button.config(state="disabled")

    def on_closing(self):
        if self.connected:
            self.disconnect_arduino()
        if self.joystick:
            self.joystick.quit()
        pygame.quit()
        self.master.destroy()

# --- Main Application ---
if __name__ == "__main__":
    root = tk.Tk()
    app = RoboticArmController(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing) # Handle window close event
    root.mainloop()