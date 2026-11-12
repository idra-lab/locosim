import tkinter as tk
from tkinter import ttk
import numpy as np
import base_controllers.params as conf

class PIDTuningGui:
     
    def __init__(self, robot, mode:str = 'none', tuning_type = 'PID',   init_freq: float = 0.5, init_amp = 0.):

        self.robot = robot
        self.tuning_type = tuning_type
        self.debug_freq = init_freq
        self.debug_amp = init_amp

        real_robot = robot.real_robot

        if  self.tuning_type == 'PID':
            rr_str = "_real" if real_robot else ""
            mode_str = "_swing" if (mode == 'swim' or mode == 'step') else ''

            # Load initial PID values from configuration
            self.initial_kp = conf.robot_params[self.robot.robot_name][f'kp{rr_str}{mode_str}']
            self.initial_kd = conf.robot_params[self.robot.robot_name][f'kd{rr_str}{mode_str}']
            self.initial_ki = conf.robot_params[self.robot.robot_name][f'ki{rr_str}{mode_str}']
            self.initial_kp_lin = conf.robot_params[self.robot.robot_name][f'kp_lin{rr_str}']
            self.initial_kd_lin = conf.robot_params[self.robot.robot_name][f'kd_lin{rr_str}']
            self.initial_kp_ang = conf.robot_params[self.robot.robot_name][f'kp_ang{rr_str}']
            self.initial_kd_ang = conf.robot_params[self.robot.robot_name][f'kd_ang{rr_str}']
    
    # Helper function to create three sliders for a parameter
    def create_triple_slider(self, label, row, initial_values, max_value, resolution, label_font):
        # Label font style

        ttk.Label(self.root, text=label, font=label_font).grid(row=row, column=0, sticky="e", padx=10, pady=10)
        sliders = []

        for i, _ in enumerate(["X", "Y", "Z"]):
            slider = tk.Scale(self.root, from_=0, to=max_value, resolution=resolution, orient="horizontal", length=150)
            slider.set(initial_values[i])
            slider.grid(row=row, column=i+1)
            sliders.append(slider)

        return sliders

    def init_pid_tuning_ui(self):

            # Initialize tkinter window for PID tuning
            self.root = tk.Tk()
            self.root.title("PID Tuning")

            # Set window size and padding
            self.root.geometry("600x600")
            self.root.minsize(600, 600)
            self.root.configure(padx=20, pady=20)

            label_font = ("Helvetica", 12, "bold")

            if self.tuning_type == 'PID':
                # Create sliders for kp, kd, ki
                self.kp_sliders = self.create_triple_slider("KP", 0, self.initial_kp, 60, 0.1, label_font)
                self.kd_sliders = self.create_triple_slider("KD", 1, self.initial_kd, 2, 0.01, label_font)
                self.ki_sliders = self.create_triple_slider("KI", 2, self.initial_ki, 5, 0.1, label_font)

                # Create sliders for kp_lin, kd_lin, kp_ang, kd_ang
                self.kp_lin_sliders = self.create_triple_slider("KP LIN", 3, self.initial_kp_lin, 1000, 0.1, label_font)
                self.kd_lin_sliders = self.create_triple_slider("KD LIN", 4, self.initial_kd_lin, 100, 0.01, label_font)
                self.kp_ang_sliders = self.create_triple_slider("KP ANG", 5, self.initial_kp_ang, 200, 0.1, label_font)
                self.kd_ang_sliders = self.create_triple_slider("KD ANG", 6, self.initial_kd_ang, 10, 0.01, label_font)

                # Add a separate row for the frequency slider
                ttk.Label(self.root, text="Freq", font=label_font).grid(row=7, column=0, sticky="e", padx=10, pady=10)
                self.freq_slider = tk.Scale(self.root, from_=0.25, to=2, resolution=0.1, orient="horizontal", length=250)
                self.freq_slider.set(self.debug_freq)
                self.freq_slider.grid(row=7, column=1, columnspan=3)  # Align with the other sliders

            if self.tuning_type == 'BODY':
                # Add a separate row for the frequency slider
                ttk.Label(self.root, text="Freq", font=label_font).grid(row=7, column=0, sticky="e", padx=10, pady=10)
                self.freq_slider = tk.Scale(self.root, from_=0.25, to=2, resolution=0.1, orient="horizontal", length=250)
                self.freq_slider.set(self.debug_freq)
                self.freq_slider.grid(row=7, column=1, columnspan=3)  # Align with the other sliders


            # Button to apply PID changes
            self.update_button = ttk.Button(self.root, text="Update PID", command=self.update_pid_values)
            self.update_button.grid(row=8, column=0, columnspan=4, pady=20)

            #  Check periodically if shutdown flag was set
            self.check_stop()

            # Run tkinter loop
            self.root.mainloop()

    def check_stop(self):
        # Correct shutdown flag: QuadrupedTasks.stop_thread
        if hasattr(self.robot, "stop_thread") and self.robot.stop_thread:
            self.root.destroy()
            return

        self.root.after(100, self.check_stop)

    # Helper function to get array values from sliders
    def get_slider_values(self, sliders, max_values):
        return np.clip([slider.get() for slider in sliders], 0, max_values)

    def update_pid_values(self):

        if self.tuning_type == 'PID':
            # Get values from each set of three sliders as arrays
            kp_values = self.get_slider_values(self.kp_sliders, 60.)
            kd_values = self.get_slider_values(self.kd_sliders, 2.)
            ki_values = self.get_slider_values(self.ki_sliders, 5.)

            # Repeat each 3-element array 4 times to create a 12-element array
            kp_array = np.tile(kp_values, 4)
            kd_array = np.tile(kd_values, 4)
            ki_array = np.tile(ki_values, 4)

            # Get values for linear and angular PID settings
            kp_lin_array = self.get_slider_values(self.kp_lin_sliders, 1000.)
            kd_lin_array = self.get_slider_values(self.kd_lin_sliders, 100.)
            kp_ang_array = self.get_slider_values(self.kp_ang_sliders, 200.)
            kd_ang_array = self.get_slider_values(self.kd_ang_sliders, 10.)

            self.debug_freq = self.freq_slider.get()

            # # Set the PID values on the robot's controller
            self.robot.pid.setPDjoints(kp_array, kd_array, ki_array)
            self.robot.wbc.setGains(kp_lin_array, kd_lin_array, kp_ang_array, kd_ang_array)

            print("PID updated:", kp_array, kd_array, ki_array)
            print("PID lin ang updated:", kp_lin_array, kd_lin_array, kp_ang_array, kd_ang_array)