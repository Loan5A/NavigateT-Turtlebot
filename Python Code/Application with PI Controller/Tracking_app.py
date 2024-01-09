#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk
from tkinter import filedialog
import subprocess
from PIL import Image, ImageTk
import psutil
import os
import threading
import subprocess
import random

def main():
    # Definition of constants
    SETPOINT_TRACKING = 0
    REFERENCE_TRACKING = 1
    REACH_STARTPOINT = 1

    # Description of the window
    root = tk.Tk()
    root.title("Tracking Options")
    root.resizable(False, False)


    def get_ssh(address):
        try:
            result = subprocess.run(['ss','-t', '-a'], capture_output=True, text = True, shell=True)

            ssh_output = result.stdout

            split_lines = ssh_output.splitlines()

            for chaine in split_lines:
                if address in chaine and "ESTAB" in chaine:
                    return True
            
            return False

        except subprocess.CalledProcessError as e:
            return None


    # Set window icon
    icon_image = Image.open("app_logo.png")  # App logo
    icon_image = ImageTk.PhotoImage(icon_image)
    root.iconphoto(True, icon_image)
    

    # Banner frame for title and logo
    banner_frame = tk.Frame(root, bd=1, relief=tk.SOLID, width=100)
    banner_frame.grid(row=0, column=0, columnspan=1, sticky="nsew")

    # Logo to the left
    logo_image = Image.open("esisar.png")  # Logo image
    logo_image = logo_image.resize((80, 40), Image.ANTIALIAS)  # Resize the logo
    logo_image = ImageTk.PhotoImage(logo_image)
    logo_label = tk.Label(banner_frame, image=logo_image)
    logo_label.grid(row=0, column=0, padx=5, pady=10, sticky="w")

    # Banner title to the right
    title_label = tk.Label(banner_frame, text="TurtleBot3 Tracking App", font=("Helvetica", 12, "bold"))
    title_label.grid(row=0, column=1, padx=0, pady=10, sticky="w")

    # Configure the column weights of the root widget
    root.grid_columnconfigure(0, weight=0)  # Fix the width of the first column
    root.grid_columnconfigure(1, weight=1)  # Let the second column expand

    # Configure the column weights of the banner_frame widget
    banner_frame.grid_columnconfigure(0, weight=0)  # Fix the width of the logo column
    banner_frame.grid_columnconfigure(1, weight=1) 

    # Radio buttons
    tracking_option = tk.IntVar()
    reference_radio = tk.Radiobutton(root, text="Reference Tracking", variable=tracking_option, value=1)
    setpoint_radio = tk.Radiobutton(root, text="Setpoint Tracking", variable=tracking_option, value=0)

    # Script name
    script = "controller_PI_v9_Obstacle_avoidance.py"
    stop_robot_script = "Stop_robot.py"

    # Checkbutton
    reach_start_checkbutton_var = tk.IntVar()
    reach_start_checkbutton = tk.Checkbutton(root, text="Reach start point first & orient robot", variable=reach_start_checkbutton_var, state=tk.DISABLED)

    # x setpoint field
    x_label = tk.Label(root, text="X:")
    x_entry = tk.Entry(root)
    x_entry.insert(0, "0")
    x_entry.focus()
    x_unit = tk.Label(root, text="[m]")

    # y setpoint field
    y_label = tk.Label(root, text="Y:")
    y_entry = tk.Entry(root)
    y_entry.insert(0, "0")
    y_unit = tk.Label(root, text="[m]")

    # theta setpoint field
    theta_label = tk.Label(root, text="θ:")
    theta_entry = tk.Entry(root)
    theta_entry.insert(0, "0")
    theta_unit = tk.Label(root, text="[rad]")

    # Connection labels
    con_tb1_label = tk.Label(root, text="\u25CF") 
    con_tb2_label = tk.Label(root, text="\u25CF") 
    con_states = [con_tb1_label, con_tb2_label]

    # Add switches (checkbuttons) with the sliding button
    odometry_switch_var = tk.IntVar()
    odometry_switch = ttk.Checkbutton(root, variable=odometry_switch_var, text="Odometry (otherwise rigid bodies)", style="Switch.TCheckbutton")
    odometry_switch_var.set(1)

    real_switch_var = tk.IntVar()
    real_switch = ttk.Checkbutton(root, variable=real_switch_var, text="Real (add 180° to the lidar)", style="Switch.TCheckbutton")
    real_switch_var.set(1)

    plots_switch_var = tk.IntVar()
    plots_switch = ttk.Checkbutton(root, variable=plots_switch_var, text="Plots", style="Switch.TCheckbutton")

    obstacle_avoidance_switch_var = tk.IntVar()
    obstacle_avoidance_switch = ttk.Checkbutton(root, variable=obstacle_avoidance_switch_var, text="Obstacle Avoidance", style="Switch.TCheckbutton")
    obstacle_avoidance_switch_var.set(1)

    # Button to validate the tracking
    ok_button = tk.Button(root, text="\u2713 OK", command=lambda: execute_script(tracking_option, x_entry, y_entry, theta_entry, reach_start_checkbutton_var, root, plots_switch_var, obstacle_avoidance_switch_var, real_switch_var, odometry_switch_var))

    # List to select turtlebot
    vlist_ipaddress = ["10.42.0.56", "10.42.0.216"]
    vlist = ["Default", "Turtlebot 1: " + vlist_ipaddress[0], "Turtlebot 2: " + vlist_ipaddress[1]]  #, "Both" ]
    topic_prefixes = ["", "tb3_1", "tb3_2"]

    selected_item = tk.StringVar()
    selected_index = 0
    global topic_prefix
    topic_prefix = topic_prefixes[selected_index]

    def update_ssh_indicator():
        # Update the indicator visually based on the SSH connection
        for i, item in enumerate(vlist):
            if item != "Default":
                connected = get_ssh(vlist_ipaddress[i-1])
                color = "green" if connected else "red"
                con_states[i-1].config(fg=color)


    def on_listbox_select(event):
        global topic_prefix
        selected_index = listbox.curselection()
        if selected_index:
            selected_index = selected_index[0]
            selected_item = listbox.get(selected_index)
            topic_prefix = topic_prefixes[selected_index]
            print(f"Selected item: {selected_item}, Topic: {topic_prefix}")
            update_ssh_indicator()

    listbox = tk.Listbox(root, selectmode=tk.SINGLE, height=len(vlist), listvariable=selected_item, width=22)
    for i in range(0, len(vlist)):
        listbox.insert(i + 1, vlist[i])
    listbox.select_set(0)
    listbox.bind("<<ListboxSelect>>", on_listbox_select)

    listbox_label = tk.Label(root, text="Turtlebot selection :")

    # Organizing buttons
    reference_radio.grid(row=1, column=0, sticky=tk.W, pady=2)
    reach_start_checkbutton.grid(row=2, column=0, columnspan=1, sticky=tk.W, pady=2)
    setpoint_radio.grid(row=3, column=0, sticky=tk.W, pady=2)

    padx_entries = 38
    padx_units = 2

    x_label.grid(row=4, column=0, sticky=tk.W, pady=2, padx=5)
    x_entry.grid(row=4, column=0, sticky="we", pady=2, padx=padx_entries)
    x_unit.grid(row = 4, column = 0, sticky=tk.E, pady=2, padx=padx_units)

    y_label.grid(row=5, column=0, sticky=tk.W, pady=2, padx=5)
    y_entry.grid(row=5, column=0, sticky="we", pady=2, padx=padx_entries)
    y_unit.grid(row = 5, column = 0, sticky=tk.E, pady=2, padx=padx_units)

    theta_label.grid(row=6, column=0, sticky=tk.W, pady=2, padx=5)
    theta_entry.grid(row=6, column=0, sticky="we", pady=2, padx=padx_entries)
    theta_unit.grid(row = 6, column = 0, sticky=tk.E, pady=2, padx=padx_units)

    tk.Label(root, text="").grid(row=7, column=0)  # Empty line

    listbox_label.grid(row=8, column=0, sticky=tk.W, pady=2)
    listbox.grid(row=9, column=0, columnspan=1, pady=5, sticky='we', padx=25)

    con_states[0].grid(row=9, column=0, sticky=tk.W)
    con_states[1].grid(row=9, column=0, sticky=tk.SW, pady=3)


    style = ttk.Style()
    style.configure("Switch.TCheckbutton", padding=(10, 5, 10, 5))

    # Add switches (checkbuttons) for set or not plots and obstacle avoidance
    odometry_switch.grid(row=10, column=0, pady=2, sticky="w")
    real_switch.grid(row=11, column=0, pady=2, sticky="w")
    plots_switch.grid(row=12, column=0, pady=2, sticky="w")
    obstacle_avoidance_switch.grid(row=13, column=0, pady=2, sticky="w")

    ok_button.grid(row=14, column=0, pady=5)

    update_ssh_indicator()



    def execute_script(tracking_option, x_entry, y_entry, theta_entry, reach_start_checkbutton_var, root, plots_switch_var, obstacle_avoidance_switch_var, real_switch_var, odometry_switch_var):
        global topic_prefix
        global subprocess_instance
          

        if tracking_option.get() == SETPOINT_TRACKING:  # Setpoint tracking
            x_value = x_entry.get()
            y_value = y_entry.get()
            theta_value = theta_entry.get()
            plots_value = plots_switch_var.get()
            obstacle_avoidance_value = obstacle_avoidance_switch_var.get()
            real_value = real_switch_var.get()
            odometry_value = odometry_switch_var.get()
            command = ["python3", script, "--tracking_type", str(SETPOINT_TRACKING), "--x", str(x_value), "--y", str(y_value), "--theta", str(theta_value), "--topic_prefix", str(topic_prefix), "--plots", str(plots_value), "--obstacle_avoidance", str(obstacle_avoidance_value), "--real", str(real_value), "--odometry", str(odometry_value)]
            subprocess_instance = subprocess.Popen(command)
        elif tracking_option.get() == REFERENCE_TRACKING:  # Reference tracking
            plots_value = plots_switch_var.get()
            obstacle_avoidance_value = obstacle_avoidance_switch_var.get()
            real_value = real_switch_var.get()
            odometry_value = odometry_switch_var.get()
            file_path = filedialog.askopenfilename(title="Select a CSV trajectory", defaultextension=".csv", filetypes=[("CSV files", "*.csv")])
            reach_startpoint = reach_start_checkbutton_var.get()
            command = ["python3", script, "--tracking_type", str(REFERENCE_TRACKING), "--file_path", str(file_path), "--reach_startpoint", str(reach_startpoint), "--topic_prefix", str(topic_prefix), "--plots", str(plots_value), "--obstacle_avoidance", str(obstacle_avoidance_value), "--real", str(real_value), "--odometry", str(odometry_value)]
            subprocess_instance = subprocess.Popen(command)
        else:
            print("ERROR: No tracking type specified.")
            root.destroy()

    def on_closing():
        global subprocess_instance
        if subprocess_instance and subprocess_instance.poll() is None:
            subprocess_instance.terminate()
            


        root.destroy()

    subprocess_instance_stop = None

    def execute_stop_robot_script():
        global subprocess_instance
        global subprocess_instance_stop

        if subprocess_instance and subprocess_instance.poll() is None:
            subprocess_instance.terminate()
    
        command=["python3", stop_robot_script]
        subprocess_instance_stop = subprocess.Popen(command)
          
            
            

    stop_subprocess_button = tk.Button(root, text="Stop subprocess", command= lambda:execute_stop_robot_script())
    stop_subprocess_button.grid(row=15, column=0, pady=5)

    def on_radio_select():
        x_entry.config(state=tk.NORMAL if tracking_option.get() == 0 else tk.DISABLED)
        y_entry.config(state=tk.NORMAL if tracking_option.get() == 0 else tk.DISABLED)
        theta_entry.config(state=tk.NORMAL if tracking_option.get() == 0 else tk.DISABLED)
        reach_start_checkbutton.config(state=tk.NORMAL if tracking_option.get() == 1 else tk.DISABLED)

    tracking_option.trace("w", lambda *args: on_radio_select())

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.bind('<Return>', lambda event: execute_script(tracking_option, x_entry, y_entry, theta_entry, reach_start_checkbutton_var, root, plots_switch_var, obstacle_avoidance_switch_var, real_switch_var, odometry_switch_var))
    root.bind('<KP_Enter>', lambda event: execute_script(tracking_option, x_entry, y_entry, theta_entry, reach_start_checkbutton_var, root, plots_switch_var, obstacle_avoidance_switch_var, real_switch_var, odometry_switch_var))

    root.mainloop()

if __name__ == "__main__":
    main()
