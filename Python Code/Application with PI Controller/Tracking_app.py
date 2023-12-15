#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk
from tkinter import filedialog
import subprocess


def main():
    # Definition of constants
    SETPOINT_TRACKING = 0
    REFERENCE_TRACKING = 1
    REACH_STARTPOINT = 1

    # Description of the window
    root = tk.Tk()
    root.title("Tracking Options")

    # Radio buttons
    tracking_option = tk.IntVar() # Variable linked with radio buttons
    reference_radio = tk.Radiobutton(root, text="Reference Tracking", variable=tracking_option, value=1)
    setpoint_radio = tk.Radiobutton(root, text="Setpoint Tracking", variable=tracking_option, value=0)

    # Script name
    script = "controller_PI_v7_Obstacle_avoidance.py"

    # Checkbutton
    reach_start_checkbutton_var = tk.IntVar() # Variable linked with checkbutton
    reach_start_checkbutton = tk.Checkbutton(root, text="Reach start point first & orient robot", variable=reach_start_checkbutton_var, state=tk.DISABLED)

    # x setpoint field
    x_label = tk.Label(root, text="X:")
    x_entry = tk.Entry(root)
    x_entry.insert(0, "0")

    # y setpoint field
    y_label = tk.Label(root, text="Y:")
    y_entry = tk.Entry(root)
    y_entry.insert(0, "0")

    # theta setpoint field
    theta_label = tk.Label(root, text="Theta:")
    theta_entry = tk.Entry(root)
    theta_entry.insert(0, "0")

    # Button to validate the tracking
    ok_button = tk.Button(root, text="OK", command=lambda: execute_script(tracking_option, x_entry, y_entry, theta_entry, reach_start_checkbutton_var, root))

    # List to select turtlebot
    vlist = ["Default", "Turtlebot 1: 10.42.0.56", "Turtlebot 2: 10.42.0.216"]#, "Both" ]
    topic_prefixes = ["", "tb3_1", "tb3_2"]

    selected_item = tk.StringVar()
    selected_index = 0
    global topic_prefix
    topic_prefix = topic_prefixes[selected_index]

    def on_listbox_select(event):
        global topic_prefix
        selected_index = listbox.curselection()
        if selected_index:
            selected_index = selected_index[0]
            selected_item = listbox.get(selected_index)
            topic_prefix = topic_prefixes[selected_index]
            print(f"Élément sélectionné: {selected_item}, Topic: {topic_prefix}")

    
    listbox = tk.Listbox(root, selectmode=tk.SINGLE, height = len(vlist), listvariable = selected_item)  
    for i in range(0, len(vlist)):
        listbox.insert(i+1,vlist[i])
    listbox.select_set(0)
    listbox.bind("<<ListboxSelect>>", on_listbox_select)

    listbox_label = tk.Label(root, text="Turtlebot selection :")


    # Organizing buttons
    reference_radio.grid(row=0, column=0, sticky=tk.W)
    reach_start_checkbutton.grid(row=1, column=0, columnspan=2, sticky=tk.W)
    setpoint_radio.grid(row=2, column=0, sticky=tk.W)

    x_label.grid(row=3, column=0, sticky=tk.W)
    x_entry.grid(row=3, column=1, sticky=tk.W)

    y_label.grid(row=4, column=0, sticky=tk.W)
    y_entry.grid(row=4, column=1, sticky=tk.W)

    theta_label.grid(row=5, column=0, sticky=tk.W)
    theta_entry.grid(row=5, column=1, sticky=tk.W)

    tk.Label(root, text="").grid(row=6, column=0) # Empty line

    listbox_label.grid(row=7, column=0, sticky=tk.W)
    listbox.grid(row=7, column=1, sticky=tk.W)

    ok_button.grid(row=8, column=0, columnspan=2)

    # Function to execute the controller according to solected options
    def execute_script(tracking_option, x_entry, y_entry, theta_entry, reach_start_checkbutton_var, root):
        global topic_prefix
        global subprocess_instance

        if tracking_option.get() == SETPOINT_TRACKING:  # Setpoint tracking
            x_value = x_entry.get()
            y_value = y_entry.get()
            theta_value = theta_entry.get()
            #root.destroy()  # Ferme la fenêtre principale après exécution du script
            command = ["python3", script, "--tracking_type", str(SETPOINT_TRACKING), "--x", str(x_value), "--y", str(y_value), "--theta", str(theta_value), "--topic_prefix", str(topic_prefix)]
            subprocess_instance = subprocess.Popen(command)
        elif tracking_option.get() == REFERENCE_TRACKING:  # Reference tracking
            file_path = filedialog.askopenfilename(title="Select a CSV trajectory", defaultextension=".csv", filetypes=[("CSV files", "*.csv")])
            #root.destroy()  # Ferme la fenêtre principale après exécution du script
            reach_startpoint = reach_start_checkbutton_var.get()
            command = ["python3", script, "--tracking_type", str(REFERENCE_TRACKING), "--file_path", str(file_path), "--reach_startpoint", str(reach_startpoint), "--topic_prefix", str(topic_prefix)]
            subprocess_instance = subprocess.Popen(command)
        else:
            print("ERROR: No tracking type specified.")
            root.destroy()  # Ferme la fenêtre principale

    def on_closing():
        root.destroy()


    def stop_subprocess():
        global subprocess_instance
        if subprocess_instance:
            subprocess_instance.terminate();
    
    stop_subprocess_button = tk.Button(root, text="Stop subprocess", command=stop_subprocess)
    stop_subprocess_button.grid(row=9, column=0, columnspan=2)


    # Enable or disable fields/buttons according to radio buttons selection
    def on_radio_select():
        # Disables / enables x, y and theta fields
        x_entry.config(state=tk.NORMAL if tracking_option.get() == 0 else tk.DISABLED)
        y_entry.config(state=tk.NORMAL if tracking_option.get() == 0 else tk.DISABLED)
        theta_entry.config(state=tk.NORMAL if tracking_option.get() == 0 else tk.DISABLED)
        # Disables / enables "Reach start point first & orient robot" button
        reach_start_checkbutton.config(state=tk.NORMAL if tracking_option.get() == 1 else tk.DISABLED)

    

    tracking_option.trace("w", lambda *args: on_radio_select())

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.bind('<Return>', lambda event: execute_script(tracking_option, x_entry, y_entry, theta_entry, reach_start_checkbutton_var, root))
    root.bind('<KP_Enter>', lambda event: execute_script(tracking_option, x_entry, y_entry, theta_entry, reach_start_checkbutton_var, root))

    

    root.mainloop()

if __name__ == "__main__":
    main()