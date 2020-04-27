#!/usr/bin/env python

import rospy
from Tkinter import *
import ttk

from sound_play.libsoundplay import SoundClient

class Gui:
    def __init__(self, master):
        self.sound = SoundClient()

        frame = Frame(master)
        frame.pack()

        self.tree = ttk.Treeview(frame, columns=["run", "goal", "pos_ampl", "pos_half_cycles", "speed_base", "speed_ampl", "speed_period"], show="headings")
        self.tree.heading("run", text="Run")
        self.tree.heading("goal", text="Goal")
        self.tree.heading("pos_ampl", text="Position amplitude (m)")
        self.tree.heading("pos_half_cycles", text="Position half-cycles")
        self.tree.heading("speed_base", text="Speed baseline (m/s)")
        self.tree.heading("speed_ampl", text="Speed amplitude (m/s)")
        self.tree.heading("speed_period", text="Speed period (s)")

        self.tree.grid(row=0, column=0, columnspan=2)

        self.welcome = Button(frame, text="Play welcome message", command=self.welcome_cmd)
        self.welcome.grid(row=1, column=0)

        self.get_noodle = Button(frame, text="Request noodle", state=DISABLED, command=self.get_noodle_cmd)
        self.get_noodle.grid(row=2)

        self.grab = Button(frame, text="Close gripper", state=DISABLED, command=self.grab_cmd)
        self.grab.grid(row=3)

        self.run = Button(frame, text="Run", state=DISABLED, command=self.run_cmd)
        self.run.grid(row=4)

        self.release = Button(frame, text="Open gripper", state=DISABLED, command=self.release_cmd)
        self.release.grid(row=5)

        self.reset = Button(frame, text="Return to start", state=DISABLED, command=self.reset_cmd)
        self.reset.grid(row=6)

        self.cancel = Button(frame, text="Stop", state=DISABLED, command=self.cancel_cmd)
        self.cancel.grid(row=1, column=1, rowspan=6)

    def welcome_cmd(self):
        # Play the welcome message
        self.sound.voiceSound("Hello. I would like you to help me carry these pool noodles to the other side of the room.")
        self.welcome["state"] = DISABLED
        self.get_noodle["state"] = NORMAL

    def get_noodle_cmd(self):
        self.get_noodle["state"] = DISABLED
        self.run["state"] = DISABLED
        # TODO Open the gripper

        # Play the request
        self.sound.voiceSound("Please place the end of the pool noodle in my hand.")
        self.grab["state"] = NORMAL

    def grab_cmd(self):
        self.grab["state"] = DISABLED
        # TODO Close the gripper

        self.get_noodle["state"] = NORMAL
        self.run["state"] = NORMAL

    def run_cmd(self):
        self.get_noodle["state"] = DISABLED
        self.run["state"] = DISABLED
        self.sound.voiceSound("Lets go!")
        self.cancel["state"] = NORMAL
        # TODO Run the nav stack
        self.release["state"] = NORMAL
        self.cancel["state"] = DISABLED

    def release_cmd(self):
        # Play the thank you
        self.sound.voiceSound("Thank you for your help. Please place the pool noodle in the pile.")

        # TODO Open the gripper
        self.reset["state"] = NORMAL
        self.release["state"] = DISABLED

    def reset_cmd(self):
        self.cancel["state"] = NORMAL
        self.reset["state"] = DISABLED
        # TODO Face back towards start

        # Ask subject to follow
        self.sound.voiceSound("")

        # TODO Drive back to start position

        # TODO Rotate back to starting orientation
        self.cancel["state"] = DISABLED
        self.get_noodle["state"] = NORMAL

    def cancel_cmd(self):
        pass

def main():
    #rospy.init_node("noodle_gui", anonymous=True)
    root = Tk()
    root.title ("Fetch Noodle")

    gui = Gui(root)


    root.mainloop()



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

