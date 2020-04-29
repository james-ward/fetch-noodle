#!/usr/bin/env python

import rospy
from Tkinter import *
import ttk

import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, GripperCommandAction
from control_msgs.msg import FollowJointTrajectoryGoal, GripperCommandGoal
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_python import MoveGroupInterface
from sound_play.libsoundplay import SoundClient


class Actions:
    def __init__(self):
        self.arm = actionlib.SimpleActionClient("/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        self.gripper = actionlib.SimpleActionClient("/gripper_controller/gripper_action", GripperCommandAction)
        self.arm.wait_for_server()
        self.gripper.wait_for_server()

        self.open_gripper = GripperCommandGoal()
        self.open_gripper.command.position = 1.0
        self.close_gripper = GripperCommandGoal()
        self.close_gripper.command.position = 0.0
        self.close_gripper.command.max_effort = 10.0  # TODO check on real robot how hard to close

        self.move_group = MoveGroupInterface("arm", "base_link")
        self.arm_up_pose = PoseStamped()
        self.arm_up_pose.header.frame_id = "base_link"
        self.arm_up_pose.pose = Pose(Point(0.4, 0.0, 0.6), Quaternion(0.5,-0.5,0.5,0.5))
        self.arm_down_pose = PoseStamped()
        self.arm_down_pose.header.frame_id = "base_link"
        self.arm_down_pose.pose = Pose(Point(0.4, 0.0, 0.5), Quaternion(0,0,0,1))


class Gui:
    def __init__(self, master):
        self.sound = SoundClient()
        self.actions = Actions()

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
        self.sound.say("Hello. I would like you to help me carry these pool noodles to the other side of the room.")
        self.welcome["state"] = DISABLED
        self.get_noodle["state"] = NORMAL

    def get_noodle_cmd(self):
        self.get_noodle["state"] = DISABLED
        self.grab["state"] = DISABLED
        self.run["state"] = DISABLED
        self.actions.arm_up_pose.header.stamp = rospy.Time.now()
        self.actions.move_group.moveToPose(self.actions.arm_up_pose, "wrist_roll_link")
        self.actions.gripper.send_goal_and_wait(self.actions.open_gripper)

        # Play the request
        self.sound.say("Please place the end of the pool noodle in my hand.")
        self.grab["state"] = NORMAL

    def grab_cmd(self):
        self.grab["state"] = DISABLED
        self.get_noodle["state"] = DISABLED
        self.actions.gripper.send_goal_and_wait(self.actions.close_gripper)

        self.get_noodle["state"] = NORMAL
        self.run["state"] = NORMAL

    def run_cmd(self):
        self.run["state"] = DISABLED
        self.get_noodle["state"] = DISABLED
        self.sound.say("Lets go!")
        self.cancel["state"] = NORMAL
        # TODO Run the nav stack
        self.release["state"] = NORMAL
        self.cancel["state"] = DISABLED

    def release_cmd(self):
        self.release["state"] = DISABLED
        # Play the thank you
        self.sound.say("Thank you for your help. Please place the pool noodle in the pile.")

        self.actions.gripper.send_goal_and_wait(self.actions.open_gripper)
        self.actions.arm_down_pose.header.stamp = rospy.Time.now()
        self.actions.move_group.moveToPose(self.actions.arm_down_pose, "wrist_roll_link")
        self.reset["state"] = NORMAL

    def reset_cmd(self):
        self.reset["state"] = DISABLED
        self.cancel["state"] = NORMAL
        # TODO Face back towards start

        # Ask subject to follow
        self.sound.say("Please follow me back to the starting position.")

        # TODO Drive back to start position

        # TODO Rotate back to starting orientation
        self.cancel["state"] = DISABLED
        self.get_noodle["state"] = NORMAL

    def cancel_cmd(self):
        pass

def main():
    rospy.init_node("noodle_gui", anonymous=True)
    root = Tk()
    root.title ("Fetch Noodle")

    gui = Gui(root)

    root.mainloop()



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

