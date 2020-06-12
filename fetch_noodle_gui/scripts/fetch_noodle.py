#!/usr/bin/env python

from Tkinter import *
import ttk

from collections import OrderedDict

import math

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryAction, GripperCommandAction
from control_msgs.msg import FollowJointTrajectoryGoal, GripperCommandGoal
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_python import MoveGroupInterface
from sound_play.libsoundplay import SoundClient
from tf.transformations import quaternion_from_euler

class Actions:
    def __init__(self):
        self.arm = actionlib.SimpleActionClient("/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        self.gripper = actionlib.SimpleActionClient("/gripper_controller/gripper_action", GripperCommandAction)
        self.move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.arm.wait_for_server()
        self.gripper.wait_for_server()
        self.move_base.wait_for_server()

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

        self.home_backwards = MoveBaseGoal()
        self.home_backwards.target_pose.header.frame_id = "map"
        self.home_backwards.target_pose.header.stamp = rospy.Time.now()
        quat_tf = quaternion_from_euler(0.0, 0.0, 0.0)
        self.home_backwards.target_pose.pose = Pose(Point(self.origin_x, self.origin_y, 0.0),
                Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3]))

        self.home = MoveBaseGoal()
        self.home.target_pose.header.frame_id = "map"
        self.home.target_pose.header.stamp = rospy.Time.now()
        quat_tf = quaternion_from_euler(0.0, 0.0, 3.14159)
        self.home.target_pose.pose = Pose(Point(self.origin_x, self.origin_y, 0.0),
                Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3]))

class Gui:
    def __init__(self, master):
        self.needs_alignment = False
        self.read_run_parameters()

        self.sound = SoundClient()
        self.actions = Actions()

        frame = Frame(master)
        frame.pack()

        style = ttk.Style()
        style.configure("Treeview.Heading", rowheight=145)

        self.tree = ttk.Treeview(frame, selectmode="browse", columns=["run", "goal", "position_amplitude", "position_half_cycles", "speed_base", "speed_amplitude", "speed_period"], show="headings")

        self.tree.heading("run", text="Run")
        self.tree.heading("goal", text="Goal")
        self.tree.heading("position_amplitude", text="Position amplitude (m)")
        self.tree.heading("position_half_cycles", text="Position half-cycles")
        self.tree.heading("speed_base", text="Speed baseline (m/s)")
        self.tree.heading("speed_amplitude", text="Speed amplitude (m/s)")
        self.tree.heading("speed_period", text="Speed period (s)")
        widths = [45,65,230,200,230,230,190]
        for col in range(7):
            self.tree.column(col, stretch=True, width=widths[col])


        for idx, run in self.runs.items():
            self.tree.insert("", "end", iid=idx, values = [idx, run['goal']['name'], run['position_amplitude'], run['position_half_cycles'],
                run['speed_baseline'], run['speed_amplitude'], run['speed_period']])
        self.tree.focus(0)
        self.tree.selection_set(0)

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

    def read_run_parameters(self):
        self.origin_x = rospy.get_param("/fetch_noodle/origin/x")
        self.origin_y = rospy.get_param("/fetch_noodle/origin/y")

        run_number = 0
        runs = {}
        while True:
            if rospy.has_param("/fetch_noodle/runs/" + str(run_number)):
                prefix = "/fetch_noodle/runs/"+str(run_number)+"/"
                run = {"goal": {
                                "name":rospy.get_param(prefix+"goal/name"),
                                "x":rospy.get_param(prefix+"goal/x"),
                                "y":rospy.get_param(prefix+"goal/y"),
                                "heading":rospy.get_param(prefix+"goal/heading")},
                        "position_amplitude": rospy.get_param(prefix+"position_amplitude"),
                        "position_half_cycles": rospy.get_param(prefix+"position_half_cycles"),
                        "speed_baseline": rospy.get_param(prefix+"speed_baseline"),
                        "speed_amplitude":rospy.get_param(prefix+"speed_amplitude"),
                        "speed_period":rospy.get_param(prefix+"speed_period")
                        }
                runs[str(run_number)] = run
                run_number += 1
            else:
                break
        self.runs = OrderedDict(sorted(runs.items()))


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
        # Run the nav stack
        # First read the parameters for this run and set them on the parameter server
        parameters = self.runs[self.tree.focus()]
        rospy.set_param("/move_base/DeviationGlobalPlanner/position/amplitude", parameters["position_amplitude"])
        rospy.set_param("/move_base/DeviationGlobalPlanner/position/half_cycles", parameters["position_half_cycles"])
        rospy.set_param("/move_base/DeviationLocalPlanner/speed/baseline", parameters["speed_baseline"])
        rospy.set_param("/move_base/DeviationLocalPlanner/speed/amplitude", parameters["speed_amplitude"])
        rospy.set_param("/move_base/DeviationLocalPlanner/speed/period", parameters["speed_period"])

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        quat_tf = quaternion_from_euler(0.0, 0.0, parameters["goal"]["heading"])
        goal.target_pose.pose = Pose(Point(parameters["goal"]["x"], parameters["goal"]["y"], 0.0),
                Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3]))
        self.current_goal = goal.target_pose.pose

        def at_goal(status, result):
            self.successful_goal = status == GoalStatus.SUCCEEDED

            self.release["state"] = NORMAL
            self.cancel["state"] = DISABLED
        def feedback(fb):
            self.current_pose = fb.base_position.pose
        self.actions.move_base.send_goal(goal, done_cb = at_goal, feedback_cb = feedback)

    def release_cmd(self):
        self.release["state"] = DISABLED
        if self.successful_goal:
            # Play the thank you if we made it to the goal without an abort
            self.sound.say("Thank you for your help. Please place the pool noodle in the pile.")
        else:
            self.sound.say("That didn't work. Lets try again.")

        self.actions.gripper.send_goal_and_wait(self.actions.open_gripper)
        self.actions.arm_down_pose.header.stamp = rospy.Time.now()
        self.actions.move_group.moveToPose(self.actions.arm_down_pose, "wrist_roll_link")
        self.reset["state"] = NORMAL

    def reset_cmd(self):
        self.reset["state"] = DISABLED
        self.cancel["state"] = NORMAL
        # Remove any deviations for straight driving
        rospy.set_param("/move_base/DeviationGlobalPlanner/position/amplitude", 0.0)
        # TODO set the return speed. For now, use the previous baseline
        rospy.set_param("/move_base/DeviationLocalPlanner/speed/amplitude", 0.0)

        # Face back towards start
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        # Get current position (might not be goal if we abort)
        bearing = math.atan2(-self.current_pose.position.y, -self.current_pose.position.x)
        quat_tf = quaternion_from_euler(0.0, 0.0, bearing)
        goal.target_pose.pose = Pose(self.current_pose.position,
                Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3]))
        self.actions.move_base.send_goal_and_wait(goal)

        # Ask subject to follow
        self.sound.say("Please follow me back to the starting position.")

        def spot_turn(status, result):
            if status != GoalStatus.SUCCEEDED:
                self.cancel["state"] = DISABLED
                self.reset["state"] = NORMAL
            else:
                self.needs_alignment = True
                # Select the next run in the list
                run_id = int(self.tree.focus())
                if run_id < len(self.runs) - 1 and self.successful_goal:
                    self.tree.focus(run_id+1)
                    self.tree.selection_set(run_id+1)
                self.cancel["state"] = DISABLED
                self.get_noodle["state"] = NORMAL
        def feedback(fb):
            self.current_pose = fb.base_position.pose
        # Drive back to start position
        self.actions.move_base.send_goal(self.actions.home_backwards, done_cb = spot_turn, feedback_cb = feedback)

    def align_to_zero(self):
        self.needs_alignment = False
        # Rotate back to starting orientation
        self.actions.move_base.send_goal_and_wait(self.actions.home)



    def cancel_cmd(self):
        self.actions.move_base.cancel_goal()

def main():
    rospy.init_node("noodle_gui", anonymous=True)
    root = Tk()
    root.title ("Fetch Noodle")
    #root.geometry("1200x200")

    gui = Gui(root)

    def check():
        if gui.needs_alignment:
            gui.align_to_zero()
        root.after(100, check)

    root.after(100, check)
    root.mainloop()



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

