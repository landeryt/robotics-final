#!/usr/bin/env python3

import rospy
import yaml
import math
import actionlib

from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from std_srvs.srv import Empty


class TaskManager:

    def __init__(self):

        rospy.init_node("task_manager")

        mission_file = rospy.get_param("~mission_file")
        rospy.loginfo(f"Loading mission file: {mission_file}")

        with open(mission_file, 'r') as f:
            self.mission = yaml.safe_load(f)

        self.rooms = self.mission["rooms"]
        self.report = {}

        self.qr_data = None
        rospy.Subscriber("/qr/result", String, self.qr_cb) # Future QR

        self.client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base")

        self.run_mission()

    # --------------------------------------------------
    
    def qr_cb(self, msg):
        self.qr_data = msg.data


    def clear_costmaps(self):
        try:
            rospy.wait_for_service("/move_base/clear_costmaps", timeout=2.0)
            clear_srv = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
            clear_srv()
            rospy.loginfo("Costmaps cleared")
        except rospy.ServiceException:
            rospy.logwarn("Failed to clear costmaps")

    # --------------------------------------------------

    def send_goal(self, goal, timeout=60.0):
        mb_goal = MoveBaseGoal()
        mb_goal.target_pose.header.frame_id = "map"
        mb_goal.target_pose.header.stamp = rospy.Time.now()

        mb_goal.target_pose.pose.position.x = goal["x"]
        mb_goal.target_pose.pose.position.y = goal["y"]

        yaw = goal.get("yaw", 0.0)
        mb_goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
        mb_goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.client.send_goal(mb_goal)

        finished = self.client.wait_for_result(rospy.Duration(timeout))

        if not finished:
            rospy.logwarn("Goal timed out")
            self.client.cancel_goal()
            return False

        state = self.client.get_state()
        return state == GoalStatus.SUCCEEDED

    # --------------------------------------------------

    def run_mission(self):
        rospy.loginfo("===== STARTING CLEANING MISSION =====")

        for room in self.rooms:
            rospy.loginfo(f"\n--- ROOM: {room} ---")

            entry = self.mission[room]["entry_goal"]

            rospy.loginfo("State: GO_TO_ROOM_ENTRY")
            if not self.send_goal(entry):
                rospy.logerr(f"{room}: FAILED to reach entry")
                self.report[room] = "FAIL"
                continue

            self.clear_costmaps()
            rospy.sleep(0.5)

            rospy.loginfo("State: QR_VERIFY (SKIPPED)") # skipped QR


            rospy.loginfo("State: EXECUTE_CLEANING")

            cleaning_goals = self.mission[room].get("cleaning_goals", [])

            # Handle rooms with no cleaning (e.g. hallway)
            if not cleaning_goals:
                rospy.loginfo(f"{room}: No cleaning required")
                self.report[room] = "SUCCESS"
                rospy.loginfo("State: REPORT")
                rospy.loginfo(f"{room} RESULT: SUCCESS")
                continue

            cleaning_failed = False

            for idx, wp in enumerate(cleaning_goals):
                rospy.loginfo(f" Cleaning waypoint {idx + 1}")

                if not self.send_goal(wp, timeout=120.0):
                    rospy.logwarn(f"{room}: Cleaning interrupted")
                    cleaning_failed = True
                    break

                self.clear_costmaps()
                rospy.sleep(0.5)

            if cleaning_failed:
                self.report[room] = "SKIPPED"
            else:
                self.report[room] = "SUCCESS"

            rospy.loginfo("State: REPORT")
            rospy.loginfo(f"{room} RESULT: {self.report[room]}")

        rospy.loginfo("\n===== MISSION FINISHED =====")
        for room, status in self.report.items():
            rospy.loginfo(f"{room}: {status}")

    # --------------------------------------------------

    def qr_verify(self, room):
        
        #Placeholder for QR verification.
        #For now, always returns True.
        
        #rospy.sleep(1.0)
        rospy.loginfo(f"QR verified for {room}")
        return True
        
    """def qr_verify(self, room):
        self.qr_data = None
        expected = self.mission[room]["qr_expected"]

        start = rospy.Time.now()
        timeout = rospy.Duration(10.0)

        while rospy.Time.now() - start < timeout:
            if self.qr_data:
                if self.qr_data == expected:
                    rospy.loginfo(f"QR verified for {room}")
                    return True
                else:
                    rospy.logwarn(f"Wrong QR detected: {self.qr_data}")
                    return False
            rospy.sleep(0.1)

        rospy.logwarn("QR verification timed out")
        return False"""
    


if __name__ == "__main__":
    try:
        TaskManager()
    except rospy.ROSInterruptException:
        pass

