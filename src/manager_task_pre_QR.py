#!/usr/bin/env python3

import rospy
import yaml
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus


class TaskManager:
    def __init__(self):
        rospy.init_node("task_manager")

        mission_file = rospy.get_param("~mission_file")
        rospy.loginfo(f"Loading mission file: {mission_file}")

        with open(mission_file, 'r') as f:
            self.mission = yaml.safe_load(f)

        self.rooms = self.mission["rooms"]
        self.report = {}

        self.client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base")

        self.run_mission()

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
        finished = self.client.wait_for_result(
            rospy.Duration(timeout))

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

            rospy.loginfo("State: QR_VERIFY")
            if not self.qr_verify(room):
                rospy.logwarn(f"{room}: QR verification failed")
                self.report[room] = "SKIPPED"
                continue

            rospy.loginfo("State: EXECUTE_CLEANING")
            cleaning_failed = False
            for idx, wp in enumerate(self.mission[room]["cleaning_goals"]):
                rospy.loginfo(f" Cleaning waypoint {idx+1}")
                if not self.send_goal(wp):
                    rospy.logwarn(f"{room}: Cleaning interrupted")
                    cleaning_failed = True
                    break

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
        """
        Placeholder for QR verification.
        For now, always returns True.
        """
        rospy.sleep(1.0)
        rospy.loginfo(f"QR verified for {room}")
        return True


if __name__ == "__main__":
    try:
        TaskManager()
    except rospy.ROSInterruptException:
        pass

