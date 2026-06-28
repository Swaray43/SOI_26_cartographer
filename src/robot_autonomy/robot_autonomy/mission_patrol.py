#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time

def main():
    # 1. Wake up the ROS 2 Python library
    rclpy.init()
    
    # 2. Create the "Commander" object to talk to Nav2
    navigator = BasicNavigator()

    # 3. Wait for the Nav2 stack to be fully awake and ready
    print("Waiting for Nav2 to boot up...")
    navigator.waitUntilNav2Active()
    print("Nav2 is ready! Starting mission...")

    # 4. Define where we want the robot to go
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'                   # We are using coordinates based on the map
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    
    # --- SET YOUR DESIRED COORDINATES HERE ---
    goal_pose.pose.position.x = 2.0                     # Drive 2 meters forward
    goal_pose.pose.position.y = 1.0                     # Drive 1 meter left
    
    # This is a Quaternion. w=1.0 means "face straight ahead without rotating"
    goal_pose.pose.orientation.w = 1.0                  

    # 5. Send the goal to Nav2
    print(f"Sending robot to X: {goal_pose.pose.position.x}, Y: {goal_pose.pose.position.y}")
    navigator.goToPose(goal_pose)

    # 6. Keep checking on the robot while it drives
    while not navigator.isTaskComplete():
        # We can ask the robot for feedback while it drives
        feedback = navigator.getFeedback()
        if feedback:
            print(f"Distance remaining: {feedback.distance_remaining:.2f} meters", end='\r')
        time.sleep(0.1)

    # 7. The robot finished driving! Let's find out what happened.
    result = navigator.getResult()
    
    print("\n") # Add a new line after the feedback loop
    if result == TaskResult.SUCCEEDED:
        print('MISSION SUCCESS: Robot arrived safely!')
    elif result == TaskResult.CANCELED:
        print('MISSION CANCELED: The goal was canceled.')
    elif result == TaskResult.FAILED:
        print('MISSION FAILED: The robot got stuck or crashed.')

    # 8. Clean up and shut down
    navigator.lifecycleShutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
