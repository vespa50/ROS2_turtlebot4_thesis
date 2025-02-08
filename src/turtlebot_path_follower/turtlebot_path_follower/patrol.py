#!/usr/bin/env python3

from math import floor
from threading import Lock, Thread
from time import sleep

import rclpy

from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import BatteryState
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from nav2_simple_commander.robot_navigator import TaskResult


BATTERY_HIGH = 0.95
BATTERY_LOW = 0.5  # when the robot will go charge
BATTERY_CRITICAL = 0.1  # when the robot will shutdown


class BatteryMonitor(Node):

    def __init__(self, lock):
        super().__init__('battery_monitor')

        self.lock = lock

        # Subscribe to the /battery_state topic
        self.battery_state_subscriber = self.create_subscription(
            BatteryState,
            'battery_state',
            self.battery_state_callback,
            qos_profile_sensor_data)

    # Callbacks
    def battery_state_callback(self, batt_msg: BatteryState):
        with self.lock:
            self.battery_percent = batt_msg.percentage

    def thread_function(self):
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        executor.spin()


def main(args=None):
    rclpy.init(args=args)

    lock = Lock()
    battery_monitor = BatteryMonitor(lock)

    navigator = TurtleBot4Navigator()
    battery_percent = None
    position_index = 0
    i = 0
    
    thread = Thread(target=battery_monitor.thread_function, daemon=True)
    thread.start()


    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before intialising pose')
        navigator.dock()
    if not navigator.getDockedStatus():
    	navigator.error('Robot failed to dock')
    	battery_monitor.destroy_node()
    	rclpy.shutdown()
    	exit()
    

    # Set initial pose
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Undock
    navigator.undock()

    # Prepare goal poses
    goal_pose = []
    goal_pose.append(navigator.getPoseStamped([-3.4507179260253906, 2.9931018352508545], TurtleBot4Directions.SOUTH))
    goal_pose.append(navigator.getPoseStamped([-3.54430890083313, -0.33151310682296753], TurtleBot4Directions.EAST))
    goal_pose.append(navigator.getPoseStamped([-0.9400812387466431, 0.7386283278465271], TurtleBot4Directions.NORTH_WEST))
    #goal_pose.append(navigator.getPoseStamped([10.0, 2.0], TurtleBot4Directions.WEST))

    while i < 4:
        with lock:
            battery_percent = battery_monitor.battery_percent

        if (battery_percent is not None):
            navigator.info(f'Battery is at {(battery_percent*100):.2f}% charge')

            # Check battery charge level
            if (battery_percent < BATTERY_CRITICAL):
                navigator.error('Battery critically low. Charge or power down')
                break
            elif (battery_percent < BATTERY_LOW):
                # Go near the dock
                navigator.info('Docking for charge')
                navigator.startToPose(navigator.getPoseStamped([-0.561712920665741, -0.03261447325348854],TurtleBot4Directions.EAST))
                navigator.dock()

                if not navigator.getDockedStatus():
                    navigator.error('Robot failed to dock')
                    break

                # Wait until charged
                navigator.info('Charging...')
                battery_percent_prev = 0
                while (battery_percent < BATTERY_HIGH):
                    sleep(15)
                    battery_percent_prev = floor(battery_percent*100)/100
                    with lock:
                        battery_percent = battery_monitor.battery_percent

                    # Print charge level every time it increases a percent
                    if battery_percent > (battery_percent_prev + 0.01):
                        navigator.info(f'Battery is at {(battery_percent*100):.2f}% charge')

                # Undock
                navigator.undock()
                position_index = 0

            else:
                # Navigate to next position
                result = navigator.startToPose(goal_pose[position_index])
                if result == TaskResult.SUCCEEDED:
                	print('info:Goal succeeded!')
                	position_index = position_index + 1
                	if position_index >= len(goal_pose):
                		i = i + 1
                		position_index = 0
                		navigator.info(f'pose index:{position_index:.2f}')
                elif result == TaskResult.CANCELED:
                	print('info:Goal was canceled!')
                elif result == TaskResult.FAILED:
                	print('info:Goal failed!')
                else:
                	print('info:Goal has an invalid return status!')
    
    navigator.info('docking')
    navigator.startToPose(navigator.getPoseStamped([-0.561712920665741, -0.03261447325348854],TurtleBot4Directions.EAST))
    navigator.dock()

    battery_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
