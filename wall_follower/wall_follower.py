#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import SetParametersResult
from wall_follower.visualization_tools import VisualizationTools


class WallFollower(Node):

    def __init__(self):
        super().__init__("wall_follower")
        # Declare parameters to make them available for use
        # DO NOT MODIFY THIS! 
        self.declare_parameter("scan_topic", "default")
        self.declare_parameter("drive_topic", "default")
        self.declare_parameter("side", "default")
        self.declare_parameter("velocity", "default")
        self.declare_parameter("desired_distance", "default")

        # Fetch constants from the ROS parameter server
        # DO NOT MODIFY THIS! This is necessary for the tests to be able to test varying parameters!
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value
		
        # This activates the parameters_callback function so that the tests are able
        # to change the parameters during testing.
        # DO NOT MODIFY THIS! 
        self.add_on_set_parameters_callback(self.parameters_callback)

        # TODO: Initialize your publishers and subscribers here
        self.simple_publisher = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)
        self.line = self.create_publisher(Marker, "wall", 10)
        timer_period = 1
        self.i =0
        self.subscription = self.create_subscription(
            LaserScan,
            self.SCAN_TOPIC,
            self.listener_callback,
            10)
        self.angle = 0.0
        self.distance = 0.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # TODO: Write your callback functions here  
        # 
    def timer_callback(self):
        self.get_logger().info('Steering Angle: "%s"' % int(self.angle*180/np.pi))
        self.get_logger().info('Wall Distance: "%s"' % self.distance)

    def listener_callback(self, msg):
        # self.get_logger().info("Message got")
        self.angle, self.distance = self.leastsq_wall(self.scan_slice_cartesian(msg))
        self.angle += self.SIDE * (self.distance - self.DESIRED_DISTANCE)/self.DESIRED_DISTANCE * 1.5

        driveCommand = AckermannDriveStamped()
        driveCommand.header.frame_id = "base_link"
        driveCommand.header.stamp = self.get_clock().now().to_msg()
        driveCommand.drive.steering_angle= self.angle
        # driveCommand.drive.acceleration= 0
        # driveCommand.drive.jerk= 0
        driveCommand.drive.speed= self.VELOCITY
        # driveCommand.drive.steering_angle_velocity= 0
        self.simple_publisher.publish(driveCommand)

    def scan_slice_cartesian(self, subscription):
        if self.SIDE == -1:
            lowerBound = int((-subscription.angle_min-np.pi/2)/subscription.angle_increment)
            upperBound = int((-subscription.angle_min+np.pi/100)/subscription.angle_increment)
        else:
            upperBound = int((-subscription.angle_min+np.pi/2)/subscription.angle_increment)
            lowerBound = int((-subscription.angle_min-np.pi/100)/subscription.angle_increment)
        sliced_scan= [[subscription.angle_min+i*subscription.angle_increment, subscription.ranges[i]] for i in range(lowerBound, upperBound)] 
        # sliced scan a list of coordinates in angle, distance form
        position_sliced_scan=np.array([[sliced_scan[i][1]*np.cos(sliced_scan[i][0]),sliced_scan[i][1]*np.sin(sliced_scan[i][0])] for i in range(len(sliced_scan))])
        return position_sliced_scan

    def leastsq_wall(self, position_sliced_scan):
        # find linear fit based on cartesian coords
        # return angle of detected wall, distance from wall
        x = position_sliced_scan[:, 0]
        y = position_sliced_scan[:, 1]
        s,r=np.linalg.lstsq(np.vstack([x, np.ones(len(x))]).T, y)[0] # slope, residue helpp
        angle=np.arctan(s) #assuming right side for now

        VisualizationTools.plot_line([0.0, 1.0], [r, s+r], self.line)


        return [angle, self.distance_to_line(s,r)]

    def distance_to_line(self, s, r):
        a = np.array([0,r]) # point on fit line
        b = np.array([1,s+r]) # point on fit line
        p = np.array([0,0]) # car
        return np.linalg.norm(np.cross(b - a, p - a)) / np.linalg.norm(b - a) # return dist to fit line

    def parameters_callback(self, params):
        """
        DO NOT MODIFY THIS CALLBACK FUNCTION!
        
        This is used by the test cases to modify the parameters during testing. 
        It's called whenever a parameter is set via 'ros2 param set'.
        """
        for param in params:
            if param.name == 'side':
                self.SIDE = param.value
                self.get_logger().info(f"Updated side to {self.SIDE}")
            elif param.name == 'velocity':
                self.VELOCITY = param.value
                self.get_logger().info(f"Updated velocity to {self.VELOCITY}")
            elif param.name == 'desired_distance':
                self.DESIRED_DISTANCE = param.value
                self.get_logger().info(f"Updated desired_distance to {self.DESIRED_DISTANCE}")
        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    