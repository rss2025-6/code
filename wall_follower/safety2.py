#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import SetParametersResult
from wall_follower.visualization_tools import VisualizationTools


class SafetyController(Node):

    def __init__(self):
        super().__init__("safety_controller")
        
        # Declare parameters for topics and fov angle
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/vesc/low_level/ackermann_cmd")
        self.declare_parameter("safety_topic", "vesc/low_level/input/safety")
        self.declare_parameter("fov_angle", 16./180*np.pi)

        # Fetch parameters
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SAFETY_TOPIC = self.get_parameter('safety_topic').get_parameter_value().string_value
        self.FOV_ANGLE = self.get_parameter('fov_angle').get_parameter_value().double_value

        # Initialize control publisher
        self.control_pub = self.create_publisher(AckermannDriveStamped, self.SAFETY_TOPIC, 10)

        self.line = self.create_publisher(Marker, "safety_wall", 10)
        self.linefit = self.create_publisher(Marker, "wallfit", 10)
        
        # Initialize subscrber to laser scan and velocity data
        self.drive_sub = self.create_subscription(AckermannDriveStamped, self.DRIVE_TOPIC, self.drive_cb, 10)
        self.laser_sub = self.create_subscription(LaserScan, self.SCAN_TOPIC, self.laser_callback, 10)

        # Set the side to -1
        self.SIDE = -1

        # Initialize car velocity to 0
        self.car_velocity = 0.

    # Callback to record car velocity
    def drive_cb(self, msg):
        self.car_velocity = msg.drive.speed

    # Get the 10th percentile distance from a set of ranges
    def get_closest_points(self, msg):
        angle_min, angle_max, angle_inc = msg.angle_min, msg.angle_max, msg.angle_increment
        angles = np.arange(angle_min, angle_max, angle_inc)
        # only consider angles within +- FOV_ANGLE
        #upperBound = int((-msg.angle_min + self.FOV_ANGLE)/msg.angle_increment)
        angle_min_filter, angle_max_filter = np.pi / 16, -np.pi / 16
        valid_indices = (angles >= angle_max_filter) & (angles <= angle_min_filter)
        valid_index_values = np.where(valid_indices)[0]
        lowerBound = valid_index_values[0]
        upperBound = valid_index_values[-1]
        #upperBound = int((msg.angle_min + 2*self.FOV_ANGLE)/msg.angle_increment)
        #lowerBound = -int((msg.angle_min + self.FOV_ANGLE)/msg.angle_increment)
        ranges = [msg.ranges[i] for i in range(lowerBound, upperBound)]

        # return 10th percentile
        return np.percentile(ranges, 10.)

    # Callback for laserscan data, also checks distance to control safety
    def laser_callback(self, msg):
        
        # Get closest distance to the car based on 10th percentile
        self.distance = self.get_closest_points(msg)
        self.get_logger().info(f"DISTANCE = {self.distance:.2f}")

        # Calculate closest allowable distance based on car velocity
        closest_dist = self.car_velocity * 0.5 + 0.05 #+ 0.15

        # If the car distance is less than the safe distance, send speed of 0
        if (self.distance < closest_dist):

            # Create drive message
            driveCommand = AckermannDriveStamped()
            driveCommand.header.frame_id = "base_link"
            driveCommand.header.stamp = self.get_clock().now().to_msg()
            
            driveCommand.drive.steering_angle= 0.
            driveCommand.drive.speed= 0.

            # Publish drive message
            self.control_pub.publish(driveCommand)

    # TODO: DELETE IF WE DON'T USE
    #def scan_slice_cartesian(self, subscription):
    #    upperBound = int((-subscription.angle_min + self.FOV_ANGLE)/subscription.angle_increment)
    #    lowerBound = -int((subscription.angle_min + self.FOV_ANGLE)/subscription.angle_increment)
    #    sliced_scan= [[subscription.angle_min+i*subscription.angle_increment, subscription.ranges[i]] for i in range(lowerBound, upperBound)]

        # sliced scan a list of coordinates in angle, distance form
    #    position_sliced_scan=np.array([[sliced_scan[i][1]*np.cos(sliced_scan[i][0]),sliced_scan[i][1]*np.sin(sliced_scan[i][0])] for i in range(len(sliced_scan))])
    #    return position_sliced_scan

    #def leastsq_wall(self, position_sliced_scan):
    #    # find linear fit based on cartesian coords
    #    # return angle of detected wall, distance from wall
    #    x = position_sliced_scan[:, 0]
    #    y = position_sliced_scan[:, 1]
    #    s,r=np.linalg.lstsq(np.vstack([x, np.ones(len(x))]).T, y)[0] # slope, residue helpp
    #    angle=np.arctan(s) #assuming right side for now

    #    VisualizationTools.plot_line(x, y, self.line, frame="/laser")
    #    VisualizationTools.plot_line([0.,1.],[r,s+r], self.linefit, frame="/laser")

    #    return [angle, self.distance_to_line(s,r)]

    #def distance_to_line(self, s, r):
    #    a = np.array([0,r]) # point on fit line
    #    b = np.array([1,s+r]) # point on fit line
    #    p = np.array([0,0]) # car
    #    return np.linalg.norm(np.cross(b - a, p - a)) / np.linalg.norm(b - a) # return dist to fit line


    #def parameters_callback(self, params):
    #    ""
    #    DO NOT MODIFY THIS CALLBACK FUNCTION!

    #    This is used by the test cases to modify the parameters during testing.
    #    It's called whenever a parameter is set via 'ros2 param set'.
    #    """
    #    for param in params:
    #        if param.name == 'side':
    #            self.SIDE = param.value
    #            self.get_logger().info(f"Updated side to {self.SIDE}")
    #        elif param.name == 'velocity':
    #            self.VELOCITY = param.value
    #            self.get_logger().info(f"Updated velocity to {self.VELOCITY}")
    #        elif param.name == 'desired_distance':
    #            self.DESIRED_DISTANCE = param.value
    #            self.get_logger().info(f"Updated desired_distance to {self.DESIRED_DISTANCE}")
    #    return SetParametersResult(successful=True)


def main():
    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

