#!/usr/bin/env python3
"""
Odometry Fusion Node
Fuses ZED visual odometry with wheel odometry from computer team using Kalman Filter
Publishes fused odometry to /autonomy/fused_odom
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
import numpy as np
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
import math

class OdometryFusion(Node):
    def __init__(self):
        super().__init__('odometry_fusion')

        # Kalman filter state: [x, y, theta, vx, vy, omega]
        self.state = np.zeros(6)

        # State covariance matrix
        self.P = np.eye(6) * 0.1

        # Process noise covariance
        self.Q = np.eye(6) * 0.01

        # Measurement noise covariance for visual odometry
        self.R_visual = np.eye(6) * 0.05

        # Measurement noise covariance for wheel odometry
        self.R_wheel = np.eye(6) * 0.1

        # Last update time
        self.last_time = None

        # QoS profile for RTAB-Map odometry (uses RELIABLE)
        rtabmap_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # QoS profile for wheel odometry (may use BEST_EFFORT)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to ZED odometry (from RTAB-Map)
        self.sub_zed_odom = self.create_subscription(
            Odometry,
            '/rtabmap/odom',
            self.zed_odom_callback,
            rtabmap_qos
        )

        # Subscribe to wheel odometry (from computer team)
        self.sub_wheel_odom = self.create_subscription(
            Odometry,
            '/wheel_odom',  # Adjust topic name based on computer team's output
            self.wheel_odom_callback,
            sensor_qos
        )

        # Publisher for fused odometry
        self.fused_odom_pub = self.create_publisher(
            Odometry,
            '/autonomy/fused_odom',
            10
        )

        self.get_logger().info('Odometry Fusion node initialized')
        self.get_logger().info('Subscribing to: /rtabmap/odom and /wheel_odom')
        self.get_logger().info('Publishing to: /autonomy/fused_odom')

    def predict(self, dt):
        """Prediction step of Kalman filter"""
        if dt <= 0:
            return

        # State transition matrix (simple constant velocity model)
        F = np.array([
            [1, 0, 0, dt, 0,  0],
            [0, 1, 0, 0,  dt, 0],
            [0, 0, 1, 0,  0,  dt],
            [0, 0, 0, 1,  0,  0],
            [0, 0, 0, 0,  1,  0],
            [0, 0, 0, 0,  0,  1]
        ])

        # Predict state
        self.state = F @ self.state

        # Normalize theta to [-pi, pi]
        self.state[2] = math.atan2(math.sin(self.state[2]), math.cos(self.state[2]))

        # Predict covariance
        self.P = F @ self.P @ F.T + self.Q

    def update(self, measurement, R):
        """Update step of Kalman filter"""
        # Measurement matrix (we observe all states)
        H = np.eye(6)

        # Innovation
        y = measurement - H @ self.state

        # Normalize angle difference
        y[2] = math.atan2(math.sin(y[2]), math.cos(y[2]))

        # Innovation covariance
        S = H @ self.P @ H.T + R

        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update state
        self.state = self.state + K @ y

        # Normalize theta
        self.state[2] = math.atan2(math.sin(self.state[2]), math.cos(self.state[2]))

        # Update covariance
        self.P = (np.eye(6) - K @ H) @ self.P

    def odom_to_state(self, odom_msg):
        """Convert Odometry message to state vector"""
        # Extract position
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y

        # Extract orientation (convert quaternion to yaw)
        qx = odom_msg.pose.pose.orientation.x
        qy = odom_msg.pose.pose.orientation.y
        qz = odom_msg.pose.pose.orientation.z
        qw = odom_msg.pose.pose.orientation.w

        # Yaw from quaternion
        theta = math.atan2(2.0 * (qw * qz + qx * qy),
                          1.0 - 2.0 * (qy * qy + qz * qz))

        # Extract velocities
        vx = odom_msg.twist.twist.linear.x
        vy = odom_msg.twist.twist.linear.y
        omega = odom_msg.twist.twist.angular.z

        return np.array([x, y, theta, vx, vy, omega])

    def zed_odom_callback(self, msg):
        """Process ZED visual odometry"""
        current_time = self.get_clock().now()

        if self.last_time is not None:
            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.predict(dt)

        # Extract measurement
        measurement = self.odom_to_state(msg)

        # Update with visual odometry (lower noise)
        self.update(measurement, self.R_visual)

        # Publish fused odometry
        self.publish_fused_odom(msg.header.frame_id, current_time)

        self.last_time = current_time

    def wheel_odom_callback(self, msg):
        """Process wheel odometry"""
        current_time = self.get_clock().now()

        if self.last_time is not None:
            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.predict(dt)

        # Extract measurement
        measurement = self.odom_to_state(msg)

        # Update with wheel odometry (higher noise)
        self.update(measurement, self.R_wheel)

        # Publish fused odometry
        self.publish_fused_odom(msg.header.frame_id, current_time)

        self.last_time = current_time

    def publish_fused_odom(self, frame_id, timestamp):
        """Publish the fused odometry estimate"""
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Position
        odom_msg.pose.pose.position.x = self.state[0]
        odom_msg.pose.pose.position.y = self.state[1]
        odom_msg.pose.pose.position.z = 0.0

        # Orientation (yaw to quaternion)
        theta = self.state[2]
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        # Pose covariance (from P matrix)
        odom_msg.pose.covariance[0] = self.P[0, 0]   # x
        odom_msg.pose.covariance[7] = self.P[1, 1]   # y
        odom_msg.pose.covariance[35] = self.P[2, 2]  # theta

        # Velocity
        odom_msg.twist.twist.linear.x = self.state[3]
        odom_msg.twist.twist.linear.y = self.state[4]
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.z = self.state[5]

        # Twist covariance
        odom_msg.twist.covariance[0] = self.P[3, 3]   # vx
        odom_msg.twist.covariance[7] = self.P[4, 4]   # vy
        odom_msg.twist.covariance[35] = self.P[5, 5]  # omega

        self.fused_odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryFusion()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
