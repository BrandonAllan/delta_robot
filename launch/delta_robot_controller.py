import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64, String

# Import your inverse kinematics C++ file
from delta_robot.inverse_kinematics import delta_calcInverse


class DeltaRobotController(Node):
    def __init__(self):
        super().__init__('delta_robot_controller')

        # Initialize the publishers and subscribers
        self.coordinates_subscription = self.create_subscription(
            Float32MultiArray,
            'delta_robot_coordinates',
            self.coordinates_callback,
            10
        )
        self.speed_subscription = self.create_subscription(
            Float64,
            'servo_speed',
            self.speed_callback,
            10
        )
        self.control_subscription = self.create_subscription(
            String,
            'control_servos_cmd',
            self.control_callback,
            10
        )
        self.feedback_publisher = self.create_publisher(
            Float32MultiArray,
            'delta_robot_feedback',
            10
        )

        # Initialize the servomotor objects and configure them accordingly
        # (replace with your actual servomotor implementation)
        self.servo1 = ServoMotor(1)
        self.servo2 = ServoMotor(2)
        self.servo3 = ServoMotor(3)

        # Default speed for servomotors
        self.servo_speed = 0.5

    def coordinates_callback(self, msg):
        # Extract the x, y, z coordinates from the received message
        x, y, z = msg.data

        # Use inverse kinematics to calculate the joint angles for the delta robot
        inv_kin = delta_calcInverse(x, y, z)
        theta1 = inv_kin.theta1
        theta2 = inv_kin.theta2
        theta3 = inv_kin.theta3

        # Set the servomotor positions based on the joint angles
        self.servo1.set_position(theta1)
        self.servo2.set_position(theta2)
        self.servo3.set_position(theta3)

        # Publish feedback information (e.g., joint angles or positions)
        feedback_msg = Float32MultiArray()
        feedback_msg.data = [theta1, theta2, theta3]
        self.feedback_publisher.publish(feedback_msg)

    def speed_callback(self, msg):
        # Update the speed for servomotors
        self.servo_speed = msg.data

    def control_callback(self, msg):
        # Extract the positions for each motor from the received message
        positions = msg.data

        # Split the positions for each motor from the received message
        positions = positions.split(',')

        # Set the positions for each motor
        self.servo1.set_position(float(positions[0]))
        self.servo2.set_position(float(positions[1]))
        self.servo3.set_position(float(positions[2]))


def main(args=None):
    rclpy.init(args=args)
    node = DeltaRobotController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()