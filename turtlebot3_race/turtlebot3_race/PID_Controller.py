import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg.Twist

class PIDControlNode(Node):
    def __init__(self):
        super().__init__('pid_control_node')
        self.namespace = self.get_namespace().strip('/')

        # PID constants for linear velocity
        self.kp_linear = 1.0
        self.ki_linear = 0.0
        self.kd_linear = 0.1

        # PID constants for angular velocity
        self.kp_angular = 1.0
        self.ki_angular = 0.0
        self.kd_angular = 0.1

        # PID errors and sums
        self.linear_error_sum = 0.0
        self.angular_error_sum = 0.0
        self.previous_linear_error = 0.0
        self.previous_angular_error = 0.0

        # Subscriptions
        self.pose_sub = self.create_subscription(Pose, f'{self.namespace}/pose', self.pose_callback, 10)
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, f'{self.namespace}/cmd_vel', 10)

        self.current_speed = 0.0
        self.current_distance = 0.0

    def distance_callback(self, data):
        self.current_pose = data

    def apply_pid_control(self):
        # Example target values (can be adjusted)
        desired_speed = 1.0  # Desired forward speed
        desired_distance = 1.0  # Target distance from opponent

        # Time delta (could be more sophisticated if time is tracked more accurately)
        dt = 0.1

        # PID for linear velocity
        linear_velocity = self.pid_control_linear(desired_speed, self.current_speed, dt)

        # PID for angular velocity
        angular_velocity = self.pid_control_angular(0.0, 0.0, dt)  # Assuming we want to keep heading straight initially

        # Publish velocity commands
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity
        self.cmd_vel_pub.publish(twist_msg)

    def pid_control_linear(self, desired_speed, current_speed, dt):
        error = desired_speed - current_speed
        self.linear_error_sum += error * dt
        derivative = (error - self.previous_linear_error) / dt
        control_signal = (self.kp_linear * error) + (self.ki_linear * self.linear_error_sum) + (self.kd_linear * derivative)
        self.previous_linear_error = error
        return control_signal
    
    def pid_control_angular(self, desired_angle, current_angle, dt):
        error = desired_angle - current_angle
        self.angular_error_sum += error * dt
        derivative = (error - self.previous_angular_error) / dt
        control_signal = (self.kp_angular * error) + (self.ki_angular * self.angular_error_sum) + (self.kd_angular * derivative)
        self.previous_angular_error = error
        return control_signal

def main(args=None):
    rclpy.init(args=args)
    node = PIDControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

