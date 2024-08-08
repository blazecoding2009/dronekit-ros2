import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State, AttitudeTarget
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from math import radians, sin, cos

class PolygonFlight(Node):
    def __init__(self):
        super().__init__('polygon_flight')

        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.local_pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.local_vel_pub = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

        self.current_state = State()
        self.offboard_setpoint = PoseStamped()

        self.timer = self.create_timer(1.0, self.timer_callback)

    def state_cb(self, msg):
        self.current_state = msg
        self.get_logger().info(f"Current State: {self.current_state}")

    def arm_and_takeoff(self, altitude):
        self.get_logger().info('Arming...')
        self.call_arming(True)

        while not self.current_state.armed:
            self.get_logger().info('Waiting for arming...')
            rclpy.spin_once(self, timeout_sec=1.0)
        
        self.get_logger().info('Armed! Taking off...')
        self.call_takeoff(altitude)

    def call_arming(self, value):
        arming_req = CommandBool.Request()
        arming_req.value = value
        future = self.arming_client.call_async(arming_req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        self.get_logger().info(f"Arming response: {result.success}")

    def call_takeoff(self, altitude):
        takeoff_req = CommandTOL.Request()
        takeoff_req.altitude = altitude
        future = self.takeoff_client.call_async(takeoff_req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        self.get_logger().info(f"Takeoff response: {result.success}")

    def set_offboard_mode(self):
        set_mode_req = SetMode.Request()
        set_mode_req.custom_mode = "OFFBOARD"
        future = self.set_mode_client.call_async(set_mode_req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        self.get_logger().info(f"Set mode response: {result.mode_sent}")

    def send_velocity(self, vx, vy, vz):
        vel_msg = TwistStamped()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.twist.linear.x = vx
        vel_msg.twist.linear.y = vy
        vel_msg.twist.linear.z = vz
        self.local_vel_pub.publish(vel_msg)

    def draw_polygon(self, sides=3, distance=5, speed=2.0):
        if sides < 3:
            self.get_logger().info("Cannot draw a polygon with less than 3 sides")
            return
        
        angle = 360 / sides
        duration = distance / speed

        for i in range(sides):
            self.get_logger().info(f"Running side {i + 1}")
            self.send_velocity(speed, 0, 0)
            rclpy.sleep(duration)
            self.condition_yaw(angle)
            rclpy.sleep(2.0)

    def condition_yaw(self, angle):
        yaw_msg = AttitudeTarget()
        yaw_msg.header.stamp = self.get_clock().now().to_msg()
        yaw_msg.type_mask = 0b100111
        yaw_msg.orientation.w = cos(radians(angle) / 2.0)
        yaw_msg.orientation.z = sin(radians(angle) / 2.0)
        self.local_pos_pub.publish(yaw_msg)

    def timer_callback(self):
        if self.current_state.mode != "OFFBOARD":
            self.set_offboard_mode()
        
        if not self.current_state.armed:
            self.call_arming(True)

    def run(self):
        self.get_logger().info('Starting Polygon Flight')
        self.arm_and_takeoff(10)
        self.draw_polygon(sides=5, distance=10)
        self.get_logger().info('Polygon Flight Complete')

def main(args=None):
    rclpy.init(args=args)
    polygon_flight = PolygonFlight()
    polygon_flight.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
