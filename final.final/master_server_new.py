import asyncio
import websockets
import json
import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import String
from final_rover.msg import ArmClient, GpsMsg as gps
from geometry_msgs.msg import Twist
from threading import Lock
from datetime import datetime, timedelta

Ip = "localhost"
port_arm = 8765
port_chassis = 8766
port_feedback = 8767
port_gps = 8768

COMMAND_TIMEOUT = 0.1  # seconds

class WebSocketServer(Node):
    def __init__(self):
        super().__init__('Master_Server_Node')

        # Add locks for thread-safe data access
        self._gps_lock = Lock()
        self._joint_lock = Lock()
        self._arm_lock = Lock()
        self._chassis_lock = Lock()

        # Initialize data
        self._gps_data = None
        self._joint_data = None
        self._joint1 = 0
        self._joint2 = 0
        self._joint3 = 0
        
        self._last_arm_command_time = datetime.now()
        self._last_chassis_command_time = datetime.now()

        # Create publishers and subscribers
        self.pub_arm = self.create_publisher(ArmClient, '/arm_controller_server', 10)
        self.pub_chassis = self.create_publisher(Twist, '/cmd_vel', 10)

        # Set up subscriptions with QoS profile
        self.create_subscription(
            gps,
            '/gps_data',
            self.gps_callback,
            10
        )

        self.create_subscription(
            String,
            '/encoder_angle',
            self.angle_callback,
            10
        )

        # Create event loop in a separate thread
        self.loop = asyncio.new_event_loop()
        self.websocket_thread = threading.Thread(target=self._run_websocket_server, daemon=True)
        self.websocket_thread.start()

    def gps_callback(self, msg):
        with self._gps_lock:
            self._gps_data = msg
            self.get_logger().info(f"GPS received: {msg}")

    def angle_callback(self, msg):
        with self._joint_lock:
            try:
                joint_data = json.loads(msg.data)
                self._joint1 = joint_data.get("joint1", 0)
                self._joint2 = joint_data.get("joint2", 0)
                self._joint3 = joint_data.get("joint3", 0)
                self._joint_data = joint_data
                self.get_logger().info(f"Feedback received: {joint_data}")
            except Exception as e:
                self.get_logger().error(f"Error in angle_callback: {e}")

    async def handle_arm(self, websocket, path):
        try:
            async for message in websocket:
                self.get_logger().info(f"Received message for arm: {message}")
                data = json.loads(message)
                arm_msg = ArmClient(
                    y=data.get('arm', {}).get('y', 0),
                    command=data.get('arm', {}).get('command', 0),
                    position=data.get('arm', {}).get('position', 0),
                    pitch=data.get('arm', {}).get('pitch', 0),
                    yaw=data.get('arm', {}).get('yaw', 0),
                    gripper=data.get('arm', {}).get('gripper', 0),
                    base=data.get('arm', {}).get('base', 0)
                )
                self.pub_arm.publish(arm_msg)
                with self._arm_lock:
                    self._last_arm_command_time = datetime.now()
        except Exception as e:
            self.get_logger().error(f"Error in handle_arm: {e}")

    async def handle_chassis(self, websocket, path):
        try:
            async for message in websocket:
                self.get_logger().info(f"Received message for rover: {message}")
                data = json.loads(message)
                chassis_command = Twist()
                chassis_command.linear.x = data.get("linear", {}).get("x", 0.0)
                chassis_command.linear.y = data.get("linear", {}).get("y", 0.0)
                chassis_command.linear.z = data.get("linear", {}).get("z", 0.0)
                chassis_command.angular.z = data.get("angular", {}).get("z", 0.0)
                self.pub_chassis.publish(chassis_command)
                with self._chassis_lock:
                    self._last_chassis_command_time = datetime.now()
        except Exception as e:
            self.get_logger().error(f"Error in handle_chassis: {e}")

    async def gps_handel(self, websocket, path):
        while True:
            try:
                with self._gps_lock:
                    gps_data = self._gps_data

                if gps_data is not None:
                    await websocket.send(json.dumps({
                        'longitude': gps_data.longitude,
                        'latitude': gps_data.latitude,
                        'altitude': gps_data.altitude
                    }))
                    self.get_logger().info(f"GPS sent : {gps_data}")
                await asyncio.sleep(0.1)  # Reduced sleep time for more frequent updates
            except Exception as e:
                self.get_logger().error(f"Error in gps_handel: {e}")
                await asyncio.sleep(1)

    async def feedback_handel(self, websocket, path):
        while True:
            try:
                with self._joint_lock:
                    joint1, joint2, joint3 = self._joint1, self._joint2, self._joint3

                await websocket.send(json.dumps({
                    'joint1': joint1,
                    'joint2': joint2,
                    'joint3': joint3
                }))
                self.get_logger().info(f"Feedback Angles : joint1 : {joint1}, joint2 : {joint2}, joint3 : {joint3} ")
                await asyncio.sleep(0.1)  # Reduced sleep time for more frequent updates
            except Exception as e:
                self.get_logger().error(f"Error in feedback_handel: {e}")
                await asyncio.sleep(1)

    async def monitor_timeouts(self):
        while True:
            now = datetime.now()
            # Handle arm timeout
            with self._arm_lock:
                if (now - self._last_arm_command_time).total_seconds() > COMMAND_TIMEOUT:
                    self.get_logger().warning("No arm command received. Sending default 0 values.")
                    arm_msg = ArmClient(y=0, command=99, position=0, pitch=0, yaw=0, gripper=0, base=0)
                    self.pub_arm.publish(arm_msg)
            # Handle chassis timeout
            with self._chassis_lock:
                if (now - self._last_chassis_command_time).total_seconds() > COMMAND_TIMEOUT:
                    self.get_logger().warning("No chassis command received. Sending default 0 values.")
                    chassis_command = Twist()
                    self.pub_chassis.publish(chassis_command)

            await asyncio.sleep(0.1)

    async def start_server(self):
        arm_server = await websockets.serve(self.handle_arm, Ip, port_arm, ping_timeout=None)
        chassis_server = await websockets.serve(self.handle_chassis, Ip, port_chassis, ping_timeout=None)
        gps_server = await websockets.serve(self.gps_handel, Ip, port_gps, ping_timeout=None)
        feedback_server = await websockets.serve(self.feedback_handel, Ip, port_feedback, ping_timeout=None)

        self.get_logger().info(f"WebSocket server started on ws://{Ip}")

        await asyncio.gather(
            arm_server.wait_closed(),
            chassis_server.wait_closed(),
            gps_server.wait_closed(),
            feedback_server.wait_closed()
        )

    def _run_websocket_server(self):
        """Run the WebSocket server in a separate thread"""
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(asyncio.gather(
            self.start_server(),
            self.monitor_timeouts()
        ))
        self.loop.run_forever()

    def run(self):
        """Main run method"""
        # Create and start the ROS executor in the main thread
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self)

        try:
            executor.spin()
        except KeyboardInterrupt:
            pass

def main(args=None):
    rclpy.init(args=args)
    server = WebSocketServer()

    try:
        server.run()
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
