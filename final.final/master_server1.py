import asyncio
import websockets
import json
import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import String, Int32
from final_rover.msg import ArmClient, GpsMsg as gps
from geometry_msgs.msg import Twist

Ip = "localhost"
port_arm = 8765
port_chassis = 8766
port_feedback = 8767
port_gps = 8768

class WebSocketServer(Node):
    def __init__(self):
        super().__init__('Master_Server_Node')

        self.pub_arm = self.create_publisher(ArmClient, '/arm_controller_server', 10)
        self.pub_chassis = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(gps, '/gps_data', self.gps_callback, 10)
        self.create_subscription(String,'/encoder_angle',self.angle_callback,10)

        self.gps_data = gps()
        self.joint_data = 0

        self.joint1 = 0
        self.joint2 = 0
        self.joint3 = 0 

        # Initialize Server loop
        self.loop = asyncio.get_event_loop()
        self.loop.run_until_complete(self.start_server())

     

    async def handle_arm(self, websocket, path):
        try:
            async for message in websocket:
                # self.get_logger().info(f"Received message for arm: {message}")

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
        except websockets.exceptions.ConnectionClosedError:
            self.get_logger().error("Connection closed unexpectedly, trying to reconnect...")
        except Exception as e:
            self.get_logger().error(f"Error in handle_arm: {e}")
        



    async def handle_chassis(self, websocket, path):
        try:
            async for message in websocket:
                # self.get_logger().info(f"Received message for rover: {message}")
                data = json.loads(message)
                chassis_command = Twist()
                chassis_command.linear.x = data.get("linear", {}).get("x", 0.0)
                chassis_command.linear.y = data.get("linear", {}).get("y", 0.0)
                chassis_command.linear.z = data.get("linear", {}).get("z", 0.0)
                chassis_command.angular.z = data.get("angular", {}).get("z", 0.0)
                self.pub_chassis.publish(chassis_command)
                
        except websockets.exceptions.ConnectionClosedError:
            self.get_logger().error("Connection closed unexpectedly, trying to reconnect...")
            # Optionally, handle reconnection logic here
        except Exception as e:
            self.get_logger().error(f"Error in handle_chassis: {e}")


    async def gps_callback(self,msg):
        self.gps_data = msg
        self.get_logger().info(f"recieved : {self.gps_data}")

    async def angle_callback(self, msg):
        self.get_logger().info(f"Received message on '/encoder_angle': {msg.data}")
        try:
            self.joint_data = json.loads(msg.data)  # Parse the JSON data
            self.joint1 = self.joint_data.get("joint1", 0)
            self.joint2 = self.joint_data.get("joint2", 0)
            self.joint3 = self.joint_data.get("joint3", 0)
            self.get_logger().info(f"Received joint angles: {self.joint1}, {self.joint2}, {self.joint3}")
        except Exception as e:
            self.get_logger().error(f"Error in angle_callback: {e}")

    async def gps_handel(self, websocket, path): 
         while True:
            if self.gps_data:
                self.get_logger().info(f"Sending GPS data: {self.gps_data}")
                await websocket.send(json.dumps({
                    'longitude': self.gps_data.longitude,
                    'latitude': self.gps_data.latitude,
                    'altitude': self.gps_data.altitude
                }))
            else:
                self.get_logger().info("No GPS data to send yet.")
            await asyncio.sleep(1)

    async def feedback_handel(self, websocket, path): 
         while True:
            if self.joint_data:
                self.get_logger().info(f"Sending Joint data: {self.joint_data}")
                await websocket.send(json.dumps({
                    'joint1': self.joint1,
                    'joint2': self.joint2,
                    'joint3': self.joint3
                }))
            else:
                self.get_logger().info("No angles send yet")
            await asyncio.sleep(1)

    async def start_server(self):
        #  Server setup
        arm_server = await websockets.serve(self.handle_arm, Ip, port_arm, ping_timeout=None)
        chassis_server = await websockets.serve(self.handle_chassis, Ip, port_chassis, ping_timeout=None)
        gps_server = await websockets.serve(self.gps_handel, Ip, port_gps, ping_timeout=None)
        feedback_server = await websockets.serve(self.feedback_handel, Ip, port_feedback, ping_timeout=None)

        self.get_logger().info(f"WebSocket server started on ws://{Ip}")

        try:
            await asyncio.gather(
                arm_server.wait_closed(),
                chassis_server.wait_closed(),
                gps_server.wait_closed(),
                feedback_server.wait_closed(),
            )
        except Exception as e:
            self.get_logger().error(f"Server error: {e}")

    def run(self):
        # Set up the ROS executor and threading
        rclpy_executor = rclpy.executors.MultiThreadedExecutor()
        rclpy_executor.add_node(self)
        rclpy_thread = threading.Thread(target=rclpy_executor.spin, daemon=True)
        rclpy_thread.start()
        # rclpy_executor.spin()

        # Start the Server 
        self.loop.run_until_complete(self.start_server())


def main(args=None):
    rclpy.init(args=args)
    websocket_server = WebSocketServer()
    
    try:
        websocket_server.run()
    except KeyboardInterrupt:
        pass
    finally:
        websocket_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
