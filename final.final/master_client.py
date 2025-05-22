import asyncio
import websockets
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from final_rover.msg import ArmClient as Arm, GpsMsg  

Ip = "192.168.1.8"
port_arm = 8765
port_chassis = 8766
port_feedback = 8767
port_gps = 8768

websocket_arm = None
websocket_rover = None
gps_data = GpsMsg()
feedback_data = 0

joint1 = 0
joint2 = 0
joint3 = 0

longi = 0
lati = 0 
alti = 0

async def receive_gps_data():
    global gps_data, gps_data_obj, lati,alti,longi
    uri_gps = f"ws://{Ip}:{port_gps}"

    while True:
        try:
            async with websockets.connect(uri_gps) as websocket:
                while True:
                    received_data = await websocket.recv()
                    gps_data = json.loads(received_data)
                    # Update the gps_data with received values
                    gps_data_obj = GpsMsg()
                    gps_data_obj.longitude = gps_data.get('longitude', 0.0)
                    gps_data_obj.latitude = gps_data.get('latitude', 0.0)
                    gps_data_obj.altitude = gps_data.get('altitude', 0.0)

                    longi = gps_data.get('longitude', 0.0)
                    lati = gps_data.get('latitude', 0.0)
                    alti = gps_data.get('altitude', 0.0)
                    print(f"Received GPS data: {gps_data_obj}")
        except (websockets.exceptions.ConnectionClosed, ConnectionRefusedError):
            print("Connection to GPS server closed. Reconnecting...")
            await asyncio.sleep(2)  # For Reconnect

async def receive_feedback_data():
    global feedback_data, joint1, joint2, joint3
    uri_feedback = f"ws://{Ip}:{port_feedback}"

    while True:
        try:
            async with websockets.connect(uri_feedback) as websocket:
                while True:
                    received_data = await websocket.recv()
                    feedback_data = json.loads(received_data)
                    # Update the global joint angles
                    joint1 = feedback_data.get('joint1', 0.0)
                    joint2 = feedback_data.get('joint2', 0.0)
                    joint3 = feedback_data.get('joint3', 0.0)
                    print(f"Received feedback data: {feedback_data}")
        except (websockets.exceptions.ConnectionClosed, ConnectionRefusedError):
            print("Connection to feedback server closed. Reconnecting...")
            await asyncio.sleep(2)  # For Reconnect

class WebSocketClient(Node):
    def __init__(self):
        super().__init__('Master_Client_Node')

        # Create Subscription 
        self.create_subscription(Arm, '/arm_client', self.callback_arm, 10)
        self.create_subscription(Twist, '/rover_client', self.callback_chassis, 10)
        self.pub_angle = self.create_publisher(String, '/joint_angles_json', 10)
        self.pub_gps = self.create_publisher(String,'/gps_publisher',10)
        
        self.gps = gps_data
        # Remove joint angle initialization here since we'll get them from globals

        # Start the WebSocket Connection
        self.loop = asyncio.get_event_loop()
        self.loop.create_task(receive_gps_data())
        self.loop.create_task(receive_feedback_data())
        self.loop.create_task(self.setup_connections())

    async def setup_connections(self):
        global websocket_arm, websocket_rover

        try:
            websocket_arm = await websockets.connect(f'ws://{Ip}:{port_arm}', ping_interval=20, ping_timeout=10)
            websocket_rover = await websockets.connect(f'ws://{Ip}:{port_chassis}', ping_interval=20, ping_timeout=10)
            self.get_logger().info("Connected to WebSocket server")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to WebSocket server: {e}")
            await asyncio.sleep(5)  # Retry Connection
            await self.setup_connections() 

    def callback_arm(self, data):
        global joint1, joint2, joint3  

        message = {
            'arm': {
                'y': data.y,
                'command': data.command,
                'position': data.position,
                'pitch': data.pitch,
                'yaw': data.yaw,
                'gripper': data.gripper,
                'base': data.base
            }
        }

        joint_angles = { 
            "angle1": joint2,
            "angle2": joint3
        }
        json_data = json.dumps(joint_angles)
        msg = String()
        msg.data = json_data

        # Send arm data to server
        asyncio.create_task(self.send_arm_server(message))
        # publish angles for simulation
        self.pub_angle.publish(msg)
        # self.get_logger().info(f"Sending to arm server: {message}")

    async def send_arm_server(self, message):
        global websocket_arm

        if websocket_arm:
            try:
                await websocket_arm.send(json.dumps(message))
                self.get_logger().info(f"Sent to Arm : {message}")
            except websockets.exceptions.ConnectionClosedError as e:
                self.get_logger().error(f"Connection closed to arm. Reconnecting ... : {e}")
                await self.setup_connections()
                await self.send_arm_server(message)
            except Exception as e:
                self.get_logger().error(f"Error: {e}")
        else:
            self.get_logger().warn("WebSocket connection to arm server is not established yet.")

    def callback_chassis(self, msg):
        global gps_data, gps_data_obj, longi, alti , lati

        x = msg.linear.x
        y = msg.linear.y
        p = msg.linear.z
        ang_z = msg.angular.z

        message = {
            "linear": {"x": x, "y": y, "z": p},
            "angular": {"z": ang_z}
        }


        gps_msg = {
            "longitude": longi , 
            "latitude": lati,
            "altitude": alti
        }
        json_data = json.dumps(gps_msg)
        gps_pub_msg = String()
        gps_pub_msg.data = json_data

        # Send chassis data 
        asyncio.create_task(self.send_rover_server(message))
        # publish gps
        self.pub_gps.publish(gps_pub_msg)
  

    async def send_rover_server(self, msg):
        global websocket_rover

        if websocket_rover:
            try:
                await websocket_rover.send(json.dumps(msg))
                self.get_logger().info(f"Sent to Rover : {msg}")
            except websockets.exceptions.ConnectionClosedError as e:
                self.get_logger().error(f"Connection to rover. Reconnecting ... : {e}")
                await self.setup_connections()
                await self.send_rover_server(msg)
            except Exception as e:
                self.get_logger().error(f"Error : {e}")
        else:
            self.get_logger().warn("WebSocket connection to rover server is not established yet.")

async def main_async():
    rclpy.init(args=None)
    node = WebSocketClient()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            await asyncio.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

def main():
    asyncio.run(main_async())

if __name__ == '__main__':
    main()