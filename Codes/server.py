import asyncio
import websockets
import json
import time
import rclpy
from rclpy.node import Node
from final_rover.msg import ArmClient
from std_msgs.msg import Int16, Float32
from sensor_msgs.msg import FluidPressure

class WebSocketServer(Node):
    def __init__(self):
        super().__init__('websocket_server')
        # self.pub_rover = self.create_publisher(RoverMsg, '/rover_controller_server', 10)
        self.pub_arm = self.create_publisher(ArmClient, '/arm_controller_server', 10)
        self.pub_science = self.create_publisher(Int16, '/science_server', 10)

        self.data_to_send = {
            "time": time.time(),
            'science': {
                'pressure': 0,
                'soil_moisture': 0
            }
        }

        self.received_data = {
            'rover': {
                'x': 0,
                'y': 0,
                'isPID': False
            },
            'arm': {
                'y': 0,
                'command': 'c',
                'position': 0,
                'pitch': 0,
                'yaw': 0,
                'gripper': 0
            },
            'science': {
                'step': 0
            }
        }

        self.create_subscription(Float32, '/moisture_feedback', self.callback_moisture, 10)
        self.create_subscription(FluidPressure, '/zed2i/zed_node/atm_press', self.callback_pressure, 10)

        self.loop = asyncio.get_event_loop()
        self.loop.run_until_complete(self.start_server())

    async def start_server(self):
        server = await websockets.serve(self.handler, 'localhost', 9090)
        print("WebSocket server started on ws://localhost:9090")
        await server.wait_closed()

    async def handler(self, websocket, path):
        while True:
            try:
                message = await websocket.recv()
                self.received_data = json.loads(message)

                arm_msg = ArmClient(
                    y=self.received_data['arm']['y'],
                    command=ord(self.received_data['arm']['command']),
                    position=self.received_data['arm']['position'],
                    pitch=self.received_data['arm']['pitch'],
                    yaw=self.received_data['arm']['yaw'],
                    gripper=self.received_data['arm']['gripper']
                )
                self.pub_arm.publish(arm_msg)

                science_msg = Int16(data=self.received_data['science']['step'])
                self.pub_science.publish(science_msg)

                print(self.received_data)

                self.data_to_send["time"] = time.time()
                await websocket.send(json.dumps(self.data_to_send))

            except websockets.exceptions.ConnectionClosed:
                print("Connection closed")
                break

    def callback_moisture(self, data):
        self.data_to_send['science']['soil_moisture'] = data.data

    def callback_pressure(self, data):
        self.data_to_send['science']['pressure'] = data.fluid_pressure * 100

def main(args=None):
    rclpy.init(args=args)
    websocket_server = WebSocketServer()
    
    try:
        rclpy.spin(websocket_server)
    except KeyboardInterrupt:
        pass
    finally:
        websocket_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
