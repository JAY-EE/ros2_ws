import asyncio
import websockets
import json
import time
import rclpy
from rclpy.node import Node
from final_rover.msg import ArmClient
from std_msgs.msg import Int16

class WebSocketClient(Node):
    def __init__(self):
        super().__init__('websocket_client')
        self.data_to_send = {
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

        self.received_dict = {
            "time": time.time(),
            'science': {
                'pressure': 0,
                'soil_moisture': 0
            }
        }

        self.create_subscription(ArmClient, '/arm_client', self.callback_arm, 10)
        self.create_subscription(Int16, '/science_client', self.callback_science, 10)

        self.loop = asyncio.get_event_loop()
        self.loop.run_until_complete(self.connect())

    async def connect(self):
        async with websockets.connect('ws://localhost:9090') as websocket:
            while True:
                json_object = json.dumps(self.data_to_send, indent=4)
                await websocket.send(json_object)

                received_message = await websocket.recv()
                self.received_dict = json.loads(received_message)
                print(self.received_dict)

                await asyncio.sleep(0.1)

    def callback_arm(self, data):
        self.data_to_send['arm'] = {
            'y': data.y,
            'command': chr(data.command),
            'position': data.position,
            'pitch': data.pitch,
            'yaw': data.yaw,
            'gripper': data.gripper
        }

    def callback_science(self, data):
        self.data_to_send['science']['step'] = data.data

def main(args=None):
    rclpy.init(args=args)
    websocket_client = WebSocketClient()

    try:
        rclpy.spin(websocket_client)
    except KeyboardInterrupt:
        pass
    finally:
        websocket_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
