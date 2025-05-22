

import rclpy
from rclpy.node import Node
from final_rover.msg import ArmClient as Arm  # Ensure this matches your message type
import asyncio
import websockets
import json

class ArmClient(Node):
    def __init__(self):
        super().__init__('arm_client_node')

        # Create a subscription to the /arm_client topic
        self.create_subscription(Arm, '/arm_client', self.callback_arm, 10)

        # Start the WebSocket client
        self.loop = asyncio.get_event_loop()
        self.loop.run_until_complete(self.connect_to_server())

    async def connect_to_server(self):
        self.websocket = await websockets.connect('ws://localhost:9090')
        self.get_logger().info("Connected to WebSocket server")

    def callback_arm(self, data):
        message = {
            'arm': {
                'y': data.y,
                'command': (data.command),  # Convert back to character
                'position': data.position,
                'pitch': data.pitch,
                'yaw': data.yaw,
                'gripper': data.gripper
            }
        }
        asyncio.run(self.send_to_server(message))

    async def send_to_server(self, message):
        await self.websocket.send(json.dumps(message))
        self.get_logger().info(f'Sent to WebSocket server: {message}')

def main(args=None):
    rclpy.init(args=args)
    arm_client = ArmClient()

    try:
        rclpy.spin(arm_client)
    finally:
        arm_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


