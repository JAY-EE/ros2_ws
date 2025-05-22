import asyncio
import websockets
import json
import rclpy
from rclpy.node import Node
from final_rover.msg import ArmClient

class WebSocketServer(Node):
    def __init__(self):
        super().__init__('websocket_server')
        self.pub_arm = self.create_publisher(ArmClient, '/arm_controller_server', 10)

        # Start the WebSocket server
        self.loop = asyncio.get_event_loop()
        self.loop.run_until_complete(self.start_server())

    async def start_server(self):
        server = await websockets.serve(
            self.handler,
            'localhost',
            9090,
            ping_interval=1,    # Send a ping every 20 seconds
            ping_timeout=None      # Wait 10 seconds for a pong response
        )
        self.get_logger().info("WebSocket server started on ws://localhost:9090")
        try:
            await server.wait_closed()
        except Exception as e:
            self.get_logger().error(f"Server error: {e}")

    async def handler(self, websocket, path):
        while True:
            try:
                message = await websocket.recv()
                received_data = json.loads(message)
                arm_msg = ArmClient(
                    y=received_data['arm']['y'],
                    command=received_data['arm']['command'],
                    position=received_data['arm']['position'],
                    pitch=received_data['arm']['pitch'],
                    yaw=received_data['arm']['yaw'],
                    gripper=received_data['arm']['gripper']
                )
                self.pub_arm.publish(arm_msg)
                self.get_logger().info(f'Published: {received_data}')
            except (websockets.exceptions.ConnectionClosed, json.JSONDecodeError, KeyError) as e:
                self.get_logger().info(f"Connection error: {e}")
                break

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
