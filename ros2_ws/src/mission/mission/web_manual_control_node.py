#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
import uvicorn
import threading
import webview
import json
import asyncio
from ament_index_python.packages import get_package_share_directory
import os
from typing import Set
from datetime import datetime
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class DroneWebControl(Node):
    def __init__(self):
        super().__init__('web_manual_control')  # Changed to super() for proper initialization
        
        # Load configuration
        self.web_dir = os.path.join(get_package_share_directory('mission'), 'control_interface')
        self.html_path = os.path.join(self.web_dir, 'index.html')

        # QoS configuration
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=8
        )

        # Publisher template expansion
        self.control_publishers = {}
        self.control_subscribers = {}
        self.sensor_data = {}
        
        self.control_publishers['/go_winch'] = self.create_publisher(String, '/go_winch', 10)
        self.control_publishers['/go_bucket_valve'] = self.create_publisher(String, '/go_bucket_valve', 10)
        self.control_publishers['/battery_changed'] = self.create_publisher(String, '/battery_changed', 10)
        self.control_publishers['/approach_target_graph'] = self.create_publisher(String, '/approach_target_graph', qos_profile)
        self.control_publishers['/bucket_number'] = self.create_publisher(Int32, '/bucket_number', 10)
        self.control_publishers['/abort_brake'] = self.create_publisher(String, '/abort_brake', qos_profile)
        self.control_publishers['/confirm_arming'] = self.create_publisher(String, '/confirm_arming', 10)
        self.control_publishers['/go_vision'] = self.create_publisher(String, '/go_vision', 10)
        self.control_publishers['/valve_state'] = self.create_publisher(String, '/valve_state', 10)
        
        self.control_subscribers['/water_qty'] = self.create_subscription(Int32, '/water_qty', self.water_qty_callback, 10)
        self.sensor_data['/water_qty'] = 0
        self.control_subscribers['/torque'] = self.create_subscription(Float32, '/torque', self.torque_callback, 10)
        self.sensor_data['/torque'] = 0
        
        # Web interface components
        self.active_connections: Set[WebSocket] = set()
        self.connections_lock = threading.Lock()
        self.app = FastAPI()
        self.app.mount(
            "/static",
            StaticFiles(directory=os.path.join(self.web_dir, "static")),
            name="static"
        )
        self.setup_web_server()
        
        # Get or create event loop
        try:
            self.loop = asyncio.get_event_loop()
        except RuntimeError:
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
                
        # Start web server in a separate thread
        self.server_thread = threading.Thread(
            target=self.run_web_server,
            daemon=True
        )
        self.server_thread.start()
        
        # Start webview
        self.start_webview()

    def water_qty_callback(self, msg):
        self.sensor_data['/water_qty'] = msg.data
        self.schedule_async_task(self.broadcast_sensor_data())

    def torque_callback(self, msg):
        self.sensor_data['/torque'] = msg.data
        self.schedule_async_task(self.broadcast_sensor_data())

    def schedule_async_task(self, coro):
        asyncio.run_coroutine_threadsafe(coro, self.loop)

    async def broadcast_sensor_data(self):
        message = json.dumps({
            'type': 'sensor_data',
            'data': self.sensor_data
        })
        await self.broadcast_message(message)

    async def send_terminal_message(self, message):
        timestamp = datetime.now().strftime("%H:%M:%S")
        message_json = json.dumps({
            'type': 'terminal',
            'message': f"[{timestamp}] {message}"
        })
        await self.broadcast_message(message_json)

    async def broadcast_message(self, message):
        with self.connections_lock:
            dead_connections = []
            for connection in self.active_connections:
                try:
                    await connection.send_text(message)
                except Exception as e:
                    self.get_logger().error(f"Message send failed: {e}")
                    dead_connections.append(connection)
            
            for connection in dead_connections:
                self.active_connections.remove(connection)

    def setup_web_server(self):
        @self.app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket):
            await websocket.accept()
            with self.connections_lock:
                self.active_connections.add(websocket)
            await self.send_terminal_message("New client connected")
            
            try:
                while True:
                    data = await websocket.receive_text()
                    await self.process_web_command(data)
            except WebSocketDisconnect:
                with self.connections_lock:
                    self.active_connections.remove(websocket)
                await self.send_terminal_message("Client disconnected")

        @self.app.get("/")
        async def serve_interface():
            with open(self.html_path, 'r', encoding='utf-8') as file:
                html_content = file.read()
            
            # CACHE BUSTING TO COMMENT IF NO CHANGES ARE MADE TO CSS/JS
            import time
            css_version = int(time.time())
            html_content = html_content.replace(
                'href="static/css/',
                f'href="static/css/?v={css_version}"'
            )
            html_content = html_content.replace(
                'src="static/index.js"',
                f'src="static/index.js?v={css_version}"'
            )

            return HTMLResponse(content=html_content, status_code=200)

    async def process_web_command(self, body):
        try:
            message = json.loads(body)
            if message['type'] == 'command':
                command = message['command']
                data = message.get('data')
                topic_type = message.get('topicType')
                self.get_logger().info(f"Received command: {command}, data: {data}, topicType: {topic_type}")
                if command in self.control_publishers:
                    if data:
                        if topic_type == 'String':
                            msg = String()
                        elif topic_type == 'Int32':
                            msg = Int32()
                            data = int(data)
                        elif topic_type == 'Float32':
                            msg = Float32()
                            data = float(data)
                        msg.data = data
                    else:
                        msg = String()
                        msg.data = command
                    self.control_publishers[command].publish(msg)
                    await self.send_terminal_message(f"Command executed: {command} with data: {data}")
                else:
                    await self.send_terminal_message(f"Unknown command: {command}")
        except Exception as e:
            await self.send_terminal_message(f"Error processing command: {str(e)}")

    def run_web_server(self):
        try:
            self.loop.run_until_complete(
                uvicorn.Server(
                    uvicorn.Config(
                        self.app,
                        host="127.0.0.1",
                        port=8000,
                        log_level="info"
                    )
                ).serve()
            )
        except Exception as e:
            self.get_logger().error(f"Web server error: {e}")

    def start_webview(self):
        try:
            window = webview.create_window(
                "Drone Control Interface",
                "http://localhost:8000",
                width=1600,
                height=800,
                resizable=True,
                on_top=True
            )
            webview.start()
        except Exception as e:
            self.get_logger().error(f"Webview error: {e}")

    async def shutdown_node_async(self):
        """Asynchronous shutdown procedure"""
        self.get_logger().info("Initiating async shutdown...")
        self.destroy_node()
        rclpy.shutdown()

    def shutdown_node(self):
        """Synchronous shutdown procedure"""
        self.get_logger().info("Initiating shutdown...")
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DroneWebControl()
    try:
        # Create an executor
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()    

if __name__ == '__main__':
    main()