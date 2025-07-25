#!/usr/bin/env python3
import sys
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

class DroneWebControl(Node):
    def __init__(self):
        Node.__init__(self, 'drone_web_control')
        
        self.web_dir = os.path.join(get_package_share_directory('mission'), 'control_interface')
        self.html_path = os.path.join(self.web_dir, 'index.html')
    
        # ROS 2 Publishers
        self.vision_pub = self.create_publisher(String, '/go_vision', 10)
        self.winch_pub = self.create_publisher(String, '/go_winch', 10)
        self.motor_init_pub = self.create_publisher(String, '/init_motor', 10)
        self.motor_stop_pub = self.create_publisher(String, '/close_motor', 10)
        self.water_source_pub = self.create_publisher(String, '/go_bucket_valve', 10)
        self.water_bucket_pub = self.create_publisher(String, '/go_bucket_valve', 10)
        self.valve_pub = self.create_publisher(String, '/valve_state', 10)
        self.manual_approach = self.create_publisher(String, '/manual', 10)
        self.finished_manual_approach_pub = self.create_publisher(String, '/task_end', 10)
        self.battery_changed_pub = self.create_publisher(String, '/battery_changed', 10)
        self.abort_state_pub = self.create_publisher(String, '/abort_state', 10)
        self.confirm_arming_pub = self.create_publisher(String, '/confirm_arming', 10)
        self.bucket_number_pub = self.create_publisher(Int32, '/bucket_number', 10)

        # ROS 2 Subscribers
        self.water_qty = 0
        self.torque = 0.0
        self.create_subscription(Int32, '/water_qty', self.water_qty_callback, 10)
        self.create_subscription(Float32, '/torque', self.torque_callback, 10)
        
        # Web interface components
        self.active_connections: Set[WebSocket] = set()
        self.connections_lock = threading.Lock()
        self.app = FastAPI()
        self.app.mount(
            "/static",
            StaticFiles(directory=self.web_dir),
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
        self.water_qty = msg.data
        self.schedule_async_task(self.broadcast_sensor_data())
        self.schedule_async_task(self.send_terminal_message(f"Water quantity updated: {msg.data} mL"))

    def torque_callback(self, msg):
        self.torque = msg.data
        self.schedule_async_task(self.broadcast_sensor_data())
        self.schedule_async_task(self.send_terminal_message(f"Torque updated: {msg.data:.2f} Nm"))

    def schedule_async_task(self, coro):
        asyncio.run_coroutine_threadsafe(coro, self.loop)

    async def broadcast_sensor_data(self):
        data = {
            'water_qty': self.water_qty,
            'torque': self.torque
        }
        message = json.dumps({
            'type': 'sensor_data',
            'data': data
        })
        
        with self.connections_lock:
            dead_connections = []
            for connection in self.active_connections:
                try:
                    await connection.send_text(message)
                except Exception as e:
                    self.get_logger().error(f"Sensor data send failed: {e}")
                    dead_connections.append(connection)
            
            for connection in dead_connections:
                self.active_connections.remove(connection)

    async def send_terminal_message(self, message):
        timestamp = datetime.now().strftime("%H:%M:%S")
        formatted_message = f"[{timestamp}] {message}"
        message_json = json.dumps({
            'type': 'terminal',
            'message': formatted_message
        })
        
        with self.connections_lock:
            dead_connections = []
            for connection in self.active_connections:
                try:
                    await connection.send_text(message_json)
                except Exception as e:
                    self.get_logger().error(f"Terminal message send failed: {e}")
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
            except Exception as e:
                self.get_logger().error(f"WebSocket error: {e}")
                with self.connections_lock:
                    self.active_connections.remove(websocket)
        
        @self.app.get("/")
        async def serve_interface():
            with open(self.html_path, 'r', encoding='utf-8') as file:
                html_content = file.read()
            return HTMLResponse(content=html_content, status_code=200)

    async def process_web_command(self, data):
        try:
            message = json.loads(data)
            if message['type'] == 'command':
                await self.process_command(message['command'], message.get('data'))
        except json.JSONDecodeError:
            await self.send_terminal_message("Invalid JSON received from client")
        except Exception as e:
            await self.send_terminal_message(f"Command processing error: {str(e)}")

    async def process_command(self, command, data=None):
        await self.send_terminal_message(f"Executing command: {command}" + (f" with data: {data}" if data else ""))
        
        msg = String()
        try:
            if command == 'vision_source':
                msg.data = "SOURCE"
                self.vision_pub.publish(msg)
                await self.send_terminal_message("Vision source command executed")
            elif command == 'vision_bucket':
                msg.data = "BUCKET"
                self.vision_pub.publish(msg)
                await self.send_terminal_message("Vision bucket command executed")
            elif command == 'manual_approach':
                msg.data = "MANUAL"
                self.manual_approach.publish(msg)
                await self.send_terminal_message("Manual approach activated")
            elif command == 'auto_approach':
                msg.data = "AUTO"
                self.manual_approach.publish(msg)
                await self.send_terminal_message("Auto approach activated")
            elif command == 'winch_down':
                msg.data = "DOWN"
                self.winch_pub.publish(msg)
                await self.send_terminal_message("Winch down command executed")
            elif command == 'winch_up':
                msg.data = "UP"
                self.winch_pub.publish(msg)
                await self.send_terminal_message("Winch up command executed")
            elif command == 'init_motor':
                msg.data = "INIT"
                self.motor_init_pub.publish(msg)
                await self.send_terminal_message("Motor initialization command executed")
            elif command == 'stop_motor':
                msg.data = "CLOSE"
                self.motor_stop_pub.publish(msg)
                await self.send_terminal_message("Motor stop command executed")
            elif command == 'water_refill':
                msg.data = "REFILL"
                self.water_source_pub.publish(msg)
                await self.send_terminal_message("Water refill command executed")
            elif command == 'water_release':
                msg.data = "RELEASE"
                self.water_bucket_pub.publish(msg)
                await self.send_terminal_message("Water release command executed")
            elif command == 'valve_open':
                msg.data = "OPEN"
                self.valve_pub.publish(msg)
                await self.send_terminal_message("Valve open command executed")
            elif command == 'valve_close':
                msg.data = "CLOSE"
                self.valve_pub.publish(msg)
                await self.send_terminal_message("Valve close command executed")
            elif command == 'set_buckets' and data:
                msg = Int32()
                msg.data = int(data)
                self.bucket_number_pub.publish(msg)
                await self.send_terminal_message(f"Bucket count set to: {data}")
            elif command == 'battery_changed':
                msg.data = "CHANGED"
                self.battery_changed_pub.publish(msg)
                await self.send_terminal_message("Battery change acknowledged")
            elif command == 'abort':
                msg.data = "ABORT"
                self.abort_state_pub.publish(msg)
                await self.send_terminal_message("!!! ABORT STATE ACTIVATED !!!")
            elif command == 'confirm_arming':
                msg.data = "ARM"
                self.confirm_arming_pub.publish(msg)
                await self.send_terminal_message("Arming confirmed - GUIDED mode")
            else:
                await self.send_terminal_message(f"Unknown command: {command}")
        except Exception as e:
            await self.send_terminal_message(f"Error executing command: {str(e)}")

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
            # Create an event to track window closure
            self.window_closed = threading.Event()
            
            def on_closed():
                self.get_logger().info("Webview window closed - shutting down node")
                self.window_closed.set()
                # Schedule node shutdown on the ROS 2 thread
                if rclpy.ok():
                    executor = rclpy.executors.MultiThreadedExecutor()
                    executor.add_node(self)
                    executor.create_task(self.shutdown_node_async())
            
            window = webview.create_window(
                "Drone Control Interface",
                "http://localhost:8000",
                width=1000,
                height=800,
                resizable=True,
                on_top=True
            )
            
            # Set the close callback
            window.events.closed += on_closed
            
            webview.start()
        except Exception as e:
            self.get_logger().error(f"Webview error: {e}")
            self.shutdown_node()

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
        
        # Spin until the window is closed or interrupt occurs
        while rclpy.ok() and not hasattr(node, 'window_closed') or not node.window_closed.is_set():
            executor.spin_once(timeout_sec=0.1)
            
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt - shutting down drone control...")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        node.get_logger().info("Drone control node shutdown complete")