#!/usr/bin/env python3
"""
Simple mock OnRobot gripper server for testing
"""
import socket
import threading
import time

class MockGripperServer:
    def __init__(self, host='127.0.0.1', port=1502):  # CHANGED: Use non-privileged port
        self.host = host
        self.port = port
        self.running = False
        
    def start(self):
        self.running = True
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.host, self.port))
        sock.listen(1)
        
        print(f"🤖 Mock Gripper Server listening on {self.host}:{self.port}")
        
        while self.running:
            try:
                client, addr = sock.accept()
                print(f"📡 Client connected: {addr}")
                # Simple echo server for testing
                data = client.recv(1024)
                if data:
                    print(f"📨 Received {len(data)} bytes")
                    # Send dummy response
                    client.send(b"\x00\x01\x00\x00\x00\x06\x01\x03\x02\x00\x00")
                client.close()
            except Exception as e:
                print(f"Server error: {e}")
                break
                
        sock.close()
        print("Mock server stopped")

if __name__ == '__main__':
    server = MockGripperServer('127.0.0.1', 1502)  # Use port 1502 instead of 502
    try:
        server.start()
    except KeyboardInterrupt:
        server.running = False
        print("\n🛑 Mock server stopped")