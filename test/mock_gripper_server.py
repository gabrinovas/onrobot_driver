#!/usr/bin/env python3
"""
Enhanced mock OnRobot gripper server for testing
"""
import socket
import threading
import time
import struct

class MockGripperServer:
    def __init__(self, host='127.0.0.1', port=1502):
        self.host = host
        self.port = port
        self.running = False
        self.current_position = 128  # Middle position
        self.is_moving = False
        
    def handle_client(self, client, addr):
        """Handle client connection"""
        print(f"📡 Client connected: {addr}")
        
        while self.running:
            try:
                data = client.recv(1024)
                if not data:
                    break
                    
                print(f"📨 Received {len(data)} bytes")
                
                # Parse Modbus-like command
                if len(data) >= 8:
                    # Simulate command processing
                    command_type = data[0]
                    if command_type == 0x01:  # Grip command
                        # Extract position from bytes 2-3
                        pos_low = data[2]
                        pos_high = data[3]
                        new_position = (pos_high << 8) | pos_low
                        
                        print(f"🤖 Mock gripper moving to position: {new_position}")
                        self.current_position = new_position
                        self.is_moving = True
                        
                        # Simulate movement
                        time.sleep(1.0)
                        self.is_moving = False
                        
                        # Send success response
                        response = self.create_status_response()
                        client.send(response)
                    else:
                        # Send status response for other commands
                        response = self.create_status_response()
                        client.send(response)
                else:
                    # Send default status response
                    response = self.create_status_response()
                    client.send(response)
                    
            except Exception as e:
                print(f"Client handling error: {e}")
                break
                
        client.close()
        print(f"📡 Client disconnected: {addr}")
    
    def create_status_response(self):
        """Create mock status response"""
        # Modbus TCP response frame
        transaction_id = 1
        protocol_id = 0
        length = 6  # unit_id + func_code + byte_count + data
        unit_id = 1
        function_code = 3
        byte_count = 4
        status = 0x01 if not self.is_moving else 0x02  # Ready/Moving
        position = self.current_position
        force = 64  # Simulated force value
        
        response = struct.pack('>HHHBBBBHH', 
                              transaction_id, protocol_id, length,
                              unit_id, function_code, byte_count,
                              status, position, force)
        return response
    
    def start(self):
        self.running = True
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.host, self.port))
        sock.listen(5)
        
        print(f"🤖 Mock Gripper Server listening on {self.host}:{self.port}")
        
        while self.running:
            try:
                client, addr = sock.accept()
                client_thread = threading.Thread(target=self.handle_client, args=(client, addr))
                client_thread.daemon = True
                client_thread.start()
            except Exception as e:
                if self.running:
                    print(f"Server error: {e}")
                break
                
        sock.close()
        print("Mock server stopped")

if __name__ == '__main__':
    server = MockGripperServer('127.0.0.1', 1502)
    try:
        server.start()
    except KeyboardInterrupt:
        server.running = False
        print("\n🛑 Mock server stopped")