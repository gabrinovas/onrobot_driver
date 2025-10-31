#!/usr/bin/env python3

import socket
import struct
import threading
import time

class MockOnRobotGripperServer:
    """
    Mock server that simulates the OnRobot Compute Box for testing.
    Implements the actual Modbus register mapping for 2FG7/2FG14 grippers.
    """
    
    def __init__(self, host='127.0.0.1', port=1502, unit_id=65):
        self.host = host
        self.port = port
        self.unit_id = unit_id
        self.socket = None
        self.running = False
        
        # Simulated gripper state
        self.registers = {
            0x0000: 0,    # Target width (1/10 mm)
            0x0001: 50,   # Target force (N)
            0x0002: 50,   # Target speed (%)
            0x0003: 0,    # Command
            0x0100: 0,    # Status
            0x0101: 500,  # External width (1/10 mm) - 50mm default
            0x0102: 300,  # Internal width (1/10 mm)
            0x0107: 0,    # Current force (N)
        }
        
        # Gripper physical limits (for 2FG7)
        self.min_width = 350  # 35mm in 1/10 mm
        self.max_width = 700  # 70mm in 1/10 mm
        self.max_force = 100  # 100N
        
        self.current_width = 500  # 50mm current position
        self.target_width = 500
        self.is_moving = False
        self.movement_thread = None
        
    def start(self):
        """Start the mock server"""
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.host, self.port))
        self.socket.listen(1)
        self.running = True
        
        print(f"🤖 Mock OnRobot Gripper Server started on {self.host}:{self.port}")
        print(f"📝 Unit ID: {self.unit_id}")
        print("📊 Available registers:")
        for addr in sorted(self.registers.keys()):
            print(f"  0x{addr:04X}: {self.registers[addr]}")
        
        # Start movement simulation thread
        self.movement_thread = threading.Thread(target=self._simulate_movement)
        self.movement_thread.daemon = True
        self.movement_thread.start()
        
        try:
            while self.running:
                client_socket, address = self.socket.accept()
                print(f"🔌 Client connected: {address}")
                
                # Handle client in a separate thread
                client_thread = threading.Thread(
                    target=self.handle_client, 
                    args=(client_socket,)
                )
                client_thread.daemon = True
                client_thread.start()
                
        except KeyboardInterrupt:
            print("\n🛑 Server stopped by user")
        finally:
            self.stop()
    
    def stop(self):
        """Stop the mock server"""
        self.running = False
        if self.socket:
            self.socket.close()
        print("✅ Mock server stopped")
    
    def _simulate_movement(self):
        """Simulate gripper movement"""
        while self.running:
            if self.is_moving:
                # Move towards target width
                if self.current_width < self.target_width:
                    self.current_width = min(self.current_width + 10, self.target_width)
                elif self.current_width > self.target_width:
                    self.current_width = max(self.current_width - 10, self.target_width)
                
                # Update external width register
                self.registers[0x0101] = self.current_width
                
                # Check if movement completed
                if self.current_width == self.target_width:
                    self.is_moving = False
                    # Clear busy bit in status register
                    self.registers[0x0100] &= ~0x0001
                    print(f"🎯 Movement completed: {self.current_width/10.0:.1f}mm")
            
            # Update status register
            if self.is_moving:
                self.registers[0x0100] |= 0x0001  # Set busy bit
            else:
                self.registers[0x0100] &= ~0x0001  # Clear busy bit
            
            # Simulate some force when gripping
            if self.is_moving and abs(self.current_width - self.target_width) < 20:
                self.registers[0x0107] = self.registers[0x0001]  # Use target force
            else:
                self.registers[0x0107] = 0
            
            time.sleep(0.1)  # 100ms update rate
    
    def handle_client(self, client_socket):
        """Handle Modbus TCP requests from client"""
        try:
            while self.running:
                data = client_socket.recv(256)
                if not data:
                    break
                
                response = self.process_modbus_request(data)
                if response:
                    client_socket.send(response)
                    
        except Exception as e:
            print(f"❌ Client handling error: {e}")
        finally:
            client_socket.close()
            print("🔌 Client disconnected")
    
    def process_modbus_request(self, data):
        """Process Modbus TCP request and return response"""
        if len(data) < 8:
            return None
        
        try:
            # Parse MBAP header
            transaction_id = struct.unpack('>H', data[0:2])[0]
            protocol_id = struct.unpack('>H', data[2:4])[0]
            length = struct.unpack('>H', data[4:6])[0]
            unit_id = data[6]
            
            # Validate header
            if protocol_id != 0 or unit_id != self.unit_id:
                return self.create_error_response(transaction_id, unit_id, 0x01)  # Illegal function
            
            # Parse PDU
            function_code = data[7]
            pdu_data = data[8:8+length-1]  # -1 for unit_id
            
            # Process based on function code
            if function_code == 0x03:  # Read Holding Registers
                return self.handle_read_holding_registers(transaction_id, unit_id, pdu_data)
            elif function_code == 0x06:  # Write Single Register
                return self.handle_write_single_register(transaction_id, unit_id, pdu_data)
            elif function_code == 0x10:  # Write Multiple Registers
                return self.handle_write_multiple_registers(transaction_id, unit_id, pdu_data)
            else:
                return self.create_error_response(transaction_id, unit_id, 0x01)  # Illegal function
                
        except Exception as e:
            print(f"❌ Error processing request: {e}")
            return None
    
    def handle_read_holding_registers(self, transaction_id, unit_id, data):
        """Handle Read Holding Registers (0x03)"""
        if len(data) != 4:
            return self.create_error_response(transaction_id, unit_id, 0x03)  # Illegal data value
        
        start_addr = struct.unpack('>H', data[0:2])[0]
        register_count = struct.unpack('>H', data[2:4])[0]
        
        # Validate address range
        for addr in range(start_addr, start_addr + register_count):
            if addr not in self.registers:
                return self.create_error_response(transaction_id, unit_id, 0x02)  # Illegal data address
        
        # Prepare response data
        byte_count = register_count * 2
        response_data = bytes([byte_count])
        
        for addr in range(start_addr, start_addr + register_count):
            value = self.registers.get(addr, 0)
            response_data += struct.pack('>H', value)
        
        return self.create_success_response(transaction_id, unit_id, 0x03, response_data)
    
    def handle_write_single_register(self, transaction_id, unit_id, data):
        """Handle Write Single Register (0x06)"""
        if len(data) != 4:
            return self.create_error_response(transaction_id, unit_id, 0x03)  # Illegal data value
        
        address = struct.unpack('>H', data[0:2])[0]
        value = struct.unpack('>H', data[2:4])[0]
        
        # Validate address
        if address not in self.registers:
            return self.create_error_response(transaction_id, unit_id, 0x02)  # Illegal data address
        
        # Update register value
        self.registers[address] = value
        
        # Handle special registers
        if address == 0x0000:  # Target width
            self.target_width = max(self.min_width, min(self.max_width, value))
            self.is_moving = True
            print(f"🎯 New target width: {self.target_width/10.0:.1f}mm")
        
        elif address == 0x0001:  # Target force
            self.registers[address] = min(self.max_force, value)
            print(f"💪 New target force: {self.registers[address]}N")
        
        elif address == 0x0003:  # Command
            if value == 1:  # Grip external
                self.is_moving = True
                print("🤖 Command: Grip external")
            elif value == 2:  # Grip internal
                self.is_moving = True
                print("🤖 Command: Grip internal")
            elif value == 3:  # Stop
                self.is_moving = False
                print("🛑 Command: Stop")
        
        # Echo the write request as response
        response_data = struct.pack('>HH', address, value)
        return self.create_success_response(transaction_id, unit_id, 0x06, response_data)
    
    def handle_write_multiple_registers(self, transaction_id, unit_id, data):
        """Handle Write Multiple Registers (0x10)"""
        if len(data) < 5:
            return self.create_error_response(transaction_id, unit_id, 0x03)  # Illegal data value
        
        start_addr = struct.unpack('>H', data[0:2])[0]
        register_count = struct.unpack('>H', data[2:4])[0]
        byte_count = data[4]
        
        # Validate data length
        if byte_count != register_count * 2 or len(data) != 5 + byte_count:
            return self.create_error_response(transaction_id, unit_id, 0x03)  # Illegal data value
        
        # Validate address range
        for addr in range(start_addr, start_addr + register_count):
            if addr not in self.registers:
                return self.create_error_response(transaction_id, unit_id, 0x02)  # Illegal data address
        
        # Update registers
        for i in range(register_count):
            addr = start_addr + i
            value = struct.unpack('>H', data[5 + i*2:5 + i*2 + 2])[0]
            self.registers[addr] = value
            
            # Handle special registers
            if addr == 0x0000:  # Target width
                self.target_width = max(self.min_width, min(self.max_width, value))
                self.is_moving = True
                print(f"🎯 New target width: {self.target_width/10.0:.1f}mm")
        
        # Return response with address and count
        response_data = struct.pack('>HH', start_addr, register_count)
        return self.create_success_response(transaction_id, unit_id, 0x10, response_data)
    
    def create_success_response(self, transaction_id, unit_id, function_code, data):
        """Create successful Modbus response"""
        length = len(data) + 1  # +1 for function code
        header = struct.pack('>HHHB', transaction_id, 0x0000, length, unit_id)
        return header + bytes([function_code]) + data
    
    def create_error_response(self, transaction_id, unit_id, error_code):
        """Create error Modbus response"""
        header = struct.pack('>HHHB', transaction_id, 0x0000, 0x0003, unit_id)
        return header + bytes([0x80 + error_code])  # Function code with error bit

def main():
    server = MockOnRobotGripperServer(host='127.0.0.1', port=1502, unit_id=65)
    try:
        server.start()
    except KeyboardInterrupt:
        server.stop()

if __name__ == '__main__':
    main()