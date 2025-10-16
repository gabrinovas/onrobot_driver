#!/usr/bin/env python3
"""
Scan for open ports and services on the Compute Box
"""
import socket
from concurrent.futures import ThreadPoolExecutor

def scan_port(host, port):
    """Scan a single port"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1.0)
        result = sock.connect_ex((host, port))
        sock.close()
        return port, result == 0
    except:
        return port, False

def scan_ports(host, start_port=1, end_port=10000):
    """Scan a range of ports"""
    print(f"🔍 Scanning {host} from port {start_port} to {end_port}...")
    open_ports = []
    
    with ThreadPoolExecutor(max_workers=100) as executor:
        futures = [executor.submit(scan_port, host, port) for port in range(start_port, end_port + 1)]
        
        for future in futures:
            port, is_open = future.result()
            if is_open:
                open_ports.append(port)
                print(f"✅ Port {port} is OPEN")
    
    return open_ports

def test_common_ports(host):
    """Test common industrial and robot ports"""
    common_ports = {
        502: "Modbus TCP",
        80: "HTTP",
        443: "HTTPS", 
        22: "SSH",
        23: "Telnet",
        21: "FTP",
        25: "SMTP",
        110: "POP3",
        143: "IMAP",
        993: "IMAPS",
        995: "POP3S",
        587: "SMTP Submission",
        8888: "Alternative HTTP",
        8080: "HTTP Alternative",
        8443: "HTTPS Alternative",
        2000: "Industrial",
        3000: "Development",
        4000: "Industrial",
        5000: "Development",
        8883: "MQTT",
        1883: "MQTT",
        4840: "OPC UA",
        102: "S7 Communication",
        44818: "EtherNet/IP",
        2222: "Alternative SSH",
        9999: "Industrial"
    }
    
    print(f"🔍 Testing common ports on {host}...")
    open_ports = []
    
    for port, description in common_ports.items():
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2.0)
            result = sock.connect_ex((host, port))
            sock.close()
            
            if result == 0:
                open_ports.append((port, description))
                print(f"✅ Port {port} ({description}): OPEN")
            else:
                print(f"❌ Port {port} ({description}): CLOSED")
        except Exception as e:
            print(f"⚠️ Port {port} ({description}): ERROR - {e}")
    
    return open_ports

if __name__ == '__main__':
    host = '192.168.1.1'
    
    print("=== OnRobot Compute Box Network Discovery ===")
    
    # First test common ports
    open_common = test_common_ports(host)
    
    if open_common:
        print(f"\n🎯 Found {len(open_common)} open ports:")
        for port, desc in open_common:
            print(f"   - {port}: {desc}")
        
        # If we found ports, scan a wider range around them
        min_port = min([p for p, _ in open_common])
        max_port = max([p for p, _ in open_common])
        
        print(f"\n🔍 Scanning around open ports ({min_port-100} to {max_port+100})...")
        wider_scan = scan_ports(host, min_port-100, max_port+100)
        
    else:
        # If no common ports found, do a broader scan
        print("\n🔍 No common ports found, scanning first 1000 ports...")
        scan_ports(host, 1, 1000)