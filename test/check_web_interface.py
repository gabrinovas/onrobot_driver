#!/usr/bin/env python3
"""
Check if Compute Box has a web interface
"""
import requests
import socket

def check_web_interface(host):
    """Check for web interface on common ports"""
    print("🌐 Checking for web interface...")
    
    web_ports = [80, 443, 8080, 8888, 8443]
    
    for port in web_ports:
        try:
            # Try HTTP
            if port in [80, 8080, 8888]:
                url = f"http://{host}:{port}"
                try:
                    response = requests.get(url, timeout=3)
                    print(f"✅ HTTP {url}: Status {response.status_code}")
                    if response.status_code == 200:
                        print(f"   Content length: {len(response.content)}")
                        # Check if it looks like OnRobot interface
                        if 'onrobot' in response.text.lower():
                            print("   🎯 Likely OnRobot web interface!")
                except requests.exceptions.RequestException as e:
                    print(f"❌ HTTP {url}: {e}")
            
            # Try HTTPS
            if port in [443, 8443]:
                url = f"https://{host}:{port}"
                try:
                    response = requests.get(url, timeout=3, verify=False)
                    print(f"✅ HTTPS {url}: Status {response.status_code}")
                    if response.status_code == 200:
                        print(f"   Content length: {len(response.content)}")
                        if 'onrobot' in response.text.lower():
                            print("   🎯 Likely OnRobot web interface!")
                except requests.exceptions.RequestException as e:
                    print(f"❌ HTTPS {url}: {e}")
                    
        except Exception as e:
            print(f"⚠️ Error checking port {port}: {e}")

def check_telnet_ssh(host):
    """Check for text-based interfaces"""
    print("\n🖥️ Checking for text-based interfaces...")
    
    text_ports = [22, 23, 2222]
    
    for port in text_ports:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(3.0)
            sock.connect((host, port))
            
            # Try to receive banner
            try:
                banner = sock.recv(1024)
                print(f"✅ Port {port}: Got banner - {banner.decode('utf-8', errors='ignore')}")
            except:
                print(f"✅ Port {port}: Connected but no banner")
                
            sock.close()
        except:
            print(f"❌ Port {port}: Cannot connect")

if __name__ == '__main__':
    host = '192.168.1.1'
    check_web_interface(host)
    check_telnet_ssh(host)