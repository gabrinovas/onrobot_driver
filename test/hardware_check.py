#!/usr/bin/env python3
"""
Basic hardware connectivity check
"""
import subprocess
import socket

def basic_checks(host):
    print("🔧 Basic Hardware Connectivity Check")
    print("=" * 50)
    
    # 1. Ping test
    print("1. Ping test...")
    try:
        result = subprocess.run(['ping', '-c', '3', host], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print("   ✅ Ping successful")
        else:
            print("   ❌ Ping failed")
            print(f"   Output: {result.stdout}")
    except Exception as e:
        print(f"   ⚠️ Ping error: {e}")
    
    # 2. ARP table check
    print("\n2. ARP table check...")
    try:
        result = subprocess.run(['arp', '-a'], capture_output=True, text=True)
        if host in result.stdout:
            print("   ✅ Host found in ARP table")
            for line in result.stdout.split('\n'):
                if host in line:
                    print(f"   ARP entry: {line.strip()}")
        else:
            print("   ⚠️ Host not in ARP table")
    except Exception as e:
        print(f"   ⚠️ ARP check error: {e}")
    
    # 3. Network interface check
    print("\n3. Network configuration...")
    try:
        result = subprocess.run(['ip', 'addr', 'show'], capture_output=True, text=True)
        print("   Network interfaces:")
        for line in result.stdout.split('\n'):
            if 'inet ' in line and '192.168.1.' in line:
                print(f"   ✅ {line.strip()}")
    except Exception as e:
        print(f"   ⚠️ Network check error: {e}")

if __name__ == '__main__':
    basic_checks('192.168.1.1')