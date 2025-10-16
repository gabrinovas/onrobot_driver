#!/usr/bin/env python3
"""
Check for OnRobot API endpoints
"""
import requests
import json

def check_api_endpoints(host):
    """Check common API endpoints"""
    print("🔌 Checking for API endpoints...")
    
    endpoints = [
        "/api/v1/status",
        "/api/status",
        "/status",
        "/gripper/status",
        "/api/gripper",
        "/onrobot/status",
        "/webapi/status",
        "/rest/api/status",
    ]
    
    for endpoint in endpoints:
        for port in [80, 443, 8080, 5000, 3000]:
            for protocol in ['http', 'https']:
                url = f"{protocol}://{host}:{port}{endpoint}"
                try:
                    response = requests.get(url, timeout=2, verify=False)
                    if response.status_code == 200:
                        print(f"✅ {url}: Status {response.status_code}")
                        try:
                            data = response.json()
                            print(f"   JSON Response: {json.dumps(data, indent=2)}")
                        except:
                            print(f"   Text Response: {response.text[:200]}...")
                except:
                    pass

if __name__ == '__main__':
    check_api_endpoints('192.168.1.1')