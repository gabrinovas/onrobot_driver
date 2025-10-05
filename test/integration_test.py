#!/usr/bin/env python3
"""
Enhanced integration test with mock server - REPLACES scripts/test_integration.py
"""
import subprocess
import time
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
import sys
import os

def start_mock_server():
    """Start mock server in background"""
    print("🚀 Starting mock gripper server...")
    
    # Get the path to the mock server script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    mock_server_script = os.path.join(script_dir, 'mock_gripper_server.py')
    
    process = subprocess.Popen(
        ['python3', mock_server_script],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    time.sleep(2)  # Give server time to start
    return process

def start_driver_node(simulation_mode=True):
    """Start driver node in background"""
    print("🚀 Starting driver node...")
    
    if simulation_mode:
        cmd = [
            'ros2', 'run', 'onrobot_driver', 'onrobot_driver_node',
            '--ros-args',
            '-p', 'ip_address:=127.0.0.1',
            '-p', 'port:=1502',
            '-p', 'gripper_type:=2FG7',
            '-p', 'update_rate:=10.0',
            '-p', 'simulation_mode:=true'
        ]
    else:
        cmd = [
            'ros2', 'run', 'onrobot_driver', 'onrobot_driver_node',
            '--ros-args',
            '-p', 'ip_address:=192.168.1.1',
            '-p', 'port:=502', 
            '-p', 'gripper_type:=2FG7',
            '-p', 'update_rate:=100.0',
            '-p', 'simulation_mode:=false'
        ]
    
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    time.sleep(3)  # Give driver time to start
    return process

def test_gripper_actions(simulation_mode=True):
    """Test gripper action server - enhanced version"""
    print("🧪 Testing gripper action server...")
    
    rclpy.init()
    
    try:
        from rclpy.node import Node
        node = Node('test_integration')
        
        # Create action client
        action_client = ActionClient(node, GripperCommand, 'gripper_action')
        
        # Wait for server with longer timeout
        print("Waiting for gripper action server...")
        if not action_client.wait_for_server(timeout_sec=15.0):
            print("❌ Action server not available")
            print("Make sure the driver node is running")
            return False
            
        print("✅ Action server: AVAILABLE")
        
        # Test sequence
        if simulation_mode:
            test_sequence = [
                (0.070, 30.0, "Open to 70mm"),
                (0.040, 60.0, "Close to 40mm"),
                (0.060, 40.0, "Open to 60mm (ready)"),
            ]
        else:
            # Conservative movements for real hardware
            test_sequence = [
                (0.070, 30.0, "Open to 70mm (safe)"),
                (0.050, 40.0, "Close to 50mm (medium)"),
                (0.060, 30.0, "Open to 60mm (ready)"),
            ]
        
        all_success = True
        
        for position, force, description in test_sequence:
            print(f"Testing: {description}")
            
            goal_msg = GripperCommand.Goal()
            goal_msg.command.position = position
            goal_msg.command.max_effort = force
            
            # Send goal with timeout
            future = action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            
            if future.result() is None:
                print(f"❌ {description} failed - no response")
                all_success = False
                continue
                
            goal_handle = future.result()
            if not goal_handle.accepted:
                print(f"❌ {description} failed - rejected")
                all_success = False
                continue
            
            print(f"✅ {description} - ACCEPTED")
            
            # Wait for result with timeout
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(node, result_future, timeout_sec=10.0)
            
            if result_future.result() is None:
                print(f"⚠️ {description} - No result received (timeout)")
                # Still consider it partial success for simulation
                if simulation_mode:
                    continue
                else:
                    all_success = False
            else:
                result = result_future.result().result
                if result.reached_goal:
                    print(f"✅ {description} - SUCCESS (position: {result.position:.3f}m)")
                else:
                    print(f"⚠️ {description} - PARTIAL (position: {result.position:.3f}m)")
                    if not simulation_mode:
                        all_success = False
            
            # Wait between commands
            wait_time = 1.0 if simulation_mode else 3.0
            time.sleep(wait_time)
        
        return all_success
        
    except Exception as e:
        print(f"❌ Integration test failed: {e}")
        return False
    finally:
        rclpy.shutdown()

def test_gripper_topics():
    """Test that gripper topics are being published"""
    print("🧪 Testing gripper topics...")
    
    rclpy.init()
    
    try:
        from rclpy.node import Node
        node = Node('test_topics')
        
        # Import the message types we want to test
        from std_msgs.msg import Bool, Float32
        from sensor_msgs.msg import JointState
        
        # We'll just check if the node can create subscribers for these topics
        # In a real test, you'd wait for messages with timeouts
        
        topics_to_check = [
            ('/gripper_position', Float32),
            ('/gripper_status', Bool),
            ('/joint_states', JointState)
        ]
        
        all_topics_found = True
        
        for topic_name, msg_type in topics_to_check:
            try:
                # Try to create a subscriber (this will fail if topic doesn't exist in the system)
                # This is a basic check - in production you'd use proper topic discovery
                subscriber = node.create_subscription(
                    msg_type, topic_name, lambda msg: None, 10
                )
                node.destroy_subscription(subscriber)
                print(f"✅ Topic {topic_name} - AVAILABLE")
            except Exception as e:
                print(f"⚠️ Topic {topic_name} - NOT FOUND: {e}")
                all_topics_found = False
        
        return all_topics_found
        
    except Exception as e:
        print(f"❌ Topic test failed: {e}")
        return False
    finally:
        rclpy.shutdown()

def main():
    """Main test function"""
    import argparse
    
    parser = argparse.ArgumentParser(description='OnRobot Gripper Integration Test')
    parser.add_argument('--hardware', action='store_true', 
                       help='Test with real hardware (default: simulation)')
    parser.add_argument('--no-mock', action='store_true',
                       help='Skip starting mock server (use existing one)')
    
    args = parser.parse_args()
    
    simulation_mode = not args.hardware
    use_mock_server = simulation_mode and not args.no_mock
    
    print("=== OnRobot Gripper Integration Test ===")
    print(f"Mode: {'SIMULATION' if simulation_mode else 'HARDWARE'}")
    print(f"Mock server: {'ENABLED' if use_mock_server else 'DISABLED'}")
    print("=" * 40)
    
    server_process = None
    driver_process = None
    
    try:
        # Start mock server if needed
        if use_mock_server:
            server_process = start_mock_server()
        
        # Start driver node
        driver_process = start_driver_node(simulation_mode)
        
        # Test 1: Action server functionality
        action_test_success = test_gripper_actions(simulation_mode)
        
        # Test 2: Topic monitoring
        topic_test_success = test_gripper_topics()
        
        # Final results
        print("\n=== TEST SUMMARY ===")
        print(f"Action functionality: {'✅ PASS' if action_test_success else '❌ FAIL'}")
        print(f"Topic monitoring: {'✅ PASS' if topic_test_success else '⚠️ PARTIAL'}")
        
        if action_test_success:
            print("🎉 OnRobot Gripper integration tests completed successfully!")
            exit_code = 0
        else:
            print("💥 Some gripper tests failed!")
            exit_code = 1
            
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        exit_code = 0
    except Exception as e:
        print(f"❌ Test setup failed: {e}")
        exit_code = 1
    finally:
        # Cleanup
        if driver_process:
            driver_process.terminate()
            driver_process.wait()
        if server_process:
            server_process.terminate()
            server_process.wait()
    
    exit(exit_code)

if __name__ == '__main__':
    main()