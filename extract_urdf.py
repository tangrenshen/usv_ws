#!/usr/bin/env python3
import sys
from mcap_ros2.reader import read_ros2_messages

def extract_robot_description(mcap_path):
    print("=== Topics in bag ===")
    topics_found = set()
    
    for msg in read_ros2_messages(mcap_path):
        topics_found.add(msg.channel.topic)
    
    for t in sorted(topics_found):
        print(f"  {t}")
    
    print("\n=== Searching for robot_description ===")
    found = False
    for msg in read_ros2_messages(mcap_path):
        if 'robot_description' in msg.channel.topic:
            found = True
            print(f"Found: {msg.channel.topic}")
            try:
                data = msg.ros_msg.data.decode('utf-8') if isinstance(msg.ros_msg.data, bytes) else str(msg.ros_msg.data)
                print(f"\n--- URDF content (first 5000 chars) ---")
                print(data[:5000])
                if len(data) > 5000:
                    print(f"... (truncated, total {len(data)} chars)")
            except Exception as e:
                print(f"Error parsing: {e}")
                print(f"Raw type: {type(msg.ros_msg.data)}")
            break
    
    if not found:
        print("robot_description NOT found in bag")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 extract_urdf.py <mcap_file>")
        sys.exit(1)
    extract_robot_description(sys.argv[1])