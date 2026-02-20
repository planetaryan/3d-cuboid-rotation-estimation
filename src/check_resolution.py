from pathlib import Path
from rosbags.highlevel import AnyReader

# ---------------- CONFIGURATION ----------------
BAG_FOLDER = Path('depth') 
TOPIC_NAME = '/depth'

def get_bag_resolutions():
    print(f"Opening bag in: {BAG_FOLDER}")
    
    with AnyReader([BAG_FOLDER]) as reader:
        # Filter for the specific depth topic
        connections = [x for x in reader.connections if x.topic == TOPIC_NAME]
        
        if not connections:
            print(f"Error: Topic {TOPIC_NAME} not found.")
            return

        print(f"{'Frame':<8} | {'Timestamp':<20} | {'Resolution (WxH)'}")
        print("-" * 50)

        for idx, (connection, timestamp, rawdata) in enumerate(reader.messages(connections=connections)):
            # Deserialize message to access metadata
            msg = reader.deserialize(rawdata, connection.msgtype)
            
            # Print resolution details per frame
            print(f"{idx:<8} | {timestamp:<20} | {msg.width} x {msg.height}")

if __name__ == "__main__":
    get_bag_resolutions()