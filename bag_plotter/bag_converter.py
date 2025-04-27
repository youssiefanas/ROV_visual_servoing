import os
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import pandas as pd
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

# Topics to extract
TOPICS = [
    '/camera_velocity',
    '/bluerov2/visual_tracker',
    '/tracked_point'
]

def find_rosbags(base_dir):
    """Recursively finds all ROS2 bag files in subdirectories."""
    bag_files = []
    for root, _, files in os.walk(base_dir):
        for file in files:
            if file.endswith(".db3"):
                bag_files.append(os.path.join(root, file))
    return bag_files

def read_rosbag_to_dataframe(bag_path, topics):
    """Reads a ROS2 bag file and converts specified topics into Pandas DataFrames."""
    try:
        rclpy.init()
        reader = SequentialReader()
        storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
        converter_options = ConverterOptions('', '')

        reader.open(storage_options, converter_options)
        topic_metadata = {t.name: t.type for t in reader.get_all_topics_and_types()}
        data = {topic: [] for topic in topics}

        print(f"Processing: {bag_path}")

        while reader.has_next():
            try:
                (topic, msg, t) = reader.read_next()
                if topic in topics and topic in topic_metadata:
                    msg_type = get_message(topic_metadata[topic])
                    deserialized_msg = deserialize_message(msg, msg_type)
                    msg_dict = {field: getattr(deserialized_msg, field) for field in deserialized_msg.__slots__}
                    msg_dict['timestamp'] = t
                    data[topic].append(msg_dict)
            except Exception as e:
                print(f"Error processing message from topic '{topic}': {e}")

        df_dict = {topic: pd.DataFrame(messages) for topic, messages in data.items() if messages}
    except Exception as e:
        print(f"Critical Error processing {bag_path}: {e}")
        df_dict = {}
    finally:
        rclpy.shutdown()

    return df_dict

def process_all_rosbags(base_dir):
    """Processes all found ROS bags, extracts data, and saves CSVs in their respective folders."""
    bag_files = find_rosbags(base_dir)
    
    for bag_file in bag_files:
        df_dict = read_rosbag_to_dataframe(bag_file, TOPICS)
        bag_dir = os.path.dirname(bag_file)  # Folder where the bag is located

        for topic, df in df_dict.items():
            try:
                filename = f"{topic.replace('/', '_')}.csv"
                file_path = os.path.join(bag_dir, filename)
                df.to_csv(file_path, index=False)
                print(f"Saved CSV: {file_path}")
            except Exception as e:
                print(f"Error saving topic '{topic}' to CSV: {e}")

if __name__ == '__main__':
    base_directory = os.getcwd()  # Change this to your base directory if needed
    process_all_rosbags(base_directory)
