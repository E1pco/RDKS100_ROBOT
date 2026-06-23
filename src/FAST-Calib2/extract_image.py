import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

def extract_image_from_bag(bag_path, image_topic, output_path):
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr',
                                         output_serialization_format='cdr')
    reader.open(storage_options, converter_options)

    bridge = CvBridge()
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic == image_topic:
            msg = deserialize_message(data, Image())
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imwrite(output_path, cv_image)
            print(f"Saved image to {output_path}")
            break

if __name__ == '__main__':
    rclpy.init()
    extract_image_from_bag(
        bag_path='/home/sunrise/fast_ws/src/FAST-Calib2/calib_data/scene_2/scene_2_0.db3',
        image_topic='/left_camera/image',
        output_path='/home/sunrise/fast_ws/src/FAST-Calib2/img/image.png'
    )
    rclpy.shutdown()