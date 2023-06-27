#!/usr/bin/env python


import numpy as np
import rospy
from PIL import Image
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool


# ros node to save map topic to pgm file and yaml file when Bool true message come to /save_map topic
# and publish map from saved file to /map topic


class MapSaver:
    def __init__(self):
        rospy.init_node('map_saver', anonymous=True)
        self.map_sub = rospy.Subscriber('/rtabmap/grid_map', OccupancyGrid, self.map_callback)
        self.save_map_sub = rospy.Subscriber('/save_map', Bool, self.save_map_callback)
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
        self.map = None
        self.map_info = None
        self.map_saved = False
        rospy.loginfo("Map saver node started")

    def map_callback(self, msg):
        self.map = msg.data
        self.map_info = msg.info
        print(self.map)
        self.map_saved = False

    def save_map_callback(self, msg):
        if msg.data:
            self.save_map()

    def publish_map_from_saved_file(self):
        pass


    def save_map(self):
        if self.map_saved:
            rospy.loginfo("Map already saved")
            return
        if self.map is None:
            rospy.loginfo("Map not received")
            return
        rospy.loginfo("Saving map")
        map_array = np.array(self.map).reshape((self.map_info.height, self.map_info.width))
        map_array = np.flip(map_array, 0)
        # map_array = np.flip(map_array, 1)
        map_array = np.where(map_array == -1, 0, map_array)
        map_array = np.where(map_array == 100, 255, map_array)
        map_array = map_array.astype(np.uint8)
        map_image = Image.fromarray(map_array)
        map_image.save("map.pgm")
        with open("map.yaml", "w") as yaml_file:
            yaml_file.write("image: map.pgm\n")
            yaml_file.write("resolution: {}\n".format(self.map_info.resolution))
            yaml_file.write("origin: [{}, {}, {}]\n".format(self.map_info.origin.position.x, self.map_info.origin.position.y, self.map_info.origin.position.z))
            yaml_file.write("negate: 0\n")
            yaml_file.write("occupied_thresh: 0.65\n")
            yaml_file.write("free_thresh: 0.196\n")
        self.map_saved = True
        rospy.loginfo("Map saved")

if __name__ == '__main__':
    map_saver = MapSaver()
    while not rospy.is_shutdown():
        map_saver.save_map()
        rospy.sleep(1)

