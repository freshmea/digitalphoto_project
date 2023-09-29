# Author: Choi sugil
# Date: 2023.09.28
# Brief: photoapp
# description: with opencv create window and show image
# usage: ros2 run digitalphoto photoapp
import rclpy
import cv2
from rclpy.node import Node
import screeninfo
from os import listdir, remove
from os.path import isfile, join
import random
import numpy as np

# todo: convert ros2 parameter and make param file

FPS = 30
SPWAN_TIME = 5
DIMMING_TIME = 1

class Photo(Node):
    def __init__(self):
        super().__init__('photo')
        self.create_timer(1/FPS, self.show_photo)
        self.screen = screeninfo.get_monitors()[0]
        self.dimming = 0
        self.img_origin = None
        self.counter = SPWAN_TIME*FPS
        self.mypath = '/home/jetson/Pictures/test'
        # window width x height full size
        cv2.namedWindow("app", cv2.WINDOW_NORMAL)
        cv2.setWindowProperty("app", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.moveWindow('app', self.screen.x - 1, self.screen.y - 1)
    
    def show_photo(self):
        # get all photo file list
        if self.counter == SPWAN_TIME*FPS:
            onlyfiles = [ join(self.mypath,f) for f in listdir(self.mypath) if isfile(join(self.mypath,f)) ]
            # file list check jpg, png, jpeg
            onlyfiles = [f for f in onlyfiles if f.endswith(('.jpg', '.png', '.jpeg', '.JPG', '.PNG', '.JPEG'))]
            # get random photo file
            file_path = random.choice(onlyfiles)
            self.img_origin = cv2.imread(file_path)
            # resize img to fit window
            self.img_origin = cv2.resize(self.img_origin, (self.screen.width, self.screen.height))
            # reset counter
            self.counter = 0
            self.dimming = 0
            # delete readed file
            self.delete_file(file_path)
        # change dimming value
        if self.counter < DIMMING_TIME*FPS:
            self.dimming += 1
            img = self.change_photo()    
        elif self.counter > (SPWAN_TIME - DIMMING_TIME)*FPS:
            self.dimming -= 1 
            img = self.change_photo()
        else:
            img = self.img_origin.copy()
        # show image
        cv2.imshow('app', img)
        cv2.waitKey(int(1000/FPS))
        # count
        self.counter += 1
    
    def change_photo(self):
        # dimming
        img = self.img_origin.copy()
        r, g, b = cv2.split(img)
        r = cv2.multiply(r, self.dimming, scale=1/FPS)
        g = cv2.multiply(g, self.dimming, scale=1/FPS)
        b = cv2.multiply(b, self.dimming, scale=1/FPS)
        img = cv2.merge((r, g, b))
        return img
    
    def delete_file(self, file):
        remove(file)

def main():
    rclpy.init()
    node = Photo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
