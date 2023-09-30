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
import time
from rcl_interfaces.msg import SetParametersResult

class Photo(Node):
    def __init__(self):
        super().__init__('photo')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('fps', 30),
                ('spawn_time', 5),
                ('dimming_time', 1),
                ('show_option', 0),
                ('mypath', '/home/jetson/Pictures/sonii')
            ]
        )
        self.fps = self.get_parameter('fps').value
        self.spawn_time = self.get_parameter('spawn_time').value
        self.dimming_time = self.get_parameter('dimming_time').value
        self.show_option = self.get_parameter('show_option').value
        self.mypath = self.get_parameter('mypath').value
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        if self.show_option == 0:
            self.create_timer(self.spawn_time, self.show_photo)
        else:
            self.create_timer(1/self.fps, self.show_photo)
        self.screen = screeninfo.get_monitors()[0]
        self.dimming = 0
        self.img_origin = None
        self.counter = self.spawn_time*self.fps
        # window width x height full size
        cv2.namedWindow("app", cv2.WINDOW_NORMAL)
        cv2.setWindowProperty("app", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.moveWindow('app', self.screen.x - 1, self.screen.y - 1)
        # get all photo file list
        self.onlyfiles = [ join(self.mypath,f) for f in listdir(self.mypath) if isfile(join(self.mypath,f)) ]
        self.onlyfiles = [f for f in self.onlyfiles if f.endswith(('.jpg', '.png', '.jpeg', '.JPG', '.PNG', '.JPEG'))]
        # time.delay(10)
        self.cnt = 0
    
    def show_photo(self):
        if self.show_option == 0:
            # get random photo file
            file_path = self.onlyfiles[self.cnt]
            self.cnt += 1
            if len(self.onlyfiles) < self.cnt:
                self.cnt = 0
            file_path = random.choice(self.onlyfiles)
            self.img_origin = cv2.imread(file_path)
            # get image size
            height, width, _ = self.img_origin.shape
            # resize img to fit window with ratio 16:9 screen
            if width * 9 > height * 16 :
                self.img_origin = cv2.resize(self.img_origin, (self.screen.width, int(self.screen.width*height/width)))
                # add white space to fit 16:9 screen both side
                white_space = np.zeros(( (self.screen.height - int(self.screen.width*height/width))//2, self.screen.width, 3), np.uint8)
                white_space[:] = (random.randint(0,255), random.randint(0,255), random.randint(0,255))
                self.img_origin = np.vstack((self.img_origin, white_space))
                self.img_origin = np.vstack((white_space, self.img_origin))
            else:
                self.img_origin = cv2.resize(self.img_origin, (int(self.screen.height*width/height), self.screen.height))
                # add white space to fit 16:9 screen both side
                white_space = np.zeros((self.screen.height, (self.screen.width - int(self.screen.height*width/height))//2, 3), np.uint8)
                white_space[:] = (random.randint(0,255), random.randint(0,255), random.randint(0,255))
                self.img_origin = np.hstack((self.img_origin, white_space))
                self.img_origin = np.hstack((white_space, self.img_origin))
            # show image
            cv2.imshow('app', self.img_origin)
            cv2.waitKey(int(1000*self.spawn_time))
            self.get_logger().info(f'{self.cnt} {len(self.onlyfiles)} {file_path}')
        elif self.show_option == 1:
            pass
        else:
            # get all photo file list
            if self.counter == self.spawn_time * self.fps:
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
            if self.counter < self.dimming_time*self.fps:
                self.dimming += 1
                img = self.change_photo()    
            elif self.counter > (self.spawn_time - self.dimming_time)*self.fps:
                self.dimming -= 1 
                img = self.change_photo()
            else:
                img = self.img_origin.copy()
            img = self.img_origin.copy()
            # show image
            cv2.imshow('app', img)
            cv2.waitKey(int(1000/self.fps))
            # count
            self.counter += 1
    
    def change_photo(self):
        # dimming
        img = self.img_origin.copy()
        r, g, b = cv2.split(img)
        r = cv2.multiply(r, self.dimming, scale=1/self.fps)
        g = cv2.multiply(g, self.dimming, scale=1/self.fps)
        b = cv2.multiply(b, self.dimming, scale=1/self.fps)
        img = cv2.merge((r, g, b))
        return img
    
    def delete_file(self, file):
        remove(file)
    
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'fps':
                self.fps = param.value
            elif param.name == 'spawn_time':
                self.spawn_time = param.value
            elif param.name == 'dimming_time':
                self.dimming_time = param.value
            elif param.name == 'show_option':
                self.show_option = param.value
            elif param.name == 'mypath':
                self.mypath = param.value
        return SetParametersResult(successful=True)

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
