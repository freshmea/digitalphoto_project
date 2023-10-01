# Author: Choi sugil
# Date: 2023.09.28
# Brief: googlephoto download
# description: with googlephoto api download photo and save file
# usage: ros2 run digitalphoto googlephotodl
import rclpy
import cv2
from rclpy.node import Node
import pickle
import os.path
from os.path import isfile, join
import datetime
from googleapiclient.discovery import build
from google_auth_oauthlib.flow import InstalledAppFlow
from google.auth.transport.requests import Request
import wget
import random
from rcl_interfaces.msg import SetParametersResult


# If modifying these scopes, delete the file token.pickle.
SCOPES = ['https://www.googleapis.com/auth/photoslibrary.readonly']

class Googlephotodl(Node):
    def __init__(self):
        super().__init__('googlephotodl')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_file_num', 10),
                ('mypath', '/home/jetson/Pictures/test')
            ]
        )
        self.mypath = self.get_parameter('mypath').value
        self.max_file_num = self.get_parameter('max_file_num').value
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.check_token()
        self.create_timer(3, self.googledl_callback)
        # self.delete_image()
        self.search_image()

    def googledl_callback(self):
        # check file number of soni directory
        files = os.listdir(self.mypath)
        file_count = len([f for f in files if isfile(join(self.mypath, f))])
        # if file number is less than 10, download image file
        if file_count < self.max_file_num:
            self.search_image()
            self.dl_image()
    
    def check_token(self):
        creds = None
        if os.path.exists('token.pickle'):
            with open('token.pickle', 'rb') as token:
                creds = pickle.load(token)
        # If there are no (valid) credentials available, let the user log in.
        if not creds or not creds.valid:
            if creds and creds.expired and creds.refresh_token:
                creds.refresh(Request())
            else:
                flow = InstalledAppFlow.from_client_secrets_file(
                    '/home/jetson/colcon_ws/src/digitalphoto/credentials.json', SCOPES)
                creds = flow.run_local_server(port=0)
            # Save the credentials for the next run
            with open('token.pickle', 'wb') as token:
                pickle.dump(creds, token)

        self.service = build('photoslibrary', 'v1', credentials=creds, static_discovery=False)
        
    def search_image(self):
        #Search image files
        
        while True:
            self.get_logger().info("Search image files...")
            dtToday = datetime.date.today()
            dtToday = dtToday - datetime.timedelta(weeks=random.randint(1,100))
            dt7DaysAgo = dtToday - datetime.timedelta(weeks=1)
            results = self.service.mediaItems().search(
                body={
                    "filters":{
                        "dateFilter":{
                            "ranges":[
                                {"startDate":{"year":dt7DaysAgo.year,"month":dt7DaysAgo.month,"day":dt7DaysAgo.day},
                                    "endDate":{"year":dtToday.year,"month":dtToday.month,"day":dtToday.day}
                                    }
                            ]
                        },
                        "mediaTypeFilter":{
                            "mediaTypes":["PHOTO"]
                        }
                    }
                }
            ).execute()
            try:
                if len(results.get("mediaItems", [])) != 0:
                    break
            except:
                pass
        self.items = results.get("mediaItems", [])
            
        print(len(self.items))
    
    def dl_image(self):
        #Download & conver image files
        self.get_logger().info("Dowload image file...")
        # for item in items:
        item = random.choice(self.items)
        filename = item["filename"]
        baseUrl = item["baseUrl"]
        
        # check duplication & download
        file_full_path = os.path.join(self.mypath, filename)
        if not os.path.isfile(file_full_path):
            wget.download(baseUrl, file_full_path)
            self.get_logger().info(f"Dowload complete {file_full_path}")

    def delete_image(self):
        # delete all jpg files
        self.get_logger().info("Deleting all jpg files...")
        command = "rm -rf "+self.mypath+"/*.jpg"
        os.system(command)
    
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'max_file_num':
                self.max_file_num = param.value
            if param.name == 'mypathdl':
                self.mypath = param.value
        return SetParametersResult(successful=True)
        
        

def main(args=None):
    rclpy.init(args=args)
    node = Googlephotodl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
