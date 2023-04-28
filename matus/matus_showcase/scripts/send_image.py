#!/usr/bin/env python

from __future__ import print_function

import socket
import rospy
import os
from talk_back import *
from clothing_publisher import *
import time
import struct


class SendImage:

    def __init__(self):
        # Specify the IP address and port number of the receiver
        self.IP_ADDRESS = '192.168.8.5'
        self.PORT = 7123
        # self.path = '../images/red_shirt.jpg'
        # self.path = '../images/20230426-170104-photo.png'
        self.path = '../images/photo.png'
        self.image_data = None

    # Open the image file
    def load_image(self, path):
        with open(path, 'rb') as f:
            self.image_data = f.read()

    def send_image(self):
        self.load_image(self.path)

        # Create a TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Connect to the receiver
        self.sock.connect((self.IP_ADDRESS, self.PORT))

        # Send the size of the image data as a 4-byte integer
        image_size = struct.pack("!L", len(self.image_data))
        self.sock.sendall(image_size)

        # Send the image data
        self.sock.sendall(self.image_data)

        #Wait for the answer
        answer = self.sock.recv(4096)
        print(answer)
        
        #publish a message to /clothes topic
        ClothingPublisher(answer)

        talker = Talker(answer)
        talker.talk()

        # Close the socket
        self.sock.close()
    

if __name__ == '__main__':
    sender = SendImage()
    try:
        sender.send_image()
    except Exception as e:
        print(e)
