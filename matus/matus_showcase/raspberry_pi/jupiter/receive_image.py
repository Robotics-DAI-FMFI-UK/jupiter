#!/usr/bin/env python3
import socket
from classify_image import *

class ImageReceiver:
    def __init__(self):
        # Specify the IP address and port number of the sender
        # Specify the path of the image save destination
        self.IP_ADDRESS = '0.0.0.0'
        self.PORT = 7123
        self.save_path = '/home/pi/jupiter/clothes/received.png'

    def receive_image(self):
        # Create a TCP/IP socket
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            # Bind socket to IP address and port number
            sock.bind((self.IP_ADDRESS, self.PORT))
            sock.listen(1)

            while True:
                # Wait for a connection
                print("Waiting for connection>>")
                conn, addr = sock.accept()
                print('Connected by', addr)

                # Receive the size of the image data as a 4-byte integer
                size_bytes = conn.recv(4)
                size = int.from_bytes(size_bytes, byteorder='big')

                # Receive the image data
                data = b''
                while len(data) < size:
                    packet = conn.recv(size - len(data))
                    if not packet:
                        break
                    data += packet

                # Save the image data to a file
                with open(self.save_path, 'wb') as f:
                    f.write(data)
                              
                result = classification()
                
                # Send the classification result as answer
                conn.send(result.encode("utf-8"))
                print(result)

                # Close the connection
                conn.close()

if __name__ == '__main__':
    reciever = ImageReceiver()
    reciever.receive_image()
