#!/usr/bin/env python
#author: Sung Jik Cha
#credits: ros turtlebot node: https://github.com/Arkapravo/turtlebot

import socket
def imu_publisher(sock):
    host="0.0.0.0"
    port=5555
    theta = 0
    gyro_x_offset = 0.0
    gyro_y_offset = 0.0
    gyro_z_offset = 0.0
    pub_freq = 10
    alpha = 0.9
    count = 0
    num_callibration_itrs = 60
    debug = False


    sock.bind((host,port))
    print("is running")

    while 1:
        data,addr = sock.recvfrom(1024)
        line = data.split(',')
        print(line)
             

if __name__ == '__main__':
    if 1:
        print("run")
        sock=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        imu_publisher(sock)

