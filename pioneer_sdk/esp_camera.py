"""
ESP32's camera is capable of streaming JPEG frames over UDP. It uses a complicated communication scheme involving
use of two sockets (TCP and UDP). This module encapsulates the complexity, namely connecting to the camera and
parsing incoming JPEG frames.
"""

import socket
import cv2
import numpy as np
from pymavlink import mavutil
import threading
import socket
import sys
import time


class EspCamera:

    IP_ADDRESS = "192.168.4.1"
    TCP_PORT = 8888  # ESP32 listens on TCP/8888. On an incoming connection, it accepts it and starts sending UDP packets to the client's UDP port w/ the same port number
    VIDEO_BUFFER_SIZE = 4 * 20 * 1024  # The expected frame size is about 15k. Double of that storage w/ a bit of slack is supposed to fullfill our needs.

    def __init__(self, autoconnect=True):
        self.__video_socket = None
        self.__video_control_socket = None
        self.__raw_video_frame = 0
        self.__video_frame_buffer = bytes()

        if autoconnect:
            self.connect()

    def connect(self):
        """
        Initializes TCP and UDP sockets, ensures that both have the same port number.
        """

        if not (self.__video_socket is None and self.__video_control_socket is None):
            raise Exception("Already connected")

        self.__video_control_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__video_control_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.__video_control_socket.settimeout(5)
        self.__video_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.__video_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.__video_socket.settimeout(5)

        # Please note the possibility of "connection error" exception
        self.__video_control_socket.connect((EspCamera.IP_ADDRESS, EspCamera.TCP_PORT,))
        self.__video_socket.bind(self.__video_control_socket.getsockname())

    def receive_frame(self):
        """
        Parses JPEG frames from a continuous stream

        :return: Parsed JPEG frame.

        Please note the possibility of socket timeout exception.
        """
        while True:
            self.__video_frame_buffer += self.__video_socket.recv(EspCamera.VIDEO_BUFFER_SIZE)
            beginning = self.__video_frame_buffer.find(b'\xff\xd8')
            end = self.__video_frame_buffer.find(b'\xff\xd9')
            if beginning != -1 and end != -1 and end > beginning:
                self.__raw_video_frame = self.__video_frame_buffer[beginning:end + 2]
                self.__video_frame_buffer = self.__video_frame_buffer[end + 2:]
                break
            else:
                print(len(self.__raw_video_frame))
                self.__video_frame_buffer = bytes()
                self.__raw_video_frame = bytes()
        return self.__raw_video_frame

    get_raw_video_frame = receive_frame  # For the purposes of partial backward compatibility


class VideoStream:
    def __init__(self):
        self.camera = EspCamera()

    def run(self, interactive=False):
        """
        Continuously receives JPEG frames from ESP32, parses them, and renders it using standard OpenCV's image rendering
        facilities.
        :param interactive: Stop stream on ESC
        """

        # Establish a connection w/ the camera.
        self.camera.connect()

        while True:
            # Try to receive a frame

            camera_frame = self.camera.receive_frame()

            if camera_frame is None:
                continue

            # Decode and render JPEG frame
            camera_frame = cv2.imdecode(np.frombuffer(camera_frame, dtype=np.uint8), cv2.IMREAD_COLOR)
            cv2.imshow('pioneer_camera_stream', camera_frame)

            # ESC key terminates the stream
            keymap = {"esc": 27}
            key = cv2.waitKey(1)

            if interactive and key == keymap["esc"]:
                cv2.destroyAllWindows()
                break


if __name__ == "__main__":
    VideoStream().run(True)

