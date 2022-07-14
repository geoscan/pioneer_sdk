import threading

import cv2
import numpy as np
import socket


class Camera:

    def __init__(self, timeout=0.5, ip='192.168.4.1', port=8888, video_buffer_size=65000, log_connection=True):
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.VIDEO_BUFFER_SIZE = video_buffer_size
        self.tcp = None
        self.udp = None
        self.raw_video_frame = 0
        self._video_frame_buffer = bytes()
        self.raw_video_frame = bytes()
        self.connected = False
        self.log_connection = log_connection

    def new_tcp(self):
        """Returns new TCP socket"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(self.timeout)
        return sock

    def new_udp(self):
        """Returns new UDP socket"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(self.timeout)
        return sock

    def connect(self):
        """Connect to TCP and UDP sockets. Creates new ones if necessary."""
        self.disconnect()
        self.tcp = self.new_tcp()
        self.udp = self.new_udp()
        try:
            self.tcp.connect((self.ip, self.port))
            self.udp.bind(self.tcp.getsockname())
        except TimeoutError:
            return False
        return True

    def disconnect(self):
        """Disconnect."""
        self.connected = False
        if self.tcp is not None:
            self.tcp.close()
            self.tcp = None
        if self.udp is not None:
            self.udp.close()
            self.udp = None

    def get_frame(self):
        """Get bytes of frame.
        If UDP socket timeout then Exception raised.
        Returns :
            boolean: True if success, False if bad data of no data in UDP socket
            bytes(): Bytes of frame.
                     If first arg is True then current frame.
                     If first arg is False then last success frame.
                     If no last success frame then None."""
        try:
            if not self.connected:
                if self.connect():
                    self.connected = True
                    if self.log_connection:
                        print('Camera CONNECTED')
                else:
                    return None
            self._video_frame_buffer, addr = self.udp.recvfrom(self.VIDEO_BUFFER_SIZE)
            beginning = self._video_frame_buffer.find(b'\xff\xd8')
            if beginning == -1:
                return None
            self._video_frame_buffer = self._video_frame_buffer[beginning:]
            end = self._video_frame_buffer.find(b'\xff\xd9')
            if end == -1:
                return None
            self.raw_video_frame = self._video_frame_buffer[:end + 2]
            return self.raw_video_frame
        except TimeoutError:
            if self.connected:
                self.connected = False
                if self.log_connection:
                    print('Camera DISCONNECTED')
            return None


class VideoStream:
    def __init__(self, logger=True):
        self.camera = Camera(log_connection=logger)
        self.logger = logger
        self._vidio_stream = None
        self._stop = threading.Event()
        self._stop.set()

    def start(self):
        if not self._stop.is_set():
            return
        self._stop.clear()
        self._vidio_stream = threading.Thread(target=self._stream, daemon=True)
        self._vidio_stream.start()

    def stop(self):
        self._stop.set()

    def _stream(self):
        """
        Continuously receives JPEG frames from ESP32, parses them, and renders it using standard OpenCV's image rendering
        facilities.
        """
        keymap = {"esc": 27}
        while True:
            key = cv2.waitKey(1)
            if key == keymap["esc"] or self._stop.is_set():
                cv2.destroyAllWindows()
                break

            # Try to receive a frame
            camera_frame = self.camera.get_frame()

            if camera_frame is None:
                if self.logger:
                    print("No frame")
                continue

            # Decode and render JPEG frame
            camera_frame = cv2.imdecode(np.frombuffer(camera_frame, dtype=np.uint8), cv2.IMREAD_COLOR)
            cv2.imshow('pioneer_camera_stream', camera_frame)
