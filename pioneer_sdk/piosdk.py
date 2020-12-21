from pymavlink import mavutil
import threading
import socket
import sys
import time


class Pioneer:
    def __init__(self, pioneer_ip='192.168.4.1', pioneer_video_port=8888, pioneer_video_control_port=8888,
                 pioneer_mavlink_port=8001, logger=True):
        self.__VIDEO_BUFFER = 65535
        video_address = (pioneer_ip, pioneer_video_port)
        video_control_address = (pioneer_ip, pioneer_video_control_port)
        self.__video_control_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__video_control_socket.settimeout(5)
        self.__video_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.__video_socket.settimeout(5)

        self.__video_frame_buffer = bytes()
        self.__raw_video_frame = 0
        self.__heartbeat_send_delay = 1
        self.__ack_timeout = 1
        self.__logger = logger

        try:
            self.__video_control_socket.connect(video_control_address)
            self.__video_socket.bind(self.__video_control_socket.getsockname())
            self.__mavlink_socket = mavutil.mavlink_connection('udpout:%s:%s' % (pioneer_ip, pioneer_mavlink_port))
        except socket.error:
            print('Can not connect to pioneer. Do you connect to drone wifi?')
            sys.exit()

        #self.__send_heartbeat()
        self.__heartbeat_thread = threading.Thread(target=self.__heartbeat_handler)
        self.__heartbeat_thread.daemon = True
        self.__heartbeat_thread.start()

    def get_raw_video_frame(self):
        try:
            self.__video_frame_buffer += self.__video_socket.recv(self.__VIDEO_BUFFER)
            beginning = self.__video_frame_buffer.find(b'\xff\xd8')
            end = self.__video_frame_buffer.find(b'\xff\xd9')
            while beginning != -1 and end != -1:
                self.__raw_video_frame = self.__video_frame_buffer[beginning:end + 2]
                self.__video_frame_buffer = self.__video_frame_buffer[end + 2:]
                return self.__raw_video_frame
        except socket.error as exc:
            print('Caught exception socket.error : ', exc)
            sys.exit()

    def __send_heartbeat(self):
        self.__mavlink_socket.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                                 mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        if self.__logger:
            print('send heartbeat')

    def __receive_heartbeat(self):
        self.__mavlink_socket.wait_heartbeat()
        if self.__logger:
            print("Heartbeat from system (system %u component %u)" % (self.__mavlink_socket.target_system,
                                                                      self.__mavlink_socket.target_component))

    def __heartbeat_handler(self):
        while True:
            self.__send_heartbeat()
            self.__receive_heartbeat()
            time.sleep(self.__heartbeat_send_delay)

    def __get_ack(self):
        command_ack = self.__mavlink_socket.recv_match(type='COMMAND_ACK', blocking=True,
                                                       timeout=self.__ack_timeout)
        if command_ack is not None:
            if command_ack.get_type() == 'COMMAND_ACK':
                if command_ack.result == 0:  # MAV_RESULT_ACCEPTED
                    if self.__logger:
                        print('MAV_RESULT_ACCEPTED')
                    return True
                elif command_ack.result == 1:  # MAV_RESULT_TEMPORARILY_REJECTED
                    if self.__logger:
                        print('MAV_RESULT_TEMPORARILY_REJECTED')
                    return None
                elif command_ack.result == 2:  # MAV_RESULT_DENIED
                    if self.__logger:
                        print('MAV_RESULT_DENIED')
                    return False
                elif command_ack.result == 3:  # MAV_RESULT_UNSUPPORTED
                    if self.__logger:
                        print('MAV_RESULT_UNSUPPORTED')
                    return False
                elif command_ack.result == 4:  # MAV_RESULT_FAILED
                    if self.__logger:
                        print('MAV_RESULT_FAILED')
                    return False
                elif command_ack.result == 5:  # MAV_RESULT_IN_PROGRESS
                    if self.__logger:
                        print('MAV_RESULT_IN_PROGRESS')
                    return self.__get_ack()
                elif command_ack.result == 6:  # MAV_RESULT_CANCELLED
                    if self.__logger:
                        print('MAV_RESULT_CANCELLED')
                    return None
        else:
            return None

    def arm(self):
        i = 0
        while True:
            self.__mavlink_socket.mav.command_long_send(
                self.__mavlink_socket.target_system,  # target_system
                self.__mavlink_socket.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # command
                i,  # confirmation
                1,  # param1
                0,  # param2 (all other params meaningless)
                0,  # param3
                0,  # param4
                0,  # param5
                0,  # param6
                0)  # param7
            ack = self.__get_ack()
            if ack is not None:
                if ack:
                    break
                else:
                    sys.exit()
            else:
                i += 1

    def disarm(self):
        i = 0
        while True:
            self.__mavlink_socket.mav.command_long_send(
                self.__mavlink_socket.target_system,  # target_system
                self.__mavlink_socket.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # command
                i,  # confirmation
                0,  # param1
                0,  # param2 (all other params meaningless)
                0,  # param3
                0,  # param4
                0,  # param5
                0,  # param6
                0)  # param7
            ack = self.__get_ack()
            if ack is not None:
                if ack:
                    break
                else:
                    self.disarm()
            else:
                i += 1

    def takeoff(self):
        i = 0
        while True:
            self.__mavlink_socket.mav.command_long_send(
                self.__mavlink_socket.target_system,  # target_system
                self.__mavlink_socket.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # command
                i,  # confirmation
                0,  # param1
                0,  # param2
                0,  # param3
                0,  # param4
                0,  # param5
                0,  # param6
                0)  # param7
            ack = self.__get_ack()
            if ack is not None:
                if ack:
                    break
                else:
                    self.disarm()
                    sys.exit()
            else:
                i += 1

    def land(self):
        i = 0
        while True:
            self.__mavlink_socket.mav.command_long_send(
                self.__mavlink_socket.target_system,  # target_system
                self.__mavlink_socket.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,  # command
                i,  # confirmation
                0,  # param1
                0,  # param2
                0,  # param3
                0,  # param4
                0,  # param5
                0,  # param6
                0)  # param7
            ack = self.__get_ack()
            if ack is not None:
                if ack:
                    break
                else:
                    self.land()
            else:
                i += 1

    def go_to_local_point(self, x=None, y=None, z=None, vx=None, vy=None, vz=None, afx=None, afy=None, afz=None,
                          yaw=None, yaw_rate=None, blocking=False, accuracy_m=0.1, accuracy_deg=5):
        parameters = [x, y, z, vx, vy, vz, afx, afy, afz, yaw, yaw_rate]
        mask = 0b0000111111111111
        element_mask = 0b0000000000000001
        for i in range(len(parameters)):
            if parameters[i] is not None:
                mask = mask ^ element_mask
            element_mask = element_mask << 1

        self.__mavlink_socket.mav.set_position_target_local_ned_send(0,  # time_boot_ms
                                                                     self.__mavlink_socket.target_system,
                                                                     self.__mavlink_socket.target_component,
                                                                     mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                                                     mask, x, y, z, vx, vy, vz,
                                                                     afx, afy, afz, yaw, yaw_rate)
        # blocking can be used only in x y z yaw flight
        if blocking:
            arrival_m = False
            arrival_deg = False
            error_vector_length_square = 0
            while not arrival_m and not arrival_deg:
                current_position = self.get_local_position(blocking=True)
                if current_position:
                    if yaw is not None:
                        if (yaw - current_position.yaw) <= accuracy_deg:
                            arrival_deg = True
                    else:
                        arrival_deg = True
                    position_vector = [current_position.x, current_position.y, current_position.z]
                    for i in range(len(position_vector)):
                        if parameters[i] is not None:
                            error_vector_length_square += pow((parameters[i]-position_vector[i]), 2)
                    if error_vector_length_square <= pow(accuracy_m, 2):
                        arrival_m = True

    def get_local_position(self, blocking=False):
        position = self.__mavlink_socket.recv_match(type='POSITION_TARGET_LOCAL_NED', blocking=blocking,
                                                    timeout=self.__ack_timeout)

        if not position:
            return
        if position.get_type() == "BAD_DATA":
            if mavutil.all_printable(position.data):
                sys.stdout.write(position.data)
                sys.stdout.flush()
        else:
            if self.__logger:
                print("X: {x}, Y: {y}, Z: {z}, YAW: {yaw}".format(x=position.x, y=position.y, z=-position.z,
                                                                  yaw=position.yaw))
            return position
