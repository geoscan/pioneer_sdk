from pymavlink import mavutil
import threading
import socket
import sys
import time


class Pioneer:
    def __init__(self, pioneer_ip='192.168.4.1', pioneer_video_port=8888, pioneer_video_control_port=8888,
                 pioneer_mavlink_port=8001, logger=True):
        self.__VIDEO_BUFFER = 65535
        video_control_address = (pioneer_ip, pioneer_video_control_port)
        self.__video_control_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__video_control_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.__video_control_socket.settimeout(5)
        self.__video_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.__video_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.__video_socket.settimeout(5)

        self.__video_frame_buffer = bytes()
        self.__raw_video_frame = 0
        self.__heartbeat_send_delay = 1
        self.__ack_timeout = 1
        self.__logger = logger

        self.__prev_point_id = None

        self.command_id = 0
        self.__pars = {}
        self.__send_time = time.time()

        self.__execute_once = True

        self.__in_air = False
        self.__landed = True

        try:
            self.__video_control_socket.connect(video_control_address)
            self.__video_socket.bind(self.__video_control_socket.getsockname())
            self.__mavlink_socket = mavutil.mavlink_connection('udpout:%s:%s' % (pioneer_ip, pioneer_mavlink_port))
        except socket.error:
            print('Can not connect to pioneer. Do you connect to drone wifi?')
            sys.exit()

        self.__init_heartbeat_event = threading.Event()

        self.__heartbeat_thread = threading.Thread(target=self.__heartbeat_handler,
                                                   args=(self.__init_heartbeat_event, ))
        self.__heartbeat_thread.daemon = True
        self.__heartbeat_thread.start()

        while not self.__init_heartbeat_event.is_set():
            pass

        while not self.point_reached():
            pass

        self.__thread_moving = threading.Thread(target=self.__thread_moving_control)
        self.__thread_moving.daemon = True
        self.__moving_done_event = threading.Event()
        self.__moving_done_event.clear()

    def get_raw_video_frame(self):
        try:
            while True:
                self.__video_frame_buffer += self.__video_socket.recv(self.__VIDEO_BUFFER)
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
        except socket.error as exc:
            print('Caught exception socket.error : ', exc)

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

    def __heartbeat_handler(self, event):
        while True:
            self.__send_heartbeat()
            self.__receive_heartbeat()
            if not event.is_set():
                event.set()
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
                    return True
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

    def __thread_moving_control(self):
        while True:
            if self.__execute_once:
                if self.command_id == 1:
                    self.__takeoff()
                    self.__execute_once = False
                elif self.command_id == 2:
                    self.__land()
                    self.__execute_once = False
                elif self.command_id == 3:
                    self.__go_to_local_point()
                    self.__execute_once = False

            if self.point_reached() or self.__moving_done_event.is_set():
                self.__moving_done_event.clear()
                self.__execute_once = True
                self.command_id = 0

    def get_task_id(self):
        return self.command_id

    def arm(self):
        i = 0
        if self.__logger:
            print('arm command send')
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
                    if self.__logger:
                        print('arming complete')
                    break
                else:
                    self.disarm()
                    sys.exit()
            else:
                i += 1
        self.__thread_moving.start()

    def disarm(self):
        i = 0
        if self.__logger:
            print('disarm command send')
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
                    if self.__logger:
                        print('disarming complete')
                    break
                else:
                    self.disarm()
            else:
                i += 1

    def __takeoff(self):
        i = 0
        self.__landed = False
        if self.__logger:
            print('takeoff command send')
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
                    if self.__logger:
                        print('takeoff complete')
                        self.__moving_done_event.set()
                    break
                else:
                    self.land()
                    sys.exit()
            else:
                i += 1
        self.__in_air = True

    def takeoff(self):
        self.command_id = 1

    def __land(self):
        i = 0
        self.__in_air = False
        if self.__logger:
            print('land command send')
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
                    if self.__logger:
                        print('landing complete')
                        self.__moving_done_event.set()
                    break
                else:
                    self.land()
            else:
                i += 1
        self.__landed = True

    def land(self):
        self.command_id = 2

    def lua_script_control(self, input_state='Stop'):
        i = 0
        target_component = 25
        state = dict(Stop=0, Start=1)
        command = state.get(input_state)
        if command is not None:
            if self.__logger:
                print('LUA script command: %s send' % input_state)
            while True:
                self.__mavlink_socket.mav.command_long_send(
                    self.__mavlink_socket.target_system,  # target_system
                    target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # command
                    i,  # confirmation
                    command,  # param1
                    0,  # param2
                    0,  # param3
                    0,  # param4
                    0,  # param5
                    0,  # param6
                    0)  # param7
                ack = self.__get_ack()
                if ack is not None:
                    if ack:
                        if self.__logger:
                            print('LUA script command: %s complete' % input_state)
                        break
                    else:
                        i += 1
                else:
                    i += 1
        else:
            if self.__logger:
                print('wrong LUA command value')

    def led_control(self, led_id=255, r=0, g=0, b=0):  # 255 all led
        max_value = 255.0
        all_led = 255
        first_led = 0
        last_led = 3
        i = 0
        led_value = [r, g, b]
        command = True

        try:
            if led_id != all_led and (led_id < first_led or led_id > last_led):
                command = False
            for i in range(len(led_value)):
                led_value[i] = float(led_value[i])
                if led_value[i] > max_value or led_value[i] < 0:
                    command = False
                    break
                led_value[i] /= max_value
        except ValueError:
            command = False

        if command:
            if led_id == all_led:
                led_id_print = 'all'
            else:
                led_id_print = led_id
            if self.__logger:
                print('LED id: %s R: %i ,G: %i, B: %i send' % (led_id_print, r, g, b))
            while True:
                self.__mavlink_socket.mav.command_long_send(
                    self.__mavlink_socket.target_system,  # target_system
                    self.__mavlink_socket.target_component,
                    mavutil.mavlink.MAV_CMD_USER_1,  # command
                    i,  # confirmation
                    led_id,  # param1
                    led_value[0],  # param2
                    led_value[1],  # param3
                    led_value[2],  # param4
                    0,  # param5
                    0,  # param6
                    0)  # param7
                ack = self.__get_ack()
                if ack is not None:
                    if ack:
                        if self.__logger:
                            print('LED id: %s RGB send complete' % led_id_print)
                        break
                    else:
                        self.led_control(led_id, r, g, b)
                else:
                    i += 1
        else:
            if self.__logger:
                print('wrong LED RGB values or id')

    def go_to_local_point(self, x=None, y=None, z=None, vx=None, vy=None, vz=None, afx=None, afy=None, afz=None,
                            yaw=None, yaw_rate=None):
        self.command_id = 3
        self.__send_time = time.time()
        self.__pars = dict(x=x, y=y, z=z, vx=vx, vy=vy, vz=vz, afx=afx, afy=afy, afz=afz, force_set=0, yaw=yaw,
                           yaw_rate=yaw_rate, mask=0b0000111111111111, el_mask=0b0000000000000001)  # 0-force_set
        for n, v in self.__pars.items():
            if n not in ('mask', 'el_mask'):
                if v is not None:
                    self.__pars['mask'] = self.__pars['mask'] ^ self.__pars['el_mask']
                else:
                    self.__pars[n] = 0.0
                self.__pars['el_mask'] = self.__pars['el_mask'] << 1
        if self.__logger:
            print('sending local point :', end=' ')
            first_output = True
            for n, v in self.__pars.items():
                if n not in ('mask', 'el_mask'):
                    if self.__pars[n] != 0.0:
                        if first_output:
                            print(n, ' = ', v, sep="", end='')
                            first_output = False
                        else:
                            print(', ', n, ' = ', v, sep="", end='')
            print(end='\n')

    def __go_to_local_point(self):
        ack_timeout = 0.1
        while True:
            if not self.__ack_receive_point():
                if (time.time() - self.__send_time) >= ack_timeout:
                    self.__mavlink_socket.mav.set_position_target_local_ned_send(0,  # time_boot_ms
                                                                                 self.__mavlink_socket.target_system,
                                                                                 self.__mavlink_socket.target_component,
                                                                                 mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                                                                 self.__pars['mask'],
                                                                                 self.__pars['x'],
                                                                                 self.__pars['y'],
                                                                                 self.__pars['z'],
                                                                                 self.__pars['vx'],
                                                                                 self.__pars['vy'],
                                                                                 self.__pars['vz'],
                                                                                 self.__pars['afx'],
                                                                                 self.__pars['afy'],
                                                                                 self.__pars['afz'],
                                                                                 self.__pars['yaw'],
                                                                                 self.__pars['yaw_rate'])
                    self.__send_time = time.time()
            else:
                self.__moving_done_event.set()
                break

    def point_reached(self, blocking=False):
        point_reached = self.__mavlink_socket.recv_match(type='MISSION_ITEM_REACHED', blocking=blocking,
                                                         timeout=self.__ack_timeout)
        if not point_reached:
            return False
        if point_reached.get_type() == "BAD_DATA":
            if mavutil.all_printable(point_reached.data):
                sys.stdout.write(point_reached.data)
                sys.stdout.flush()
                return False
        else:
            point_id = point_reached.seq
            if self.__prev_point_id is None:
                self.__prev_point_id = point_id
                new_point = True
            elif point_id > self.__prev_point_id:
                self.__prev_point_id = point_id
                new_point = True
            else:
                new_point = False
            if new_point:
                if self.__logger:
                    print("point reached, id: ", point_id)
                return True
            else:
                return False

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

    def get_dist_sensor_data(self, blocking=False):
        dist_sensor_data = self.__mavlink_socket.recv_match(type='DISTANCE_SENSOR', blocking=blocking,
                                                            timeout=self.__ack_timeout)
        if not dist_sensor_data:
            return
        if dist_sensor_data.get_type() == "BAD_DATA":
            if mavutil.all_printable(dist_sensor_data.data):
                sys.stdout.write(dist_sensor_data.data)
                sys.stdout.flush()
                return
        else:
            curr_distance = float(dist_sensor_data.current_distance)/100  # cm to m
            if self.__logger:
                print("get dist sensor data: %5.2f m" % curr_distance)
            return curr_distance

    def __ack_receive_point(self, blocking=False, timeout=None):
        if timeout is None:
            timeout = self.__ack_timeout
        ack = self.__mavlink_socket.recv_match(type='POSITION_TARGET_LOCAL_NED', blocking=blocking,
                                               timeout=timeout)
        if not ack:
            return False
        if ack.get_type() == "BAD_DATA":
            if mavutil.all_printable(ack.data):
                sys.stdout.write(ack.data)
                sys.stdout.flush()
            return False
        else:
            return True

    def send_rc_channels(self, channel_1=0xFF, channel_2=0xFF, channel_3=0xFF, channel_4=0xFF,
                           channel_5=0xFF, channel_6=0xFF, channel_7=0xFF, channel_8=0xFF):
        self.__mavlink_socket.mav.rc_channels_override_send(self.__mavlink_socket.target_system,
                                                            self.__mavlink_socket.target_component, channel_1,
                                                            channel_2, channel_3, channel_4, channel_5, channel_6,
                                                            channel_7, channel_8)

    def in_air(self):
        return self.__in_air

    def landed(self):
        return self.__landed
