from pymavlink import mavutil
from pymavlink.dialects.v20 import common
import json
import threading
import socket
import sys
import time
import serial


class Pioneer:
    def __init__(self, method=0, pioneer_ip='192.168.4.1', pioneer_mavlink_port=8001, device='/dev/serial0',
                 baud=115200, logger=True):

        self.__heartbeat_send_delay = 0.25
        self.__ack_timeout = 1
        self.__logger = logger

        self.__prev_point_id = None

        self.__mavlink_socket = None

        self.t_start = time.time()

        self.autopilot_state = {
            0: "ROOT",
            1: "DISARMED",
            2: "IDLE",
            3: "TEST_ACTUATION",
            4: "TEST_PARACHUTE",
            5: "TEST_ENGINE",
            6: "PARACHUTE",
            7: "WAIT_FOR_LANDING",
            8: "LANDED",
            9: "CATAPULT",
            10: "PREFLIGHT",
            11: "ARMED",
            12: "TAKEOFF",
            13: "WAIT_FOR_GPS",
            14: "WIND_MEASURE",
            15: "MISSION",
            16: "ASCEND",
            17: "DESCEND",
            18: "RTL",
            19: "UNCONDITIONAL_RTL",
            20: "MANUAL_HEADING",
            21: "MANUAL_ROLL",
            22: "MANUAL_SPEED",
            23: "LANDING",
            24: "ON_DEMAND"
        }

        self.cur_state = -1
        self.preflight_state = dict(BatteryLow=None,
                                    NavSystem=None,
                                    Area=None,
                                    Attitude=None,
                                    RcExpected=None,
                                    RcMode=None,
                                    RcUnexpected=None,
                                    UavStartAllowed=None)

        if method == 0:
            if self.__logger:
                print("Connection method", method)
            try:
                if self.__logger:
                    print("Connected through Wi-Fi as host to", pioneer_ip, pioneer_mavlink_port)
                self.__mavlink_socket = mavutil.mavlink_connection("udpin:%s:%s" % (pioneer_ip, pioneer_mavlink_port))
            except socket.error:
                print("Can not connect to pioneer. Do you connect to drone wifi?")
                sys.exit()
        elif method == 1:
            if self.__logger:
                print("Connection method", method)
            try:
                print('Connected through UART to', device, baud)
                self.__mavlink_socket = mavutil.mavlink_connection(device=device, baud=baud)

            except serial.SerialException:
                print('Serial error')
                sys.exit()
        elif method == 2:
            if self.__logger:
                print("Connection method", method)
            try:
                print("Connected through Wi-Fi as client to", pioneer_ip, pioneer_mavlink_port)
                self.__mavlink_socket = mavutil.mavlink_connection('udpout:%s:%s' % (pioneer_ip, pioneer_mavlink_port))
            except socket.error:
                print('Can not connect to pioneer. Do you connect to drone wifi?')
                sys.exit()
        else:
            print('Unsupported method was called')

        self.__init_heartbeat_event = threading.Event()

        self.__heartbeat_thread = threading.Thread(target=self.__heartbeat_handler,
                                                   args=(self.__init_heartbeat_event,))
        self.__heartbeat_thread.daemon = True
        self.__heartbeat_thread.start()
        while not self.__init_heartbeat_event.is_set():
            pass
        time.sleep(0.5)
        while not self.point_reached():
            pass

    def __send_heartbeat(self):
        self.__mavlink_socket.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                                 mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        if self.__logger:
            print('Send heartbeat')

    def __receive_heartbeat(self):
        heartbeat = self.__mavlink_socket.recv_match(type="HEARTBEAT", blocking=True)
        try:
            if heartbeat._header.srcComponent == 1:
                custom_mode = heartbeat.custom_mode
                custom_mode_buf = format(custom_mode, "032b")
                status_autopilot = custom_mode_buf[24:]
                self.cur_state = self.autopilot_state[int(status_autopilot, 2)]
        except:
            pass

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

    def __get_ack(self, timeout=0.1):
        command_ack = self.__mavlink_socket.recv_match(type='COMMAND_ACK', blocking=True,
                                                       timeout=timeout)
        if command_ack is not None:
            print(command_ack)
            if command_ack.get_type() == 'COMMAND_ACK':

                if command_ack.result == 0:  # MAV_RESULT_ACCEPTED
                    if self.__logger:
                        print('MAV_RESULT_ACCEPTED')
                    return True, command_ack.command
                elif command_ack.result == 1:  # MAV_RESULT_TEMPORARILY_REJECTED
                    if self.__logger:
                        print('MAV_RESULT_TEMPORARILY_REJECTED')
                    return None, command_ack.command
                elif command_ack.result == 2:  # MAV_RESULT_DENIED
                    if self.__logger:
                        print('MAV_RESULT_DENIED')
                    return True, command_ack.command
                elif command_ack.result == 3:  # MAV_RESULT_UNSUPPORTED
                    if self.__logger:
                        print('MAV_RESULT_UNSUPPORTED')
                    return False, command_ack.command
                elif command_ack.result == 4:  # MAV_RESULT_FAILED
                    if self.__logger:
                        print('MAV_RESULT_FAILED')
                    if command_ack.result_param2 is not None:
                        self.preflight_state.update(BatteryLow=command_ack.result_param2 & 0b00000001)
                        self.preflight_state.update(NavSystem=command_ack.result_param2 & 0b00000010)
                        self.preflight_state.update(Area=command_ack.result_param2 & 0b00000100)
                        self.preflight_state.update(Attitude=command_ack.result_param2 & 0b00001000)
                        self.preflight_state.update(RcExpected=command_ack.result_param2 & 0b00010000)
                        self.preflight_state.update(RcMode=command_ack.result_param2 & 0b00100000)
                        self.preflight_state.update(RcUnexpected=command_ack.result_param2 & 0b01000000)
                        self.preflight_state.update(UavStartAllowed=command_ack.result_param2 & 0b10000000)
                    return False, command_ack.command
                elif command_ack.result == 5:  # MAV_RESULT_IN_PROGRESS
                    if self.__logger:
                        print('MAV_RESULT_IN_PROGRESS')
                    return self.__get_ack()
                elif command_ack.result == 6:  # MAV_RESULT_CANCELLED
                    if self.__logger:
                        print('MAV_RESULT_CANCELLED')
                    return None, command_ack.command
        else:
            return None, None

    def raspberry_poweroff(self):
        """ Shutdown the raspberry pi """
        i = 0
        n_attempts = 25
        target_component = 42
        if self.__logger:
            print('Raspberry poweroff command send')
        while True:
            self.__mavlink_socket.mav.command_long_send(
                self.__mavlink_socket.target_system,  # target_system
                target_component,  # target_component
                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,  # command
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
                if ack[0] and ack[1] == mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                    if self.__logger:
                        print('Raspberry poweroff complete')
                    return True
                else:
                    i += 1
                    if i > n_attempts:
                        return False
            else:
                i += 1
                if i > n_attempts:
                    return False

    def raspberry_reboot(self):
        """ Reboot the raspberry pi """
        i = 0
        n_attempts = 25
        target_component = 43
        if self.__logger:
            print('Raspberry reboot command send')
        while True:
            self.__mavlink_socket.mav.command_long_send(
                self.__mavlink_socket.target_system,  # target_system
                target_component,  # target_component
                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,  # command
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
                if ack[0] and ack[1] == mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                    if self.__logger:
                        print('Raspberry reboot complete')
                    return True
                else:
                    i += 1
                    if i > n_attempts:
                        return False

            else:
                i += 1
                if i > n_attempts:
                    return False

    def arm(self):
        """ Arming command for drone """
        try:
            i = 0
            n_attempts = 15
            if self.__logger:
                print('Arm command send')
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
                ack = self.__get_ack(0.5)
                if ack is not None:
                    if ack[0] and ack[1] == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                        if self.__logger:
                            print('Arming complete')
                        return True
                    else:
                        i += 1
                        if i > n_attempts:
                            return False
                else:
                    i += 1
                    if i > n_attempts:
                        return False
        except:
            pass

    def disarm(self):
        """ Disarming command for drone """
        try:
            i = 0
            n_attempts = 15
            if self.__logger:
                print('Disarm command send')
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
                ack = self.__get_ack(0.5)
                if ack is not None:
                    if ack[0] and ack[1] == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                        if self.__logger:
                            print('Disarming complete')
                        return True
                    else:
                        i += 1
                        if i > n_attempts:
                            return False
                else:
                    i += 1
                    if i > n_attempts:
                        return False
        except:
            pass

    def takeoff(self):
        """ Takeoff command for drone"""
        i = 0
        n_attempts = 15
        if self.__logger:
            print('Takeoff command send')
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
            ack = self.__get_ack(0.5)
            if ack is not None:
                if ack[0] and ack[1] == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                    if self.__logger:
                        print('Takeoff complete')
                    return True
                else:
                    i += 1
                    if i > n_attempts:
                        return False
            else:
                i += 1
                if i > n_attempts:
                    return False

    def land(self):
        """ Landing command for drone """
        i = 0
        n_attempts = 15
        if self.__logger:
            print('Land command send')
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
            ack = self.__get_ack(0.5)
            if ack is not None:
                if ack[0] and ack[1] == mavutil.mavlink.MAV_CMD_NAV_LAND:
                    if self.__logger:
                        print('Landing complete')
                    return True
                else:
                    i += 1
                    if i > n_attempts:
                        return False
            else:
                i += 1
                if i > n_attempts:
                    return False

    def reboot_board(self):
        """ Restart the flight controller """
        try:
            i = 0
            n_attempts = 255
            if self.__logger:
                print('arm command send')
            while True:
                self.__mavlink_socket.mav.command_long_send(
                    self.__mavlink_socket.target_system,  # target_system
                    1,
                    mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,  # command
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
                    if ack[0] and ack[1] == mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                        if self.__logger:
                            print('arming complete')
                        return True
                    else:
                        i += 1
                        if i > n_attempts:
                            return False
                else:
                    i += 1
                    if i > n_attempts:
                        return False
        except:
            pass

    def lua_script_control(self, input_state='Stop'):
        """ Start/stop loaded to board LUA script """
        i = 0
        n_attempts = 25
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
                        return True
                    else:
                        i += 1
                        if i > n_attempts:
                            return False
                else:
                    i += 1
                    if i > n_attempts:
                        return False
        else:
            if self.__logger:
                print('wrong LUA command value')

    def led_control(self, led_id=255, r=0, g=0, b=0):  # 255 all led
        """ Change LED color for device without raspberry pi"""
        max_value = 255.0
        all_led = 255
        first_led = 0
        last_led = 3
        i = 0
        n_attempts = 25
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
                    if ack[0] and ack[1] == mavutil.mavlink.MAV_CMD_USER_1:
                        if self.__logger:
                            print('LED id: %s RGB send complete' % led_id_print)
                        return True
                    else:
                        i += 1
                        if i > n_attempts:
                            return False
                else:
                    i += 1
                    if i > n_attempts:
                        return False
        else:
            if self.__logger:
                print('wrong LED RGB values or id')

    def raspberry_led_custom(self, mode=1, timer=0, color1=(0, 0, 0), color2=(0, 0, 0)):
        """ Change LED color for device with raspberry pi"""
        param2 = (((color1[0] << 8) | color1[1]) << 8) | color1[2]
        param3 = (((color2[0] << 8) | color2[1]) << 8) | color2[2]
        param5 = mode
        param6 = timer
        i = 0
        n_attempts = 25
        while True:
            self.__mavlink_socket.mav.command_long_send(
                0,  # target_system
                0,
                mavutil.mavlink.MAV_CMD_USER_3,  # command
                i,  # confirmation
                0,  # param1
                param2,  # param2
                param3,  # param3
                0,  # param4
                param5,  # param5
                param6,  # param6
                0)  # param7
            if self.__logger:
                print("Raspberry custom led sent message")
            ack = self.__get_ack()
            if ack is not None:
                if ack[0] and ack[1] == mavutil.mavlink.MAV_CMD_USER_2:
                    if self.__logger:
                        print("Raspberry custom led complete")
                    return True
                else:
                    i += 1
                    if i > n_attempts:
                        return False
            else:
                i += 1
                if i > n_attempts:
                    return False

    def go_to_local_point(self, x=None, y=None, z=None, vx=None, vy=None, vz=None, afx=None, afy=None, afz=None,
                          yaw=None, yaw_rate=None):

        """ Flight to point in position system's coordinate """

        ack_timeout = 0.1
        send_time = time.time()
        parameters = dict(x=x, y=y, z=z, vx=vx, vy=vy, vz=vz, afx=afx, afy=afy, afz=afz, force_set=0, yaw=yaw,
                          yaw_rate=yaw_rate)  # 0-force_set
        mask = 0b0000111111111111
        element_mask = 0b0000000000000001
        for n, v in parameters.items():
            if v is not None:
                mask = mask ^ element_mask
            else:
                parameters[n] = 0.0
            element_mask = element_mask << 1
        if self.__logger:
            print('Sending local point :', end=' ')
            first_output = True
            for n, v in parameters.items():
                if parameters[n] != 0.0:
                    if first_output:
                        print(n, ' = ', v, sep="", end='')
                        first_output = False
                    else:
                        print(', ', n, ' = ', v, sep="", end='')
            print(end='\n')
        counter = 1
        while True:
            if not self.__ack_receive_point():
                if (time.time() - send_time) >= ack_timeout:
                    counter += 1
                    self.__mavlink_socket.mav.set_position_target_local_ned_send(0,  # time_boot_ms
                                                                                 self.__mavlink_socket.target_system,
                                                                                 self.__mavlink_socket.target_component,
                                                                                 mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                                                                 mask, parameters['x'], parameters['y'],
                                                                                 parameters['z'], parameters['vx'],
                                                                                 parameters['vy'], parameters['vz'],
                                                                                 parameters['afx'], parameters['afy'],
                                                                                 parameters['afz'], parameters['yaw'],
                                                                                 parameters['yaw_rate'])
                    send_time = time.time()
                if counter > 25:
                    return False
            else:
                return True

    def go_to_local_point_body_fixed(self, x=None, y=None, z=None, vx=0.3, vy=0.3, vz=0.8, afx=None, afy=None, afz=None,
                                     yaw=None, yaw_rate=0.5):
        """ Flight to point relative to the current position """

        ack_timeout = 0.1
        send_time = time.time()
        parameters = dict(x=x, y=y, z=z, vx=vx, vy=vy, vz=vz, afx=afx, afy=afy, afz=afz, force_set=0, yaw=yaw,
                          yaw_rate=yaw_rate)  # 0-force_set
        mask = 0b0000111111111111
        element_mask = 0b0000000000000001
        for n, v in parameters.items():
            if v is not None:
                mask = mask ^ element_mask
            else:
                parameters[n] = 0.0
            element_mask = element_mask << 1
        if self.__logger:
            print('Sending relative local point :', end=' ')
            first_output = True
            for n, v in parameters.items():
                if parameters[n] != 0.0:
                    if first_output:
                        print(n, ' = ', v, sep="", end='')
                        first_output = False
                    else:
                        print(', ', n, ' = ', v, sep="", end='')
            print(end='\n')
        counter = 1
        while True:
            if not self.__ack_receive_point():
                if (time.time() - send_time) >= ack_timeout:
                    counter += 1
                    self.__mavlink_socket.mav.set_position_target_local_ned_send(0,  # time_boot_ms
                                                                                 self.__mavlink_socket.target_system,
                                                                                 self.__mavlink_socket.target_component,
                                                                                 mavutil.mavlink.MAV_FRAME_BODY_FRD,
                                                                                 mask, parameters['x'], parameters['y'],
                                                                                 parameters['z'], parameters['vx'],
                                                                                 parameters['vy'], parameters['vz'],
                                                                                 parameters['afx'], parameters['afy'],
                                                                                 parameters['afz'], parameters['yaw'],
                                                                                 parameters['yaw_rate'])
                    send_time = time.time()
                if counter > 25:
                    return False
            else:
                return True

    def point_reached(self, blocking=False):
        """ Callback of destination point in mission flight """
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
                    print("Point reached, id: ", point_id)
                return True
            else:
                return False

    def get_local_position_lps(self, blocking=False):
        """ Returning position from LPS """
        position = self.__mavlink_socket.recv_match(type='LOCAL_POSITION_NED', blocking=blocking,
                                                    timeout=self.__ack_timeout)
        if not position:
            return None
        if position.get_type() == "BAD_DATA":
            if mavutil.all_printable(position.data):
                sys.stdout.write(position.data)
                sys.stdout.flush()
        else:
            if position._header.srcComponent == 26:
                return [position.x, position.y, position.z]
            else:
                return None

    def get_dist_sensor_data(self, blocking=False):
        """ Returning altitude from TOF """
        dist_sensor_data = self.__mavlink_socket.recv_match(type='DISTANCE_SENSOR', blocking=blocking,
                                                            timeout=self.__ack_timeout)
        if not dist_sensor_data:
            return None
        if dist_sensor_data.get_type() == "BAD_DATA":
            if mavutil.all_printable(dist_sensor_data.data):
                sys.stdout.write(dist_sensor_data.data)
                sys.stdout.flush()
                return None
        else:
            curr_distance = float(dist_sensor_data.current_distance) / 100  # cm to m
            if self.__logger:
                print("get dist sensor data: %5.2f m" % curr_distance)
            return curr_distance

    def get_optical_data(self, blocking=False):
        """ Returning data from OPT """
        optical_data = self.__mavlink_socket.recv_match(type='OPTICAL_FLOW_RAD', blocking=blocking,
                                                        timeout=self.__ack_timeout)
        if not optical_data:
            return None
        if optical_data.get_type() == "BAD_DATA":
            if mavutil.all_printable(optical_data.data):
                sys.stdout.write(optical_data.data)
                sys.stdout.flush()
                return
        else:
            return optical_data

    def get_battery_status(self, blocking=False):
        """ Returning battery remaining voltage """
        bat_state = self.__mavlink_socket.recv_match(type='BATTERY_STATUS', blocking=blocking,
                                                     timeout=self.__ack_timeout)
        if bat_state is None:
            return None
        if bat_state.get_type() == "BAD_DATA":
            if mavutil.all_printable(bat_state.data):
                sys.stdout.write(bat_state.data)
                sys.stdout.flush()
                return None
        else:
            voltage = bat_state.voltages[0]
            if self.__logger:
                print("voltage %f" % voltage)
            return voltage

    def get_preflight_state(self, field):
        return self.preflight_state

    def get_autopilot_version(self):
        """ Returning autopilot version """
        i = 0
        n_attempts = 15
        while True:
            self.__mavlink_socket.mav.command_long_send(
                self.__mavlink_socket.target_system,  # target_system
                1,
                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,  # command
                i,  # confirmation
                mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,  # param1
                0,  # param2
                0,  # param3
                0,  # param4
                0,  # param5
                0,  # param6
                0)  # param7
            ap_ver = self.__mavlink_socket.recv_match(type="AUTOPILOT_VERSION", blocking=True,
                                                      timeout=self.__ack_timeout)

            if ap_ver is None:
                i += 1
                if i > n_attempts:
                    return None
                else:
                    continue
            else:
                if ap_ver.get_type() == "AUTOPILOT_VERSION":
                    return [ap_ver.flight_sw_version, ap_ver.board_version, ap_ver.flight_custom_version]
                else:
                    i += 1
                    if i > n_attempts:
                        return None

    def __ack_receive_point(self, blocking=False, timeout=None):
        if timeout is None:
            timeout = self.__ack_timeout
        ack = self.__mavlink_socket.recv_match(type='POSITION_TARGET_LOCAL_NED', blocking=blocking,
                                               timeout=timeout)
        if not ack:
            return False
        else:
            if ack._header.srcComponent == 1:
                return True
        if ack.get_type() == "BAD_DATA":
            if mavutil.all_printable(ack.data):
                sys.stdout.write(ack.data)
                sys.stdout.flush()
            return False

    def send_rc_channels(self, channel_1=0xFF, channel_2=0xFF, channel_3=0xFF, channel_4=0xFF,
                         channel_5=0xFF, channel_6=0xFF, channel_7=0xFF, channel_8=0xFF):
        """ RC unit signal send to drone """
        # channel_1 = throttle channel value
        # channel_2 = yaw channel value
        # channel_3 = pitch channel value
        # channel_4 = roll channel value
        # channel_5 = mode. 2000 - program

        self.__mavlink_socket.mav.rc_channels_override_send(self.__mavlink_socket.target_system,
                                                            self.__mavlink_socket.target_component, channel_1,
                                                            channel_2, channel_3, channel_4, channel_5, channel_6,
                                                            channel_7, channel_8)

    def raspberry_start_capture(self, interval=0.1, total_images=0, sequence_number=0):
        """ Raspberry pi camera start capturing """
        # param2 interval - delay between each frame
        # param3 total_images - number of images to capture. 0 = do capture till stop_capture is called
        # param4 sequence_number - if total_images = 1 increment after each frame, else use 0

        try:
            i = 0
            n_attempts = 15
            if self.__logger:
                print('Raspberry pi camera start_capture send')

            while True:
                self.__mavlink_socket.mav.command_long_send(
                    self.__mavlink_socket.target_system,  # target_system
                    self.__mavlink_socket.target_component,
                    mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE,  # command
                    i,  # confirmation
                    0,  # param1
                    interval,  # param2
                    total_images,  # param3
                    sequence_number,  # param4
                    0,  # param5
                    0,  # param6
                    0)  # param7

                ack = self.__get_ack()
                if ack is not None:
                    if ack[0] and ack[1] == mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE:
                        if self.__logger:
                            print('Raspberry pi camera start_capture complete, iter:', i)
                        return True
                    else:
                        i += 1
                        if i > n_attempts:
                            return False
                else:
                    i += 1
                    if i > n_attempts:
                        return False
        except:
            pass

    def raspberry_stop_capture(self):
        """ End of Raspberry pi capture """

        try:
            i = 0
            n_attempts = 15
            if self.__logger:
                print('Raspberry pi stop_capture send')

            while True:
                self.__mavlink_socket.mav.command_long_send(
                    self.__mavlink_socket.target_system,  # target_system
                    self.__mavlink_socket.target_component,
                    mavutil.mavlink.MAV_CMD_IMAGE_STOP_CAPTURE,  # command
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
                    if ack[0] and ack[1] == mavutil.mavlink.MAV_CMD_IMAGE_STOP_CAPTURE:
                        if self.__logger:
                            print('Raspberry pi camera stop_capture complete, iter:', i)
                        return True
                    else:
                        i += 1
                        if i > n_attempts:
                            return False
                else:
                    i += 1
                    if i > n_attempts:
                        return False
        except:
            pass

    def mission_flights(self, mission_file=None, start_event=None, finish_event=None, callback=None):
        """ Autonomous flight by mission """

        # mission_file - mission plan with points
        # start_event \ finish_event - callbacks for begining/ending of flight
        # callback - raises on every point reaching

        if mission_file is None:
            print('Mission flight is missed')
            return
        elif type(mission_file) == str:
            with open(mission_file, encoding='utf-8') as read_file:
                mission_file = json.load(read_file)

        points = []
        try:
            mission_point = mission_file
            for point_ind in mission_point:
                points.append(mission_point[point_ind])

        except:
            pass

        if start_event is not None:
            start_event()

        self.arm()
        time.sleep(0.5)
        self.takeoff()
        time.sleep(8)

        current_point = points.pop(0)
        self.go_to_local_point(x=current_point['x'], y=current_point['y'] * -1, z=current_point['z'] * -1 + 1, yaw=0,
                               vx=0.1, vy=0.1, vz=0.1)
        while True:

            if len(points) == 0 and self.point_reached():
                break

            if self.point_reached():
                if callback is not None:
                    callback()

                current_point = points.pop(0)
                self.go_to_local_point(x=current_point['x'], y=current_point['y'] * -1, z=current_point['z'] * -1 + 1,
                                       yaw=0,
                                       vx=0.1, vy=0.1, vz=0.1)
            time.sleep(0.1)

        if finish_event is not None:
            finish_event()

        self.land()
