from pymavlink import mavutil
from .mavsub import ftp as mavftp
from .tools import lua
import threading
import socket
import sys
import time


class Pioneer:
    mav_result = {
        -1: 'SEND_TIMEOUT',
        0: 'ACCEPTED',
        1: 'TEMPORARILY_REJECTED',
        2: 'DENIED',
        3: 'UNSUPPORTED',
        4: 'FAILED',
        5: 'IN_PROGRESS',
        6: 'CANCELLED'
    }
    autopilot_state = {
        0: 'ROOT',
        1: 'DISARMED',
        2: 'IDLE',
        3: 'TEST_ACTUATION',
        4: 'TEST_PARACHUTE',
        5: 'TEST_ENGINE',
        6: 'PARACHUTE',
        7: 'WAIT_FOR_LANDING',
        8: 'LANDED',
        9: 'CATAPULT',
        10: 'PREFLIGHT',
        11: 'ARMED',
        12: 'TAKEOFF',
        13: 'WAIT_FOR_GPS',
        14: 'WIND_MEASURE',
        15: 'MISSION',
        16: 'ASCEND',
        17: 'DESCEND',
        18: 'RTL',
        19: 'UNCONDITIONAL_RTL',
        20: 'MANUAL_HEADING',
        21: 'MANUAL_ROLL',
        22: 'MANUAL_SPEED',
        23: 'LANDING',
        24: 'ON_DEMAND'
    }

    def __init__(self, name='pioneer', ip='192.168.4.1', mavlink_port=8001, connection_method=2,
                 device='/dev/serial0', baud=115200,
                 logger=True, log_connection=True):

        self.name = name

        self._is_connected = False
        self._is_connected_timeout = 1
        self._last_msg_time = time.time() - self._is_connected_timeout

        self._heartbeat_timeout = 1
        self._heartbeat_send_time = time.time() - self._heartbeat_timeout

        self._mavlink_send_timeout = 0.5
        self._mavlink_send_long_timeout = 1
        self._mavlink_send_number = 10

        self._logger = logger
        self._log_connection = log_connection

        self._point_seq = None
        self._point_reached = False

        self._cur_state = None
        self._preflight_state = dict(BatteryLow=None,
                                     NavSystem=None,
                                     Area=None,
                                     Attitude=None,
                                     RcExpected=None,
                                     RcMode=None,
                                     RcUnexpected=None,
                                     UavStartAllowed=None)
        self.mavlink_socket = None
        try:
            match connection_method:
                case 0:
                    self.mavlink_socket = mavutil.mavlink_connection('udpin:%s:%s' % (ip, mavlink_port))
                case 1:
                    self.mavlink_socket = mavutil.mavlink_connection(device=device, baud=baud)
                case 2:
                    self.mavlink_socket = mavutil.mavlink_connection('udpout:%s:%s' % (ip, mavlink_port))
                case _:
                    print(f"Unknown connection method: {connection_method}")
        except socket.error as e:
            print('Connection error. Can not connect to drone')
            sys.exit()

        self.msg_archive = dict()
        self.wait_msg = dict()

        self._message_handler_thread = threading.Thread(target=self._message_handler, daemon=True)
        self._message_handler_thread.daemon = True
        self._message_handler_thread.start()

        if self._log_connection:
            self.log(msg_type='connection', msg='Connecting to drone...')

    def log(self, msg, msg_type=None):
        if msg_type is None:
            print(f"[{self.name}] {msg}")
        else:
            print(f"[{self.name}] <{msg_type}> {msg}")

    def connected(self):
        return self._is_connected

    def set_logger(self, value: bool = True):
        self._logger = value

    def set_log_connection(self, value: bool = True):
        self._log_connection = value

    def _send_heartbeat(self):
        self.mavlink_socket.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                               mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        self._heartbeat_send_time = time.time()

    def _receive_heartbeat(self, msg):
        try:
            if msg._header.srcComponent == 1:
                custom_mode = msg.custom_mode
                custom_mode_buf = format(custom_mode, "032b")
                status_autopilot = custom_mode_buf[24:]
                self._cur_state = Pioneer.autopilot_state[int(status_autopilot, 2)]
        except Exception:
            pass

    def _message_handler(self):
        while True:
            if time.time() - self._heartbeat_send_time >= self._heartbeat_timeout:
                self._send_heartbeat()
            msg = self.mavlink_socket.recv_msg()
            if msg is not None:
                self._last_msg_time = time.time()
                if not self._is_connected:
                    self._is_connected = True
                    if self._log_connection:
                        self.log(msg_type='connection', msg='CONNECTED')
                match msg.get_type():
                    case 'HEARTBEAT':
                        self._receive_heartbeat(msg)
                    case 'MISSION_ITEM_REACHED':
                        if self._point_seq is None:
                            self._point_seq = msg.seq
                        if msg.seq > self._point_seq:
                            self._point_reached = True
                            if self._logger:
                                self.log(msg_type='POINT_REACHED', msg=f'point_id: {msg.seq}')
                        self._point_seq = msg.seq
                    case 'COMMAND_ACK':
                        msg._type += f'_{msg.command}'
                        # if msg.command == 400 and msg.result_param2 is not None:
                        #     print("PREFLIGHT_STATE")
                        #     self._preflight_state.update(BatteryLow=msg.result_param2 & 0b00000001)
                        #     self._preflight_state.update(NavSystem=msg.result_param2 & 0b00000010)
                        #     self._preflight_state.update(Area=msg.result_param2 & 0b00000100)
                        #     self._preflight_state.update(Attitude=msg.result_param2 & 0b00001000)
                        #     self._preflight_state.update(RcExpected=msg.result_param2 & 0b00010000)
                        #     self._preflight_state.update(RcMode=msg.result_param2 & 0b00100000)
                        #     self._preflight_state.update(RcUnexpected=msg.result_param2 & 0b01000000)
                        #     self._preflight_state.update(UavStartAllowed=msg.result_param2 & 0b10000000)
                if msg.get_type() in self.wait_msg:
                    self.wait_msg[msg.get_type()].set()
                self.msg_archive.update({msg.get_type(): {'msg': msg, 'is_read': threading.Event()}})
            elif self._is_connected and (time.time() - self._last_msg_time > self._is_connected_timeout):
                self._is_connected = False
                if self._log_connection:
                    self.log(msg_type='connection', msg='DISCONNECTED')

    def _send_command_long(self, command_name, command, param1: float = 0, param2: float = 0, param3: float = 0,
                           param4: float = 0, param5: float = 0, param6: float = 0, param7: float = 0,
                           target_system=None, target_component=None, sending_log_msg='sending...'):
        if self._logger:
            self.log(msg_type=command_name, msg=sending_log_msg)
        if target_system is None:
            target_system = self.mavlink_socket.target_system
        if target_component is None:
            target_component = self.mavlink_socket.target_component
        if_send = True
        in_progress = False
        confirm = 0
        msg_to_wait = f'COMMAND_ACK_{command}'
        event = threading.Event()
        self.wait_msg[msg_to_wait] = event
        try:
            while True:
                if if_send:
                    self.mavlink_socket.mav.command_long_send(target_system, target_component, command, confirm,
                                                              param1, param2, param3, param4, param5, param6, param7)
                    confirm += 1

                if in_progress:
                    event.wait(self._mavlink_send_long_timeout)
                else:
                    event.wait(self._mavlink_send_timeout)

                if event.is_set():
                    if_send = False
                    msg = self.msg_archive[msg_to_wait]['msg']
                    self.msg_archive[msg_to_wait]['is_read'].set()
                    if msg.result == 5:  # IN_PROGRESS
                        in_progress = True
                        if self._logger:
                            self.log(msg_type=command_name,
                                     msg=Pioneer.mav_result[msg.result])
                        event.clear()
                    else:
                        if self._logger:
                            self.log(msg_type=command_name,
                                     msg=Pioneer.mav_result[msg.result])
                        return msg.result in [0, 2]
                else:
                    if_send = True
                if confirm >= self._mavlink_send_number:
                    if self._logger:
                        self.log(msg_type=command_name, msg=Pioneer.mav_result[-1])
                    return False
        finally:
            if msg_to_wait in self.wait_msg:
                del self.wait_msg[msg_to_wait]

    def arm(self):
        return self._send_command_long(command_name='ARM', command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                       param1=1)

    def disarm(self):
        return self._send_command_long(command_name='DISARM', command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                       param1=0)

    def takeoff(self):
        return self._send_command_long(command_name='TAKEOFF', command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)

    def land(self):
        return self._send_command_long(command_name='LAND', command=mavutil.mavlink.MAV_CMD_NAV_LAND)

    def lua_script_upload(self, lua_source):
        """
        Compiles lua code from a source file, and uploads it to the UAV using MAVLink FTP subprotocol.
        Works on both Pioneer and Pioneer Mini.

        :param mavlink_connection: "MAVLink connection object".
        """
        lua_compiled = str(lua.compile(lua_source))
        dest_file_name = "/dev/LuaScript/main.lua"

        ftp_wrapper = mavftp.FtpWrapper(self.mavlink_socket)
        ftp_wrapper.reset_sessions()
        ftp_wrapper.upload_file(lua_compiled, dest_file_name)

    def lua_script_control(self, state='Stop'):
        states = {'Stop': 0, 'Start': 1}
        if state not in states:
            raise ValueError(
                f"Argument 'state' must be one of the following values: {*states.keys(),}. But your value is '{state}'.")
        return self._send_command_long(command_name='LUA', command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                       target_component=25, param1=states[state],
                                       sending_log_msg=f'sending {state} ...')

    def led_control(self, led_id=255, r=0, g=0, b=0):  # 255 all led
        if led_id not in [255, 0, 1, 2, 3]:
            raise ValueError(
                f"Argument 'led_id' must have one of the following values: 0, 1, 2, 3, 255. But your value is {led_id}.")
        if r < 0 or r > 255 or g < 0 or g > 255 or b < 0 or b > 255:
            raise ValueError(
                f"Arguments 'r', 'g', 'b' must have value in [0, 255]. But your values is r={r}, g={g}, b={b}.")
        return self._send_command_long(command_name='LED', command=mavutil.mavlink.MAV_CMD_USER_1,
                                       param1=led_id, param2=r, param3=g, param4=b,
                                       sending_log_msg=f'sending LED_ID={led_id} RGB=({r}, {g}, {b}) ...')

    def raspberry_poweroff(self):
        return self._send_command_long(command_name='RPi_POWEROFF',
                                       command=mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                                       target_component=42)

    def raspberry_reboot(self):
        return self._send_command_long(command_name='RPi_REBOOT',
                                       command=mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                                       target_component=43)

    def reboot_board(self):
        return self._send_command_long(command_name='REBOOT_BOARD',
                                       command=mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                                       target_component=1)

    def raspberry_led_custom(self, mode=1, timer=0, color1=(0, 0, 0), color2=(0, 0, 0)):
        """ Change LED color for device with raspberry pi"""
        param2 = (((color1[0] << 8) | color1[1]) << 8) | color1[2]
        param3 = (((color2[0] << 8) | color2[1]) << 8) | color2[2]
        param5 = mode
        param6 = timer
        return self._send_command_long('RPi_LED', mavutil.mavlink.MAV_CMD_USER_3, param2=param2, param3=param3,
                                       param5=param5, param6=param6, target_system=0, target_component=0)

    def _send_position_target_local_ned(self, command_name, coordinate_system, mask=0b0000_11_0_111_111_111, x=0, y=0,
                                        z=0, vx=0, vy=0, vz=0, afx=0, afy=0, afz=0, yaw=0, yaw_rate=0,
                                        target_system=None, target_component=None):
        if target_system is None:
            target_system = self.mavlink_socket.target_system
        if target_component is None:
            target_component = self.mavlink_socket.target_component
        event = threading.Event()
        self.wait_msg['POSITION_TARGET_LOCAL_NED'] = event
        try:
            for confirm in range(self._mavlink_send_number):
                self.mavlink_socket.mav.set_position_target_local_ned_send(0, target_system, target_component,
                                                                           coordinate_system,
                                                                           mask, x, y, z, vx, vy, vz, afx, afy, afz,
                                                                           yaw, yaw_rate)
                event.wait(self._mavlink_send_timeout)
                if event.is_set():
                    msg = self.msg_archive['POSITION_TARGET_LOCAL_NED']['msg']
                    self.msg_archive['POSITION_TARGET_LOCAL_NED']['is_read'].set()
                    if msg.type_mask == mask:
                        if self._logger:
                            self.log(msg_type=command_name, msg=Pioneer.mav_result[0])
                    else:
                        if self._logger:
                            self.log(msg_type=command_name, msg=Pioneer.mav_result[2])
                    return True
            if self._logger:
                self.log(msg_type=command_name, msg=Pioneer.mav_result[-1])
            return False
        finally:
            if 'POSITION_TARGET_LOCAL_NED' in self.wait_msg:
                del self.wait_msg['POSITION_TARGET_LOCAL_NED']

    def go_to_local_point(self, x, y, z, yaw):
        """ Flight to point in the current navigation system's coordinate frame """
        cmd_name = 'GO_TO_POINT'
        if self._logger:
            self.log(msg_type=cmd_name, msg=f'sending point {{LOCAL, x:{x}, y:{y}, z:{z}, yaw:{yaw}}} ...')
        mask = 0b0000_10_0_111_111_000  # _ _ _ _ yaw_rate yaw   force_set   afz afy afx   vz vy vx   z y x
        x, y, z = y, x, -z  # ENU coordinates to NED coordinates
        self._point_reached = False
        return self._send_position_target_local_ned(command_name=cmd_name,
                                                    coordinate_system=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                                    mask=mask, x=x, y=y, z=z, yaw=yaw)

    def go_to_local_point_body_fixed(self, x, y, z, yaw):
        """ Flight to point relative to the current position """

        cmd_name = 'GO_TO_POINT'
        if self._logger:
            self.log(msg_type=cmd_name, msg=f'sending point {{BODY_FIX, x:{x}, y:{y}, z:{z}, yaw:{yaw}}} ...')
        mask = 0b0000_10_0_111_111_000  # _ _ _ _ yaw_rate yaw   force_set   afz afy afx   vz vy vx   z y x
        x, y, z = y, x, -z  # ENU coordinates to NED coordinates
        self._point_reached = False
        return self._send_position_target_local_ned(command_name=cmd_name,
                                                    coordinate_system=mavutil.mavlink.MAV_FRAME_BODY_FRD,
                                                    mask=mask, x=x, y=y, z=z, yaw=yaw)

    def set_manual_speed(self, vx, vy, vz, yaw_rate):
        """ Set manual speed """
        cmd_name = 'MANUAL_SPEED'
        if self._logger:
            self.log(msg_type=cmd_name,
                     msg=f'sending speed {{LOCAL, vx:{vx}, vy:{vy}, vz:{vz}, yaw_rate:{yaw_rate}}} ...')
        mask = 0b0000_01_0_111_000_111  # _ _ _ _ yaw_rate yaw   force_set   afz afy afx   vz vy vx   z y x
        vx, vy, vz = vy, vx, -vz  # ENU coordinates to NED coordinates
        return self._send_position_target_local_ned(command_name=cmd_name,
                                                    coordinate_system=mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                                    mask=mask, vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate)

    def set_manual_speed_body_fixed(self, vx, vy, vz, yaw_rate):
        """ Set manual speed in an external coordinate frame (that of a currently used local navigation system) """
        cmd_name = 'MANUAL_SPEED'
        if self._logger:
            self.log(msg_type=cmd_name,
                     msg=f'sending speed {{BODY_FIX, vx:{vx}, vy:{vy}, vz:{vz}, yaw_rate:{yaw_rate}}} ...')
        mask = 0b0000_01_0_111_000_111  # _ _ _ _ yaw_rate yaw   force_set   afz afy afx   vz vy vx   z y x
        vx, vy, vz = vy, vx, -vz  # ENU coordinates to NED coordinates
        return self._send_position_target_local_ned(command_name=cmd_name,
                                                    coordinate_system=mavutil.mavlink.MAV_FRAME_BODY_FRD,
                                                    mask=mask, vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate)

    def point_reached(self):
        if self._point_reached:
            self._point_reached = False
            return True
        else:
            return False

    def get_local_position_lps(self, get_last_received: bool = False):
        """ Get position LPS"""
        if 'LOCAL_POSITION_NED' in self.msg_archive:
            msg_dict = self.msg_archive['LOCAL_POSITION_NED']
            msg = msg_dict['msg']
            if not msg_dict['is_read'].is_set() or (msg_dict['is_read'].is_set() and get_last_received):
                msg_dict['is_read'].set()
                return [msg.x, msg.y, msg.z]
            else:
                return None
        else:
            return None

    def get_dist_sensor_data(self, get_last_received: bool = False):
        """ Get altitude from TOF"""
        if 'DISTANCE_SENSOR' in self.msg_archive:
            msg_dict = self.msg_archive['DISTANCE_SENSOR']
            if not msg_dict['is_read'].is_set() or (msg_dict['is_read'].is_set() and get_last_received):
                msg_dict['is_read'].set()
                return msg_dict['msg'].current_distance/100
            else:
                return None
        else:
            return None

    def get_optical_data(self, get_last_received: bool = False):
        """ Returning data from OPT """
        if 'OPTICAL_FLOW_RAD' in self.msg_archive:
            msg_dict = self.msg_archive['OPTICAL_FLOW_RAD']
            msg = msg_dict['msg']
            if not msg_dict['is_read'].is_set() or (msg_dict['is_read'].is_set() and get_last_received):
                msg_dict['is_read'].set()
                return {'integration_time_us': msg.integration_time_us, 'integrated_x': msg.integrated_x,
                        'integrated_y': msg.integrated_y, 'distance': msg.distance,
                        'integrated_xgyro': msg.integrated_xgyro, 'integrated_ygyro': msg.integrated_ygyro,
                        'integrated_zgyro': msg.integrated_zgyro, 'temperature': msg.temperature,
                        'quality': msg.quality}
            else:
                return None
        else:
            return None

    def get_battery_status(self, get_last_received: bool = False):
        """ Returning battery remaining voltage """
        if 'BATTERY_STATUS' in self.msg_archive:
            msg_dict = self.msg_archive['BATTERY_STATUS']
            if not msg_dict['is_read'].is_set() or (msg_dict['is_read'].is_set() and get_last_received):
                msg_dict['is_read'].set()
                return msg_dict['msg'].voltages[0] / 100
            else:
                return None
        else:
            return None

    def get_preflight_state(self):
        return self._preflight_state

    def get_autopilot_state(self):
        return self._cur_state

    def get_autopilot_version(self):
        """ Returning autopilot version """
        event = threading.Event()
        self.wait_msg['AUTOPILOT_VERSION'] = event
        try:
            for confirm in range(self._mavlink_send_number):
                self.mavlink_socket.mav.command_long_send(self.mavlink_socket.target_system, 1,
                                                          mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, confirm,
                                                          mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION,
                                                          0, 0, 0, 0, 0, 0)
                event.wait(self._mavlink_send_timeout)
                if event.is_set():
                    msg = self.msg_archive['AUTOPILOT_VERSION']['msg']
                    self.msg_archive['AUTOPILOT_VERSION']['is_read'].set()
                    if self._logger:
                        self.log(msg_type='AUTOPILOT_VERSION',
                                 msg=str([msg.flight_sw_version, msg.board_version, msg.flight_custom_version]))
                    return [msg.flight_sw_version, msg.board_version, msg.flight_custom_version]
            if self._logger:
                self.log(msg_type='AUTOPILOT_VERSION', msg='send timeout')
            return None, None
        finally:
            if 'AUTOPILOT_VERSION' in self.wait_msg:
                del self.wait_msg['AUTOPILOT_VERSION']

    def send_rc_channels(self, channel_1=0xFF, channel_2=0xFF, channel_3=0xFF, channel_4=0xFF,
                         channel_5=0xFF, channel_6=0xFF, channel_7=0xFF, channel_8=0xFF):
        """ RC unit signal send to drone """
        # channel_1 = throttle channel value
        # channel_2 = yaw channel value
        # channel_3 = pitch channel value
        # channel_4 = roll channel value
        # channel_5 = mode. 2000 - program

        self.mavlink_socket.mav.rc_channels_override_send(self.mavlink_socket.target_system,
                                                          self.mavlink_socket.target_component, channel_1,
                                                          channel_2, channel_3, channel_4, channel_5, channel_6,
                                                          channel_7, channel_8)

    def raspberry_start_capture(self, interval=0.1, total_images=0, sequence_number=0):
        """ Raspberry pi camera start capturing """
        # param2 interval - delay between each frame
        # param3 total_images - number of images to capture. 0 = do capture till stop_capture is called
        # param4 sequence_number - if total_images = 1 increment after each frame, else use 0
        return self._send_command_long(command_name='RPi_START_CAPTURE',
                                       command=mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE,
                                       param2=interval, param3=total_images, param4=sequence_number)

    def raspberry_stop_capture(self):
        """ End of Raspberry pi capture """
        return self._send_command_long(command_name='RPi_START_CAPTURE',
                                       command=mavutil.mavlink.MAV_CMD_IMAGE_STOP_CAPTURE)
