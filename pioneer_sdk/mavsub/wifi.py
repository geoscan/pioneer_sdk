"""
Enables control over the vehicle Wi-Fi configurations over MAVLink protocol.
The API is quite succinct, as it is built around just one message:
`WIFI_CONFIG_AP`.
More: http://mavlink.io/en/messages/common.html#WIFI_CONFIG_AP
Also, for the details of this particular implementation, refer to `./README.md`
"""

# Bring the necessary paths into the scope
import sys
import pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
import struct
from ..generic import Logging
import pymavlink
from pymavlink.dialects.v20.common import MAVLink_wifi_config_ap_message
import pymavlink
import hashlib

RECEIVE_TIMEOUT_SEC = .3
RECEIVE_ATTEMPTS = 20
SEND_RECEIVE_ATTEMPTS = 3
RECEIVE_IS_BLOCKING = True
WIFI_CONFIG_MODE_AP = 1
WIFI_CONFIG_MODE_STATION = 2
PASSWORD_MAX_LEN = 64
SSID_MAX_LEN = 32
WIFI_CMD_REQUEST_MESSAGE_PARAM_2_AP = 0
WIFI_CMD_REQUEST_MESSAGE_PARAM_2_STA = 1
WIFI_CMD_TARGET_COMPONENT = pymavlink.dialects.v20.common.MAV_COMP_ID_UDP_BRIDGE
WIFI_CMD_TARGET_SYSTEM = 1


class Wifi:
    """
    ETL-encapsulating convenience wrapper
    """

    _FIELD_MOCK_PLACEHOLDER = bytes([0x00, 0xFF])

    def __init__(self, connection):
        self.connection = connection

    def receive_wifi_config_ap(self, blocking=RECEIVE_IS_BLOCKING,
            timeout=RECEIVE_TIMEOUT_SEC):
        """
        Makes an attempt to receive a WIFI_CONFIG_AP message using both
        socket buffer, and the connection object's internal cache
        """
        msg = None

        for _ in range(RECEIVE_ATTEMPTS):
            message_type = "WIFI_CONFIG_AP"
            msg = self.connection.recv_match(type=message_type,
                                             blocking=blocking,
                                             timeout=timeout)

            if not msg:
                try:
                    msg = self.connection.messages.pop(message_type)

                    break
                except KeyError:
                    Logging.warning(__file__, Wifi, Wifi.receive_wifi_config_ap,
                                    "failed to receive")
                    msg = None

        return msg

    @staticmethod
    def _ssid_ensure_format(ssid):
        """
        Ensures that SSID is of correct binary format.

        The current implementation uses a set of special sequences. Also, there
        are subject area limitations on SSID and password lengths.
        """
        if ssid is None:
            ssid = Wifi._FIELD_MOCK_PLACEHOLDER
        else:
            ssid = bytes(ssid, encoding="ascii")
            ssid = ssid[:SSID_MAX_LEN]

        return ssid

    @staticmethod
    def _password_ensure_format(password):
        """
        Ensures that password is of correct binary format.

        The current implementation uses a set of special sequences. Also, there
        are subject area limitations on SSID and password lengths.
        """
        if password is None:
            password = Wifi._FIELD_MOCK_PLACEHOLDER
        else:
            password = bytes(password, encoding="ascii")
            password = password[:PASSWORD_MAX_LEN]

        return password

    def _send_wifi_config_ap(self, ssid, password):
        """
        Basic boilerplate-reducing wrapper. Performs formattings, and
        ensures consistency w/ the protocol implementation that is being used.
        """
        ssid = self._ssid_ensure_format(ssid)
        password = self._password_ensure_format(password)
        wifi_config_ap_msg = MAVLink_wifi_config_ap_message(ssid, password)
        self.connection.mav.send(wifi_config_ap_msg)

    def send_sta_connect(self, ssid, password):
        self._send_wifi_config_ap(ssid=ssid, password=password)

    def send_sta_disconnect(self):
        self._send_wifi_config_ap(ssid=None, password=None)

    def send_ap_ssid(self, ssid):
        self._send_wifi_config_ap(ssid=ssid, password=None)

    def send_ap_password(self, password):
        self._send_wifi_config_ap(ssid=None, password=password)

    @staticmethod
    def _wifi_config_ap_response_is_ok(wifi_config_ap, ssid, password):
        """
        Using the knowledge on the nuances of the protocol implementation,
        infers whether a response produced by the UAV signifies success or
        a failure of the request complementary to the response.
        """
        assert wifi_config_ap is not None
        received_ssid = bytes(wifi_config_ap.ssid, encoding="ascii")
        received_password = wifi_config_ap.password
        use_password = (password is not None)
        use_ssid = (ssid is not None)
        is_ok = True

        if use_ssid:
            ssid = Wifi._ssid_ensure_format(ssid)
            ssid_ok = (received_ssid == ssid)

            if not ssid_ok:
                Logging.warning(__file__, Wifi,
                                f"received SSID {received_ssid} " \
                                f"is not equal to expected SSID {ssid}")

            is_ok = is_ok and ssid_ok

        if use_password:
            password = Wifi._password_ensure_format(password)

            if password != Wifi._FIELD_MOCK_PLACEHOLDER:
                password = hashlib.md5(password).hexdigest()

            password_ok = (password == received_password)

            if not password_ok:
                Logging.warning(__file__, Wifi, f"received password digest " \
                                f"{received_password} is not equal to expected " \
                                f"password digest {password}")

            is_ok = is_ok and password_ok

        return is_ok

    @staticmethod
    def _wifi_config_ap_response_connect_is_ok(wifi_config_ap, ssid, password):
        return Wifi._wifi_config_ap_response_is_ok(wifi_config_ap, ssid=ssid,
                                                  password=password)

    @staticmethod
    def _wifi_config_ap_response_disconnect_is_ok(wifi_config_ap):
        return Wifi._wifi_config_ap_response_is_ok(wifi_config_ap, ssid=None,
                                                  password=None)

    @staticmethod
    def _wifi_config_ap_response_ap_set_ssid_is_ok(wifi_config_ap, ssid):
        return Wifi._wifi_config_ap_response_is_ok(wifi_config_ap, ssid=ssid,
                                                  password=None)

    @staticmethod
    def _wifi_config_ap_response_ap_set_password_is_ok(wifi_config_ap, password):
        return Wifi._wifi_config_ap_response_is_ok(wifi_config_ap, ssid=None,
                                                  password=password)

    def ap_set_ssid(self, ssid):
        """
        Sets the AP's SSID
        """

        for _ in range(SEND_RECEIVE_ATTEMPTS):
            self.send_ap_ssid(ssid)
            response = self.receive_wifi_config_ap()

            if response is not None:
                ret = self._wifi_config_ap_response_ap_set_ssid_is_ok(
                    response, ssid=ssid)

                if not ret:
                    Logging.warning(Wifi, "failed to set SSID", response)

                return ret

        Logging.error(Wifi, "failed to receive in", SEND_RECEIVE_ATTEMPTS,
                        "attempts")

        return False

    def ap_set_password(self, password):
        """
        Sets the AP's password
        """
        for _ in range(SEND_RECEIVE_ATTEMPTS):
            self.send_ap_password(password)
            response = self.receive_wifi_config_ap()

            if response is not None:
                ret = self._wifi_config_ap_response_ap_set_password_is_ok(
                    response, password=password)

                if not ret:
                    Logging.warning(Wifi, "failed to set password")

                return ret

        Logging.error(Wifi, "failed to receive in", SEND_RECEIVE_ATTEMPTS,
                        "attempts")

        return False

    def sta_connect(self, ssid, password):
        """
        Makes an attempt to connect the UAV to a remote AP
        """
        for _ in range(SEND_RECEIVE_ATTEMPTS):
            self.send_sta_connect(ssid, password)
            response = self.receive_wifi_config_ap()

            if response is not None:
                ret = self._wifi_config_ap_response_connect_is_ok(
                    response, ssid=ssid, password=password)

                if not ret:
                    Logging.warning(Wifi, "failed to connect to an AP")

                return ret

        Logging.warning(Wifi, "failed to receive in", SEND_RECEIVE_ATTEMPTS,
                        "attempts")

        return False

    def sta_disconnect(self):
        """
        Disconnects the UAV from whatever AP it happens to be connected to
        """
        for _ in range(SEND_RECEIVE_ATTEMPTS):
            self.send_sta_disconnect()
            response = self.receive_wifi_config_ap()

            if response is not None:
                ret = self._wifi_config_ap_response_disconnect_is_ok(response)

                if not ret:
                    Logging.warning(Wifi, "failed to disconnect from an AP")

                return ret

        Logging.warning(Wifi, "failed to receive in", SEND_RECEIVE_ATTEMPTS,
                        "attempts")

        return False

    def _try_get_status(self, param2=WIFI_CMD_REQUEST_MESSAGE_PARAM_2_AP):
        """
        Makes an attempt to acquire the STA's current SSID and password.
        Please note that the password will be represented as an MD5 stringified
        digest.

        On success, returns (SSID, password) pair. (None, None) otherwise.
        """
        msgid = pymavlink.dialects.v20.common.MAVLINK_MSG_ID_WIFI_CONFIG_AP
        command = pymavlink.dialects.v20.common.MAV_CMD_REQUEST_MESSAGE
        ssid = None
        password = None

        for i in range(SEND_RECEIVE_ATTEMPTS):
            self.connection.mav.command_long_send(
                target_system=WIFI_CMD_TARGET_SYSTEM,
                target_component=WIFI_CMD_TARGET_COMPONENT, command=command,
                confirmation=i, param1=msgid, param2=param2, param3=0, param4=0,
                param5=0, param6=0, param7=0)
            response = self.receive_wifi_config_ap()

            if response is not None:
                ssid = response.ssid
                password = response.password

                break

        if ssid is None:
            Logging.warning(Wifi,
                "failed to receive requested WIFI_CONFIG_AP message in ",
                SEND_RECEIVE_ATTEMPTS, "attempts")

        return ssid, password

    def try_get_sta_status(self):
        return self._try_get_status(param2=WIFI_CMD_REQUEST_MESSAGE_PARAM_2_STA)

    def try_get_ap_status(self):
        return self._try_get_status(param2=WIFI_CMD_REQUEST_MESSAGE_PARAM_2_AP)
