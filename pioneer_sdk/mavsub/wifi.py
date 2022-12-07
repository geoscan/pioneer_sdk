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

RECEIVE_TIMEOUT_SEC = .2
RECEIVE_ATTEMPTS = 8
RECEIVE_IS_BLOCKING = True
WIFI_CONFIG_MODE_AP = 1
WIFI_CONFIG_MODE_STATION = 2
PASSWORD_MAX_LEN = 64
SSID_MAX_LEN = 32


class WifiConfigApMessage(MAVLink_wifi_config_ap_message):
    """

    The definition stored the library does not comply with the MAVLink 2's
    message definitions. It lacks `mode` and `response` fields.

    This class is a little hack that brings the definition in compliance
    with the modern standard.

    For how-s and why-s, please refer to `common.py` ("pymavlink" library).
    """

    id = 299
    msgname = "WIFI_CONFIG_AP"
    fieldnames = ["ssid", "password", "mode", "response"]
    ordered_fieldnames = ["ssid", "password", "mode", "response"]
    fieldtypes = ["char", "char", "int8_t", "int8_t"]
    fielddisplays_by_name = {}
    fieldenums_by_name = {}
    fieldunits_by_name = {}
    native_format = bytearray("<ccbb", "ascii")
    orders = [0, 1, 2, 3]
    lengths = [1, 1, 1, 1]
    array_lengths = [32, 64, 0, 0]
    crc_extra = 19
    unpacker = struct.Struct("<32s64sbb")
    instance_field = None
    instance_offset = -1

    def __init__(self, ssid, password, mode, response):
        MAVLink_wifi_config_ap_message.__init__(self, ssid, password)
        self.mode = mode
        self.response = response

    def pack(self, mav, force_mavlink1=False):
        packed = self.unpacker.pack(self.ssid, self.password, self.mode,
                                    self.response)
        return self._pack(mav, self.crc_extra, packed,
                          force_mavlink1=force_mavlink1)


class Wifi:
    """
    ETL-encapsulating convenience wrapper
    """

    _FIELD_MOCK_PLACEHOLDER = b"\x00\xff"

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
            password = password[:PASSWORD_MAX_LEN]

        return password

    def send_wifi_config_ap(self, ssid, password):
        """
        Basic boilerplate-reducing wrapper. Performs formattings, and
        ensures consistency w/ the protocol implementation that is being used.
        """
        ssid = self._ssid_ensure_format(ssid)
        password = self._password_ensure_format(password)
        wifi_config_ap_msg = MAVLink_wifi_config_ap_message(ssid, password)
        self.connection.mav.send(wifi_config_ap_msg)

    def send_sta_connect(self, ssid, password):
        self.send_wifi_config_ap(ssid=ssid, password=password)

    def send_sta_disconnect(self, ssid, password):
        self.send_wifi_config_ap(ssid=None, password=None)

    def send_ap_ssid(self, ssid):
        self.send_wifi_config_ap(ssid=ssid, password=None)

    def send_ap_password(self, password):
        self.send_wifi_config_ap(ssid=None, password=password)

    @staticmethod
    def wifi_config_ap_response_is_ok(wifi_config_ap, ssid, password):
        """
        Using the knowledge on the nuances of the protocol implementation,
        infers whether a response produced by the UAV signifies success or
        a failure of the request complementary to the response.
        """
        assert wifi_config_ap is not None
        ssid = Wifi._ssid_ensure_format(ssid)
        password = Wifi._password_ensure_format(password)

        if password != Wifi._FIELD_MOCK_PLACEHOLDER:
            password = hashlib.md5(password).hexdigest()

        is_ok = wifi_config_ap.ssid == ssid \
            and wifi_config_ap.password == password

        return is_ok

    @staticmethod
    def wifi_config_ap_response_connect_is_ok(wifi_config_ap, ssid, password):
        return Wifi.wifi_config_ap_response_is_ok(wifi_config_ap, ssid=ssid,
                                                  password=password)

    @staticmethod
    def wifi_config_ap_response_disconnect_is_ok(wifi_config_ap):
        return Wifi.wifi_config_ap_response_is_ok(wifi_config_ap, ssid=None,
                                                  password=None)

    @staticmethod
    def wifi_config_ap_response_ap_set_ssid_is_ok(wifi_config_ap, ssid):
        return Wifi.wifi_config_ap_response_is_ok(wifi_config_ap, ssid=ssid,
                                                  password=None)

    @staticmethod
    def wifi_config_ap_response_ap_set_password_is_ok(wifi_config_ap, password):
        return Wifi.wifi_config_ap_response_is_ok(wifi_config_ap, ssid=None,
                                                  password=password)

    def ap_set_ssid(self, ssid):
        """
        Sets the AP's SSID
        """
        self.send_ap_ssid(ssid)
        response = self.receive_wifi_config_ap()

        if response is not None:
            # TODO request missing message
            return self.wifi_config_ap_response_ap_set_ssid_is_ok(
                response, ssid=ssid)

        return False

    def ap_set_password(self, password):
        """
        Sets the AP's password
        """
        self.send_ap_password(password)
        response = self.receive_wifi_config_ap()

        if response is not None:
            # TODO request missing message
            return self.wifi_config_ap_response_ap_set_password_is_ok(
                response, password=password)

        return False

    def sta_connect(self, ssid, password):
        """
        Makes an attempt to connect the UAV to a remote AP
        """
        self.send_sta_connect(ssid, password)
        response = self.receive_wifi_config_ap()

        if response is not None:
            # TODO request missing message
            return self.wifi_config_ap_response_connect_is_ok(
                response, ssid=ssid, password=password)

        return False

    def sta_disconnect(self):
        """
        Disconnects the UAV from whatever AP it happens to be connected to
        """
        self.send_sta_disconnect()
        response = self.receive_wifi_config_ap()

        if response is not None:
            # TODO request missing message
            return self.wifi_config_ap_response_disconnect_is_ok(response)
        return False
