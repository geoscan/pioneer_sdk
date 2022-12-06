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

RECEIVE_TIMEOUT_SEC = .2
RECEIVE_ATTEMPTS = 6
RECEIVE_IS_BLOCKING = True
WIFI_CONFIG_MODE_AP = 1
WIFI_CONFIG_MODE_STATION = 2



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
            msg = self.connection.recv_match(type=message_type, blocking=blocking,
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

    def send_wifi_config_ap(self, ssid, password, mode):
        ssid = bytes(ssid, encoding="ascii")
        password = bytes(password, encoding="ascii")
        wifi_config_ap_msg = WifiConfigApMessage(ssid, password, mode, 0)
        self.connection.mav.send(wifi_config_ap_msg, force_mavlink1=False)
