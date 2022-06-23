"""
Demonstration of file exchange operations implemented over MAVLink FTP subprotocol.
"""

import pathlib, sys, os, time
from pioneer_sdk.mavsub import ftp as mavftp
from pioneer_sdk.piosdk import MavlinkConnectionFactory, Pioneer
from pioneer_sdk.tools import lua


def list_directory(mavlink_connection):
    ftp_wrapper = mavftp.FtpWrapper(mavlink_connection)
    print(ftp_wrapper.list_directory("/dev/"))


def main():
    mavlink_connection = MavlinkConnectionFactory.make_connected_udp_instantiate()
    drone = Pioneer(mavlink_connection=mavlink_connection, logger=True)
    list_directory(mavlink_connection)
    drone.lua_script_upload("pioneer_led_blink.lua")
    drone.lua_script_control("Start")
    time.sleep(2)
    drone.lua_script_control("Stop")


if __name__ == "__main__":
    main()
