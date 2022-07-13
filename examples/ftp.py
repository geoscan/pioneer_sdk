"""
Demonstration of file exchange operations implemented over MAVLink FTP subprotocol.
"""

import time
from pioneer_sdk.mavsub import ftp as mavftp
from pioneer_sdk import Pioneer


def list_directory(mavlink_connection):
    ftp_wrapper = mavftp.FtpWrapper(mavlink_connection)
    print(ftp_wrapper.list_directory("/dev/"))


def main():
    drone = Pioneer(logger=True)
    list_directory(drone.mavlink_socket)
    drone.lua_script_upload("pioneer_led_blink.lua")
    drone.lua_script_control("Start")
    time.sleep(2)
    drone.lua_script_control("Stop")


if __name__ == "__main__":
    main()
