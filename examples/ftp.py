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


def lua_script_upload(mavlink_connection):
    """
    Compiles lua code from a source file, and uploads it to the UAV using MAVLink FTP subprotocol.
    Works on both Pioneer and Pioneer Mini.

    :param mavlink_connection: "MAVLink connection object".
    """
    lua_source = "pioneer_led_blink.lua"
    lua_compiled = str(lua.compile(lua_source))
    dest_file_name = "/dev/LuaScript/main.lua"

    ftp_wrapper = mavftp.FtpWrapper(mavlink_connection)
    ftp_wrapper.reset_sessions()
    ftp_wrapper.upload_file(lua_compiled, dest_file_name)


def main():
    mavlink_connection = MavlinkConnectionFactory.make_connected_udp_instantiate()
    drone = Pioneer(mavlink_connection)
    list_directory(mavlink_connection)
    lua_script_upload(mavlink_connection)
    drone.lua_script_control("Start")
    time.sleep(2)
    drone.lua_script_control("Stop")


if __name__ == "__main__":
    main()
