"""
Demonstration of file exchange operations implemented over MAVLink FTP subprotocol.
"""

import time
from pioneer_sdk.mavsub import ftp as mavftp
from pioneer_sdk import Pioneer


def main():
    drone = Pioneer(logger=True)
    drone.lua_script_upload("pioneer_led_blink.lua")
    drone.lua_script_control("Start")
    time.sleep(2)
    drone.lua_script_control("Stop")
    drone.close_connection()


if __name__ == "__main__":
    main()
