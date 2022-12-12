#!/usr/bin/python3

import sys
import pioneer_sdk
import time


AP_SSID = "echothere"
AP_PASSWORD = "password"
STA_REMOTE_SSID = "ssid"
STA_REMOTE_PASSWORD = "password"

def debug_sleep():
    time.sleep(2)

def test_set_ap_connect_disconnect(drone):
    set_ssid = False
    set_password = False
    connect = False
    disconnect = True

    if set_ssid:
        if drone.ap_set_ssid(AP_SSID):
            debug_sleep()
            print("Successfully set SSID")

    if set_password:
        if drone.ap_set_password(AP_PASSWORD):
            debug_sleep()
            print("Successfully set password")

    if connect:
        if drone.sta_connect(ssid=STA_REMOTE_SSID, password=STA_REMOTE_PASSWORD):
            print("Successfully connected")
            debug_sleep()

    if disconnect:
        if drone.sta_disconnect():
            print("Successfully disconnected")

def main():
    drone = pioneer_sdk.piosdk.Pioneer()
    test_set_ap_connect_disconnect(drone)
    drone.close_connection()


if __name__ == "__main__":
    main()
