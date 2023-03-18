#!/usr/bin/python3

import sys
import pioneer_sdk
import time

# Access point connection credentials (the UAV is the AP)
AP_SSID = "echothere"
AP_PASSWORD = "password"
# Remote access point connection credentials (an access point the UAV attempts to connect to)
STA_REMOTE_SSID = "somessid"
STA_REMOTE_PASSWORD = "password"


def debug_sleep():
    time.sleep(2)


def test(drone):
    # Tweak the values below to test the relevant Wi-Fi functionality
    set_ssid = False
    set_password = False
    connect = True
    disconnect = False
    sta_state = True
    ap_state = True

    if set_ssid:
        if drone.ap_set_ssid(AP_SSID):
            debug_sleep()
            print("- Successfully set SSID")

    if set_password:
        if drone.ap_set_password(AP_PASSWORD):
            debug_sleep()
            print("- Successfully set password")

    if connect:
        if drone.sta_connect(ssid=STA_REMOTE_SSID, password=STA_REMOTE_PASSWORD):
            print("- Successfully connected")
            debug_sleep()

    if disconnect:
        if drone.sta_disconnect():
            print("- Successfully disconnected")

    if sta_state:
        print("-", drone.try_get_sta_status())

    if ap_state:
        print("-", drone.try_get_ap_status())


def main():
    drone = pioneer_sdk.piosdk.Pioneer()
    test(drone)
    drone.close_connection()


if __name__ == "__main__":
    main()
