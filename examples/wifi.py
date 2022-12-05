#!/usr/bin/python3

import sys
import pioneer_sdk


def test_set_ap_name(drone):
    drone.set_wifi_ap_settings("echothere", "nopasswordyet")


def main():
    drone = pioneer_sdk.piosdk.Pioneer()
    test_set_ap_name(drone)


if __name__ == "__main__":
    main()
