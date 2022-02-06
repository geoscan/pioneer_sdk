from pioneer_sdk.synchronous import Pioneer
import sys

if __name__ == '__main__':
    pioneer_mini = Pioneer()
    pioneer_mini.lua_script_control('Start')
    while True:
        try:
            pass
        except KeyboardInterrupt:
            pioneer_mini.lua_script_control('Stop')
            sys.exit()

