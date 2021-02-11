from pioneer_sdk import Pioneer
import sys

pioneer_mini = Pioneer()
if __name__ == '__main__':
    pioneer_mini.lua_script_control('Start')
    while True:
        try:
            pass
        except KeyboardInterrupt:
            pioneer_mini.lua_script_control('Stop')
            sys.exit()

