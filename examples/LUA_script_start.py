from pioneer_sdk import Pioneer

if __name__ == '__main__':
    pioneer_mini = Pioneer()
    try:
        while True:
            cmd = input()
            match cmd:
                case 'start':
                    pioneer_mini.lua_script_control('Start')
                case 'stop':
                    pioneer_mini.lua_script_control('Stop')
    except KeyboardInterrupt:
        pioneer_mini.lua_script_control('Stop')

