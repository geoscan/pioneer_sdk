from pioneer_sdk import Pioneer

if __name__ == '__main__':
    pioneer_mini = Pioneer()
    try:
        while True:
            cmd = input()
            if cmd == 'start':
                pioneer_mini.lua_script_control('Start')
            elif cmd == 'stop':
                pioneer_mini.lua_script_control('Stop')
    except KeyboardInterrupt:
        pioneer_mini.lua_script_control('Stop')
        
        pioneer_mini.close_connection()
        del pioneer_mini

