from pioneer_sdk import VideoStream

if __name__ == '__main__':
    stream = VideoStream()
    while True:
        cmd = input()
        if cmd == 'start':
            stream.start()
        elif cmd == 'stop':
            stream.stop()
