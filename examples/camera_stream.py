from pioneer_sdk import VideoStream

if __name__ == '__main__':
    stream = VideoStream()
    while True:
        cmd = input()
        match cmd:
            case 'start':
                stream.start()
            case 'stop':
                stream.stop()
