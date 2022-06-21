from pioneer_sdk import Pioneer
import time


if __name__ == '__main__':
    print('start')
    pioneer_mini = Pioneer()
    pioneer_mini.arm()
    pioneer_mini.takeoff()
    t = time.time()
    while True:
        pioneer_mini.set_manual_speed(vx=0, vy=0, vz=1, yaw_rate=0)
        time.sleep(0.05)
        if time.time() - t > 2:
            break
    time.sleep(4)
    t = time.time()
    while True:
        pioneer_mini.set_manual_speed(vx=1, vy=0, vz=0, yaw_rate=0)
        time.sleep(0.05)
        if time.time() - t > 3:
            break
    time.sleep(4)
    pioneer_mini.land()
