from pioneer_sdk import Pioneer
import time

delta_time = 0.1

r = 0
g = 0
b = 0

height_1 = 0.25
height_2 = 0.5
height_3 = 0.75
#  <------- height_1 ------- height_2 ------- height_3 ------->
#     red             green            blue             white

if __name__ == '__main__':
    pioneer_mini = Pioneer(logger=False)
    curr_time = time.time()
    while True:
        if time.time()-curr_time > delta_time:
            tof_data = pioneer_mini.get_dist_sensor_data()
            print(tof_data)
            if tof_data is not None:
                if tof_data <= height_1:
                    r = 255
                    g = 0
                    b = 0
                elif height_1 < tof_data <= height_2:
                    r = 0
                    g = 255
                    b = 0
                elif height_2 < tof_data <= height_3:
                    r = 0
                    g = 0
                    b = 255
                elif tof_data >= height_3:
                    r = 255
                    g = 255
                    b = 255
                pioneer_mini.led_control(r=r, g=g, b=b)
                curr_time = time.time()
