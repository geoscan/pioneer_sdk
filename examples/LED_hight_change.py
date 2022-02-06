from pioneer_sdk.synchronous import Pioneer
import time
delta = 0.1
m_to_led = 1000

led_min = 0
led_max = 255

r = led_min
g = led_min
b = led_min

low = 0.25
mid = 0.5
high = 0.75

if __name__ == '__main__':
    pioneer_mini = Pioneer(logger=False)
    curr_time = time.time()
    while True:
        if time.time()-curr_time > delta:
            tof_data = pioneer_mini.get_dist_sensor_data()
            if tof_data is not None:
                if tof_data <= low:
                    r = tof_data*m_to_led
                    g = led_min
                    b = led_min
                elif low < tof_data <= mid:
                    r = (tof_data-low)*m_to_led
                    g = (tof_data-low)*m_to_led
                    b = led_min
                elif mid < tof_data <= high:
                    r = led_min
                    g = (tof_data-mid) * m_to_led
                    b = led_min
                elif tof_data >= high:
                    r = led_min
                    g = led_min
                    b = led_max
                pioneer_mini.led_control(r=r, g=g, b=b)
                curr_time = time.time()
