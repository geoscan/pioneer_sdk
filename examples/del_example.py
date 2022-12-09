from pioneer_sdk import Pioneer
import time

pioneer = Pioneer()

time.sleep(5)

pioneer.close_connection()
del pioneer

time.sleep(3)
