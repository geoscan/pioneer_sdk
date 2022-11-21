# This example demonstrates deleting an object of the Pioneer class.

import time
from pioneer_sdk import Pioneer

pioneer = Pioneer()
time.sleep(5)
pioneer.close_connection()
del pioneer
time.sleep(2)
