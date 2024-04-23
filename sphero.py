import time
from spherov2 import scanner
from spherov2.sphero_edu import SpheroEduAPI
from spherov2.types import Color
from spherov2.adapter.tcp_adapter import get_tcp_adapter
from spherov2.sphero_edu import SpheroEduAPI

toy = scanner.find_toy(toy_name="SB-6A6C")
# toys = scanner.find_toys()
print(toy)

with SpheroEduAPI(toy) as droid:
    droid.set_main_led(Color(r=0, g=0, b=255))
    droid.set_speed(30)
    time.sleep(2)
    droid.spin(360, 10)
    droid.set_speed(0)
