import math
import numpy as np
from std_msgs.msg import Float32MultiArray
import time


def timed(fn):
    def wrapper(self, *args, **kwargs):
        start = time.time()
        ret = fn(self, *args, **kwargs)
        class_name = self.__class__.__name__ if hasattr(self, "__class__") else ""
        print_yellow(
            f"{class_name}.{fn.__name__} takes {(time.time() - start)*1000:.3f} ms"
        )
        return ret

    return wrapper


def print_green(x):
    return print(f"\033[92m {x}\033[00m")


def print_yellow(x):
    return print(f"\033[93m {x}\033[00m")


def print_red(x):
    return print(f"\033[91m {x}\033[00m")


# put this at top-level, outside the class
def change_status(status: str):
    def wrapper(fn):
        def inner(self, *args, **kwargs):
            try:
                self.status = status
                return fn(self, *args, **kwargs)
            finally:
                self.status = "IDLE"

        return inner

    return wrapper
