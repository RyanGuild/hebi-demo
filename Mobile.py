from typing import List
import hebi

from hebi._internal.group import Group
from util import get_lookup
from hebi import Lookup, GroupFeedback
from hebi._internal import lookup, mobile_io
from hebi._internal.ffi._message_types import Feedback
from numpy import ndarray


class Mobile:
    mobile: mobile_io.MobileIO
    def __init__(self, family_name: str, mobile_name: str, enable_logs=False, logfile_name="mobile-logs", logfile_dir="logs", feedback_hz=10.0):
        lookup = get_lookup()
        self.mobile = hebi.util.create_mobile_io(lookup, family=family_name, name=mobile_name)
        self.mobile.update()
        if self.mobile is None:
            print("the mobile device init failed")
            exit(1)
        
    
    def ar_position(self) -> ndarray:
        if self.mobile.position is None:
            print("the mobile device does not support AR position")
            exit(1)
        
        return self.mobile.position

    def ar_orientation(self) -> ndarray:
        if self.mobile.orientation is None:
            print("the mobile device does not support AR orientation")
            exit(1)
        
        return self.mobile.orientation

    def ar_error(self) -> int:
        if self.mobile.orientation is None:
            print("the mobile device does not support AR error statet")
            exit(1)
        return self.mobile.state

    def get_sensor_data(self) -> Feedback:
        if self.mobile.update():
            return self.mobile.get_last_feedback()
        raise Exception("Could not update mobile IO")
    
    def __str__(self) -> str:
        return "Mobile(bat:{}%, {})".format(self.get_sensor_data().battery_level * 100, self.get_sensor_data().sender_id)
