import socket
import time
import can
import cantools
from pprint import pprint
import os

from enum import Enum

bus1 = can.Bus(channel="vcan1", interface='socketcan')

def main():
    db = cantools.database.load_file("hytech_141.dbc")
    dynamics = db.get_message_by_name("INV1_DYNAMICS")

    inverter_on_msg = dynamics.encode({'actual_power_w': 0, 'actual_torque_nm': 0, 'actual_speed_rpm': 6969})

    while(1):
        msg = can.Message(arbitration_id=dynamics.frame_id, is_extended_id=False, data = inverter_on_msg)
        bus1.send(msg)
        
        time.sleep(0.004)

if __name__ == "__main__":
    main()