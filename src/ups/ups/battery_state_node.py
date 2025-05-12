#!/usr/bin/env python3
#
# battery_state_node.py – publish UPS status as sensor_msgs/BatteryState
#

import time
import os
import smbus
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

ADDR     = 0x2d
LOW_CELL = 3150
BUS      = smbus.SMBus(1)

class UpsBatteryPublisher(Node):
    def __init__(self):
        super().__init__("ups_battery_publisher")

        period = self.declare_parameter("period", 2.0).get_parameter_value().double_value
        self.pub  = self.create_publisher(BatteryState, "ups/battery_state", 10)
        self.timer = self.create_timer(period, self.read_and_publish)

        self.low_counter = 0

    # ────────────────────────────────────────────────────────────────
    def read_and_publish(self):
        msg = BatteryState()
        try:
            # 1 – charge / discharge status
            st = BUS.read_i2c_block_data(ADDR, 0x02, 1)[0]
            msg.power_supply_status = (
                BatteryState.POWER_SUPPLY_STATUS_CHARGING  if st & 0x80 else
                BatteryState.POWER_SUPPLY_STATUS_DISCHARGING if st & 0x20 else
                BatteryState.POWER_SUPPLY_STATUS_FULL       if st & 0x40 else
                BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
            )

            # 2 – VBUS numbers (not essential but useful)
            vbus = BUS.read_i2c_block_data(ADDR, 0x10, 6)
            msg.voltage         = (vbus[0] | vbus[1] << 8) / 1000.0      # V
            current_raw         =  BUS.read_i2c_block_data(ADDR, 0x20, 4)
            current             = (current_raw[2] | current_raw[3] << 8)
            if current > 0x7FFF:
                current -= 0x10000
            msg.current         = current / 1000.0                       # A

            batt = BUS.read_i2c_block_data(ADDR, 0x20, 12)
            msg.percentage      = ((batt[4] | batt[5] << 8) / 100.0)     # 0–1
            msg.capacity        = (batt[6] | batt[7] << 8) / 1000.0      # Ah

            # Warn if *any* cell goes below LOW_CELL
            cells = BUS.read_i2c_block_data(ADDR, 0x30, 8)
            cell_mv = [cells[i] | cells[i+1] << 8 for i in range(0, 8, 2)]
            if any(v < LOW_CELL for v in cell_mv):
                self.low_counter += 1
                if self.low_counter >= 30:
                    self.get_logger().fatal("Battery critically low, shutting down")
                    # Optional shutdown logic …
                else:
                    self.get_logger().warn(
                        f"Low cell voltage detected – will shut down in {(60-2*self.low_counter):d}s")
            else:
                self.low_counter = 0

            self.pub.publish(msg)

        except OSError as e:
            self.get_logger().error(f"I2C read failed: {e}")

# ────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = UpsBatteryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
