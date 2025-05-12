#!/usr/bin/env python3
"""
ups_status_node.py  – publish full UPS telemetry as ups_interfaces/UpsStatus
"""
import os, smbus
import rclpy
from rclpy.node import Node
from ups_interfaces.msg import UpsStatus        # ← the custom msg

ADDR, BUS = 0x2d, smbus.SMBus(1)

class UpsStatusPublisher(Node):
    def __init__(self):
        super().__init__("ups_status_publisher")
        period = self.declare_parameter("period", 2.0).value
        self.pub = self.create_publisher(UpsStatus, "ups/status", 10)
        self.create_timer(period, self.read_and_publish)

    def read_and_publish(self):
        msg = UpsStatus()
        try:
            # VBUS
            vbus = BUS.read_i2c_block_data(ADDR, 0x10, 6)
            msg.vbus_voltage_mv = vbus[0] | vbus[1] << 8
            msg.vbus_current_ma = vbus[2] | vbus[3] << 8
            msg.vbus_power_mw   = vbus[4] | vbus[5] << 8

            # Battery block
            batt = BUS.read_i2c_block_data(ADDR, 0x20, 12)
            msg.battery_voltage_mv = batt[0] | batt[1] << 8
            cur = batt[2] | batt[3] << 8
            if cur > 0x7FFF:
                cur -= 0x10000
            msg.battery_current_ma = cur
            msg.battery_percent = (batt[4] | batt[5] << 8)
            msg.remaining_capacity_mah = batt[6] | batt[7] << 8
            msg.runtime_to_empty_min   = batt[8] | batt[9] << 8 \
                                          if cur < 0 else \
                                          -(batt[10] | batt[11] << 8)

            # Cell voltages
            cells = BUS.read_i2c_block_data(ADDR, 0x30, 8)
            msg.cell_voltage_mv = [
                cells[i] | cells[i+1] << 8 for i in range(0, 8, 2)
            ]

            self.pub.publish(msg)

        except OSError as e:
            self.get_logger().error(f"I²C read failed: {e}")

def main():
    rclpy.init()
    rclpy.spin(UpsStatusPublisher())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
