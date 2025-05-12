# Complete UPS telemetry

uint16  vbus_voltage_mv        # mV
int16   vbus_current_ma        # mA
uint16  vbus_power_mw          # mW

uint16  battery_voltage_mv     # mV
int16   battery_current_ma     # mA
uint8   battery_percent        # 0–100 %
uint16  remaining_capacity_mah # mAh
int16   runtime_to_empty_min   # minutes (negative = time‑to‑full)

uint16[4] cell_voltage_mv      # 4‑cell pack, each in mV