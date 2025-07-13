import time, serial.tools.list_ports

# Optional: force reset by toggling DTR
try:
    with serial.Serial("COM6", 1200) as s:
        s.setDTR(False)
        time.sleep(0.2)
        s.setDTR(True)
except Exception:
    pass

time.sleep(2)
