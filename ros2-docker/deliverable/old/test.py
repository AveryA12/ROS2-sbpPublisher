# sbp_sniff.py
import time, sys
from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client import Framer

PORT = "/dev/ttyACM0"
BAUD = 115200

def main():
    try:
        driver = PySerialDriver(PORT, BAUD)
    except Exception as e:
        print(f"[ERR] Could not open {PORT} @ {BAUD}: {e}")
        sys.exit(1)

    framer = Framer(driver.read, driver.write, verbose=False)
    print("[OK] Listening for SBP… (10s)")
    t0 = time.time()
    count = 0
    seen = {}

    for msg, _ in framer:
        mtype = type(msg).__name__
        seen[mtype] = seen.get(mtype, 0) + 1
        count += 1
        if time.time() - t0 > 10:
            break

    print(f"[DONE] Read {count} messages in ~10s")
    for k in sorted(seen.keys()):
        print(f"{k}: {seen[k]}")

if __name__ == "__main__":
    main()

