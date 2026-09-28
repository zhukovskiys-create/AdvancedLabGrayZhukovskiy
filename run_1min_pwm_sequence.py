import csv
import re
import serial
import time

PORT = 'COM3'
BAUD_RATE = 9600
OUTPUT_FILE = 'one_minute_heat_run.csv'
LEVELS = [(25, 20), (50, 20), (75, 20)]
LINE_PATTERN = re.compile(
    r"^Temperature \(C\):\s*([0-9\.-]+),\s*Time \(s\):\s*([0-9\.-]+),\s*PWM:\s*([0-9]+),\s*Heat/Cool:\s*([01])"
)

rows = []

try:
    ser = serial.Serial(PORT, BAUD_RATE, timeout=0.5)
    time.sleep(2)
    ser.reset_input_buffer()
    ser.flush()
    print(f"Connected to {PORT} at {BAUD_RATE} baud")

    for percent, seconds in LEVELS:
        pwm_value = round((percent / 100.0) * 255)
        command = f"SET PWM {pwm_value} DIR HEAT\n".encode('utf-8')
        ser.write(command)
        print(f"Set HEAT to {percent}% (PWM {pwm_value}) for {seconds}s")

        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            while ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                match = LINE_PATTERN.match(line)
                if not match:
                    continue
                temp_c = float(match.group(1))
                time_s = float(match.group(2))
                pwm = int(match.group(3))
                heat_cool = int(match.group(4))
                rows.append((time_s, temp_c, pwm, heat_cool))
                print(f"{time_s:.2f}s, {temp_c:.2f}C, PWM={pwm}, Heat/Cool={heat_cool}")
            time.sleep(0.1)

    with open(OUTPUT_FILE, 'w', newline='') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(['time_s', 'temperature_C', 'pwm', 'heat_cool'])
        writer.writerows(rows)

    print(f"Saved {len(rows)} samples to {OUTPUT_FILE}")

except Exception as exc:
    print(f"Serial run failed: {exc}")
finally:
    try:
        ser.close()
    except Exception:
        pass
