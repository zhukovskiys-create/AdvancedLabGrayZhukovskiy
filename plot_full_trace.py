import csv
import glob
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def pick_trace_candidates():
    preferred = []
    for pattern in ["temperature_control_log.csv", "full_minute_heat_run.csv", "run_*.csv", "*_heat_run.csv"]:
        preferred.extend(sorted(glob.glob(pattern)))

    seen = set()
    unique = []
    for path in preferred:
        if path not in seen and not path.startswith("steady_state"):
            seen.add(path)
            unique.append(path)

    if unique:
        return unique

    fallback = sorted(glob.glob("*.csv"))
    fallback = [p for p in fallback if not p.startswith("steady_state")]
    return fallback


def load_rows(path):
    with open(path, newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        if not reader.fieldnames:
            raise ValueError(f"No CSV header found in {path}")
        return list(reader)


def split_segments(rows):
    if not rows:
        return []

    segments = []
    current = []
    current_dir = None

    for row in rows:
        if "heat_cool" not in row:
            continue
        try:
            direction = int(float(row["heat_cool"]))
        except (TypeError, ValueError):
            continue

        if current_dir is None:
            current_dir = direction
            current = [row]
        elif direction == current_dir:
            current.append(row)
        else:
            segments.append((current_dir, current))
            current = [row]
            current_dir = direction

    if current:
        segments.append((current_dir, current))

    return segments


def save_segment_plot(segment_index, direction, rows, root_name):
    times = []
    temps = []
    pwms = []
    for row in rows:
        try:
            times.append(float(row["time_s"]))
            temps.append(float(row["temperature_C"]))
            pwms.append(int(float(row["pwm"])))
        except (TypeError, ValueError, KeyError):
            continue

    if not times or not temps:
        return None

    pwm_value = pwms[0] if pwms else 0
    pwm_value_text = f"PWM {pwm_value}"

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.plot(times, temps, color="red" if direction == 1 else "blue", linewidth=2, label=pwm_value_text)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Temperature (°C)")
    ax.set_title(f"Segment {segment_index}: {'HEAT' if direction == 1 else 'COOL'} | {pwm_value_text}")
    ax.grid(True, linestyle="--", alpha=0.5)
    ax.legend(loc="best")
    fig.tight_layout()

    filename = f"{root_name}_segment_{segment_index:02d}_{'HEAT' if direction == 1 else 'COOL'}_PWM{pwm_value}.png"
    fig.savefig(filename, dpi=200)
    plt.close(fig)
    return filename


files = pick_trace_candidates()
if not files:
    raise FileNotFoundError("No CSV trace files were found in the workspace root.")

all_segment_files = []
for file_path in files:
    rows = load_rows(file_path)
    segments = split_segments(rows)
    if not segments:
        continue

    for idx, (direction, segment_rows) in enumerate(segments, start=1):
        name = save_segment_plot(idx, direction, segment_rows, os.path.splitext(os.path.basename(file_path))[0])
        if name:
            all_segment_files.append(name)

if not all_segment_files:
    raise ValueError("No valid heating/cooling segments were found in the CSV files.")

print(f"Saved {len(all_segment_files)} segment plots:")
for item in all_segment_files:
    print(item)
