# run_analyzer
"""Analyze run data to get data rate."""

file = "run_data.py"

def process_line(line):
    """
    If line is a timestamp, like -> 11:07:45.32
    return value of total seconds
    otherwise return None
    """
    try:
        h, m, s = line.split(':')
        h = int(h)
        m = int(m)
        s = float(s)
        seconds = s + (m * 60) + (h * 3600)
        return seconds
    except ValueError as e:
        pass

# Build a list of the time values (seconds)
times = []  # list of time values after start
started = False
with open(file) as f:
    lines = f.readlines()
    print("Total number of lines in file: ", len(lines))
    # start analysis after start command received
    for line in lines:
        if "Robot has received" in line:
            started = True

        if started:
            seconds = process_line(line)
            if seconds:  # Ignore None values
                times.append(seconds)

# Get delta_time between adjacent times
deltas = []
prev_value = times[0]
for value in times:
    if value == prev_value:
        pass
    else:
        deltas.append(value - prev_value)
        prev_value = value

print(f"Total number of time steps: {len(deltas)}")
print(f"Average value of time step: {sum(deltas) / len(deltas):.2f} seconds")
print(f"Data rate: {len(deltas) / sum(deltas):.2f} Hz")
print(f"Duration of run: {times[-1] - times[0]:.2f} seconds")
