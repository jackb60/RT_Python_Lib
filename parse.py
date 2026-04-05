import csv
import ast
headers = [
    "time", "pyros", "servos", "accelerometer", "barometer",
    "baro_filtered_alt", "baro_filtered_velocity", "temp", "gyro",
    "magnetometer", "heading", "gps_fix", "lat", "lon", "gps_alt",
    "pdop", "hdop", "vdop", "flight_time", "last_rec",
    "yaw_gyro_int", "pitch_gyro_int", "roll_gyro_int",
    "batt_voltage", "state", "pktnum", "rssi",
    "armed", "fired", "bad_packets", "rxrssi",
    "accel_integrated_velo", "baro_max_alt", "gps_max_alt"
]

def parse_and_print(line: str):
    reader = csv.reader([line])
    values = next(reader)

    def convert(v):
        # Try to parse lists/tuples/etc first
        try: 
            return ast.literal_eval(v)
        except:
            pass
        # Try numeric conversion
        try:
            if '.' in v:
                return float(v)
            return int(v)
        except:
            return v  # Default to string

    values = [convert(v) for v in values]

    for name, val in zip(headers, values):
        print(f"{name:25}: {val}")

# Example use
line = """1761878333.8147173,"[1, 1, 1, 1, 1, 1, 1, 1]","[0, 0, 0, 0, 0, 0, 1641, 1566]","[16.81840475, -0.48052585, 1.9221034]",6204120,118.07688903808594,1.2669144868850708,8965624,"[-0.762939453, 8.94165038916, 4.9133300773199995]","[-396.25, 241.1875, -181.1875]",114.06781768798828,0,0.0,0,0.0,99.98999786376953,99.98999786376953,99.98999786376953,357532,20,7.168768405914307,-8.686909675598145,-23.188751220703125,11.32800006866455,state.GROUND_TESTING,6034,-41,"[0, 0, 0, 0, 0, 0, 0, 0]","[0, 0, 0, 0, 0, 0, 0, 0]",1,-57,-3503.94482421875,136.16708374023438,0.0"""


parse_and_print(line)