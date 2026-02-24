############################################################
# APRS GPS POSITION → HDLC → AFSK1200 WAV
############################################################

import math
import struct
import wave

FLAG = 0x7E


############################################################
# FORMAT APRS GPS POSITION
############################################################
def aprs_position(lat, lon):

    lat_dir = 'N' if lat >= 0 else 'S'
    lon_dir = 'E' if lon >= 0 else 'W'

    lat = abs(lat)
    lon = abs(lon)

    lat_deg = int(lat)
    lon_deg = int(lon)

    lat_min = (lat - lat_deg) * 60
    lon_min = (lon - lon_deg) * 60

    lat_str = f"{lat_deg:02d}{lat_min:05.2f}{lat_dir}"
    lon_str = f"{lon_deg:03d}{lon_min:05.2f}{lon_dir}"

    return f"!{lat_str}/{lon_str}-"


############################################################
# AX25 CALLSIGN
############################################################
def encode_callsign(call, ssid, last=False):
    call = call.upper().ljust(6)

    addr = [(ord(c) << 1) for c in call]

    ssid_byte = 0b01100000 | ((ssid & 0x0F) << 1)

    if last:
        ssid_byte |= 1

    addr.append(ssid_byte)
    return addr


############################################################
# CRC-16 CCITT
############################################################
def crc_ccitt(data):
    crc = 0xFFFF

    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1

    crc ^= 0xFFFF
    return [crc & 0xFF, (crc >> 8) & 0xFF]


############################################################
# BUILD AX25 FRAME
############################################################
def build_ax25(info):

    frame = []

    frame += encode_callsign("APRS", 0)
    frame += encode_callsign("N0CALL", 11)

    path = [("WIDE1",1),("WIDE2",2)]

    for i,(c,s) in enumerate(path):
        frame += encode_callsign(c,s,last=(i==len(path)-1))

    frame += [0x03, 0xF0]
    frame += list(info.encode("ascii"))

    return frame


############################################################
# BIT HELPERS
############################################################
def bytes_to_bits_lsb(data):
    return [(b >> i) & 1 for b in data for i in range(8)]


def bitstuff(bits):
    out=[]
    ones=0

    for b in bits:
        out.append(b)
        if b:
            ones+=1
            if ones==5:
                out.append(0)
                ones=0
        else:
            ones=0
    return out


def nrzi(payload_bits):
    level = 1
    out = []

    for b in payload_bits:
        if b == 0:
            level ^= 1
        out.append(level)

    return out


############################################################
# HDLC
############################################################
def hdlc(ax25):

    FLAG = 0x7E

    frame = ax25 + crc_ccitt(ax25)

    payload_bits = bitstuff(bytes_to_bits_lsb(frame))
    flag_bits = bytes_to_bits_lsb([FLAG])

    bits = []

    # 300ms preamble (~50 flags)
    bits += flag_bits * 50

    bits += payload_bits

    bits += flag_bits * 3

    # NRZI EVERYTHING
    return nrzi(payload_bits=bits)


############################################################
# AFSK1200 WAV
############################################################
def afsk1200(nrzi_bits, filename="gps_aprs.wav"):

    import wave, math, struct

    samplerate = 48000
    baud = 1200

    MARK = 1200
    SPACE = 2200

    samples_per_bit = samplerate // baud

    phase = 0.0
    tone = MARK   # start tone

    audio = []

    for bit in nrzi_bits:

        # APRS rule:
        # 0 = transition
        # 1 = no transition
        if bit == 0:
            tone = SPACE if tone == MARK else MARK

        phase_inc = 2 * math.pi * tone / samplerate

        for _ in range(samples_per_bit):
            phase += phase_inc
            audio.append(math.sin(phase))

    peak = max(abs(x) for x in audio)
    audio = [x / peak for x in audio]

    with wave.open(filename, "w") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(samplerate)

        for s in audio:
            wf.writeframes(struct.pack("<h", int(s * 32767)))

    print("WAV written:", filename)


############################################################
# MAIN
############################################################
if __name__=="__main__":

    lat = 42.356537
    lon = -71.095185

    position = aprs_position(lat, lon)

    print("APRS payload:", position)

    ax25 = build_ax25(position)

    bits = nrzi(hdlc(ax25))

    afsk1200(bits, "gps_aprs.wav")