#!/usr/bin/env python3
"""
Generate an APRS packet as an AX.25 UI frame and modulate it into a WAV file
using Bell 202 AFSK (1200 baud), implemented from scratch.

This is inspired by the building blocks used by software TNCs such as Dire Wolf,
but does not import Dire Wolf or depend on external DSP libraries.

Features implemented here:
- APRS text packet creation: SRC>DEST,PATH1,PATH2:payload
- AX.25 address encoding
- AX.25 UI frame generation
- CRC-16/X.25 (reversed CCITT) FCS
- HDLC bit stuffing
- NRZI encoding
- Bell 202 AFSK modulation (1200 Hz mark, 2200 Hz space)
- 16-bit PCM mono WAV output

Example:
    python aprs_wav_generator.py \
        --source N0CALL-9 \
        --dest APRS \
        --path WIDE1-1,WIDE2-1 \
        --info ">Hello from Python" \
        --output aprs.wav
"""

from __future__ import annotations

import argparse
import math
import struct
import wave
from typing import Iterable, List, Sequence, Tuple


SAMPLE_RATE = 48000
BAUD = 1200
MARK_FREQ = 1200.0   # Bell 202 mark
SPACE_FREQ = 2200.0  # Bell 202 space
AMPLITUDE = 0.6      # 0.0 .. 1.0
PREAMBLE_FLAGS = 50
POSTAMBLE_FLAGS = 3


def parse_callsign(call: str) -> Tuple[str, int]:
    """Parse CALL or CALL-SSID into (callsign, ssid)."""
    call = call.strip().upper()
    if not call:
        raise ValueError("Empty callsign")

    if "-" in call:
        base, ssid_text = call.split("-", 1)
        if not ssid_text.isdigit():
            raise ValueError(f"Invalid SSID in callsign: {call}")
        ssid = int(ssid_text)
    else:
        base = call
        ssid = 0

    if not (1 <= len(base) <= 6):
        raise ValueError(f"Callsign base must be 1..6 chars: {call}")
    if not all(ch.isalnum() for ch in base):
        raise ValueError(f"Invalid callsign characters: {call}")
    if not (0 <= ssid <= 15):
        raise ValueError(f"SSID out of range 0..15: {call}")

    return base, ssid


def encode_ax25_address(addr: str, last: bool, repeated: bool = False) -> bytes:
    """
    Encode one AX.25 address field (7 bytes).

    Address byte layout:
    - 6 bytes callsign, left shifted by 1 bit, padded with spaces
    - 1 SSID byte

    SSID byte bits commonly used:
      bit 0: extension bit (1 for last address field)
      bit 1: reserved, usually 1
      bit 2: reserved, usually 1
      bit 5..2 contain SSID shifted left one place
      bit 7 has been-used/repeated bit for digipeaters
    """
    call, ssid = parse_callsign(addr)
    padded = call.ljust(6)

    out = bytearray()
    for ch in padded:
        out.append(ord(ch) << 1)

    ssid_byte = 0
    ssid_byte |= 0x60              # set reserved bits per AX.25 convention
    ssid_byte |= (ssid & 0x0F) << 1
    if repeated:
        ssid_byte |= 0x80
    if last:
        ssid_byte |= 0x01

    out.append(ssid_byte)
    return bytes(out)


def build_aprs_text_packet(source: str, dest: str, info: str, path: Sequence[str] | None = None) -> str:
    """Return classic APRS text line, mainly for display/debug."""
    header = f"{source.upper()}>{dest.upper()}"
    if path:
        clean = [p.strip().upper() for p in path if p.strip()]
        if clean:
            header += "," + ",".join(clean)
    return f"{header}:{info}"


def build_ax25_ui_frame(source: str, dest: str, info: str, path: Sequence[str] | None = None) -> bytes:
    """
    Build the AX.25 UI frame bytes before HDLC flag insertion.

    Frame:
      destination (7)
      source      (7)
      digis...    (7 each)
      control     0x03  (UI frame)
      pid         0xF0  (no layer 3)
      information
      fcs         2 bytes, little-endian on the wire because bits are sent LSB first
    """
    path = [p.strip().upper() for p in (path or []) if p.strip()]

    frame = bytearray()
    addresses = [dest.upper(), source.upper(), *path]
    for i, addr in enumerate(addresses):
        frame.extend(encode_ax25_address(addr, last=(i == len(addresses) - 1), repeated=False))

    frame.append(0x03)  # UI frame
    frame.append(0xF0)  # no layer 3
    frame.extend(info.encode("ascii", errors="replace"))

    fcs = crc16_x25(frame)
    frame.extend(struct.pack("<H", fcs))
    return bytes(frame)


def crc16_x25(data: bytes) -> int:
    """
    AX.25/HDLC FCS: CRC-16/X.25, reflected, init=0xFFFF, xorout=0xFFFF.
    Polynomial in reflected form: 0x8408.
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    crc ^= 0xFFFF
    return crc & 0xFFFF


def bytes_to_bits_lsb_first(data: bytes) -> List[int]:
    bits: List[int] = []
    for byte in data:
        for i in range(8):
            bits.append((byte >> i) & 1)
    return bits


def apply_bit_stuffing(bits: Iterable[int]) -> List[int]:
    """Insert a 0 after five consecutive 1 bits."""
    stuffed: List[int] = []
    one_count = 0
    for bit in bits:
        stuffed.append(bit)
        if bit:
            one_count += 1
            if one_count == 5:
                stuffed.append(0)
                one_count = 0
        else:
            one_count = 0
    return stuffed


def flag_bits() -> List[int]:
    # 0x7E = 01111110, sent LSB first => 0,1,1,1,1,1,1,0
    return bytes_to_bits_lsb_first(b"\x7E")


def nrzi_encode(bits: Iterable[int], initial_level: int = 1) -> List[int]:
    """
    HDLC/AX.25 NRZI:
      bit 0 => transition
      bit 1 => no transition
    Return symbol levels as 0/1 per bit interval.
    """
    level = initial_level
    out: List[int] = []
    for bit in bits:
        if bit == 0:
            level ^= 1
        out.append(level)
    return out


def afsk_modulate(nrzi_levels: Sequence[int],
                  sample_rate: int = SAMPLE_RATE,
                  baud: int = BAUD,
                  mark_freq: float = MARK_FREQ,
                  space_freq: float = SPACE_FREQ,
                  amplitude: float = AMPLITUDE) -> List[int]:
    """
    Convert NRZI levels into Bell 202 AFSK audio samples.

    By convention here:
      level 1 -> mark  (1200 Hz)
      level 0 -> space (2200 Hz)

    Continuous phase is preserved across symbols.
    """
    samples_per_bit = sample_rate / baud
    total_bits = len(nrzi_levels)
    total_samples = int(round(total_bits * samples_per_bit))

    pcm: List[int] = []
    phase = 0.0
    max_int16 = 32767

    for n in range(total_samples):
        bit_index = min(int(n / samples_per_bit), total_bits - 1)
        freq = mark_freq if nrzi_levels[bit_index] else space_freq
        phase += 2.0 * math.pi * freq / sample_rate
        # Keep phase bounded to avoid numerical growth.
        if phase > 2.0 * math.pi:
            phase %= 2.0 * math.pi
        sample = int(max(-1.0, min(1.0, amplitude * math.sin(phase))) * max_int16)
        pcm.append(sample)

    return pcm


def write_wav(filename: str, samples: Sequence[int], sample_rate: int = SAMPLE_RATE) -> None:
    with wave.open(filename, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)  # 16-bit PCM
        wf.setframerate(sample_rate)
        frames = struct.pack("<" + "h" * len(samples), *samples)
        wf.writeframes(frames)


def generate_aprs_wav(source: str,
                      dest: str,
                      info: str,
                      output_wav: str,
                      path: Sequence[str] | None = None,
                      preamble_flags: int = PREAMBLE_FLAGS,
                      postamble_flags: int = POSTAMBLE_FLAGS) -> str:
    """Generate APRS WAV and return the packet text."""
    packet_text = build_aprs_text_packet(source, dest, info, path)
    frame = build_ax25_ui_frame(source, dest, info, path)

    payload_bits = bytes_to_bits_lsb_first(frame)
    stuffed_payload_bits = apply_bit_stuffing(payload_bits)

    bits = []
    for _ in range(preamble_flags):
        bits.extend(flag_bits())
    bits.extend(stuffed_payload_bits)
    for _ in range(postamble_flags):
        bits.extend(flag_bits())

    nrzi = nrzi_encode(bits, initial_level=1)
    samples = afsk_modulate(nrzi)
    write_wav(output_wav, samples)
    return packet_text


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate APRS AFSK WAV from a text packet")
    parser.add_argument("--source", required=True, help="Source callsign, e.g. N0CALL-9")
    parser.add_argument("--dest", default="APRS", help="Destination callsign/app ID")
    parser.add_argument("--path", default="", help="Comma-separated digipeater path")
    parser.add_argument("--info", required=True, help="APRS information field, e.g. '>hello' or '!4903.50N/07201.75W-Test'")
    parser.add_argument("--output", default="aprs.wav", help="Output WAV filename")
    parser.add_argument("--preamble-flags", type=int, default=PREAMBLE_FLAGS, help="Number of opening HDLC flags")
    parser.add_argument("--postamble-flags", type=int, default=POSTAMBLE_FLAGS, help="Number of closing HDLC flags")
    args = parser.parse_args()

    path = [p.strip() for p in args.path.split(",") if p.strip()] if args.path else []

    packet_text = generate_aprs_wav(
        source=args.source,
        dest=args.dest,
        info=args.info,
        output_wav=args.output,
        path=path,
        preamble_flags=args.preamble_flags,
        postamble_flags=args.postamble_flags,
    )

    print("Packet:")
    print(packet_text)
    print(f"WAV written to: {args.output}")


if __name__ == "__main__":
    main()
