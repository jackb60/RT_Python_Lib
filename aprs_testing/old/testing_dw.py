#!/usr/bin/env python3
import argparse
import wave
import numpy as np

def read_wav_pcm(path: str):
    """Read PCM WAV via stdlib wave. Returns (fs, mono_float32)."""
    with wave.open(path, "rb") as w:
        nch = w.getnchannels()
        fs = w.getframerate()
        nframes = w.getnframes()
        sampwidth = w.getsampwidth()
        raw = w.readframes(nframes)

    # Decode PCM
    if sampwidth == 1:
        # 8-bit WAV is unsigned
        x = np.frombuffer(raw, dtype=np.uint8).astype(np.int16) - 128
        maxv = 128.0
    elif sampwidth == 2:
        x = np.frombuffer(raw, dtype=np.int16)
        maxv = 32768.0
    elif sampwidth == 3:
        # 24-bit little-endian: manual sign-extend into int32
        b = np.frombuffer(raw, dtype=np.uint8)
        b = b.reshape(-1, 3)
        x = (b[:, 0].astype(np.int32) |
             (b[:, 1].astype(np.int32) << 8) |
             (b[:, 2].astype(np.int32) << 16))
        # sign extension for 24-bit
        sign = (x & 0x800000) != 0
        x[sign] -= 1 << 24
        maxv = float(1 << 23)
    elif sampwidth == 4:
        x = np.frombuffer(raw, dtype=np.int32)
        maxv = float(1 << 31)
    else:
        raise ValueError(f"Unsupported sample width: {sampwidth} bytes")

    # Deinterleave channels and convert to mono float32
    x = x.reshape(-1, nch)
    mono = x.mean(axis=1).astype(np.float32) / maxv
    return fs, mono

def tone_power_blocks(x: np.ndarray, fs: int, f: float, start: int, block_len: int):
    """Return power per block at frequency f using a single-bin complex correlator."""
    nblocks = (len(x) - start) // block_len
    if nblocks <= 0:
        return np.array([], dtype=np.float32)

    # Precompute reference for one block
    n = np.arange(block_len, dtype=np.float32)
    ref = np.exp(-1j * 2.0 * np.pi * (f / fs) * n).astype(np.complex64)

    p = np.empty(nblocks, dtype=np.float32)
    idx = start
    for i in range(nblocks):
        blk = x[idx:idx + block_len]
        z = np.dot(blk.astype(np.float32), ref)  # complex
        p[i] = (z.real * z.real + z.imag * z.imag)
        idx += block_len
    return p

def demod_bits(x, fs, baud, f0, f1, map_f0_to_bit0=True, offset_samples=0):
    """
    Decide each bit by comparing power at f0 vs f1 in each bit window.
    Convention requested: 2200->0, 1200->1 (default).
    """
    block_len = int(round(fs / baud))
    if block_len <= 4:
        raise ValueError("Bit window too small. Check fs/baud.")

    p0 = tone_power_blocks(x, fs, f0, offset_samples, block_len)
    p1 = tone_power_blocks(x, fs, f1, offset_samples, block_len)
    if len(p0) == 0:
        return np.array([], dtype=np.uint8), block_len

    # Decide which frequency dominates
    f0_dom = p0 > p1  # True where f0 dominates
    if map_f0_to_bit0:
        # f0 -> 0, f1 -> 1
        bits = (~f0_dom).astype(np.uint8)
    else:
        # f0 -> 1, f1 -> 0
        bits = (f0_dom).astype(np.uint8)

    # Confidence (used for auto-offset selection)
    eps = 1e-12
    conf = np.mean(np.abs(p0 - p1) / (p0 + p1 + eps))
    return bits, block_len, conf

def pack_bits_msb_first(bits: np.ndarray) -> bytes:
    """Pack bits (0/1) into bytes, MSB-first. Pads last byte with zeros."""
    n = len(bits)
    pad = (-n) % 8
    if pad:
        bits = np.concatenate([bits, np.zeros(pad, dtype=np.uint8)])
    bits = bits.reshape(-1, 8)
    weights = np.array([128, 64, 32, 16, 8, 4, 2, 1], dtype=np.uint16)
    out = (bits * weights).sum(axis=1).astype(np.uint8)
    return out.tobytes()

def main():
    ap = argparse.ArgumentParser(
        description="Convert 1200/2200 Hz tone WAV to bits (2200->0, 1200->1 by default)."
    )
    ap.add_argument("input_wav", help="Input PCM .wav file")
    ap.add_argument("output_bin", help="Output .bin (bit-packed bytes, MSB-first)")
    ap.add_argument("--baud", type=float, default=1200.0, help="Bit rate (default: 1200)")
    ap.add_argument("--f0", type=float, default=2200.0, help="Frequency representing bit 0 (default: 2200)")
    ap.add_argument("--f1", type=float, default=1200.0, help="Frequency representing bit 1 (default: 1200)")
    ap.add_argument("--no-auto-offset", action="store_true",
                    help="Disable offset scan; start at sample 0")
    ap.add_argument("--offset", type=float, default=None,
                    help="Manual start offset in seconds (overrides auto-offset scan)")
    ap.add_argument("--raw-bits", default=None,
                    help="Also write ASCII '0'/'1' to this file (optional)")
    args = ap.parse_args()

    fs, x = read_wav_pcm(args.input_wav)

    # Basic DC removal (helps sometimes)
    x = x - np.mean(x)

    block_len = int(round(fs / args.baud))

    # Choose offset
    if args.offset is not None:
        offset_samples = int(round(args.offset * fs))
    elif args.no_auto_offset:
        offset_samples = 0
    else:
        # Scan a handful of offsets within one bit window and pick best confidence
        steps = 20
        cand = np.unique(np.linspace(0, max(block_len - 1, 0), steps).astype(int))
        best = (None, -1.0, None)  # (offset, conf, bits)
        for off in cand:
            bits, _, conf = demod_bits(
                x, fs, args.baud, args.f0, args.f1,
                map_f0_to_bit0=True, offset_samples=int(off)
            )
            if len(bits) > 16 and conf > best[1]:
                best = (int(off), float(conf), bits)
        if best[0] is None:
            offset_samples = 0
        else:
            offset_samples = best[0]

    bits, _, conf = demod_bits(
        x, fs, args.baud, args.f0, args.f1,
        map_f0_to_bit0=True, offset_samples=offset_samples
    )

    data = pack_bits_msb_first(bits)
    with open(args.output_bin, "wb") as f:
        f.write(data)

    if args.raw_bits:
        with open(args.raw_bits, "w", encoding="utf-8") as f:
            f.write("".join("1" if b else "0" for b in bits))

    print(f"fs={fs} Hz, baud={args.baud}, bit_window={block_len} samples")
    print(f"offset={offset_samples} samples ({offset_samples/fs:.6f} s), confidence={conf:.4f}")
    print(f"bits={len(bits)}, bytes_written={len(data)} -> {args.output_bin}")
    if args.raw_bits:
        print(f"raw bits written -> {args.raw_bits}")

if __name__ == "__main__":
    main()