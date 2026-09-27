"""Minimal PGM reader (P2 ascii + P5 binary), no external image library.

Netpbm greymap. Header tokens (magic, width, height, maxval) are whitespace
separated and may carry `#` comments; ascii pixel data likewise flows across
line breaks, so we tokenize the whole stream rather than trusting line layout.
"""

from __future__ import annotations

import numpy as np


def _read_ascii_tokens(data: bytes, start: int, count: int) -> tuple[list[int], int]:
    """Read ``count`` whitespace-separated ints from ``data`` starting at ``start``."""
    tokens: list[int] = []
    i = start
    n = len(data)
    while len(tokens) < count:
        while i < n and data[i : i + 1].isspace():
            i += 1
        if i < n and data[i : i + 1] == b"#":  # comment to end of line
            while i < n and data[i : i + 1] != b"\n":
                i += 1
            continue
        j = i
        while j < n and not data[j : j + 1].isspace():
            j += 1
        if j == i:
            break
        tokens.append(int(data[i:j]))
        i = j
    if len(tokens) < count:
        raise ValueError(f"PGM truncated: expected {count} values, got {len(tokens)}")
    return tokens, i


def _read_header_field(data: bytes, i: int) -> tuple[int, int]:
    """Read one whitespace-delimited integer header field, skipping comments."""
    n = len(data)
    while i < n and data[i : i + 1].isspace():
        i += 1
    while i < n and data[i : i + 1] == b"#":
        while i < n and data[i : i + 1] != b"\n":
            i += 1
        while i < n and data[i : i + 1].isspace():
            i += 1
    j = i
    while j < n and not data[j : j + 1].isspace():
        j += 1
    return int(data[i:j]), j


def read_pgm(path: str) -> tuple[int, int, np.ndarray]:
    """Return (width, height, pixels[H, W] uint16 in [0, maxval])."""
    with open(path, "rb") as fh:
        data = fh.read()
    magic = data[:2]
    if magic not in (b"P2", b"P5"):
        raise ValueError(f"unsupported PGM magic {magic!r} (only P2/P5)")
    width, i = _read_header_field(data, 2)
    height, i = _read_header_field(data, i)
    maxval, i = _read_header_field(data, i)
    count = width * height
    if magic == b"P2":
        tokens, _ = _read_ascii_tokens(data, i, count)
        pixels = np.array(tokens, dtype=np.uint16).reshape(height, width)
    else:
        i += 1  # exactly one whitespace char separates header from binary payload
        if maxval < 256:
            raw = np.frombuffer(data, dtype=np.uint8, count=count, offset=i)
        else:  # 2 bytes per sample, big-endian
            raw = np.frombuffer(data, dtype=">u2", count=count, offset=i)
        pixels = raw.astype(np.uint16).reshape(height, width)
    return width, height, pixels
