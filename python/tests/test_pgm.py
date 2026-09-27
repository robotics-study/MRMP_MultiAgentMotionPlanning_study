"""PGM reader: P2 ascii and P5 binary yield identical grids; comments tolerated."""

from __future__ import annotations

import struct
from pathlib import Path

import numpy as np
import pytest

from mrmp.maps.pgm import read_pgm


def test_p2_ascii_with_comment(tmp_path: Path) -> None:
    p = tmp_path / "a.pgm"
    p.write_text("P2\n# a comment\n3 2\n255\n0 255 0\n255 0 255\n", encoding="utf-8")
    w, h, pixels = read_pgm(str(p))
    assert (w, h) == (3, 2)
    assert pixels.tolist() == [[0, 255, 0], [255, 0, 255]]


def test_p5_binary_matches_p2(tmp_path: Path) -> None:
    values = [0, 255, 128, 64, 200, 10]
    p5 = tmp_path / "b.pgm"
    p5.write_bytes(b"P5\n3 2\n255\n" + bytes(values))
    w, h, pixels = read_pgm(str(p5))
    assert (w, h) == (3, 2)
    assert pixels.tolist() == np.array(values, dtype=np.uint16).reshape(2, 3).tolist()


def test_p5_16bit_big_endian(tmp_path: Path) -> None:
    values = [0, 65535, 300, 1000]  # requires 2 bytes/sample
    p = tmp_path / "c.pgm"
    p.write_bytes(b"P5\n2 2\n65535\n" + b"".join(struct.pack(">H", v) for v in values))
    _, _, pixels = read_pgm(str(p))
    assert pixels.tolist() == [[0, 65535], [300, 1000]]


def test_truncated_ascii_raises(tmp_path: Path) -> None:
    p = tmp_path / "d.pgm"
    p.write_text("P2\n3 3\n255\n0 255 0\n", encoding="utf-8")  # only 3 of 9 values
    with pytest.raises(ValueError):
        read_pgm(str(p))


def test_unsupported_magic_raises(tmp_path: Path) -> None:
    p = tmp_path / "e.pgm"
    p.write_bytes(b"P3\n1 1\n255\n0 0 0\n")  # P3 = PPM color, unsupported
    with pytest.raises(ValueError):
        read_pgm(str(p))
