"""ParamSet loading + declaration validation."""

from __future__ import annotations

from pathlib import Path

import pytest
from conftest import write_config

from mrmp.core.params import ParamError, ParamSet


def test_empty_params_loads_and_values_is_empty(tmp_path: Path) -> None:
    cfg = write_config(tmp_path / "ok.yaml", "x", [])
    ps = ParamSet.from_yaml(cfg)
    assert ps.algorithm == "x" and ps.section == "search"
    assert ps.values() == {}


def test_out_of_range_default_raises(tmp_path: Path) -> None:
    cfg = write_config(
        tmp_path / "bad.yaml",
        "x",
        [{"name": "weight", "type": "float", "default": 9.0,
          "min": 1.0, "max": 5.0, "description": "x"}],
    )
    with pytest.raises(ParamError):
        ParamSet.from_yaml(cfg)


def test_type_mismatch_raises(tmp_path: Path) -> None:
    cfg = write_config(
        tmp_path / "bad.yaml",
        "x",
        [{"name": "n", "type": "int", "default": 1.5, "description": "x"}],
    )
    with pytest.raises(ParamError):
        ParamSet.from_yaml(cfg)


def test_enum_choice_violation_raises(tmp_path: Path) -> None:
    cfg = write_config(
        tmp_path / "bad.yaml",
        "x",
        [{"name": "mode", "type": "enum", "default": "z",
          "choices": ["a", "b"], "description": "x"}],
    )
    with pytest.raises(ParamError):
        ParamSet.from_yaml(cfg)


def test_missing_required_key_raises(tmp_path: Path) -> None:
    p = tmp_path / "bad.yaml"
    p.write_text("algorithm: x\nsection: search\n", encoding="utf-8")
    with pytest.raises(ParamError):
        ParamSet.from_yaml(p)


def test_wrong_type_access_raises(tmp_path: Path) -> None:
    cfg = write_config(
        tmp_path / "mix.yaml",
        "mix",
        [{"name": "weight", "type": "float", "default": 1.0, "description": "x"}],
    )
    ps = ParamSet.from_yaml(cfg)
    with pytest.raises(ParamError):
        ps.get_int("weight")  # declared float


def test_typed_getters_across_types(tmp_path: Path) -> None:
    cfg = write_config(
        tmp_path / "mix.yaml",
        "mix",
        [
            {"name": "flag", "type": "bool", "default": True, "description": "x"},
            {"name": "label", "type": "string", "default": "hi", "description": "x"},
            {"name": "mode", "type": "enum", "default": "a",
             "choices": ["a", "b"], "description": "x"},
            {"name": "n", "type": "int", "default": 3, "min": 0, "max": 10, "description": "x"},
        ],
    )
    ps = ParamSet.from_yaml(cfg)
    assert ps.get_bool("flag") is True
    assert ps.get_string("label") == "hi"
    assert ps.get_string("mode") == "a"  # enum surfaces as string
    assert ps.get_int("n") == 3
    assert ps.has("flag") and not ps.has("missing")
    with pytest.raises(ParamError):
        ps.get_string("n")  # int is not a string/enum
    with pytest.raises(ParamError):
        ps.get_float("missing")  # unknown parameter


def test_bad_default_types_raise(tmp_path: Path) -> None:
    for bad in (
        {"name": "flag", "type": "bool", "default": 1, "description": "x"},
        {"name": "label", "type": "string", "default": 5, "description": "x"},
        {"name": "n", "type": "int", "default": True, "description": "x"},  # bool != int
    ):
        cfg = write_config(tmp_path / "bad.yaml", "x", [bad])
        with pytest.raises(ParamError):
            ParamSet.from_yaml(cfg)


def test_scenarios_round_trip_and_gate(tmp_path: Path) -> None:
    # scenarios is a REQUIRED top-level key (per-config routing for bench/export).
    cfg = write_config(
        tmp_path / "ok.yaml", "x", [], section="sampling",
        scenarios=["open01_cross_discs"],
    )
    assert ParamSet.from_yaml(cfg).scenarios == ["open01_cross_discs"]
    # Missing key raises (the params-key check above already proves requiredness
    # is enforced per key); a non-list value raises too.
    bad = tmp_path / "bad.yaml"
    bad.write_text("algorithm: x\nsection: search\nscenarios: nope\nparams: []",
                   encoding="utf-8")
    with pytest.raises(ParamError):
        ParamSet.from_yaml(bad)
