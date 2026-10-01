"""Declarative algorithm parameters loaded from configs/<section>/<algo>.yaml.

Mirrors the C++ `core/params.hpp`. The yaml *is* the declaration (per
`spec/param_schema.json`) carrying defaults; the loader validates type / range /
choices at load time so no magic numbers leak into algorithm code. A config also
declares the scenarios its algorithm is run on (`scenarios:`) — the matrix runner
and the web exporter read that list instead of running every algorithm against
every scenario (a continuous-planner config does not belong on discrete-only
scenarios and vice versa).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

ParamValue = int | float | bool | str

_VALID_TYPES = ("int", "float", "bool", "string", "enum")


@dataclass(frozen=True)
class ParamDecl:
    name: str
    type: str
    default: ParamValue
    min: float | None = None
    max: float | None = None
    choices: list[str] | None = None
    description: str = ""


class ParamError(Exception):
    """Raised on any parameter declaration / value validation failure."""


def _check_default(decl: ParamDecl) -> ParamValue:
    """Coerce and range-check a declared default, raising ParamError on failure."""
    value = decl.default
    if decl.type == "int":
        # bool is an int subclass in Python; reject it explicitly.
        if isinstance(value, bool) or not isinstance(value, int):
            raise ParamError(f"param error: '{decl.name}' default must be int, got {value!r}")
        return _check_range(decl, float(value), value)
    if decl.type == "float":
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ParamError(f"param error: '{decl.name}' default must be float, got {value!r}")
        fvalue = float(value)
        return _check_range(decl, fvalue, fvalue)
    if decl.type == "bool":
        if not isinstance(value, bool):
            raise ParamError(f"param error: '{decl.name}' default must be bool, got {value!r}")
        return value
    if decl.type in ("string", "enum"):
        if not isinstance(value, str):
            raise ParamError(f"param error: '{decl.name}' default must be string, got {value!r}")
        if decl.type == "enum":
            if not decl.choices:
                raise ParamError(f"param error: enum '{decl.name}' declares no choices")
            if value not in decl.choices:
                raise ParamError(
                    f"param error: '{decl.name}' default {value!r} not in choices {decl.choices}"
                )
        return value
    raise ParamError(f"param error: '{decl.name}' has unknown type {decl.type!r}")


def _check_range(decl: ParamDecl, as_float: float, value: ParamValue) -> ParamValue:
    if decl.min is not None and as_float < decl.min:
        raise ParamError(f"param error: '{decl.name}' default {value} < min {decl.min}")
    if decl.max is not None and as_float > decl.max:
        raise ParamError(f"param error: '{decl.name}' default {value} > max {decl.max}")
    return value


class ParamSet:
    def __init__(
        self,
        algorithm: str,
        section: str,
        decls: dict[str, ParamDecl],
        scenarios: list[str] | None = None,
    ) -> None:
        self.algorithm = algorithm
        self.section = section
        # Scenario slugs (maps/scenarios/<slug>.yaml) this algorithm is run on.
        self.scenarios: list[str] = list(scenarios or [])
        self._decls = decls
        self._values: dict[str, ParamValue] = {
            name: _check_default(decl) for name, decl in decls.items()
        }

    @classmethod
    def from_yaml(cls, path: str | Path) -> ParamSet:
        with open(path, encoding="utf-8") as fh:
            raw = yaml.safe_load(fh)
        if not isinstance(raw, dict):
            raise ParamError(f"param error: {path} is not a mapping")
        for key in ("algorithm", "section", "params", "scenarios"):
            if key not in raw:
                raise ParamError(f"param error: {path} missing required key '{key}'")
        # A config declares which family (site section) its algorithm belongs to —
        # a config declaring anything else is stale. Mirrored in C++.
        if raw["section"] not in ("search", "sampling", "decentralized"):
            raise ParamError(f"param error: unknown section {raw['section']!r}")
        # Scenario slugs this algorithm runs on — required, list of strings
        # (possibly empty). The matrix runner and web exporter route per config.
        if not isinstance(raw["scenarios"], list) or any(
            not isinstance(s, str) for s in raw["scenarios"]
        ):
            raise ParamError(f"param error: {path} 'scenarios' must be a list of strings")
        params = raw["params"]
        if not isinstance(params, list):
            raise ParamError(f"param error: {path} 'params' must be a list")
        decls: dict[str, ParamDecl] = {}
        for entry in params:
            if not isinstance(entry, dict):
                raise ParamError(f"param error: {path} param entry is not a mapping")
            for req in ("name", "type", "default", "description"):
                if req not in entry:
                    raise ParamError(f"param error: {path} param missing '{req}'")
            if entry["type"] not in _VALID_TYPES:
                raise ParamError(f"param error: unknown type {entry['type']!r} for {entry['name']}")
            decl = ParamDecl(
                name=entry["name"],
                type=entry["type"],
                default=entry["default"],
                min=entry.get("min"),
                max=entry.get("max"),
                choices=entry.get("choices"),
                description=entry["description"],
            )
            decls[decl.name] = decl
        return cls(raw["algorithm"], raw["section"], decls, list(raw["scenarios"]))

    def _typed(self, name: str, expected: str) -> ParamValue:
        if name not in self._decls:
            raise ParamError(f"param error: unknown parameter '{name}'")
        decl = self._decls[name]
        if decl.type != expected:
            raise ParamError(
                f"param error: '{name}' is {decl.type}, requested as {expected}"
            )
        return self._values[name]

    def get_int(self, name: str) -> int:
        value = self._typed(name, "int")
        assert isinstance(value, int)
        return value

    def get_float(self, name: str) -> float:
        value = self._typed(name, "float")
        assert isinstance(value, (int, float))
        return float(value)

    def get_bool(self, name: str) -> bool:
        value = self._typed(name, "bool")
        assert isinstance(value, bool)
        return value

    def get_string(self, name: str) -> str:
        # 'string' and 'enum' are both surfaced as strings.
        if name not in self._decls:
            raise ParamError(f"param error: unknown parameter '{name}'")
        if self._decls[name].type not in ("string", "enum"):
            raise ParamError(f"param error: '{name}' is not a string/enum")
        value = self._values[name]
        assert isinstance(value, str)
        return value

    def has(self, name: str) -> bool:
        return name in self._decls

    def values(self) -> dict[str, ParamValue]:
        return dict(self._values)
