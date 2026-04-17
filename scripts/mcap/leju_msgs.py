#!/usr/bin/env python3
"""
leju::msgs 解码库 / Decoders for leju::msgs::* CDR payloads.

优先用 **mcap 自带的 IDL 文本动态构造解码器** (pycdr2),
在 pycdr2 不可用或 IDL 解析失败时回退到内置硬编码解码器.

两条路径都返回 **plain dict**, 含 IDL 里全部字段 —— 尤其是每条消息自带的
`header_sec / header_nanosec` 采集时间戳, 以及老版本漏掉的 `JointCmd.modes`.

典型用法:
    from mcap.reader import make_reader
    import leju_msgs

    with open(path, "rb") as f:
        reader = make_reader(f)
        leju_msgs.register_from_mcap_summary(reader.get_summary())   # ←一次性登记
        for sch, ch, m in reader.iter_messages():
            payload = leju_msgs.decode(sch.name, m.data)   # dict
            ...

API:
    is_supported(schema_name) -> bool
    supported_schemas()       -> frozenset[str]
    register_schema(name, idl_text)          -> bool
    register_from_mcap_summary(summary)      -> dict[str, str]   # name -> "dynamic"/"hardcoded"/err
    decode(schema_name, data) -> dict
    UnsupportedSchema  (exception)

扩展新 schema:
    录制时 recorder 会把 IDL 写进 mcap —— 只要你调用了 register_from_mcap_summary,
    新 schema **零代码改动**即可被 decode(). 硬编码表保留是为了
    (a) pycdr2 没装也能跑这 7 个已知类型;
    (b) spike_idl_decode.py 做 IDL 解析器回归校验.
"""

from __future__ import annotations

import re
import struct
import sys
from typing import Any

__all__ = [
    "UnsupportedSchema",
    "is_supported",
    "supported_schemas",
    "register_schema",
    "register_from_mcap_summary",
    "decode",
]


class UnsupportedSchema(Exception):
    """No decoder (dynamic or hardcoded) for the requested schema."""


# ---------------------------------------------------------------------------
# pycdr2 optional dependency
# ---------------------------------------------------------------------------

try:  # pragma: no cover - environment-dependent
    import pycdr2  # noqa: F401
    import pycdr2.types as _pt
    from pycdr2 import make_idl_struct as _make_idl_struct
    _PYCDR2_OK = True
except ImportError:  # pragma: no cover
    _pt = None  # type: ignore[assignment]
    _make_idl_struct = None  # type: ignore[assignment]
    _PYCDR2_OK = False


# ---------------------------------------------------------------------------
# Minimal OMG IDL parser — Leju subset only
# ---------------------------------------------------------------------------
# 支持: module, (@anno)* struct, primitive, fixed array T[N], sequence<T>[,N],
#       string<[N]>. 未支持: typedef / enum / union / 嵌套 struct 引用.

_PRIMS: dict[str, str | None] = {
    # IDL name -> pycdr2.types attribute name (None = Python builtin)
    "bool":   None,   # Python bool
    "int8":   "int8",   "uint8":  "uint8",
    "int16":  "int16",  "uint16": "uint16",
    "int32":  "int32",  "uint32": "uint32",
    "int64":  "int64",  "uint64": "uint64",
    "float":  "float32",
    "double": "float64",
    "octet":  "uint8",
    "char":   "char",
}

_TOKEN = re.compile(r"@|::|[A-Za-z_][A-Za-z_0-9]*|\d+|[<>{}\[\];,()]")


def _strip_comments(src: str) -> str:
    src = re.sub(r"//[^\n]*", "", src)
    src = re.sub(r"/\*.*?\*/", "", src, flags=re.DOTALL)
    return src


def _tokenize(src: str) -> list[str]:
    return _TOKEN.findall(_strip_comments(src))


class _IDLParser:
    def __init__(self, tokens: list[str]):
        self.toks = tokens
        self.i = 0

    def _peek(self) -> str | None:
        return self.toks[self.i] if self.i < len(self.toks) else None

    def _pop(self) -> str:
        t = self.toks[self.i]
        self.i += 1
        return t

    def _expect(self, tok: str) -> None:
        got = self._pop()
        if got != tok:
            raise ValueError(f"expected {tok!r}, got {got!r} at token #{self.i}")

    # Return {fully_qualified_type_name: [(field_name, type_spec), ...]}.
    def parse(self) -> dict[str, list[tuple[str, Any]]]:
        out: dict[str, list] = {}
        self._parse_scope([], out)
        return out

    def _parse_scope(self, mods: list[str], out: dict) -> None:
        while True:
            tok = self._peek()
            if tok is None or tok == "}":
                return
            if tok == "module":
                self._pop()
                name = self._pop()
                self._expect("{")
                self._parse_scope(mods + [name], out)
                self._expect("}")
                if self._peek() == ";":
                    self._pop()
            elif tok == "@":
                self._skip_annotation()
            elif tok == "struct":
                self._pop()
                name = self._pop()
                self._expect("{")
                fields = self._parse_fields()
                self._expect("}")
                if self._peek() == ";":
                    self._pop()
                out["::".join(mods + [name])] = fields
            else:
                raise ValueError(f"unexpected token at module scope: {tok!r}")

    def _skip_annotation(self) -> None:
        self._pop()  # '@'
        self._pop()  # name
        if self._peek() == "(":
            depth = 0
            while True:
                t = self._pop()
                if t == "(":
                    depth += 1
                elif t == ")":
                    depth -= 1
                    if depth == 0:
                        return

    def _parse_fields(self) -> list[tuple[str, Any]]:
        fields: list[tuple[str, Any]] = []
        while self._peek() != "}":
            if self._peek() == "@":
                self._skip_annotation()
                continue
            tspec = self._parse_type()
            name = self._pop()
            if self._peek() == "[":
                self._pop()
                dim = int(self._pop())
                self._expect("]")
                tspec = ("array", tspec, dim)
            self._expect(";")
            fields.append((name, tspec))
        return fields

    def _parse_type(self) -> Any:
        tok = self._pop()
        if tok == "sequence":
            self._expect("<")
            inner = self._parse_type()
            bound = None
            if self._peek() == ",":
                self._pop()
                bound = int(self._pop())
            self._expect(">")
            return ("sequence", inner, bound)
        if tok == "string":
            if self._peek() == "<":
                self._pop()
                self._pop()  # bound — ignored
                self._expect(">")
            return ("string",)
        if tok in _PRIMS:
            return ("prim", tok)
        # scoped ref like leju::msgs::Foo — unsupported
        name = tok
        while self._peek() == "::":
            self._pop()
            name += "::" + self._pop()
        return ("ref", name)


def _parse_idl(text: str) -> dict[str, list[tuple[str, Any]]]:
    return _IDLParser(_tokenize(text)).parse()


# ---------------------------------------------------------------------------
# Spec -> pycdr2 type translation & dynamic class building
# ---------------------------------------------------------------------------

def _spec_to_pycdr2(spec: Any) -> Any:
    if _pt is None:
        raise RuntimeError("pycdr2 not available")
    kind = spec[0]
    if kind == "prim":
        name = spec[1]
        if name == "bool":
            return bool
        return getattr(_pt, _PRIMS[name])
    if kind == "string":
        return str
    if kind == "sequence":
        inner = _spec_to_pycdr2(spec[1])
        return _pt.sequence[inner]
    if kind == "array":
        inner = _spec_to_pycdr2(spec[1])
        return _pt.array[inner, spec[2]]
    if kind == "ref":
        raise NotImplementedError(
            f"struct reference {spec[1]!r} — nested user types aren't supported by this parser yet"
        )
    raise ValueError(f"bad type spec: {spec!r}")


def _build_pycdr2_class(fq_name: str, fields: list[tuple[str, Any]]):
    py_fields = {fname: _spec_to_pycdr2(fspec) for fname, fspec in fields}
    simple = fq_name.split("::")[-1]
    return _make_idl_struct(class_name=simple, typename=fq_name, fields=py_fields)


# ---------------------------------------------------------------------------
# Dynamic-class registry
# ---------------------------------------------------------------------------

_DYNAMIC: dict[str, Any] = {}


def register_schema(schema_name: str, idl_text: str) -> bool:
    """Parse `idl_text`, build a pycdr2 class, cache it. Returns True on success.

    Silent no-op (returns False) if pycdr2 isn't installed or the parse fails —
    the caller will just fall back to the hardcoded decoder.
    """
    if not _PYCDR2_OK:
        return False
    try:
        parsed = _parse_idl(idl_text)
    except Exception:
        return False
    if schema_name not in parsed:
        return False
    try:
        _DYNAMIC[schema_name] = _build_pycdr2_class(schema_name, parsed[schema_name])
        return True
    except Exception:
        _DYNAMIC.pop(schema_name, None)
        return False


def register_from_mcap_summary(summary) -> dict[str, str]:
    """Register every omgidl schema in the summary. Returns {name: status}.

    `status` is one of: "dynamic" (pycdr2), "hardcoded" (fallback already covers it),
    or "error: <msg>" (nothing can decode it — caller should check this).
    """
    result: dict[str, str] = {}
    for sch in summary.schemas.values():
        if sch.encoding != "omgidl":
            result[sch.name] = f"error: non-omgidl encoding {sch.encoding!r}"
            continue
        if register_schema(sch.name, sch.data.decode("utf-8", errors="replace")):
            result[sch.name] = "dynamic"
        elif sch.name in _HARDCODED:
            result[sch.name] = "hardcoded"
        else:
            result[sch.name] = "error: IDL parse failed and no hardcoded fallback"
    return result


# ---------------------------------------------------------------------------
# Hardcoded CDR fallback decoders — covering the 7 baseline schemas
# ---------------------------------------------------------------------------
# Kept as a second implementation so that:
#   (a) pycdr2 need not be installed to use this script set against a Leju bag
#       containing only the 7 known schemas.
#   (b) The IDL parser can be regression-checked against a known-good reference.
#
# Every decoder returns the SAME dict shape as the dynamic path for these schemas.
# CDR alignment is computed stream-relative (see `_CDRCursor`).


class _CDRCursor:
    ENCAP_HEADER = 4

    def __init__(self, data: bytes):
        if len(data) < self.ENCAP_HEADER:
            raise ValueError(f"payload too short for CDR encapsulation: {len(data)} bytes")
        self._data = data
        self.pos = 0

    @property
    def _raw(self) -> int:
        return self.ENCAP_HEADER + self.pos

    def align(self, to: int) -> None:
        self.pos += (-self.pos) & (to - 1)

    def skip(self, n: int) -> None:
        self.pos += n

    def read_i32(self) -> int:
        self.align(4)
        v = struct.unpack_from("<i", self._data, self._raw)[0]
        self.pos += 4
        return v

    def read_u32(self) -> int:
        self.align(4)
        v = struct.unpack_from("<I", self._data, self._raw)[0]
        self.pos += 4
        return v

    def read_f32(self) -> float:
        self.align(4)
        v = struct.unpack_from("<f", self._data, self._raw)[0]
        self.pos += 4
        return v

    def read_f64(self) -> float:
        self.align(8)
        v = struct.unpack_from("<d", self._data, self._raw)[0]
        self.pos += 8
        return v

    def read_seq_f64(self) -> list[float]:
        n = self.read_u32()
        if n == 0:
            return []
        self.align(8)
        out = list(struct.unpack_from(f"<{n}d", self._data, self._raw))
        self.pos += 8 * n
        return out

    def read_seq_f32(self) -> list[float]:
        n = self.read_u32()
        if n == 0:
            return []
        self.align(4)
        out = list(struct.unpack_from(f"<{n}f", self._data, self._raw))
        self.pos += 4 * n
        return out

    def read_seq_i32(self) -> list[int]:
        n = self.read_u32()
        if n == 0:
            return []
        self.align(4)
        out = list(struct.unpack_from(f"<{n}i", self._data, self._raw))
        self.pos += 4 * n
        return out

    def read_seq_u8(self) -> list[int]:
        n = self.read_u32()
        if n == 0:
            return []
        out = list(struct.unpack_from(f"<{n}B", self._data, self._raw))
        self.pos += n
        return out

    def read_string(self) -> str:
        n = self.read_u32()
        if n == 0:
            return ""
        raw = self._data[self._raw : self._raw + n]
        self.pos += n
        if raw.endswith(b"\x00"):
            raw = raw[:-1]
        return raw.decode("utf-8", errors="replace")


def _read_header(c: _CDRCursor) -> tuple[int, int]:
    return c.read_i32(), c.read_u32()


def _decode_float64(data: bytes) -> dict:
    c = _CDRCursor(data)
    hs, hn = _read_header(c)
    return {"header_sec": hs, "header_nanosec": hn, "data": c.read_f64()}


def _decode_float64_array(data: bytes) -> dict:
    c = _CDRCursor(data)
    hs, hn = _read_header(c)
    return {"header_sec": hs, "header_nanosec": hn, "data": c.read_seq_f64()}


def _decode_string_data(data: bytes) -> dict:
    c = _CDRCursor(data)
    hs, hn = _read_header(c)
    return {"header_sec": hs, "header_nanosec": hn, "data": c.read_string()}


def _decode_joint_state(data: bytes) -> dict:
    c = _CDRCursor(data)
    hs, hn = _read_header(c)
    return {
        "header_sec": hs, "header_nanosec": hn,
        "q":   c.read_seq_f64(),
        "v":   c.read_seq_f64(),
        "vd":  c.read_seq_f64(),
        "tau": c.read_seq_f64(),
    }


def _decode_joint_cmd(data: bytes) -> dict:
    c = _CDRCursor(data)
    hs, hn = _read_header(c)
    return {
        "header_sec": hs, "header_nanosec": hn,
        "q":     c.read_seq_f64(),
        "v":     c.read_seq_f64(),
        "tau":   c.read_seq_f64(),
        "kp":    c.read_seq_f64(),
        "kd":    c.read_seq_f64(),
        "modes": c.read_seq_u8(),
    }


def _decode_imu_data(data: bytes) -> dict:
    c = _CDRCursor(data)
    hs, hn = _read_header(c)
    return {
        "header_sec": hs, "header_nanosec": hn,
        "gyro":     [c.read_f64() for _ in range(3)],
        "acc":      [c.read_f64() for _ in range(3)],
        "free_acc": [c.read_f64() for _ in range(3)],
        "quat":     [c.read_f64() for _ in range(4)],
    }


def _decode_joy(data: bytes) -> dict:
    c = _CDRCursor(data)
    hs, hn = _read_header(c)
    return {
        "header_sec": hs, "header_nanosec": hn,
        "axes":    c.read_seq_f32(),
        "buttons": c.read_seq_i32(),
    }


_HARDCODED: dict[str, Any] = {
    "leju::msgs::Float64":      _decode_float64,
    "leju::msgs::Float64Array": _decode_float64_array,
    "leju::msgs::StringData":   _decode_string_data,
    "leju::msgs::JointState":   _decode_joint_state,
    "leju::msgs::JointCmd":     _decode_joint_cmd,
    "leju::msgs::ImuData":      _decode_imu_data,
    "leju::msgs::Joy":          _decode_joy,
}


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def is_supported(schema_name: str) -> bool:
    return schema_name in _DYNAMIC or schema_name in _HARDCODED


def supported_schemas() -> frozenset[str]:
    return frozenset(_DYNAMIC) | frozenset(_HARDCODED)


def _to_plain(obj: Any) -> Any:
    """Recursively turn an IdlStruct tree into dict/list/primitive."""
    if isinstance(obj, list):
        return [_to_plain(x) for x in obj]
    if isinstance(obj, tuple):
        return [_to_plain(x) for x in obj]
    # IdlStruct instances have __dict__ of plain fields
    if hasattr(obj, "__dict__") and not isinstance(obj, type):
        return {k: _to_plain(v) for k, v in obj.__dict__.items()}
    return obj


def decode(schema_name: str, data: bytes) -> dict:
    """Decode `data` using the dynamic pycdr2 class if registered, else the hardcoded fallback.

    Always returns a plain dict (with IDL field names as keys). Raises
    `UnsupportedSchema` if neither path can handle this schema.
    """
    if schema_name in _DYNAMIC:
        obj = _DYNAMIC[schema_name].deserialize(data)
        return _to_plain(obj)
    if schema_name in _HARDCODED:
        return _HARDCODED[schema_name](data)
    raise UnsupportedSchema(
        f"no decoder for schema {schema_name!r}. "
        f"Did you forget leju_msgs.register_from_mcap_summary(summary)? "
        f"Known hardcoded: {sorted(_HARDCODED)}"
    )
