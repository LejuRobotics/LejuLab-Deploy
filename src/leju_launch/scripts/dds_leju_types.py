"""Shared CycloneDDS IDL types for leju_launch debug scripts."""

from dataclasses import dataclass

from cyclonedds.idl import IdlStruct
from cyclonedds.idl.types import float64, int32, sequence, uint32


@dataclass
class Float64Array(IdlStruct, typename="leju::msgs::Float64Array"):
    header_sec: int32
    header_nanosec: uint32
    data: sequence[float64]


@dataclass
class JointTrajectoryPoint(IdlStruct, typename="leju::msgs::JointTrajectoryPoint"):
    header_sec: int32
    header_nanosec: uint32
    q: sequence[float64]
    v: sequence[float64]
    acc: sequence[float64]


@dataclass
class JointState(IdlStruct, typename="leju::msgs::JointState"):
    header_sec: int32
    header_nanosec: uint32
    q: sequence[float64]
    v: sequence[float64]
    vd: sequence[float64]
    tau: sequence[float64]
