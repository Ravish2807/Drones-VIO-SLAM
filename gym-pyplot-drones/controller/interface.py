"""Shared controller input/output types for fair controller substitution."""
from dataclasses import dataclass, field
import numpy as np


@dataclass(frozen=True)
class ControlState:
    position: np.ndarray
    velocity: np.ndarray
    rotation: np.ndarray
    angular_velocity: np.ndarray

    @classmethod
    def from_mapping(cls, state):
        return cls(*(np.asarray(state[k],float).copy() for k in ("position","velocity","R","omega")))


@dataclass(frozen=True)
class DesiredState:
    position: np.ndarray
    velocity: np.ndarray = field(default_factory=lambda:np.zeros(3))
    acceleration: np.ndarray = field(default_factory=lambda:np.zeros(3))
    yaw: float = 0.0
    rotation: np.ndarray | None = None
    angular_velocity: np.ndarray = field(default_factory=lambda:np.zeros(3))
    thrust: float | None = None
    attitude_only: bool = False

    @classmethod
    def from_mapping(cls, desired):
        return cls(position=np.asarray(desired["position"],float),
                   velocity=np.asarray(desired.get("velocity",np.zeros(3)),float),
                   acceleration=np.asarray(desired.get("acceleration",np.zeros(3)),float),
                   yaw=float(desired.get("yaw",0.)),
                   rotation=None if desired.get("R") is None else np.asarray(desired["R"],float),
                   angular_velocity=np.asarray(desired.get("omega",np.zeros(3)),float),
                   thrust=None if desired.get("thrust") is None else float(desired["thrust"]),
                   attitude_only=bool(desired.get("attitude_only",False)))


@dataclass(frozen=True)
class ControlOutput:
    """Collective thrust and body torque, plus logged controller diagnostics."""
    thrust: float
    torque: np.ndarray
    desired_rotation: np.ndarray
    position_error: np.ndarray
    velocity_error: np.ndarray
    desired_acceleration: np.ndarray
    desired_force: np.ndarray
    attitude_error: np.ndarray
    angular_velocity_error: np.ndarray
    terms: dict = field(default_factory=dict)


class Controller:
    """Protocol-like base contract: compute(state, desired) returns ControlOutput."""
    name = "controller"
    def compute(self, state: ControlState, desired: DesiredState) -> ControlOutput:
        raise NotImplementedError
