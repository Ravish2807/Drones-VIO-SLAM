from enum import Enum, auto


class FlightState(Enum):
    WAIT_FOR_CONNECTION = auto()
    PREARM = auto()
    GUIDED = auto()
    ARMED = auto()
    TAKEOFF = auto()
    FLYING = auto()
    POSITION_CONTROL = auto()
    TARGET_REACHED = auto()
    HOLD = auto()
    FAILSAFE = auto()


class FlightManager:

    def __init__(self, position_tolerance: float = 0.10, velocity_tolerance: float = 0.05):
        self.state = FlightState.WAIT_FOR_CONNECTION
        self.position_tolerance = float(position_tolerance)
        self.velocity_tolerance = float(velocity_tolerance)

    def update(
        self,
        is_connected: bool,
        target_reached: bool,
        armed: bool = False,
        flying: bool = False,
        failsafe: bool = False
    ) -> FlightState:

        if failsafe:
            self.state = FlightState.FAILSAFE
            return self.state

        if not is_connected:
            self.state = FlightState.WAIT_FOR_CONNECTION
            return self.state

        if self.state == FlightState.WAIT_FOR_CONNECTION:
            self.state = FlightState.PREARM

        if self.state in [FlightState.PREARM, FlightState.GUIDED]:
            if armed:
                self.state = FlightState.ARMED
            else:
                self.state = FlightState.GUIDED

        if self.state == FlightState.ARMED:
            if flying:
                self.state = FlightState.FLYING
            else:
                self.state = FlightState.POSITION_CONTROL

        if self.state in [FlightState.FLYING, FlightState.POSITION_CONTROL]:
            if target_reached:
                self.state = FlightState.TARGET_REACHED

        elif self.state == FlightState.TARGET_REACHED:
            if not target_reached:
                self.state = FlightState.POSITION_CONTROL

        return self.state

    def trigger_failsafe(self):
        self.state = FlightState.FAILSAFE
