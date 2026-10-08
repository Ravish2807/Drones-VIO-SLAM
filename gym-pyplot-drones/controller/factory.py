from controller.baseline.controller import BaselineController
from controller.geometric.controller import GeometricSO3Controller


def create_controller(name,config):
    if name=="baseline": return BaselineController(config)
    if name in ("geometric","geometric_so3"): return GeometricSO3Controller(config)
    raise ValueError(f"Unknown controller {name!r}; available: baseline, geometric_so3")
