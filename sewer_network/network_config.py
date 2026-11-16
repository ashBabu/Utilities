import yaml
from pxr import Gf
from fractions import Fraction
from dataclasses import dataclass


@dataclass
class Manhole:
    name: str
    start: Gf.Vec3d
    outer_dia: float
    height: float
    wall_thickness: float
    floor_thickness: float


@dataclass
class Pipe:
    name: str
    manhole: str
    entry_depth: float
    segment_length: float
    segment_count: int
    socket_length: float
    outer_dia: float
    wall_thickness: float
    angle: float
    outgoing: bool
    gradient: float

@dataclass
class Connection:
    name: str
    between: tuple
    entry_depth: float
    segment_length: float
    socket_length: float
    outer_dia: float
    wall_thickness: float
    gradient: float


def load_config(config_path):
    with open(config_path) as f:
        config = yaml.safe_load(f)

    manholes, pipes, connections = [], [], []

    for name, p in config.items():
        if name.startswith("Manhole"):
            manholes.append(Manhole(
                name=name,
                start=Gf.Vec3d(*p["start_coordinate"]),
                outer_dia=p["outer_diameter"],
                height=p["height"],
                wall_thickness=p["wall_thickness"],
                floor_thickness=p["floor_thickness"],
            ))
        elif name.startswith("Pipe"):
            if "between" in p:
                connections.append(Connection(
                    name=name,
                    between=tuple(p["between"]),
                    entry_depth=p["entry_depth"],
                    segment_length=p["segment_length"],
                    socket_length=p["socket_length"],
                    outer_dia=p["outer_diameter"],
                    wall_thickness=p["wall_thickness"],
                    gradient=float(Fraction(str(p["gradient"]))),
                ))
            else:
                pipes.append(Pipe(
                name=name,
                manhole=p["manhole"],
                entry_depth=p["entry_depth"],
                segment_length=p["segment_length"],
                segment_count=p["segment_count"],
                socket_length=p["socket_length"],
                outer_dia=p["outer_diameter"],
                wall_thickness=p["wall_thickness"],
                angle=p["angle"],
                outgoing=p["outgoing"],
                gradient=float(Fraction(str(p["gradient"]))),
            ))
    return manholes, pipes, connections