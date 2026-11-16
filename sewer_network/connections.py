import math
from network_config import Pipe


def plan_connection(conn, manhole_a, manhole_b):
    """
    Turn a Connection into two Pipe descriptions that the rest of the code already understands:
      start_pipe : leaves manhole_a heading towards manhole_b (this one gets the segments)
      end_pipe   : where the run arrives at manhole_b (only used to cut the opening there)
    """
    dx = manhole_b.start[0] - manhole_a.start[0]
    dy = manhole_b.start[1] - manhole_a.start[1]
    distance = math.hypot(dx, dy)
    angle = math.degrees(math.atan2(dy, dx))            # direction from A to B

    # the pipe runs from the inner wall of A to the inner wall of B
    r_in_a = manhole_a.outer_dia / 2 - manhole_a.wall_thickness
    r_in_b = manhole_b.outer_dia / 2 - manhole_b.wall_thickness
    run = distance - r_in_a - r_in_b
    if run <= 0:
        raise ValueError(f"{conn.name}: the manholes are too close to connect")

    axis_length = run / math.cos(math.atan(conn.gradient))   # length along the tilted pipe
    drop = run * conn.gradient                                # how much lower it is at B

    # Each segment adds `pitch` to the run, and the last socket adds `socket_length` once
    s = conn.socket_length
    count = max(1, round((axis_length - s) / (conn.segment_length - s)))
    pitch = (axis_length - s) / count                         # stretched a little to fit exactly
    segment_length = pitch + s
    if pitch <= conn.wall_thickness:
        raise ValueError(f"{conn.name}: segments are too short for this gap")

    # depth of the pipe's centre line below ground, at each end
    depth_a = conn.entry_depth
    depth_b = depth_a + drop + (manhole_b.start[2] - manhole_a.start[2])
    r = conn.outer_dia / 2
    for m, depth in ((manhole_a, depth_a), (manhole_b, depth_b)):
        if depth - r < 0 or depth + r > m.height:
            raise ValueError(f"{conn.name}: pipe would be above ground or below the floor "
                             f"of {m.name} (depth {depth:.2f} m, manhole height {m.height} m)")

    common = dict(segment_length=segment_length, socket_length=s, outer_dia=conn.outer_dia,
                  wall_thickness=conn.wall_thickness, gradient=conn.gradient)
    start_pipe = Pipe(name=conn.name, manhole=manhole_a.name, entry_depth=depth_a,
                      segment_count=count, angle=angle, outgoing=True, **common)
    end_pipe = Pipe(name=conn.name + "_end", manhole=manhole_b.name, entry_depth=depth_b,
                    segment_count=0, angle=angle + 180, outgoing=False, **common)
    return start_pipe, end_pipe