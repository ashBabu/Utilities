import math
from pxr import UsdGeom, Gf

def get_segment_end(start, rotation, segment_length):
        axis = rotation.TransformDir(Gf.Vec3d(0, 0, 1))   # where the pipe's local Z points in the world
        return start + axis * segment_length

def get_pipe_rotation(start, manhole_center, gradient, outgoing=True):
    """
    Outgoing: pipes that go from the manhole taking the water out, hence sloping down.
    The pipe is created with its axis along Z. 
    start: is a point on the inner circumference on the manhole where the pipe joins it
        ((manhole_outer_radius - wall_thickness) * cos/sin(theta) and -entry_depth being it
        distance below the ground)
    rotation_angle: Pipe needs to be rotated along its local y axis
    """
    if outgoing:
        rotation_angle = 90 + math.degrees(math.atan(gradient))
    else:
        rotation_angle = 90 - math.degrees(math.atan(gradient))

    local_pipe_z_axis = Gf.Vec3d(0, 0, 1)
    local_pipe_x_axis = Gf.Vec3d(
        start[0] - manhole_center[0], 
        start[1] - manhole_center[1], 
        0
    )

    rotation_axis = Gf.Cross(local_pipe_z_axis, local_pipe_x_axis)

    if rotation_axis.GetLength() < 1e-9:
        raise ValueError("Invalid pipe direction")

    rotation_axis.Normalize()

    return Gf.Rotation(rotation_axis, rotation_angle)


class Geometry:
    """Builds the manhole and pipe shapes on the stage."""

    def __init__(self, stage):
        self.stage = stage

    @staticmethod
    def _check_wall(outer_dia, wall_thickness):
        if wall_thickness <= 0 or wall_thickness >= outer_dia / 2:
            raise ValueError("Wall thickness must be between 0 and the radius")

    def create_revolved_solid(self, prim_path, profile, segments=32):
        # profile = list of (radius, z) points, counter-clockwise around the wall's
        # cross-section (solid on your left as you walk the outline).
        # A point with radius 0 sits on the axis, so it gets a single vertex.
        UsdGeom.Xform.Define(self.stage, prim_path)
        mesh = UsdGeom.Mesh.Define(self.stage, prim_path + '/Geometry')

        points, face_counts, face_indices = [], [], []
        ring_start = []                              # index of each ring's first vertex

        for (r, z) in profile:
            ring_start.append(len(points))
            if abs(r) < 1e-9:                        # on the axis: one vertex is enough
                points.append(Gf.Vec3f(0, 0, z))
            else:                                    # otherwise a full ring of points
                for i in range(segments):
                    angle = 2 * math.pi * i / segments
                    points.append(Gf.Vec3f(r * math.cos(angle), r * math.sin(angle), z))

        def vertex(k, i):
            on_axis = abs(profile[k][0]) < 1e-9
            return ring_start[k] + (0 if on_axis else i)

        n = len(profile)
        for k in range(n):                           # join each ring to the next ring
            k2 = (k + 1) % n
            for i in range(segments):
                j = (i + 1) % segments
                corners = [vertex(k, i), vertex(k, j), vertex(k2, j), vertex(k2, i)]
                # next to the axis, corners repeat: the quad becomes a triangle (or vanishes)
                face = [c for idx, c in enumerate(corners) if c != corners[idx - 1]]
                if len(face) >= 3:
                    face_counts.append(len(face))
                    face_indices.extend(face)

        mesh.GetPointsAttr().Set(points)
        mesh.GetFaceVertexCountsAttr().Set(face_counts)
        mesh.GetFaceVertexIndicesAttr().Set(face_indices)

    def create_hollow_cylinder(self, prim_path, length, outer_dia, wall_thickness, segments=32):
        self._check_wall(outer_dia, wall_thickness)
        r_out = outer_dia / 2
        r_in = r_out - wall_thickness
        profile = [(r_out, 0), (r_out, length), (r_in, length), (r_in, 0)]
        self.create_revolved_solid(prim_path, profile, segments)

    def create_manhole(self, prim_path, start, length, outer_dia, wall_thickness, segments=32):
        """Currently this is assumed to be a vertical cylinder closed at the bottom."""
        self.create_hollow_cylinder(prim_path, length, outer_dia, wall_thickness, segments)
        xformable = UsdGeom.Xformable(self.stage.GetPrimAtPath(prim_path))
        xformable.AddTranslateOp().Set(start - Gf.Vec3d(0, 0, length))   # to make it underground

    def create_manhole_base(self, manhole_path, outer_dia, thickness, overhang=0.1, segments=32):
        """
        A solid slab under the manhole, made as a child of the manhole prim so it moves
        with it. It sticks out `overhang` past the wall, which gives a visible edge.
        """
        r = outer_dia / 2 + overhang
        profile = [(0, -thickness), (r, -thickness), (r, 0), (0, 0)]   # just below the manhole's z = 0
        self.create_revolved_solid(manhole_path + "/Base", profile, segments)

    def create_pipe(self, prim_path, start, rotation, length, outer_dia, wall_thickness,
                    socket_length=0.1, socket_clearance=0.005, segments=32):
        self._check_wall(outer_dia, wall_thickness)
        """
            ┆       E┌──<─┐D         z = L    socket end face
            ┆ G     F│░░░░│
            ┆ ┌───<──┘░░░░│          z = z_top   shoulder top (next pipe butts here)
            ┆ │░░░░░░░░░░░│
            ┆ │░░░░┌───>──┘C         z = z_bot   shoulder underside
            ┆ │░░░░│B
            ┆H└──>─┘A                z = 0
        The dotted vertical line is the pipe axis. Point A (r_out, z=0). 
        Travel anticlockwise from A --> B --> C --> D --> E --> F --> G --> H to get the profile
        """
        r_out = outer_dia / 2
        r_in = r_out - wall_thickness
        r_sock_in = r_out + socket_clearance            # socket bore: just big enough for the next pipe
        r_sock_out = r_sock_in + wall_thickness         # socket outer wall
        z_top = length - socket_length                  # where the step happens
        z_bot = z_top - wall_thickness                  # underside of the shoulder


        profile = [
            (r_out, 0), (r_out, z_bot),               # normal pipe outside
            (r_sock_out, z_bot), (r_sock_out, length),# step out, then along the socket
            (r_sock_in, length), (r_sock_in, z_top),  # socket end face, then inside the socket
            (r_in, z_top), (r_in, 0),                 # step back in, then the normal inside
        ]
        self.create_revolved_solid(prim_path, profile, segments)

        xformable = UsdGeom.Xformable(self.stage.GetPrimAtPath(prim_path))
        xformable.AddTranslateOp().Set(start)
        xformable.AddOrientOp(
            precision=UsdGeom.XformOp.PrecisionDouble).Set(rotation.GetQuat())