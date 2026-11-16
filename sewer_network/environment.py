import manifold3d as m3d
from pxr import UsdGeom
from cutting import Cutter


class Environment:
    """The ground surface and the soil around the network."""

    def __init__(self, stage, ground_z=0.0):
        self.stage = stage
        self.ground_z = ground_z              # height of the ground surface
        self.cutter = Cutter(stage)

    def _box_with_holes(self, bounds, z_top, z_bottom, holes, segments=32):
        xmin, ymin, xmax, ymax = bounds
        height = z_top - z_bottom
        box = m3d.Manifold.cube((xmax - xmin, ymax - ymin, height)).translate((xmin, ymin, z_bottom))
        for (x, y, r) in holes:               # one round hole per manhole, all the way through
            hole = m3d.Manifold.cylinder(height + 2, r, r, segments, False)
            box = box - hole.translate((x, y, z_bottom - 1))
        return box

    def create_ground(self, prim_path, bounds, holes, thickness=0.2):
        """A slab whose top is at ground level, with a hole where each manhole comes up."""
        UsdGeom.Xform.Define(self.stage, prim_path)
        box = self._box_with_holes(bounds, self.ground_z, self.ground_z - thickness, holes)
        self.cutter.write_manifold(prim_path + "/Geometry", box)

    def create_soil(self, prim_path, bounds, holes, depth, ground_thickness=0.2):
        """A block of soil from under the ground slab down to `depth` below ground level."""
        UsdGeom.Xform.Define(self.stage, prim_path)
        box = self._box_with_holes(bounds, self.ground_z - ground_thickness,
                                   self.ground_z - depth, holes)
        self.cutter.write_manifold(prim_path + "/Geometry", box)