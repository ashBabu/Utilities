import os
import math
from pxr import Usd, UsdGeom, Gf

from cutting import Cutter
from environment import Environment
from network_config import load_config
from connections import plan_connection
from materials import Materials, make_brick_texture
from geometry import Geometry, get_pipe_rotation, get_segment_end
         
class SewerNetwork:

    def __init__(self, config_path, usda_path="class_sewer_network.usda"):
        self.stage = Usd.Stage.CreateNew(usda_path)
        self.materials = Materials(self.stage)
        self.cutter = Cutter(self.stage)
        self.geometry = Geometry(self.stage)
        self.environment = Environment(self.stage)
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)  # setting the stage up axis to Z
        self.manholes, self.pipes, self.connections = load_config(config_path)

    def _get_manhole(self, name):
        for m in self.manholes:
            if m.name == name:
                return m
        raise ValueError(f"Manhole {name} not found")
    def _get_bounds(self):
        reach = max((p.segment_count * p.segment_length for p in self.pipes), default=0) + 2.0
        r = max(m.outer_dia / 2 for m in self.manholes)
        xs = [m.start[0] for m in self.manholes]
        ys = [m.start[1] for m in self.manholes]
        return (min(xs) - reach - r, min(ys) - reach - r,
                max(xs) + reach + r, max(ys) + reach + r)

    def get_pipe_start(self, pipe):
        """
        Returns a vector that is on the inner surface of the manhole where the pipe starts
        """
        m = self._get_manhole(pipe.manhole)
        r_in = m.outer_dia / 2 - m.wall_thickness      # inner wall of the manhole
        a = math.radians(pipe.angle)
        return Gf.Vec3d(
            m.start[0] + r_in * math.cos(a),
            m.start[1] + r_in * math.sin(a),
            m.start[2] - pipe.entry_depth,
        )

    def cut_pipe_opening(self, pipe):
        m = self._get_manhole(pipe.manhole)
        start = self.get_pipe_start(pipe)
        rotation = get_pipe_rotation(start, m.start, pipe.gradient, pipe.outgoing)

        self.cutter.cut_opening(
            f"/Sewer/Manholes/{m.name}",
            position=start,
            rotation=rotation,
            radius=pipe.outer_dia / 2 + 0.005,
            wall_thickness=m.wall_thickness,
        )
        
    def build_pipe(self, pipe, material):
        m = self._get_manhole(pipe.manhole)
        start = self.get_pipe_start(pipe)
        rotation = get_pipe_rotation(start, m.start, pipe.gradient, pipe.outgoing)

        for i in range(pipe.segment_count):
            seg_path = f"/Sewer/Pipes/{pipe.name}/Seg{i+1:02d}"
            self.geometry.create_pipe(
                seg_path,
                start=start,
                rotation=rotation,
                length=pipe.segment_length,
                outer_dia=pipe.outer_dia,
                wall_thickness=pipe.wall_thickness,
                socket_length=pipe.socket_length,
            )
            self.materials.assign_uvs(seg_path + "/Geometry", pipe.outer_dia)
            self.materials.bind_material(seg_path, material)
            start = get_segment_end(start, rotation, pipe.segment_length - pipe.socket_length)
    
    def build(self):
        make_brick_texture("textures/brick.png")
        concrete = self.materials.create_material("/Sewer/Looks/Concrete", (0.50, 0.50, 0.47), roughness=0.95)
        brick = self.materials.create_textured_material("/Sewer/Looks/Brick", "./textures/brick.png")
        base_mat = self.materials.create_material("/Sewer/Looks/BaseConcrete", (0.34, 0.34, 0.32), roughness=0.95)
        
        for m in self.manholes:
            path = f"/Sewer/Manholes/{m.name}"
            self.geometry.create_manhole(
                path,
                start=m.start,
                length=m.height,
                outer_dia=m.outer_dia,
                wall_thickness=m.wall_thickness,
            )
            self.geometry.create_manhole_base(path, outer_dia=m.outer_dia, thickness=m.floor_thickness)
            self.materials.bind_material(path, concrete)
            self.materials.bind_material(path + "/Base", base_mat)

        for p in self.pipes:
            self.build_pipe(p, brick)
            self.cut_pipe_opening(p)

        for c in self.connections:
            a = self._get_manhole(c.between[0])
            b = self._get_manhole(c.between[1])
            start_pipe, end_pipe = plan_connection(c, a, b)
            self.build_pipe(start_pipe, brick)      # the segments, leaving the first manhole
            self.cut_pipe_opening(start_pipe)       # opening in the first manhole
            self.cut_pipe_opening(end_pipe)         # opening in the second manhole

        bounds = self._get_bounds()
        holes = [(m.start[0], m.start[1], m.outer_dia / 2) for m in self.manholes]
        depth = max(m.height + m.floor_thickness for m in self.manholes) + 1.0   # 1 m below the deepest base

        ### Ground
        # ground_mat = self.materials.create_material("/Sewer/Looks/Ground", (0.28, 0.33, 0.18), roughness=1.0)
        # self.environment.create_ground("/Sewer/Environment/Ground", bounds, holes)
        # self.materials.bind_material("/Sewer/Environment/Ground", ground_mat)

        ### Soil
        soil_mat = self.materials.create_material("/Sewer/Looks/Soil", (0.36, 0.26, 0.17), roughness=1.0, opacity=0.25)
        self.environment.create_soil("/Sewer/Environment/Soil", bounds, holes, depth)
        self.materials.bind_material("/Sewer/Environment/Soil", soil_mat)

        self.stage.GetRootLayer().Save()


if __name__ == "__main__":
    # Get the directory where this script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Build the absolute path to your YAML file
    config = os.path.join(script_dir, "sewer_network.yaml")
    swn = SewerNetwork(config_path=config)
    swn.build()
    
