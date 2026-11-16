import numpy as np
import manifold3d as m3d
from pxr import Usd, UsdGeom, Gf


class Cutter:
    """Cuts openings in meshes already on the stage, using manifold3d."""

    def __init__(self, stage):
        self.stage = stage

    def cut_opening(self, mesh_prim_path, position, rotation, radius, wall_thickness):
        """
        Cut a round hole through the wall of the mesh at mesh_prim_path.
        position  : world position of the hole, on the inner surface of the wall
        rotation  : Gf.Rotation that points the hole's axis through the wall
        radius    : radius of the hole
        """
        cutter = self._create_cutter(radius, wall_thickness)

        local_pos = self._get_parent_to_local(mesh_prim_path).Transform(position)
        R = Gf.Matrix3d(rotation)
        transform = np.array(
            [[R[j][i] for j in range(3)] + [local_pos[i]] for i in range(3)],
            dtype=np.float64
        )
        cutter = cutter.transform(transform)

        mesh_geom_path = mesh_prim_path + "/Geometry"
        result = self._usd_mesh_to_manifold(mesh_geom_path) - cutter
        self.write_manifold(mesh_geom_path, result)

    def _get_parent_to_local(self, prim_path):
        prim = self.stage.GetPrimAtPath(prim_path)
        local = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        return local.GetInverse()

    def _create_cutter(self, radius, wall_thickness, margin=0.1):
        cutter = m3d.Manifold.cylinder(
            wall_thickness + 2 * margin,     # sticks out by `margin` on both sides of the wall
            radius, radius, 32, False
        )
        return cutter.translate((0, 0, -margin))

    def _usd_mesh_to_manifold(self, mesh_path):
        mesh = UsdGeom.Mesh.Get(self.stage, mesh_path)
        points = mesh.GetPointsAttr().Get()
        face_counts = mesh.GetFaceVertexCountsAttr().Get()
        face_indices = mesh.GetFaceVertexIndicesAttr().Get()

        vertices = np.array([[p[0], p[1], p[2]] for p in points], dtype=np.float32)

        triangles, offset = [], 0
        for count in face_counts:
            face = face_indices[offset:offset + count]
            offset += count
            for i in range(1, count - 1):          # fan-triangulate each polygon
                triangles.append([face[0], face[i], face[i + 1]])

        return m3d.Manifold(m3d.Mesh(
            vert_properties=vertices,
            tri_verts=np.array(triangles, dtype=np.uint32)
        ))

    def write_manifold(self, mesh_path, manifold):
        res = manifold.to_mesh()
        mesh = UsdGeom.Mesh.Define(self.stage, mesh_path)
        mesh.GetPointsAttr().Set(
            [Gf.Vec3f(float(v[0]), float(v[1]), float(v[2])) for v in res.vert_properties])
        mesh.GetFaceVertexIndicesAttr().Set(res.tri_verts.flatten().tolist())
        mesh.GetFaceVertexCountsAttr().Set([3] * len(res.tri_verts))