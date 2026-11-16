import os
import math
import numpy as np
from PIL import Image
from pxr import Usd, UsdGeom, Gf, UsdShade, Sdf


def tileable_noise(h, w, sigma, rng):
    """Soft random blotches that repeat seamlessly at the image edges."""
    f = np.fft.fft2(rng.random((h, w)))
    fy = np.fft.fftfreq(h)[:, None]
    fx = np.fft.fftfreq(w)[None, :]
    f *= np.exp(-2 * (np.pi * sigma) ** 2 * (fx ** 2 + fy ** 2))
    n = np.real(np.fft.ifft2(f))
    return (n - n.min()) / (n.max() - n.min())

def make_brick_texture(path, nx=4, ny=4, bw=320, ch=112, mortar=14, seed=1):
    rng = np.random.default_rng(seed)
    W, H = nx * bw, ny * ch                         # tile = 4 bricks wide, 4 courses high
    y, x = np.mgrid[0:H, 0:W]

    course = y // ch                                # which row of bricks
    xs = (x - (course % 2) * (bw // 2)) % W         # every 2nd row shifted half a brick
    in_bed = (y % ch < mortar // 2) | (y % ch >= ch - mortar // 2)       # horizontal mortar
    in_head = (xs % bw < mortar // 2) | (xs % bw >= bw - mortar // 2)    # vertical mortar
    is_mortar = in_bed | in_head

    brick_id = (course * nx + xs // bw).astype(int)
    tint = rng.uniform(0.8, 1.2, size=nx * ny)[brick_id]    # each brick a bit different

    blotch = tileable_noise(H, W, 40, rng)          # big stains
    grain = tileable_noise(H, W, 2, rng)            # fine grain

    brick_col = np.array([0.42, 0.40, 0.34])
    mortar_col = np.array([0.58, 0.58, 0.55])
    col = np.where(is_mortar[..., None], mortar_col, brick_col * tint[..., None])
    col = col * (0.65 + 0.5 * blotch[..., None]) * (0.9 + 0.2 * grain[..., None])

    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    Image.fromarray((np.clip(col, 0, 1) * 255).astype(np.uint8)).save(path)


class Materials:

    def __init__(self, stage):
        self.stage = stage

    
    def create_material(self, path, color, roughness=0.9, opacity=1.0):
        material = UsdShade.Material.Define(self.stage, path)
        shader = UsdShade.Shader.Define(self.stage, path + "/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        return material

    def bind_material(self, prim_path, material):
        mesh_prim = self.stage.GetPrimAtPath(prim_path + "/Geometry")
        UsdShade.MaterialBindingAPI.Apply(mesh_prim).Bind(material)

    def create_textured_material(self, path, texture_file, roughness=0.9):
        material = UsdShade.Material.Define(self.stage, path)

        st = UsdShade.Shader.Define(self.stage, path + "/STReader")     # reads the UVs
        st.CreateIdAttr("UsdPrimvarReader_float2")
        st.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        st.CreateOutput("result", Sdf.ValueTypeNames.Float2)

        tex = UsdShade.Shader.Define(self.stage, path + "/Texture")     # reads the image
        tex.CreateIdAttr("UsdUVTexture")
        tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(texture_file))
        tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st.ConnectableAPI(), "result")
        tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
        tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
        tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)

        shader = UsdShade.Shader.Define(self.stage, path + "/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        return material

    def assign_uvs(self, mesh_path, outer_dia, nx=4, ny=4, brick_len=0.225, course_h=0.075):
        mesh = UsdGeom.Mesh.Get(self.stage, mesh_path)
        points = mesh.GetPointsAttr().Get()
        counts = mesh.GetFaceVertexCountsAttr().Get()
        indices = mesh.GetFaceVertexIndicesAttr().Get()

        # how many times the image repeats around the pipe (a whole number, so the ends meet)
        tiles_u = max(1, round(math.pi * outer_dia / (nx * brick_len)))
        tile_h = ny * course_h                      # height of one image, in metres

        uvs, pos = [], 0
        for c in counts:
            face = []
            for idx in indices[pos:pos + c]:
                p = points[idx]
                u = (math.atan2(p[1], p[0]) / (2 * math.pi)) % 1.0    # 0 to 1 around the pipe
                face.append([u, p[2]])                                # z = distance along the pipe
            pos += c

            if max(f[0] for f in face) - min(f[0] for f in face) > 0.5:   # face crosses the seam
                for f in face:
                    if f[0] < 0.5:
                        f[0] += 1.0

            for u, z in face:
                uvs.append(Gf.Vec2f(u * tiles_u, z / tile_h))

        st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying)
        st.Set(uvs)