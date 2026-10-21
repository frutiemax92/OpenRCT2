"""Minimal glTF 2.0 binary (.glb) writer for the placeholder 3D models (R3D-001, GLF-030).

Models are made of boxes hung on a tree of nodes; clips rotate or move those nodes. Model space is glTF: +Y up and the
model faces -X (the game turns it by the yaw of the entity)."""

import json
import math
import struct


def quat_z(degrees):
    half = math.radians(degrees) / 2
    return [0.0, 0.0, math.sin(half), math.cos(half)]


def quat_x(degrees):
    half = math.radians(degrees) / 2
    return [math.sin(half), 0.0, 0.0, math.cos(half)]


class Builder:
    def __init__(self):
        self.buffer = bytearray()
        self.views = []
        self.accessors = []
        self.materials = []
        self.meshes = []
        self.nodes = []
        self.animations = []

    def _view(self, data, target=None):
        while len(self.buffer) % 4:
            self.buffer.append(0)
        view = {"buffer": 0, "byteOffset": len(self.buffer), "byteLength": len(data)}
        if target:
            view["target"] = target
        self.buffer.extend(data)
        self.views.append(view)
        return len(self.views) - 1

    def _accessor(self, view, component, count, kind, minimum=None, maximum=None):
        accessor = {"bufferView": view, "componentType": component, "count": count, "type": kind}
        if minimum is not None:
            accessor["min"] = minimum
            accessor["max"] = maximum
        self.accessors.append(accessor)
        return len(self.accessors) - 1

    def material(self, name, colour):
        self.materials.append({
            "name": name,
            "pbrMetallicRoughness": {
                "baseColorFactor": [colour[0] / 255, colour[1] / 255, colour[2] / 255, 1.0],
                "metallicFactor": 0.0,
                "roughnessFactor": 1.0,
            },
        })
        return len(self.materials) - 1

    def box(self, size, centre, material):
        """A mesh index for an axis aligned box of the given size centred on `centre` (in the node's frame)."""
        hx, hy, hz = size[0] / 2, size[1] / 2, size[2] / 2
        cx, cy, cz = centre
        faces = [
            ((1, 0, 0), [(1, -1, -1), (1, 1, -1), (1, 1, 1), (1, -1, 1)]),
            ((-1, 0, 0), [(-1, -1, 1), (-1, 1, 1), (-1, 1, -1), (-1, -1, -1)]),
            ((0, 1, 0), [(-1, 1, -1), (-1, 1, 1), (1, 1, 1), (1, 1, -1)]),
            ((0, -1, 0), [(-1, -1, 1), (-1, -1, -1), (1, -1, -1), (1, -1, 1)]),
            ((0, 0, 1), [(-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1)]),
            ((0, 0, -1), [(1, -1, -1), (-1, -1, -1), (-1, 1, -1), (1, 1, -1)]),
        ]
        positions, normals, indices = [], [], []
        for normal, corners in faces:
            base = len(positions)
            for sx, sy, sz in corners:
                positions.append((cx + sx * hx, cy + sy * hy, cz + sz * hz))
                normals.append(normal)
            indices.extend([base, base + 1, base + 2, base, base + 2, base + 3])
        pos_bytes = b"".join(struct.pack("<3f", *p) for p in positions)
        nor_bytes = b"".join(struct.pack("<3f", *n) for n in normals)
        idx_bytes = b"".join(struct.pack("<H", i) for i in indices)
        lo = [min(p[k] for p in positions) for k in range(3)]
        hi = [max(p[k] for p in positions) for k in range(3)]
        pos = self._accessor(self._view(pos_bytes, 34962), 5126, len(positions), "VEC3", lo, hi)
        nor = self._accessor(self._view(nor_bytes, 34962), 5126, len(normals), "VEC3")
        idx = self._accessor(self._view(idx_bytes, 34963), 5123, len(indices), "SCALAR")
        self.meshes.append({"primitives": [{
            "attributes": {"POSITION": pos, "NORMAL": nor}, "indices": idx, "material": material}]})
        return len(self.meshes) - 1

    def node(self, name, translation=(0, 0, 0), mesh=None, children=()):
        node = {"name": name, "children": list(children)}
        if not node["children"]:
            del node["children"]
        if any(translation):
            node["translation"] = list(translation)
        if mesh is not None:
            node["mesh"] = mesh
        self.nodes.append(node)
        return len(self.nodes) - 1

    def clip(self, name, channels):
        """channels: list of (node, "rotation"|"translation", [(time, value), ...])."""
        samplers, out = [], []
        for node, path, keys in channels:
            times = struct.pack("<%df" % len(keys), *[k[0] for k in keys])
            values = b"".join(struct.pack("<%df" % len(k[1]), *k[1]) for k in keys)
            t_acc = self._accessor(self._view(times), 5126, len(keys), "SCALAR", [keys[0][0]], [keys[-1][0]])
            v_acc = self._accessor(self._view(values), 5126, len(keys), "VEC4" if path == "rotation" else "VEC3")
            samplers.append({"input": t_acc, "output": v_acc, "interpolation": "LINEAR"})
            out.append({"sampler": len(samplers) - 1, "target": {"node": node, "path": path}})
        self.animations.append({"name": name, "samplers": samplers, "channels": out})

    def build(self, root):
        doc = {
            "asset": {"version": "2.0", "generator": "open-golf-simulator placeholders"},
            "scene": 0,
            "scenes": [{"nodes": [root]}],
            "nodes": self.nodes,
            "meshes": self.meshes,
            "materials": self.materials,
            "accessors": self.accessors,
            "bufferViews": self.views,
            "buffers": [{"byteLength": len(self.buffer)}],
            "animations": self.animations,
        }
        js = json.dumps(doc, separators=(",", ":")).encode("utf-8")
        js += b" " * (-len(js) % 4)
        binary = bytes(self.buffer) + b"\0" * (-len(self.buffer) % 4)
        total = 12 + 8 + len(js) + 8 + len(binary)
        return (struct.pack("<III", 0x46546C67, 2, total) + struct.pack("<II", len(js), 0x4E4F534A) + js
                + struct.pack("<II", len(binary), 0x004E4942) + binary)


def golfer_glb():
    """The placeholder golfer (about 1.8 m): shirt, trousers and cap are the three remappable materials."""
    b = Builder()
    skin = b.material("skin", (232, 184, 150))
    shirt = b.material("remap_primary", (200, 60, 60))
    trousers = b.material("remap_secondary", (60, 70, 140))
    cap = b.material("remap_tertiary", (240, 240, 240))
    club = b.material("club", (90, 90, 95))

    torso = b.node("torso_mesh", mesh=b.box((0.30, 0.60, 0.46), (0, 0.30, 0), shirt))
    head = b.node("head_mesh", mesh=b.box((0.24, 0.26, 0.24), (0, 0.13, 0), skin))
    cap_mesh = b.node("cap_mesh", mesh=b.box((0.28, 0.08, 0.28), (-0.02, 0.28, 0), cap))
    head_pivot = b.node("head", (0, 0.60, 0), children=[head, cap_mesh])
    chest = b.node("chest", (0, 0.78, 0), children=[torso, head_pivot])
    # Arms hang from the shoulders at the sides (+Z and -Z); they swing about the Z axis.
    arm_l_mesh = b.node("arm_l_mesh", mesh=b.box((0.12, 0.55, 0.12), (0, -0.275, 0), shirt))
    club_mesh = b.node("club_mesh", (0, -0.55, 0), mesh=b.box((0.04, 0.9, 0.04), (0, -0.2, 0), club))
    arm_l = b.node("arm_l", (0, 1.30, 0.27), children=[arm_l_mesh])
    arm_r_mesh = b.node("arm_r_mesh", mesh=b.box((0.12, 0.55, 0.12), (0, -0.275, 0), shirt))
    arm_r = b.node("arm_r", (0, 1.30, -0.27), children=[arm_r_mesh, club_mesh])
    leg_l_mesh = b.node("leg_l_mesh", mesh=b.box((0.15, 0.80, 0.15), (0, -0.40, 0), trousers))
    leg_r_mesh = b.node("leg_r_mesh", mesh=b.box((0.15, 0.80, 0.15), (0, -0.40, 0), trousers))
    leg_l = b.node("leg_l", (0, 0.82, 0.10), children=[leg_l_mesh])
    leg_r = b.node("leg_r", (0, 0.82, -0.10), children=[leg_r_mesh])
    upper = b.node("upper", (0, 0.0, 0), children=[chest, arm_l, arm_r])
    root = b.node("golfer", children=[upper, leg_l, leg_r])
    # The chest node above sits at 0.78 and the arms at 1.30 in the upper frame; legs hang from 0.82.

    def rot(node, keys):
        return (node, "rotation", [(t, quat_z(a)) for t, a in keys])

    def lift(node, keys):
        return (node, "translation", [(t, v) for t, v in keys])

    still = [(0.0, 0.0), (1.0, 0.0)]
    # Forward is -X, and a rotation about +Z by a negative angle swings a hanging limb forward.
    b.clip("idle", [rot(arm_l, [(0, 2), (1, -2), (2, 2)]), rot(arm_r, [(0, -2), (1, 2), (2, -2)])])
    b.clip("walk", [
        rot(leg_l, [(0, -28), (0.5, 28), (1, -28)]), rot(leg_r, [(0, 28), (0.5, -28), (1, 28)]),
        rot(arm_l, [(0, 25), (0.5, -25), (1, 25)]), rot(arm_r, [(0, -25), (0.5, 25), (1, -25)]),
    ])
    b.clip("sit_bench", [
        rot(leg_l, [(0, -90), (1, -90)]), rot(leg_r, [(0, -90), (1, -90)]),
        lift(root, [(0, (0, -0.42, 0)), (1, (0, -0.42, 0))]),
        rot(arm_l, [(0, -10), (1, -10)]), rot(arm_r, [(0, -10), (1, -10)]),
    ])
    b.clip("talk", [
        rot(arm_r, [(0, -70), (0.4, -100), (0.8, -70), (1.2, -100)]), rot(arm_l, [(0, 5), (1.2, 5)]),
    ])
    b.clip("swing_full", [
        rot(arm_l, [(0, -20), (0.35, 150), (0.55, -140), (0.9, -20)]),
        rot(arm_r, [(0, -20), (0.35, 150), (0.55, -140), (0.9, -20)]),
    ])
    b.clip("swing_putt", [
        rot(arm_l, [(0, -25), (0.4, -5), (0.7, -45), (1.1, -25)]),
        rot(arm_r, [(0, -25), (0.4, -5), (0.7, -45), (1.1, -25)]),
    ])
    b.clip("cheer", [
        rot(arm_l, [(0, 170), (0.3, 130), (0.6, 170)]), rot(arm_r, [(0, 170), (0.3, 130), (0.6, 170)]),
        lift(root, [(0, (0, 0, 0)), (0.3, (0, 0.1, 0)), (0.6, (0, 0, 0))]),
    ])
    b.clip("groan", [
        rot(upper, [(0, 0), (0.6, -22), (1.4, -22), (2.0, 0)]),
        rot(arm_l, [(0, 0), (0.6, -30), (1.4, -30), (2.0, 0)]),
        rot(arm_r, [(0, 0), (0.6, -30), (1.4, -30), (2.0, 0)]),
    ])
    b.clip("celebrate", [
        rot(arm_l, [(0, 175), (0.25, 120), (0.5, 175)]), rot(arm_r, [(0, 120), (0.25, 175), (0.5, 120)]),
        lift(root, [(0, (0, 0, 0)), (0.25, (0, 0.25, 0)), (0.5, (0, 0, 0))]),
    ])
    b.clip("rage_quit", [
        rot(arm_r, [(0, 150), (0.15, 100), (0.3, 150)]), rot(arm_l, [(0, 100), (0.15, 150), (0.3, 100)]),
        rot(leg_l, [(0, -35), (0.15, 20), (0.3, -35)]), rot(leg_r, [(0, 20), (0.15, -35), (0.3, 20)]),
    ])
    return b.build(root)


def vehicle_glb(length, width, body_height, roof_length, seats_cart):
    """A car or a golf cart facing -X: a body in the first remappable colour, a roof in the second, four tyres, and glass
    (cars) or seats (carts). Sizes are in metres."""
    b = Builder()
    paint = b.material("remap_primary", (60, 90, 200))
    roof_paint = b.material("remap_secondary", (235, 235, 235))
    tyre = b.material("tyre", (25, 25, 28))
    glass = b.material("glass", (150, 190, 215))
    seat = b.material("seat", (90, 70, 50))

    wheel_r = 0.17
    body_y = wheel_r + body_height / 2
    parts = [b.node("body", mesh=b.box((length, body_height, width), (0, body_y, 0), paint))]
    if seats_cart:
        # An open cart: a roof on four posts over two seats.
        parts.append(b.node("seat", mesh=b.box((0.5, 0.18, width * 0.8), (0.15, body_y + body_height / 2 + 0.09, 0), seat)))
        parts.append(b.node("roof", mesh=b.box((roof_length, 0.06, width), (0.1, body_y + body_height / 2 + 0.75, 0), roof_paint)))
        for px in (-roof_length / 2 + 0.1, roof_length / 2 - 0.1):
            for pz in (-width / 2 + 0.05, width / 2 - 0.05):
                parts.append(b.node("post", mesh=b.box((0.05, 0.7, 0.05), (0.1 + px, body_y + body_height / 2 + 0.4, pz), tyre)))
    else:
        top = body_y + body_height / 2
        parts.append(b.node("cabin", mesh=b.box((roof_length, 0.28, width * 0.9), (0.15, top + 0.14, 0), glass)))
        parts.append(b.node("roof", mesh=b.box((roof_length, 0.05, width * 0.92), (0.15, top + 0.305, 0), roof_paint)))
    for wx in (-length * 0.32, length * 0.32):
        for wz in (-width / 2, width / 2):
            parts.append(b.node("wheel", mesh=b.box((wheel_r * 2, wheel_r * 2, 0.1), (wx, wheel_r, wz), tyre)))
    root = b.node("vehicle", children=parts)
    return b.build(root)


def car_glb():
    return vehicle_glb(2.4, 1.2, 0.45, 1.2, False)


def cart_glb():
    return vehicle_glb(2.0, 1.1, 0.3, 1.5, True)


if __name__ == "__main__":
    import sys
    open(sys.argv[1], "wb").write(golfer_glb())
