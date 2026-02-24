#!/usr/bin/env python3
"""Self-contained Pygame/OpenGL M2 model viewer.

Launched as a subprocess from the asset pipeline GUI to avoid Tkinter/Pygame conflicts.
Supports textured rendering, skeletal animation playback, and orbit camera controls.
"""

from __future__ import annotations

import hashlib
import math
import multiprocessing
import os
import shutil
import struct
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

# ---------------------------------------------------------------------------
# Matrix math utilities (pure NumPy, no external 3D lib needed)
# ---------------------------------------------------------------------------

def perspective(fov_deg: float, aspect: float, near: float, far: float) -> np.ndarray:
    f = 1.0 / math.tan(math.radians(fov_deg) / 2.0)
    m = np.zeros((4, 4), dtype=np.float32)
    m[0, 0] = f / aspect
    m[1, 1] = f
    m[2, 2] = (far + near) / (near - far)
    m[2, 3] = (2.0 * far * near) / (near - far)
    m[3, 2] = -1.0
    return m


def look_at(eye: np.ndarray, target: np.ndarray, up: np.ndarray) -> np.ndarray:
    f = target - eye
    f = f / np.linalg.norm(f)
    s = np.cross(f, up)
    s = s / (np.linalg.norm(s) + 1e-12)
    u = np.cross(s, f)
    m = np.eye(4, dtype=np.float32)
    m[0, :3] = s
    m[1, :3] = u
    m[2, :3] = -f
    m[0, 3] = -np.dot(s, eye)
    m[1, 3] = -np.dot(u, eye)
    m[2, 3] = np.dot(f, eye)
    return m


def translate(tx: float, ty: float, tz: float) -> np.ndarray:
    m = np.eye(4, dtype=np.float32)
    m[0, 3] = tx
    m[1, 3] = ty
    m[2, 3] = tz
    return m


def scale_mat4(sx: float, sy: float, sz: float) -> np.ndarray:
    m = np.eye(4, dtype=np.float32)
    m[0, 0] = sx
    m[1, 1] = sy
    m[2, 2] = sz
    return m


def quat_to_mat4(q: np.ndarray) -> np.ndarray:
    """Quaternion (x,y,z,w) to 4x4 rotation matrix."""
    x, y, z, w = q
    m = np.eye(4, dtype=np.float32)
    m[0, 0] = 1 - 2 * (y * y + z * z)
    m[0, 1] = 2 * (x * y - z * w)
    m[0, 2] = 2 * (x * z + y * w)
    m[1, 0] = 2 * (x * y + z * w)
    m[1, 1] = 1 - 2 * (x * x + z * z)
    m[1, 2] = 2 * (y * z - x * w)
    m[2, 0] = 2 * (x * z - y * w)
    m[2, 1] = 2 * (y * z + x * w)
    m[2, 2] = 1 - 2 * (x * x + y * y)
    return m


def slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    dot = np.dot(q0, q1)
    if dot < 0:
        q1 = -q1
        dot = -dot
    dot = min(dot, 1.0)
    if dot > 0.9995:
        result = q0 + t * (q1 - q0)
        return result / np.linalg.norm(result)
    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    a = math.sin((1 - t) * theta) / sin_theta
    b = math.sin(t * theta) / sin_theta
    result = a * q0 + b * q1
    return result / np.linalg.norm(result)


# ---------------------------------------------------------------------------
# M2 Parser
# ---------------------------------------------------------------------------

@dataclass
class M2Track:
    """Parsed animation track with per-sequence timestamps and keyframes."""
    interp: int = 0
    global_sequence: int = -1
    timestamps: list[np.ndarray] = field(default_factory=list)  # list of uint32 arrays per seq
    keys: list[np.ndarray] = field(default_factory=list)        # list of value arrays per seq


@dataclass
class M2Bone:
    key_bone_id: int = -1
    flags: int = 0
    parent: int = -1
    pivot: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    translation: M2Track = field(default_factory=M2Track)
    rotation: M2Track = field(default_factory=M2Track)
    scale: M2Track = field(default_factory=M2Track)


@dataclass
class M2Submesh:
    vertex_start: int = 0
    vertex_count: int = 0
    index_start: int = 0
    index_count: int = 0


@dataclass
class M2Batch:
    submesh_index: int = 0
    texture_combo_index: int = 0


@dataclass
class M2Animation:
    anim_id: int = 0
    variation: int = 0
    duration: int = 0
    speed: float = 0.0
    flags: int = 0


class M2Parser:
    """Parse M2 binary data for rendering: vertices, UVs, normals, bones, skins, textures."""

    def __init__(self, data: bytes):
        self.data = data
        self.version = struct.unpack_from("<I", data, 4)[0]
        self.is_vanilla = self.version <= 256

        # Parsed data
        self.positions: np.ndarray = np.empty((0, 3), dtype=np.float32)
        self.normals: np.ndarray = np.empty((0, 3), dtype=np.float32)
        self.uvs: np.ndarray = np.empty((0, 2), dtype=np.float32)
        self.bone_weights: np.ndarray = np.empty((0, 4), dtype=np.uint8)
        self.bone_indices: np.ndarray = np.empty((0, 4), dtype=np.uint8)

        self.vertex_lookup: np.ndarray = np.empty(0, dtype=np.uint16)
        self.triangles: np.ndarray = np.empty(0, dtype=np.uint16)
        self.resolved_indices: np.ndarray = np.empty(0, dtype=np.uint16)  # global vertex indices
        self.submeshes: list[M2Submesh] = []
        self.batches: list[M2Batch] = []

        self.textures: list[dict] = []  # {type, flags, filename}
        self.texture_lookup: list[int] = []
        self.bone_lookup: list[int] = []
        self.bones: list[M2Bone] = []
        self.animations: list[M2Animation] = []
        self.global_sequences: list[int] = []

        self._parse()

    def _hdr(self, field_name: str) -> int:
        """Return header offset for a given field, version-gated."""
        offsets_wotlk = {
            "nGlobalSeq": 20, "ofsGlobalSeq": 24,
            "nAnims": 28, "ofsAnims": 32,
            "nBones": 44, "ofsBones": 48,
            "nVerts": 60, "ofsVerts": 64,
            "nTextures": 80, "ofsTextures": 84,
            "nTextureLookup": 128, "ofsTextureLookup": 132,
            "nBoneLookup": 120, "ofsBoneLookup": 124,
        }
        offsets_vanilla = {
            "nGlobalSeq": 20, "ofsGlobalSeq": 24,
            "nAnims": 28, "ofsAnims": 32,
            "nBones": 52, "ofsBones": 56,
            "nVerts": 68, "ofsVerts": 72,
            "nTextures": 92, "ofsTextures": 96,
            "nTextureLookup": 148, "ofsTextureLookup": 152,
            "nBoneLookup": 140, "ofsBoneLookup": 144,
        }
        table = offsets_vanilla if self.is_vanilla else offsets_wotlk
        return table[field_name]

    def _read_u32(self, offset: int) -> int:
        return struct.unpack_from("<I", self.data, offset)[0]

    def _read_m2array(self, field_name: str) -> tuple[int, int]:
        """Read count, offset for an M2Array header field."""
        n_off = self._hdr(f"n{field_name}")
        o_off = self._hdr(f"ofs{field_name}")
        n = self._read_u32(n_off)
        o = self._read_u32(o_off)
        return n, o

    def _parse(self):
        self._parse_global_sequences()
        self._parse_vertices()
        self._parse_textures()
        self._parse_texture_lookup()
        self._parse_bone_lookup()
        self._parse_animations()
        self._parse_bones()
        self._parse_skin()

    def _parse_global_sequences(self):
        n, ofs = self._read_m2array("GlobalSeq")
        if n == 0 or n > 10000 or ofs + n * 4 > len(self.data):
            return
        self.global_sequences = list(struct.unpack_from(f"<{n}I", self.data, ofs))

    def _parse_vertices(self):
        n, ofs = self._read_m2array("Verts")
        if n == 0 or n > 500000 or ofs + n * 48 > len(self.data):
            return

        # Parse all vertex fields using numpy for speed
        positions = np.empty((n, 3), dtype=np.float32)
        normals = np.empty((n, 3), dtype=np.float32)
        uvs = np.empty((n, 2), dtype=np.float32)
        bone_weights = np.empty((n, 4), dtype=np.uint8)
        bone_indices = np.empty((n, 4), dtype=np.uint8)

        for i in range(n):
            base = ofs + i * 48
            positions[i] = struct.unpack_from("<3f", self.data, base)
            bone_weights[i] = struct.unpack_from("<4B", self.data, base + 12)
            bone_indices[i] = struct.unpack_from("<4B", self.data, base + 16)
            normals[i] = struct.unpack_from("<3f", self.data, base + 20)
            uvs[i] = struct.unpack_from("<2f", self.data, base + 32)

        self.positions = positions
        self.normals = normals
        self.uvs = uvs
        self.bone_weights = bone_weights
        self.bone_indices = bone_indices

    def _parse_textures(self):
        n, ofs = self._read_m2array("Textures")
        if n == 0 or n > 1000 or ofs + n * 16 > len(self.data):
            return
        for i in range(n):
            base = ofs + i * 16
            tex_type, tex_flags = struct.unpack_from("<II", self.data, base)
            name_len, name_ofs = struct.unpack_from("<II", self.data, base + 8)
            filename = ""
            if tex_type == 0 and name_len > 1 and name_ofs + name_len <= len(self.data):
                raw = self.data[name_ofs:name_ofs + name_len]
                filename = raw.split(b"\x00", 1)[0].decode("ascii", errors="replace")
            self.textures.append({"type": tex_type, "flags": tex_flags, "filename": filename})

    def _parse_texture_lookup(self):
        n, ofs = self._read_m2array("TextureLookup")
        if n == 0 or n > 10000 or ofs + n * 2 > len(self.data):
            return
        self.texture_lookup = list(struct.unpack_from(f"<{n}H", self.data, ofs))

    def _parse_bone_lookup(self):
        n, ofs = self._read_m2array("BoneLookup")
        if n == 0 or n > 10000 or ofs + n * 2 > len(self.data):
            return
        self.bone_lookup = list(struct.unpack_from(f"<{n}H", self.data, ofs))

    def _parse_animations(self):
        n, ofs = self._read_m2array("Anims")
        if n == 0 or n > 5000:
            return
        seq_size = 68 if self.is_vanilla else 64
        if ofs + n * seq_size > len(self.data):
            return
        for i in range(n):
            base = ofs + i * seq_size
            anim_id, variation = struct.unpack_from("<HH", self.data, base)
            if self.is_vanilla:
                start_ts, end_ts = struct.unpack_from("<II", self.data, base + 4)
                duration = end_ts - start_ts
                speed = struct.unpack_from("<f", self.data, base + 12)[0]
                flags = struct.unpack_from("<I", self.data, base + 16)[0]
            else:
                duration = struct.unpack_from("<I", self.data, base + 4)[0]
                speed = struct.unpack_from("<f", self.data, base + 8)[0]
                flags = struct.unpack_from("<I", self.data, base + 12)[0]
            self.animations.append(M2Animation(
                anim_id=anim_id, variation=variation,
                duration=duration, speed=speed, flags=flags,
            ))

    def _parse_track_wotlk(self, base: int, key_size: int, key_dtype: str) -> M2Track:
        """Parse a WotLK M2TrackDisk (20 bytes) at given offset."""
        track = M2Track()
        if base + 20 > len(self.data):
            return track
        interp, global_seq = struct.unpack_from("<hh", self.data, base)
        n_ts, ofs_ts, n_keys, ofs_keys = struct.unpack_from("<IIII", self.data, base + 4)
        track.interp = interp
        track.global_sequence = global_seq

        if n_ts > 5000 or n_keys > 5000:
            return track

        # Each entry in n_ts is a sub-array header: {count(4), offset(4)}
        for s in range(n_ts):
            ts_hdr = ofs_ts + s * 8
            if ts_hdr + 8 > len(self.data):
                track.timestamps.append(np.empty(0, dtype=np.uint32))
                continue
            sub_count, sub_ofs = struct.unpack_from("<II", self.data, ts_hdr)
            if sub_count > 50000 or sub_ofs + sub_count * 4 > len(self.data):
                track.timestamps.append(np.empty(0, dtype=np.uint32))
                continue
            ts_data = np.frombuffer(self.data, dtype=np.uint32, count=sub_count, offset=sub_ofs)
            track.timestamps.append(ts_data.copy())

        for s in range(n_keys):
            key_hdr = ofs_keys + s * 8
            if key_hdr + 8 > len(self.data):
                track.keys.append(np.empty(0, dtype=np.float32))
                continue
            sub_count, sub_ofs = struct.unpack_from("<II", self.data, key_hdr)
            if sub_count > 50000 or sub_ofs + sub_count * key_size > len(self.data):
                track.keys.append(np.empty(0, dtype=np.float32))
                continue
            if key_dtype == "compressed_quat":
                raw = np.frombuffer(self.data, dtype=np.int16, count=sub_count * 4, offset=sub_ofs)
                raw = raw.reshape(sub_count, 4).astype(np.float32)
                # Decompress: (v < 0 ? v+32768 : v-32767) / 32767.0
                result = np.where(raw < 0, raw + 32768.0, raw - 32767.0) / 32767.0
                # Normalize each quaternion
                norms = np.linalg.norm(result, axis=1, keepdims=True)
                norms = np.maximum(norms, 1e-10)
                result = result / norms
                track.keys.append(result)
            elif key_dtype == "vec3":
                vals = np.frombuffer(self.data, dtype=np.float32, count=sub_count * 3, offset=sub_ofs)
                track.keys.append(vals.reshape(sub_count, 3).copy())
            elif key_dtype == "float":
                vals = np.frombuffer(self.data, dtype=np.float32, count=sub_count, offset=sub_ofs)
                track.keys.append(vals.copy())

        return track

    def _parse_track_vanilla(self, base: int, key_size: int, key_dtype: str) -> M2Track:
        """Parse a Vanilla M2TrackDiskVanilla (28 bytes) — flat arrays with M2Range indexing."""
        track = M2Track()
        if base + 28 > len(self.data):
            return track
        interp, global_seq = struct.unpack_from("<hh", self.data, base)
        n_ranges, ofs_ranges = struct.unpack_from("<II", self.data, base + 4)
        n_ts, ofs_ts = struct.unpack_from("<II", self.data, base + 12)
        n_keys, ofs_keys = struct.unpack_from("<II", self.data, base + 20)
        track.interp = interp
        track.global_sequence = global_seq

        if n_ts > 500000 or n_keys > 500000:
            return track

        # Read flat timestamp array
        all_ts = np.empty(0, dtype=np.uint32)
        if n_ts > 0 and ofs_ts + n_ts * 4 <= len(self.data):
            all_ts = np.frombuffer(self.data, dtype=np.uint32, count=n_ts, offset=ofs_ts).copy()

        # Read flat key array
        if key_dtype == "c4quat":
            all_keys_flat = np.empty(0, dtype=np.float32)
            if n_keys > 0 and ofs_keys + n_keys * 16 <= len(self.data):
                all_keys_flat = np.frombuffer(self.data, dtype=np.float32, count=n_keys * 4, offset=ofs_keys)
                all_keys_flat = all_keys_flat.reshape(n_keys, 4).copy()
        elif key_dtype == "vec3":
            all_keys_flat = np.empty((0, 3), dtype=np.float32)
            if n_keys > 0 and ofs_keys + n_keys * 12 <= len(self.data):
                all_keys_flat = np.frombuffer(self.data, dtype=np.float32, count=n_keys * 3, offset=ofs_keys)
                all_keys_flat = all_keys_flat.reshape(n_keys, 3).copy()
        else:
            all_keys_flat = np.empty(0, dtype=np.float32)
            if n_keys > 0 and ofs_keys + n_keys * key_size <= len(self.data):
                all_keys_flat = np.frombuffer(self.data, dtype=np.float32, count=n_keys, offset=ofs_keys).copy()

        # Read ranges and split into per-sequence arrays
        if n_ranges > 0 and n_ranges < 5000 and ofs_ranges + n_ranges * 8 <= len(self.data):
            for r in range(n_ranges):
                rng_start, rng_end = struct.unpack_from("<II", self.data, ofs_ranges + r * 8)
                if rng_end > rng_start and rng_end <= len(all_ts):
                    track.timestamps.append(all_ts[rng_start:rng_end])
                    if key_dtype in ("c4quat", "vec3") and rng_end <= len(all_keys_flat):
                        track.keys.append(all_keys_flat[rng_start:rng_end])
                    elif rng_end <= len(all_keys_flat):
                        track.keys.append(all_keys_flat[rng_start:rng_end])
                    else:
                        track.keys.append(np.empty(0, dtype=np.float32))
                else:
                    track.timestamps.append(np.empty(0, dtype=np.uint32))
                    track.keys.append(np.empty(0, dtype=np.float32))
        else:
            # No ranges — treat entire array as single sequence
            if len(all_ts) > 0:
                track.timestamps.append(all_ts)
                track.keys.append(all_keys_flat if len(all_keys_flat) > 0 else np.empty(0, dtype=np.float32))

        return track

    def _parse_bones(self):
        n, ofs = self._read_m2array("Bones")
        if n == 0 or n > 5000:
            return

        if self.is_vanilla:
            bone_size = 108  # No boneNameCRC, 28-byte tracks: 4+4+2+2+3×28+12=108
            for i in range(n):
                base = ofs + i * bone_size
                if base + bone_size > len(self.data):
                    break
                bone = M2Bone()
                bone.key_bone_id = struct.unpack_from("<i", self.data, base)[0]
                bone.flags = struct.unpack_from("<I", self.data, base + 4)[0]
                bone.parent = struct.unpack_from("<h", self.data, base + 8)[0]
                # submeshId at +10 (2 bytes), then 3 vanilla tracks at +12, each 28 bytes
                bone.translation = self._parse_track_vanilla(base + 12, 12, "vec3")
                bone.rotation = self._parse_track_vanilla(base + 40, 16, "c4quat")
                bone.scale = self._parse_track_vanilla(base + 68, 12, "vec3")
                # pivot at 12 + 3×28 = 96
                bone.pivot = np.array(struct.unpack_from("<3f", self.data, base + 96), dtype=np.float32)
                self.bones.append(bone)
        else:
            bone_size = 88  # WotLK with boneNameCRC
            for i in range(n):
                base = ofs + i * bone_size
                if base + bone_size > len(self.data):
                    break
                bone = M2Bone()
                bone.key_bone_id = struct.unpack_from("<i", self.data, base)[0]
                bone.flags = struct.unpack_from("<I", self.data, base + 4)[0]
                bone.parent = struct.unpack_from("<h", self.data, base + 8)[0]
                # submeshId(2) + boneNameCRC(4) = 6 more bytes before tracks
                # Tracks start at base+16, each 20 bytes
                bone.translation = self._parse_track_wotlk(base + 16, 12, "vec3")
                bone.rotation = self._parse_track_wotlk(base + 36, 8, "compressed_quat")
                bone.scale = self._parse_track_wotlk(base + 56, 12, "vec3")
                bone.pivot = np.array(struct.unpack_from("<3f", self.data, base + 76), dtype=np.float32)
                self.bones.append(bone)

    def _parse_skin(self):
        """Parse skin file (external .skin or embedded for vanilla)."""
        # This will be called externally with skin data
        pass

    def parse_skin_data(self, skin_data: bytes):
        """Parse skin file binary data for vertex lookup, triangles, submeshes, batches."""
        if len(skin_data) < 48:
            return

        off = 0
        if skin_data[:4] == b"SKIN":
            off = 4

        n_indices, ofs_indices = struct.unpack_from("<II", skin_data, off + 0)
        n_tris, ofs_tris = struct.unpack_from("<II", skin_data, off + 8)
        # Properties at +16
        n_submeshes, ofs_submeshes = struct.unpack_from("<II", skin_data, off + 24)
        n_batches, ofs_batches = struct.unpack_from("<II", skin_data, off + 32)

        if n_indices == 0 or n_indices > 500000:
            return
        if n_tris == 0 or n_tris > 500000:
            return

        # Vertex lookup
        if ofs_indices + n_indices * 2 <= len(skin_data):
            self.vertex_lookup = np.frombuffer(skin_data, dtype=np.uint16,
                                               count=n_indices, offset=ofs_indices).copy()

        # Raw triangle indices (indices into vertex_lookup)
        if ofs_tris + n_tris * 2 <= len(skin_data):
            self.triangles = np.frombuffer(skin_data, dtype=np.uint16,
                                           count=n_tris, offset=ofs_tris).copy()

        # Resolve two-level indirection: triangle idx -> vertex_lookup -> global vertex idx
        # This matches the C++ approach: model.indices stores global vertex indices
        if len(self.triangles) > 0 and len(self.vertex_lookup) > 0:
            n_verts = len(self.positions) if len(self.positions) > 0 else 65536
            resolved = np.zeros(len(self.triangles), dtype=np.uint16)
            for i, tri_idx in enumerate(self.triangles):
                if tri_idx < len(self.vertex_lookup):
                    global_idx = self.vertex_lookup[tri_idx]
                    resolved[i] = global_idx if global_idx < n_verts else 0
                else:
                    resolved[i] = 0
            self.resolved_indices = resolved

        # Submeshes (WotLK: 48 bytes, Vanilla: 32 bytes)
        submesh_size = 32 if self.is_vanilla else 48
        if n_submeshes > 0 and n_submeshes < 10000 and ofs_submeshes + n_submeshes * submesh_size <= len(skin_data):
            for i in range(n_submeshes):
                base = ofs_submeshes + i * submesh_size
                sm = M2Submesh()
                # WotLK M2SkinSection: +0=skinSectionId(2), +2=Level(2),
                # +4=vertexStart(2), +6=vertexCount(2), +8=indexStart(2), +10=indexCount(2)
                sm.vertex_start = struct.unpack_from("<H", skin_data, base + 4)[0]
                sm.vertex_count = struct.unpack_from("<H", skin_data, base + 6)[0]
                sm.index_start = struct.unpack_from("<H", skin_data, base + 8)[0]
                sm.index_count = struct.unpack_from("<H", skin_data, base + 10)[0]
                self.submeshes.append(sm)

        # Batches (24 bytes each)
        if n_batches > 0 and n_batches < 10000 and ofs_batches + n_batches * 24 <= len(skin_data):
            for i in range(n_batches):
                base = ofs_batches + i * 24
                batch = M2Batch()
                # M2Batch: flags(1) + priority(1) + shaderId(2) + skinSectionIndex(2)
                # + geosetIndex(2) + colorIndex(2) + materialIndex(2) + materialLayer(2)
                # + textureCount(2) + textureComboIndex(2) + ...
                batch.submesh_index = struct.unpack_from("<H", skin_data, base + 4)[0]
                batch.texture_combo_index = struct.unpack_from("<H", skin_data, base + 16)[0]
                self.batches.append(batch)


# ---------------------------------------------------------------------------
# Animation System
# ---------------------------------------------------------------------------

_ANIM_NAMES: dict[int, str] = {
    0: "Stand", 1: "Death", 2: "Spell", 3: "Stop", 4: "Walk", 5: "Run",
    6: "Dead", 7: "Rise", 8: "StandWound", 9: "CombatWound", 10: "CombatCritical",
    11: "ShuffleLeft", 12: "ShuffleRight", 13: "Walkbackwards", 14: "Stun",
    15: "HandsClosed", 16: "AttackUnarmed", 17: "Attack1H", 18: "Attack2H",
    24: "ShieldBlock", 25: "ReadyUnarmed", 26: "Ready1H",
    27: "Ready2H", 34: "NPCWelcome", 35: "NPCGoodbye",
    37: "JumpStart", 38: "Jump", 39: "JumpEnd", 40: "Fall",
    41: "SwimIdle", 42: "Swim", 60: "SpellChannelDirected",
    69: "CombatAbility", 138: "Fly", 157: "EmoteTalk", 185: "FlyIdle",
}


class AnimationSystem:
    """Evaluates bone hierarchy each frame, producing world-space bone matrices."""

    def __init__(self, parser: M2Parser):
        self.parser = parser
        self.bone_matrices: np.ndarray = np.empty(0)
        self.current_seq: int = 0
        self.playing: bool = True
        self.speed: float = 1.0
        self.time_ms: float = 0.0
        self._identity = np.eye(4, dtype=np.float32)

    def set_sequence(self, idx: int):
        self.current_seq = max(0, min(idx, len(self.parser.animations) - 1))
        self.time_ms = 0.0

    def update(self, dt: float):
        """Advance animation time and compute bone matrices."""
        if not self.parser.bones:
            return

        n_bones = len(self.parser.bones)
        if len(self.bone_matrices) == 0:
            self.bone_matrices = np.tile(self._identity, (n_bones, 1, 1)).copy()

        if self.playing and self.parser.animations:
            anim = self.parser.animations[self.current_seq]
            if anim.duration > 0:
                self.time_ms += dt * 1000.0 * self.speed
                self.time_ms = self.time_ms % anim.duration

        seq_idx = self.current_seq
        t = self.time_ms

        for i, bone in enumerate(self.parser.bones):
            local = self._eval_bone(bone, seq_idx, t)
            if bone.parent >= 0 and bone.parent < n_bones:
                self.bone_matrices[i] = self.bone_matrices[bone.parent] @ local
            else:
                self.bone_matrices[i] = local

    def _eval_bone(self, bone: M2Bone, seq_idx: int, time_ms: float) -> np.ndarray:
        """Compute local bone transform for one bone at given time."""
        trans = self._interp_vec3(bone.translation, seq_idx, time_ms, np.zeros(3, dtype=np.float32))
        rot = self._interp_quat(bone.rotation, seq_idx, time_ms)
        scl = self._interp_vec3(bone.scale, seq_idx, time_ms, np.ones(3, dtype=np.float32))

        # local = T(pivot) * T(trans) * R(rot) * S(scl) * T(-pivot)
        p = bone.pivot
        m = translate(p[0], p[1], p[2])
        m = m @ translate(trans[0], trans[1], trans[2])
        m = m @ quat_to_mat4(rot)
        m = m @ scale_mat4(scl[0], scl[1], scl[2])
        m = m @ translate(-p[0], -p[1], -p[2])
        return m

    def _get_time_and_seq(self, track: M2Track, seq_idx: int, time_ms: float) -> tuple[int, float]:
        """Resolve sequence index and time, handling global sequences."""
        if track.global_sequence >= 0 and track.global_sequence < len(self.parser.global_sequences):
            gs_dur = self.parser.global_sequences[track.global_sequence]
            actual_seq = 0
            actual_time = time_ms % gs_dur if gs_dur > 0 else 0
        else:
            actual_seq = seq_idx
            actual_time = time_ms
        return actual_seq, actual_time

    def _interp_vec3(self, track: M2Track, seq_idx: int, time_ms: float,
                     default: np.ndarray) -> np.ndarray:
        si, t = self._get_time_and_seq(track, seq_idx, time_ms)
        if si >= len(track.timestamps) or si >= len(track.keys):
            return default
        ts = track.timestamps[si]
        keys = track.keys[si]
        if len(ts) == 0 or len(keys) == 0:
            return default
        if len(keys.shape) == 1:
            return default

        if t <= ts[0]:
            return keys[0]
        if t >= ts[-1]:
            return keys[-1]

        # Binary search
        idx = np.searchsorted(ts, t, side='right') - 1
        idx = max(0, min(idx, len(ts) - 2))
        t0, t1 = float(ts[idx]), float(ts[idx + 1])
        frac = (t - t0) / (t1 - t0) if t1 != t0 else 0.0

        if track.interp == 0:
            return keys[idx]
        return keys[idx] * (1.0 - frac) + keys[idx + 1] * frac

    def _interp_quat(self, track: M2Track, seq_idx: int, time_ms: float) -> np.ndarray:
        default = np.array([0, 0, 0, 1], dtype=np.float32)
        si, t = self._get_time_and_seq(track, seq_idx, time_ms)
        if si >= len(track.timestamps) or si >= len(track.keys):
            return default
        ts = track.timestamps[si]
        keys = track.keys[si]
        if len(ts) == 0 or len(keys) == 0:
            return default
        if len(keys.shape) == 1:
            return default

        if t <= ts[0]:
            return keys[0]
        if t >= ts[-1]:
            return keys[-1]

        idx = np.searchsorted(ts, t, side='right') - 1
        idx = max(0, min(idx, len(ts) - 2))
        t0, t1 = float(ts[idx]), float(ts[idx + 1])
        frac = (t - t0) / (t1 - t0) if t1 != t0 else 0.0

        if track.interp == 0:
            return keys[idx]
        return slerp(keys[idx], keys[idx + 1], frac)

    def skin_vertices(self, positions: np.ndarray, bone_weights: np.ndarray,
                      bone_indices: np.ndarray, bone_lookup: list[int]) -> np.ndarray:
        """CPU vertex skinning (NumPy vectorized). Returns transformed positions."""
        if len(self.bone_matrices) == 0 or len(bone_lookup) == 0:
            return positions.copy()

        n = len(positions)
        n_bones = len(self.bone_matrices)
        n_lookup = len(bone_lookup)
        lookup_arr = np.array(bone_lookup, dtype=np.int32)

        # Build homogeneous positions (n, 4)
        pos4 = np.ones((n, 4), dtype=np.float32)
        pos4[:, :3] = positions

        # Weights normalized to float (n, 4)
        weights = bone_weights.astype(np.float32) / 255.0

        result = np.zeros((n, 4), dtype=np.float32)

        for j in range(4):
            w = weights[:, j]  # (n,)
            mask = w > 0.001
            if not np.any(mask):
                continue

            bi = bone_indices[mask, j].astype(np.int32)
            # Clamp bone lookup indices
            valid = bi < n_lookup
            bi = np.where(valid, bi, 0)
            global_bones = lookup_arr[bi]
            global_bones = np.where(valid, global_bones, 0)
            valid2 = valid & (global_bones < n_bones)
            global_bones = np.where(valid2, global_bones, 0)

            # Gather bone matrices for these vertices: (count, 4, 4)
            mats = self.bone_matrices[global_bones]
            # Transform: (count, 4, 4) @ (count, 4, 1) -> (count, 4, 1)
            transformed = np.einsum('nij,nj->ni', mats, pos4[mask])
            # Apply weight and validity
            weighted = transformed * w[mask, np.newaxis]
            weighted[~valid2] = 0
            result[mask] += weighted

        # De-homogenize
        w_col = result[:, 3:4]
        w_col = np.where(np.abs(w_col) > 0.001, w_col, 1.0)
        return (result[:, :3] / w_col).astype(np.float32)


# ---------------------------------------------------------------------------
# Orbit Camera
# ---------------------------------------------------------------------------

class OrbitCamera:
    def __init__(self):
        self.azimuth: float = 0.0
        self.elevation: float = 0.3
        self.distance: float = 5.0
        self.target: np.ndarray = np.zeros(3, dtype=np.float32)
        self.pan_x: float = 0.0
        self.pan_y: float = 0.0

    def get_view_matrix(self) -> np.ndarray:
        eye = self._eye_pos()
        up = np.array([0, 0, 1], dtype=np.float32)
        target = self.target + np.array([self.pan_x, self.pan_y, 0], dtype=np.float32)
        return look_at(eye, target, up)

    def _eye_pos(self) -> np.ndarray:
        x = self.distance * math.cos(self.elevation) * math.cos(self.azimuth)
        y = self.distance * math.cos(self.elevation) * math.sin(self.azimuth)
        z = self.distance * math.sin(self.elevation)
        target = self.target + np.array([self.pan_x, self.pan_y, 0], dtype=np.float32)
        return target + np.array([x, y, z], dtype=np.float32)

    def orbit(self, dx: float, dy: float):
        self.azimuth += dx * 0.01
        self.elevation = max(-math.pi / 2 + 0.01, min(math.pi / 2 - 0.01,
                                                        self.elevation + dy * 0.01))

    def zoom(self, delta: float):
        self.distance = max(0.5, self.distance * (1.0 - delta * 0.1))

    def pan(self, dx: float, dy: float):
        self.pan_x += dx * self.distance * 0.002
        self.pan_y += dy * self.distance * 0.002


# ---------------------------------------------------------------------------
# M2 Renderer (OpenGL 3.3)
# ---------------------------------------------------------------------------

VERT_SHADER = """
#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNormal;
layout(location=2) in vec2 aUV;

uniform mat4 uMVP;
uniform mat4 uModel;

out vec3 vNormal;
out vec2 vUV;
out vec3 vWorldPos;

void main() {
    gl_Position = uMVP * vec4(aPos, 1.0);
    vNormal = mat3(uModel) * aNormal;
    vUV = aUV;
    vWorldPos = (uModel * vec4(aPos, 1.0)).xyz;
}
"""

FRAG_SHADER = """
#version 330 core
in vec3 vNormal;
in vec2 vUV;
in vec3 vWorldPos;

uniform sampler2D uTexture;
uniform int uHasTexture;
uniform vec3 uLightDir;

out vec4 FragColor;

void main() {
    vec3 N = normalize(vNormal);
    float NdotL = abs(dot(N, uLightDir));
    float ambient = 0.35;
    float diffuse = 0.65 * NdotL;
    float light = ambient + diffuse;

    vec4 texColor;
    if (uHasTexture == 1) {
        texColor = texture(uTexture, vUV);
        if (texColor.a < 0.1) discard;
    } else {
        texColor = vec4(0.6, 0.6, 0.65, 1.0);
    }

    FragColor = vec4(texColor.rgb * light, texColor.a);
}
"""

WIRE_VERT = """
#version 330 core
layout(location=0) in vec3 aPos;
uniform mat4 uMVP;
void main() {
    gl_Position = uMVP * vec4(aPos, 1.0);
}
"""

WIRE_FRAG = """
#version 330 core
out vec4 FragColor;
void main() {
    FragColor = vec4(0.0, 0.8, 1.0, 0.4);
}
"""


class M2Renderer:
    """OpenGL 3.3 renderer for M2 models."""

    def __init__(self, parser: M2Parser, blp_paths: dict[str, str], blp_convert: str):
        self.parser = parser
        self.blp_paths = blp_paths  # texture filename -> filesystem path
        self.blp_convert_path = blp_convert

        self.vao = 0
        self.vbo = 0
        self.ebo = 0
        self.wire_vao = 0
        self.wire_vbo = 0
        self.wire_ebo = 0
        self.shader = 0
        self.wire_shader = 0
        self.gl_textures: dict[int, int] = {}  # batch index -> GL texture ID
        self.batch_texture_map: dict[int, int] = {}  # batch idx -> texture array index

        self.show_wireframe = False
        self.n_indices = 0
        self.n_wire_indices = 0
        self.n_verts = 0

    def init_gl(self):
        import OpenGL.GL as gl

        self._gl = gl

        # Build shaders
        self.shader = self._compile_program(VERT_SHADER, FRAG_SHADER)
        self.wire_shader = self._compile_program(WIRE_VERT, WIRE_FRAG)

        p = self.parser
        n_verts = len(p.positions)
        if n_verts == 0:
            return
        self.n_verts = n_verts

        # VBO: ALL model vertices, interleaved pos(12) + normal(12) + uv(8) = 32 bytes
        vbo_data = np.zeros((n_verts, 8), dtype=np.float32)
        vbo_data[:, 0:3] = p.positions
        vbo_data[:, 3:6] = p.normals if len(p.normals) == n_verts else np.zeros((n_verts, 3), dtype=np.float32)
        vbo_data[:, 6:8] = p.uvs if len(p.uvs) == n_verts else np.zeros((n_verts, 2), dtype=np.float32)

        # EBO: resolved global vertex indices (after two-level skin indirection)
        if len(p.resolved_indices) > 0:
            idx_data = p.resolved_indices.astype(np.uint16)
        elif len(p.triangles) > 0:
            idx_data = p.triangles.astype(np.uint16)
        else:
            idx_data = np.empty(0, dtype=np.uint16)

        # Create main VAO/VBO/EBO
        self.vao = gl.glGenVertexArrays(1)
        self.vbo = gl.glGenBuffers(1)
        self.ebo = gl.glGenBuffers(1)

        gl.glBindVertexArray(self.vao)

        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.vbo)
        gl.glBufferData(gl.GL_ARRAY_BUFFER, vbo_data.nbytes, vbo_data, gl.GL_DYNAMIC_DRAW)

        if len(idx_data) > 0:
            self.n_indices = len(idx_data)
            gl.glBindBuffer(gl.GL_ELEMENT_ARRAY_BUFFER, self.ebo)
            gl.glBufferData(gl.GL_ELEMENT_ARRAY_BUFFER, idx_data.nbytes, idx_data, gl.GL_STATIC_DRAW)

        stride = 32
        gl.glVertexAttribPointer(0, 3, gl.GL_FLOAT, gl.GL_FALSE, stride, gl.ctypes.c_void_p(0))
        gl.glEnableVertexAttribArray(0)
        gl.glVertexAttribPointer(1, 3, gl.GL_FLOAT, gl.GL_FALSE, stride, gl.ctypes.c_void_p(12))
        gl.glEnableVertexAttribArray(1)
        gl.glVertexAttribPointer(2, 2, gl.GL_FLOAT, gl.GL_FALSE, stride, gl.ctypes.c_void_p(24))
        gl.glEnableVertexAttribArray(2)

        gl.glBindVertexArray(0)

        # Wireframe VAO (positions only, same indices)
        self.wire_vao = gl.glGenVertexArrays(1)
        self.wire_vbo = gl.glGenBuffers(1)
        self.wire_ebo = gl.glGenBuffers(1)

        gl.glBindVertexArray(self.wire_vao)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.wire_vbo)
        gl.glBufferData(gl.GL_ARRAY_BUFFER, p.positions.nbytes, p.positions, gl.GL_DYNAMIC_DRAW)

        if len(idx_data) > 0:
            self.n_wire_indices = len(idx_data)
            gl.glBindBuffer(gl.GL_ELEMENT_ARRAY_BUFFER, self.wire_ebo)
            gl.glBufferData(gl.GL_ELEMENT_ARRAY_BUFFER, idx_data.nbytes, idx_data, gl.GL_STATIC_DRAW)

        gl.glVertexAttribPointer(0, 3, gl.GL_FLOAT, gl.GL_FALSE, 12, gl.ctypes.c_void_p(0))
        gl.glEnableVertexAttribArray(0)
        gl.glBindVertexArray(0)

        # Load textures
        self._load_textures()

        # Map batches to textures
        self._map_batch_textures()

    def _compile_program(self, vert_src: str, frag_src: str) -> int:
        gl = self._gl
        vs = gl.glCreateShader(gl.GL_VERTEX_SHADER)
        gl.glShaderSource(vs, vert_src)
        gl.glCompileShader(vs)
        if gl.glGetShaderiv(vs, gl.GL_COMPILE_STATUS) != gl.GL_TRUE:
            log = gl.glGetShaderInfoLog(vs).decode()
            print(f"Vertex shader error: {log}")

        fs = gl.glCreateShader(gl.GL_FRAGMENT_SHADER)
        gl.glShaderSource(fs, frag_src)
        gl.glCompileShader(fs)
        if gl.glGetShaderiv(fs, gl.GL_COMPILE_STATUS) != gl.GL_TRUE:
            log = gl.glGetShaderInfoLog(fs).decode()
            print(f"Fragment shader error: {log}")

        prog = gl.glCreateProgram()
        gl.glAttachShader(prog, vs)
        gl.glAttachShader(prog, fs)
        gl.glLinkProgram(prog)
        if gl.glGetProgramiv(prog, gl.GL_LINK_STATUS) != gl.GL_TRUE:
            log = gl.glGetProgramInfoLog(prog).decode()
            print(f"Program link error: {log}")

        gl.glDeleteShader(vs)
        gl.glDeleteShader(fs)
        return prog

    def _load_textures(self):
        """Load BLP textures via blp_convert → PIL → GL texture."""
        gl = self._gl
        try:
            from PIL import Image
        except ImportError:
            print("PIL not available, textures disabled")
            return

        cache_dir = Path(os.path.expanduser("~/.cache/m2_viewer"))
        cache_dir.mkdir(parents=True, exist_ok=True)

        for i, tex in enumerate(self.parser.textures):
            if tex["type"] != 0 or not tex["filename"]:
                continue

            fname = tex["filename"].replace("\\", "/")
            blp_path = self.blp_paths.get(fname) or self.blp_paths.get(fname.lower())
            if not blp_path:
                continue

            # Convert BLP to PNG
            cache_key = hashlib.md5(blp_path.encode()).hexdigest()
            cached_png = cache_dir / f"{cache_key}.png"

            if not cached_png.exists():
                try:
                    # Copy BLP to temp dir for conversion (avoids read-only source dirs)
                    import tempfile
                    with tempfile.TemporaryDirectory() as tmpdir:
                        tmp_blp = Path(tmpdir) / Path(blp_path).name
                        shutil.copy2(blp_path, str(tmp_blp))
                        result = subprocess.run(
                            [self.blp_convert_path, "--to-png", str(tmp_blp)],
                            capture_output=True, text=True, timeout=10,
                        )
                        output_png = tmp_blp.with_suffix(".png")
                        if result.returncode != 0 or not output_png.exists():
                            print(f"blp_convert failed for {fname}: {result.stderr}")
                            continue
                        shutil.move(str(output_png), str(cached_png))
                except Exception as e:
                    print(f"BLP convert failed for {fname}: {e}")
                    continue

            try:
                img = Image.open(cached_png)
                img = img.transpose(Image.FLIP_TOP_BOTTOM)
                if img.mode != "RGBA":
                    img = img.convert("RGBA")
                img_data = np.array(img, dtype=np.uint8)

                tex_id = gl.glGenTextures(1)
                gl.glBindTexture(gl.GL_TEXTURE_2D, tex_id)
                gl.glTexImage2D(gl.GL_TEXTURE_2D, 0, gl.GL_RGBA, img.width, img.height,
                                0, gl.GL_RGBA, gl.GL_UNSIGNED_BYTE, img_data)
                gl.glGenerateMipmap(gl.GL_TEXTURE_2D)
                gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_LINEAR_MIPMAP_LINEAR)
                gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_LINEAR)
                gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_S, gl.GL_REPEAT)
                gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_T, gl.GL_REPEAT)

                self.gl_textures[i] = tex_id
            except Exception as e:
                print(f"Texture load failed for {fname}: {e}")

    def _map_batch_textures(self):
        """Resolve batch → texture combo → texture lookup → GL texture mapping."""
        for bi, batch in enumerate(self.parser.batches):
            tci = batch.texture_combo_index
            if tci < len(self.parser.texture_lookup):
                tex_idx = self.parser.texture_lookup[tci]
                if tex_idx in self.gl_textures:
                    self.batch_texture_map[bi] = self.gl_textures[tex_idx]

    def update_vertices(self, skinned_positions: np.ndarray):
        """Upload new skinned vertex positions to VBO."""
        gl = self._gl
        if self.vao == 0 or len(skinned_positions) == 0:
            return

        p = self.parser
        n_verts = len(skinned_positions)

        # Rebuild interleaved VBO data with new positions
        vbo_data = np.zeros((n_verts, 8), dtype=np.float32)
        vbo_data[:, 0:3] = skinned_positions
        vbo_data[:, 3:6] = p.normals if len(p.normals) == n_verts else np.zeros((n_verts, 3), dtype=np.float32)
        vbo_data[:, 6:8] = p.uvs if len(p.uvs) == n_verts else np.zeros((n_verts, 2), dtype=np.float32)

        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.vbo)
        gl.glBufferSubData(gl.GL_ARRAY_BUFFER, 0, vbo_data.nbytes, vbo_data)

        # Update wireframe VBO too
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self.wire_vbo)
        gl.glBufferSubData(gl.GL_ARRAY_BUFFER, 0, skinned_positions.nbytes, skinned_positions)

    def render(self, mvp: np.ndarray, model: np.ndarray):
        gl = self._gl
        if self.vao == 0 or self.n_indices == 0:
            return

        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glDisable(gl.GL_CULL_FACE)

        gl.glUseProgram(self.shader)

        mvp_loc = gl.glGetUniformLocation(self.shader, "uMVP")
        model_loc = gl.glGetUniformLocation(self.shader, "uModel")
        tex_loc = gl.glGetUniformLocation(self.shader, "uTexture")
        has_tex_loc = gl.glGetUniformLocation(self.shader, "uHasTexture")
        light_loc = gl.glGetUniformLocation(self.shader, "uLightDir")

        gl.glUniformMatrix4fv(mvp_loc, 1, gl.GL_TRUE, mvp)
        gl.glUniformMatrix4fv(model_loc, 1, gl.GL_TRUE, model)
        gl.glUniform1i(tex_loc, 0)

        # Light direction (normalized)
        light_dir = np.array([0.5, 0.3, 0.8], dtype=np.float32)
        light_dir /= np.linalg.norm(light_dir)
        gl.glUniform3fv(light_loc, 1, light_dir)

        gl.glBindVertexArray(self.vao)

        if self.parser.batches and self.parser.submeshes:
            # Per-batch rendering
            for bi, batch in enumerate(self.parser.batches):
                si = batch.submesh_index
                if si >= len(self.parser.submeshes):
                    continue
                sm = self.parser.submeshes[si]

                # Bind texture if available
                gl_tex = self.batch_texture_map.get(bi)
                if gl_tex:
                    gl.glActiveTexture(gl.GL_TEXTURE0)
                    gl.glBindTexture(gl.GL_TEXTURE_2D, gl_tex)
                    gl.glUniform1i(has_tex_loc, 1)
                else:
                    gl.glUniform1i(has_tex_loc, 0)

                # Draw this submesh's triangles
                idx_start = sm.index_start
                idx_count = sm.index_count
                if idx_start + idx_count <= self.n_indices:
                    gl.glDrawElements(gl.GL_TRIANGLES, idx_count, gl.GL_UNSIGNED_SHORT,
                                      gl.ctypes.c_void_p(idx_start * 2))
        else:
            # Fallback: draw all triangles with no texture
            gl.glUniform1i(has_tex_loc, 0)
            gl.glDrawElements(gl.GL_TRIANGLES, self.n_indices, gl.GL_UNSIGNED_SHORT,
                              gl.ctypes.c_void_p(0))

        gl.glBindVertexArray(0)

        # Wireframe overlay
        if self.show_wireframe and self.wire_vao and self.n_wire_indices > 0:
            gl.glUseProgram(self.wire_shader)
            wire_mvp_loc = gl.glGetUniformLocation(self.wire_shader, "uMVP")
            gl.glUniformMatrix4fv(wire_mvp_loc, 1, gl.GL_TRUE, mvp)

            gl.glEnable(gl.GL_BLEND)
            gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
            gl.glPolygonMode(gl.GL_FRONT_AND_BACK, gl.GL_LINE)
            gl.glDisable(gl.GL_CULL_FACE)

            gl.glBindVertexArray(self.wire_vao)
            gl.glDrawElements(gl.GL_TRIANGLES, self.n_wire_indices, gl.GL_UNSIGNED_SHORT,
                              gl.ctypes.c_void_p(0))
            gl.glBindVertexArray(0)

            gl.glPolygonMode(gl.GL_FRONT_AND_BACK, gl.GL_FILL)
            gl.glDisable(gl.GL_BLEND)


# ---------------------------------------------------------------------------
# M2 Viewer Window (Pygame main loop)
# ---------------------------------------------------------------------------

class M2ViewerWindow:
    """Pygame + OpenGL M2 model viewer window."""

    def __init__(self, m2_path: str, blp_paths: dict[str, str], blp_convert: str):
        self.m2_path = m2_path
        self.blp_paths = blp_paths
        self.blp_convert = blp_convert
        self.parser: M2Parser | None = None
        self.anim_system: AnimationSystem | None = None
        self.renderer: M2Renderer | None = None
        self.camera = OrbitCamera()
        self.width = 1024
        self.height = 768
        self.running = True
        self.fps_clock = None
        self.font = None

        self._dragging = False
        self._panning = False
        self._last_mouse = (0, 0)

    def run(self):
        """Main entry point — parse, init GL, run loop."""
        import pygame
        from pygame.locals import (
            DOUBLEBUF, OPENGL, RESIZABLE, QUIT, KEYDOWN, MOUSEBUTTONDOWN,
            MOUSEBUTTONUP, MOUSEMOTION, VIDEORESIZE,
            K_SPACE, K_LEFT, K_RIGHT, K_PLUS, K_MINUS, K_EQUALS, K_r, K_w,
            K_ESCAPE,
        )

        # Parse M2
        data = Path(self.m2_path).read_bytes()
        if len(data) < 8 or data[:4] != b"MD20":
            print(f"Not a valid M2 file: {self.m2_path}")
            return

        self.parser = M2Parser(data)

        # Load skin file
        m2_p = Path(self.m2_path)
        skin_path = m2_p.with_name(m2_p.stem + "00.skin")
        if skin_path.exists():
            self.parser.parse_skin_data(skin_path.read_bytes())
        elif self.parser.is_vanilla:
            # Embedded skin at ofsViews
            if self.parser.version <= 256:
                # Read ofsViews from vanilla header
                if len(data) > 108:
                    ofs_views = struct.unpack_from("<I", data, 100)[0]
                    if ofs_views > 0 and ofs_views < len(data):
                        self.parser.parse_skin_data(data[ofs_views:])

        # Init animation
        self.anim_system = AnimationSystem(self.parser)
        if self.parser.animations:
            self.anim_system.set_sequence(0)

        # Auto-fit camera
        if len(self.parser.positions) > 0:
            mins = self.parser.positions.min(axis=0)
            maxs = self.parser.positions.max(axis=0)
            center = (mins + maxs) / 2.0
            extent = np.linalg.norm(maxs - mins)
            self.camera.target = center
            self.camera.distance = max(extent * 1.2, 1.0)

        # Init Pygame + OpenGL
        pygame.init()
        pygame.display.set_caption(f"M2 Viewer — {Path(self.m2_path).name}")
        pygame.display.gl_set_attribute(pygame.GL_CONTEXT_MAJOR_VERSION, 3)
        pygame.display.gl_set_attribute(pygame.GL_CONTEXT_MINOR_VERSION, 3)
        pygame.display.gl_set_attribute(pygame.GL_CONTEXT_PROFILE_MASK,
                                        pygame.GL_CONTEXT_PROFILE_CORE)
        pygame.display.set_mode((self.width, self.height), DOUBLEBUF | OPENGL | RESIZABLE)

        self.fps_clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("monospace", 14)

        import OpenGL.GL as gl

        # Init renderer
        self.renderer = M2Renderer(self.parser, self.blp_paths, self.blp_convert)
        self.renderer.init_gl()

        gl.glClearColor(0.12, 0.12, 0.18, 1.0)
        gl.glEnable(gl.GL_DEPTH_TEST)

        # Main loop
        while self.running:
            dt = self.fps_clock.tick(60) / 1000.0

            for event in pygame.event.get():
                if event.type == QUIT:
                    self.running = False
                elif event.type == VIDEORESIZE:
                    self.width, self.height = event.w, event.h
                    pygame.display.set_mode((self.width, self.height),
                                            DOUBLEBUF | OPENGL | RESIZABLE)
                elif event.type == KEYDOWN:
                    self._handle_key(event.key)
                elif event.type == MOUSEBUTTONDOWN:
                    if event.button == 1:
                        self._dragging = True
                        self._last_mouse = event.pos
                    elif event.button == 3:
                        self._panning = True
                        self._last_mouse = event.pos
                    elif event.button == 4:
                        self.camera.zoom(1)
                    elif event.button == 5:
                        self.camera.zoom(-1)
                elif event.type == MOUSEBUTTONUP:
                    if event.button == 1:
                        self._dragging = False
                    elif event.button == 3:
                        self._panning = False
                elif event.type == MOUSEMOTION:
                    if self._dragging:
                        dx = event.pos[0] - self._last_mouse[0]
                        dy = event.pos[1] - self._last_mouse[1]
                        self.camera.orbit(dx, dy)
                        self._last_mouse = event.pos
                    elif self._panning:
                        dx = event.pos[0] - self._last_mouse[0]
                        dy = event.pos[1] - self._last_mouse[1]
                        self.camera.pan(-dx, dy)
                        self._last_mouse = event.pos

            # Update animation + skinning
            if self.anim_system:
                self.anim_system.update(dt)
                if (len(self.anim_system.bone_matrices) > 0
                        and len(self.parser.bone_lookup) > 0):
                    skinned = self.anim_system.skin_vertices(
                        self.parser.positions,
                        self.parser.bone_weights,
                        self.parser.bone_indices,
                        self.parser.bone_lookup,
                    )
                    self.renderer.update_vertices(skinned)

            # Render
            gl.glViewport(0, 0, self.width, self.height)
            gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)

            aspect = self.width / max(self.height, 1)
            proj = perspective(45.0, aspect, 0.01, 5000.0)
            view = self.camera.get_view_matrix()
            model = np.eye(4, dtype=np.float32)
            mvp = proj @ view @ model

            self.renderer.render(mvp, model)

            # HUD overlay
            self._draw_hud(pygame, gl)

            pygame.display.flip()

        pygame.quit()

    def _handle_key(self, key):
        import pygame
        if key == pygame.K_ESCAPE:
            self.running = False
        elif key == pygame.K_SPACE:
            if self.anim_system:
                self.anim_system.playing = not self.anim_system.playing
        elif key == pygame.K_RIGHT:
            if self.anim_system and self.parser.animations:
                idx = (self.anim_system.current_seq + 1) % len(self.parser.animations)
                self.anim_system.set_sequence(idx)
        elif key == pygame.K_LEFT:
            if self.anim_system and self.parser.animations:
                idx = (self.anim_system.current_seq - 1) % len(self.parser.animations)
                self.anim_system.set_sequence(idx)
        elif key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
            if self.anim_system:
                self.anim_system.speed = min(self.anim_system.speed + 0.25, 5.0)
        elif key in (pygame.K_MINUS, pygame.K_KP_MINUS):
            if self.anim_system:
                self.anim_system.speed = max(self.anim_system.speed - 0.25, 0.0)
        elif key == pygame.K_r:
            if self.anim_system:
                self.anim_system.time_ms = 0.0
                self.anim_system.playing = False
                self.anim_system.bone_matrices = np.empty(0)
        elif key == pygame.K_w:
            if self.renderer:
                self.renderer.show_wireframe = not self.renderer.show_wireframe

    def _draw_hud(self, pygame, gl):
        """Draw text overlay using Pygame font → texture approach."""
        if not self.font:
            return

        lines = [Path(self.m2_path).name]

        n_verts = len(self.parser.positions)
        n_tris = len(self.parser.triangles) // 3
        lines.append(f"{n_verts} verts, {n_tris} tris, {len(self.parser.textures)} tex")

        if self.parser.animations and self.anim_system:
            anim = self.parser.animations[self.anim_system.current_seq]
            name = _ANIM_NAMES.get(anim.anim_id, f"Anim {anim.anim_id}")
            state = "Playing" if self.anim_system.playing else "Paused"
            lines.append(f"[{self.anim_system.current_seq + 1}/{len(self.parser.animations)}] "
                         f"{name} ({anim.duration}ms) - {state} x{self.anim_system.speed:.1f}")
        else:
            lines.append("No animations")

        fps = self.fps_clock.get_fps() if self.fps_clock else 0
        lines.append(f"FPS: {fps:.0f}")

        lines.append("")
        lines.append("LMB: orbit | RMB: pan | Scroll: zoom")
        lines.append("Space: play/pause | Left/Right: anim | +/-: speed")
        lines.append("W: wireframe | R: reset | Esc: quit")

        # Render text to surface, then blit via orthographic projection
        # Use a simple texture-based approach
        line_height = 18
        total_height = len(lines) * line_height + 8
        surf_width = 450
        surf = pygame.Surface((surf_width, total_height), pygame.SRCALPHA)
        surf.fill((0, 0, 0, 160))

        for i, line in enumerate(lines):
            text_surf = self.font.render(line, True, (220, 220, 240))
            surf.blit(text_surf, (6, 4 + i * line_height))

        # Convert to OpenGL texture and draw
        text_data = pygame.image.tostring(surf, "RGBA", True)
        tex_id = gl.glGenTextures(1)
        gl.glBindTexture(gl.GL_TEXTURE_2D, tex_id)
        gl.glTexImage2D(gl.GL_TEXTURE_2D, 0, gl.GL_RGBA, surf_width, total_height,
                        0, gl.GL_RGBA, gl.GL_UNSIGNED_BYTE, text_data)
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_NEAREST)
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_NEAREST)

        # Draw fullscreen quad in ortho — use compatibility approach with glWindowPos + glDrawPixels
        # Simpler: use a small shader-less blit via fixed function emulation
        # Actually, let's just use the modern approach with a screen quad
        self._blit_texture(gl, tex_id, 8, self.height - total_height - 8, surf_width, total_height)

        gl.glDeleteTextures(1, [tex_id])

    def _blit_texture(self, gl, tex_id, x, y, w, h):
        """Blit a texture to screen at (x,y) using a temporary screen-space quad."""
        # Simple blit using glBlitFramebuffer alternative:
        # Create a minimal screen-space shader + quad
        if not hasattr(self, '_blit_shader'):
            blit_vert = """
#version 330 core
layout(location=0) in vec2 aPos;
layout(location=1) in vec2 aUV;
out vec2 vUV;
void main() {
    gl_Position = vec4(aPos, 0.0, 1.0);
    vUV = aUV;
}
"""
            blit_frag = """
#version 330 core
in vec2 vUV;
uniform sampler2D uTex;
out vec4 FragColor;
void main() {
    FragColor = texture(uTex, vUV);
}
"""
            self._blit_shader = self.renderer._compile_program(blit_vert, blit_frag)
            self._blit_vao = gl.glGenVertexArrays(1)
            self._blit_vbo = gl.glGenBuffers(1)

        # Convert pixel coords to NDC
        x0 = 2.0 * x / self.width - 1.0
        y0 = 2.0 * y / self.height - 1.0
        x1 = 2.0 * (x + w) / self.width - 1.0
        y1 = 2.0 * (y + h) / self.height - 1.0

        quad = np.array([
            x0, y0, 0.0, 0.0,
            x1, y0, 1.0, 0.0,
            x1, y1, 1.0, 1.0,
            x0, y0, 0.0, 0.0,
            x1, y1, 1.0, 1.0,
            x0, y1, 0.0, 1.0,
        ], dtype=np.float32)

        gl.glBindVertexArray(self._blit_vao)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self._blit_vbo)
        gl.glBufferData(gl.GL_ARRAY_BUFFER, quad.nbytes, quad, gl.GL_DYNAMIC_DRAW)
        gl.glVertexAttribPointer(0, 2, gl.GL_FLOAT, gl.GL_FALSE, 16, gl.ctypes.c_void_p(0))
        gl.glEnableVertexAttribArray(0)
        gl.glVertexAttribPointer(1, 2, gl.GL_FLOAT, gl.GL_FALSE, 16, gl.ctypes.c_void_p(8))
        gl.glEnableVertexAttribArray(1)

        gl.glDisable(gl.GL_DEPTH_TEST)
        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)

        gl.glUseProgram(self._blit_shader)
        gl.glActiveTexture(gl.GL_TEXTURE0)
        gl.glBindTexture(gl.GL_TEXTURE_2D, tex_id)
        gl.glUniform1i(gl.glGetUniformLocation(self._blit_shader, "uTex"), 0)

        gl.glDrawArrays(gl.GL_TRIANGLES, 0, 6)

        gl.glBindVertexArray(0)
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glDisable(gl.GL_BLEND)


# ---------------------------------------------------------------------------
# WMO Parser
# ---------------------------------------------------------------------------

@dataclass
class WMOBatch:
    start_index: int = 0
    index_count: int = 0
    material_id: int = 0


@dataclass
class WMOMaterial:
    flags: int = 0
    shader: int = 0
    blend_mode: int = 0
    texture1_ofs: int = 0
    texture2_ofs: int = 0
    texture3_ofs: int = 0
    color1: int = 0
    color2: int = 0


@dataclass
class WMOGroup:
    positions: np.ndarray = field(default_factory=lambda: np.empty((0, 3), dtype=np.float32))
    normals: np.ndarray = field(default_factory=lambda: np.empty((0, 3), dtype=np.float32))
    uvs: np.ndarray = field(default_factory=lambda: np.empty((0, 2), dtype=np.float32))
    indices: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=np.uint16))
    batches: list = field(default_factory=list)


class WMOParser:
    """Parse WMO root + group files for rendering."""

    def __init__(self):
        self.textures: list[str] = []
        self.texture_offset_map: dict[int, int] = {}  # MOTX byte offset -> texture index
        self.materials: list[WMOMaterial] = []
        self.groups: list[WMOGroup] = []
        self.n_groups_expected: int = 0

    def parse_root(self, data: bytes):
        """Parse root WMO file for textures and materials."""
        pos = 0
        while pos + 8 <= len(data):
            chunk_id = data[pos:pos + 4]
            chunk_size = struct.unpack_from("<I", data, pos + 4)[0]
            chunk_start = pos + 8
            chunk_end = chunk_start + chunk_size

            if chunk_end > len(data):
                break

            cid = chunk_id if chunk_id[:1] == b"M" else chunk_id[::-1]

            if cid == b"MOHD" and chunk_size >= 16:
                # nTextures at +0, nGroups at +4
                self.n_groups_expected = struct.unpack_from("<I", data, chunk_start + 4)[0]

            elif cid == b"MOTX":
                # Null-terminated string block
                off = 0
                tex_idx = 0
                while off < chunk_size:
                    end = data.find(b"\x00", chunk_start + off, chunk_end)
                    if end < 0:
                        break
                    s = data[chunk_start + off:end].decode("ascii", errors="replace")
                    if s:
                        self.texture_offset_map[off] = tex_idx
                        self.textures.append(s)
                        tex_idx += 1
                        off = end - chunk_start + 1
                    else:
                        off += 1

            elif cid == b"MOMT":
                n_mats = chunk_size // 64
                for i in range(n_mats):
                    base = chunk_start + i * 64
                    fields = struct.unpack_from("<16I", data, base)
                    mat = WMOMaterial()
                    mat.flags = fields[0]
                    mat.shader = fields[1]
                    mat.blend_mode = fields[2]
                    mat.texture1_ofs = fields[3]
                    mat.color1 = fields[4]
                    mat.texture2_ofs = fields[6]
                    mat.color2 = fields[7]
                    mat.texture3_ofs = fields[9]
                    self.materials.append(mat)

            pos = chunk_end

    def parse_group(self, data: bytes) -> WMOGroup:
        """Parse a WMO group file for geometry."""
        group = WMOGroup()
        pos = 0

        # Scan for MOGP chunk which wraps all sub-chunks
        mogp_start = -1
        mogp_end = len(data)
        while pos + 8 <= len(data):
            chunk_id = data[pos:pos + 4]
            chunk_size = struct.unpack_from("<I", data, pos + 4)[0]
            cid = chunk_id if chunk_id[:1] == b"M" else chunk_id[::-1]
            if cid == b"MOGP":
                mogp_start = pos + 8 + 68  # Skip MOGP header (68 bytes)
                mogp_end = pos + 8 + chunk_size
                break
            pos += 8 + chunk_size

        # Parse sub-chunks inside MOGP (or scan whole file if no MOGP found)
        scan_start = mogp_start if mogp_start >= 0 else 0
        pos = scan_start
        while pos + 8 <= mogp_end:
            chunk_id = data[pos:pos + 4]
            chunk_size = struct.unpack_from("<I", data, pos + 4)[0]
            chunk_start = pos + 8
            chunk_end = chunk_start + chunk_size

            if chunk_end > mogp_end:
                break

            cid = chunk_id if chunk_id[:1] == b"M" else chunk_id[::-1]

            if cid == b"MOVT":
                n = chunk_size // 12
                group.positions = np.zeros((n, 3), dtype=np.float32)
                for i in range(n):
                    group.positions[i] = struct.unpack_from("<3f", data, chunk_start + i * 12)

            elif cid == b"MOVI":
                n = chunk_size // 2
                group.indices = np.frombuffer(data, dtype=np.uint16,
                                              count=n, offset=chunk_start).copy()

            elif cid == b"MONR":
                n = chunk_size // 12
                group.normals = np.zeros((n, 3), dtype=np.float32)
                for i in range(n):
                    group.normals[i] = struct.unpack_from("<3f", data, chunk_start + i * 12)

            elif cid == b"MOTV":
                n = chunk_size // 8
                group.uvs = np.zeros((n, 2), dtype=np.float32)
                for i in range(n):
                    group.uvs[i] = struct.unpack_from("<2f", data, chunk_start + i * 8)

            elif cid == b"MOBA":
                n = chunk_size // 24
                for i in range(n):
                    base = chunk_start + i * 24
                    batch = WMOBatch()
                    batch.start_index = struct.unpack_from("<I", data, base + 12)[0]
                    batch.index_count = struct.unpack_from("<H", data, base + 16)[0]
                    # skip startVertex(2) + lastVertex(2) + flags(1)
                    batch.material_id = struct.unpack_from("<B", data, base + 23)[0]
                    group.batches.append(batch)

            pos = chunk_end

        return group

    def get_texture_name(self, motx_offset: int) -> str:
        """Resolve a MOTX byte offset to a texture filename."""
        idx = self.texture_offset_map.get(motx_offset)
        if idx is not None and idx < len(self.textures):
            return self.textures[idx]
        return ""


# ---------------------------------------------------------------------------
# WMO Renderer
# ---------------------------------------------------------------------------

class WMORenderer:
    """OpenGL 3.3 renderer for WMO models."""

    def __init__(self, parser: WMOParser, blp_paths: dict[str, str], blp_convert: str):
        self.parser = parser
        self.blp_paths = blp_paths
        self.blp_convert_path = blp_convert
        self.show_wireframe = False

        # Per-group GL state
        self._group_vaos: list[int] = []
        self._group_vbos: list[int] = []
        self._group_ebos: list[int] = []
        self._group_n_indices: list[int] = []
        self._group_batches: list[list[WMOBatch]] = []

        self.shader = 0
        self.wire_shader = 0
        self._gl = None

        # material_id -> GL texture id
        self._mat_textures: dict[int, int] = {}

    def init_gl(self):
        import OpenGL.GL as gl
        self._gl = gl

        self.shader = self._compile_program(VERT_SHADER, FRAG_SHADER)
        self.wire_shader = self._compile_program(WIRE_VERT, WIRE_FRAG)

        self._load_textures()

        for group in self.parser.groups:
            self._upload_group(group)

    def _upload_group(self, group: WMOGroup):
        gl = self._gl
        n_verts = len(group.positions)
        if n_verts == 0:
            self._group_vaos.append(0)
            self._group_vbos.append(0)
            self._group_ebos.append(0)
            self._group_n_indices.append(0)
            self._group_batches.append([])
            return

        # Interleaved: pos(12) + normal(12) + uv(8) = 32 bytes
        vbo_data = np.zeros((n_verts, 8), dtype=np.float32)
        vbo_data[:, 0:3] = group.positions
        if len(group.normals) == n_verts:
            vbo_data[:, 3:6] = group.normals
        if len(group.uvs) == n_verts:
            vbo_data[:, 6:8] = group.uvs

        vao = gl.glGenVertexArrays(1)
        vbo = gl.glGenBuffers(1)
        ebo = gl.glGenBuffers(1)

        gl.glBindVertexArray(vao)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, vbo)
        gl.glBufferData(gl.GL_ARRAY_BUFFER, vbo_data.nbytes, vbo_data, gl.GL_STATIC_DRAW)

        n_idx = 0
        if len(group.indices) > 0:
            idx_data = group.indices.astype(np.uint16)
            n_idx = len(idx_data)
            gl.glBindBuffer(gl.GL_ELEMENT_ARRAY_BUFFER, ebo)
            gl.glBufferData(gl.GL_ELEMENT_ARRAY_BUFFER, idx_data.nbytes, idx_data, gl.GL_STATIC_DRAW)

        stride = 32
        gl.glVertexAttribPointer(0, 3, gl.GL_FLOAT, gl.GL_FALSE, stride, gl.ctypes.c_void_p(0))
        gl.glEnableVertexAttribArray(0)
        gl.glVertexAttribPointer(1, 3, gl.GL_FLOAT, gl.GL_FALSE, stride, gl.ctypes.c_void_p(12))
        gl.glEnableVertexAttribArray(1)
        gl.glVertexAttribPointer(2, 2, gl.GL_FLOAT, gl.GL_FALSE, stride, gl.ctypes.c_void_p(24))
        gl.glEnableVertexAttribArray(2)
        gl.glBindVertexArray(0)

        self._group_vaos.append(vao)
        self._group_vbos.append(vbo)
        self._group_ebos.append(ebo)
        self._group_n_indices.append(n_idx)
        self._group_batches.append(group.batches)

    def _load_textures(self):
        gl = self._gl
        try:
            from PIL import Image
        except ImportError:
            return

        cache_dir = Path(os.path.expanduser("~/.cache/m2_viewer"))
        cache_dir.mkdir(parents=True, exist_ok=True)

        loaded: dict[str, int] = {}  # filename -> GL tex id

        for mat_idx, mat in enumerate(self.parser.materials):
            tex_name = self.parser.get_texture_name(mat.texture1_ofs)
            if not tex_name:
                continue

            if tex_name in loaded:
                self._mat_textures[mat_idx] = loaded[tex_name]
                continue

            norm = tex_name.replace("\\", "/")
            blp_path = self.blp_paths.get(norm) or self.blp_paths.get(norm.lower())
            if not blp_path:
                continue

            cache_key = hashlib.md5(blp_path.encode()).hexdigest()
            cached_png = cache_dir / f"{cache_key}.png"

            if not cached_png.exists():
                try:
                    import tempfile
                    with tempfile.TemporaryDirectory() as tmpdir:
                        tmp_blp = Path(tmpdir) / Path(blp_path).name
                        shutil.copy2(blp_path, str(tmp_blp))
                        result = subprocess.run(
                            [self.blp_convert_path, "--to-png", str(tmp_blp)],
                            capture_output=True, text=True, timeout=10,
                        )
                        output_png = tmp_blp.with_suffix(".png")
                        if result.returncode != 0 or not output_png.exists():
                            continue
                        shutil.move(str(output_png), str(cached_png))
                except Exception:
                    continue

            try:
                img = Image.open(cached_png)
                img = img.transpose(Image.FLIP_TOP_BOTTOM)
                if img.mode != "RGBA":
                    img = img.convert("RGBA")
                img_data = np.array(img, dtype=np.uint8)

                tex_id = gl.glGenTextures(1)
                gl.glBindTexture(gl.GL_TEXTURE_2D, tex_id)
                gl.glTexImage2D(gl.GL_TEXTURE_2D, 0, gl.GL_RGBA, img.width, img.height,
                                0, gl.GL_RGBA, gl.GL_UNSIGNED_BYTE, img_data)
                gl.glGenerateMipmap(gl.GL_TEXTURE_2D)
                gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_LINEAR_MIPMAP_LINEAR)
                gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_LINEAR)
                gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_S, gl.GL_REPEAT)
                gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_WRAP_T, gl.GL_REPEAT)

                loaded[tex_name] = tex_id
                self._mat_textures[mat_idx] = tex_id
            except Exception:
                continue

    def _compile_program(self, vert_src: str, frag_src: str) -> int:
        gl = self._gl
        vs = gl.glCreateShader(gl.GL_VERTEX_SHADER)
        gl.glShaderSource(vs, vert_src)
        gl.glCompileShader(vs)
        fs = gl.glCreateShader(gl.GL_FRAGMENT_SHADER)
        gl.glShaderSource(fs, frag_src)
        gl.glCompileShader(fs)
        prog = gl.glCreateProgram()
        gl.glAttachShader(prog, vs)
        gl.glAttachShader(prog, fs)
        gl.glLinkProgram(prog)
        gl.glDeleteShader(vs)
        gl.glDeleteShader(fs)
        return prog

    def render(self, mvp: np.ndarray, model: np.ndarray):
        gl = self._gl
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glDisable(gl.GL_CULL_FACE)

        gl.glUseProgram(self.shader)
        mvp_loc = gl.glGetUniformLocation(self.shader, "uMVP")
        model_loc = gl.glGetUniformLocation(self.shader, "uModel")
        tex_loc = gl.glGetUniformLocation(self.shader, "uTexture")
        has_tex_loc = gl.glGetUniformLocation(self.shader, "uHasTexture")
        light_loc = gl.glGetUniformLocation(self.shader, "uLightDir")

        gl.glUniformMatrix4fv(mvp_loc, 1, gl.GL_TRUE, mvp)
        gl.glUniformMatrix4fv(model_loc, 1, gl.GL_TRUE, model)
        gl.glUniform1i(tex_loc, 0)

        light_dir = np.array([0.5, 0.3, 0.8], dtype=np.float32)
        light_dir /= np.linalg.norm(light_dir)
        gl.glUniform3fv(light_loc, 1, light_dir)

        for gi in range(len(self._group_vaos)):
            vao = self._group_vaos[gi]
            n_idx = self._group_n_indices[gi]
            batches = self._group_batches[gi]
            if vao == 0 or n_idx == 0:
                continue

            gl.glBindVertexArray(vao)

            if batches:
                for batch in batches:
                    gl_tex = self._mat_textures.get(batch.material_id)
                    if gl_tex:
                        gl.glActiveTexture(gl.GL_TEXTURE0)
                        gl.glBindTexture(gl.GL_TEXTURE_2D, gl_tex)
                        gl.glUniform1i(has_tex_loc, 1)
                    else:
                        gl.glUniform1i(has_tex_loc, 0)

                    si = batch.start_index
                    ic = batch.index_count
                    if si + ic <= n_idx:
                        gl.glDrawElements(gl.GL_TRIANGLES, ic, gl.GL_UNSIGNED_SHORT,
                                          gl.ctypes.c_void_p(si * 2))
            else:
                gl.glUniform1i(has_tex_loc, 0)
                gl.glDrawElements(gl.GL_TRIANGLES, n_idx, gl.GL_UNSIGNED_SHORT,
                                  gl.ctypes.c_void_p(0))

            gl.glBindVertexArray(0)

        # Wireframe overlay
        if self.show_wireframe:
            gl.glUseProgram(self.wire_shader)
            wire_mvp_loc = gl.glGetUniformLocation(self.wire_shader, "uMVP")
            gl.glUniformMatrix4fv(wire_mvp_loc, 1, gl.GL_TRUE, mvp)
            gl.glEnable(gl.GL_BLEND)
            gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
            gl.glPolygonMode(gl.GL_FRONT_AND_BACK, gl.GL_LINE)

            for gi in range(len(self._group_vaos)):
                vao = self._group_vaos[gi]
                n_idx = self._group_n_indices[gi]
                if vao == 0 or n_idx == 0:
                    continue
                gl.glBindVertexArray(vao)
                gl.glDrawElements(gl.GL_TRIANGLES, n_idx, gl.GL_UNSIGNED_SHORT,
                                  gl.ctypes.c_void_p(0))
                gl.glBindVertexArray(0)

            gl.glPolygonMode(gl.GL_FRONT_AND_BACK, gl.GL_FILL)
            gl.glDisable(gl.GL_BLEND)


# ---------------------------------------------------------------------------
# WMO Viewer Window
# ---------------------------------------------------------------------------

class WMOViewerWindow:
    """Pygame + OpenGL WMO model viewer window."""

    def __init__(self, wmo_root_path: str, group_paths: list[str],
                 blp_paths: dict[str, str], blp_convert: str):
        self.wmo_root_path = wmo_root_path
        self.group_paths = group_paths
        self.blp_paths = blp_paths
        self.blp_convert = blp_convert
        self.parser: WMOParser | None = None
        self.renderer: WMORenderer | None = None
        self.camera = OrbitCamera()
        self.width = 1024
        self.height = 768
        self.running = True
        self.fps_clock = None
        self.font = None
        self._dragging = False
        self._panning = False
        self._last_mouse = (0, 0)

    def run(self):
        import pygame
        from pygame.locals import (
            DOUBLEBUF, OPENGL, RESIZABLE, QUIT, KEYDOWN, MOUSEBUTTONDOWN,
            MOUSEBUTTONUP, MOUSEMOTION, VIDEORESIZE,
        )

        # Parse WMO
        self.parser = WMOParser()

        if self.wmo_root_path and Path(self.wmo_root_path).exists():
            self.parser.parse_root(Path(self.wmo_root_path).read_bytes())

        total_verts = 0
        total_tris = 0
        for gp in self.group_paths:
            if Path(gp).exists():
                group = self.parser.parse_group(Path(gp).read_bytes())
                self.parser.groups.append(group)
                total_verts += len(group.positions)
                total_tris += len(group.indices) // 3

        if total_verts == 0:
            print("No geometry found in WMO groups")
            return

        # Auto-fit camera
        all_pos = np.vstack([g.positions for g in self.parser.groups if len(g.positions) > 0])
        mins = all_pos.min(axis=0)
        maxs = all_pos.max(axis=0)
        center = (mins + maxs) / 2.0
        extent = np.linalg.norm(maxs - mins)
        self.camera.target = center
        self.camera.distance = max(extent * 1.2, 1.0)

        # Init Pygame
        pygame.init()
        name = Path(self.wmo_root_path or self.group_paths[0]).stem
        pygame.display.set_caption(f"WMO Viewer — {name}")
        pygame.display.gl_set_attribute(pygame.GL_CONTEXT_MAJOR_VERSION, 3)
        pygame.display.gl_set_attribute(pygame.GL_CONTEXT_MINOR_VERSION, 3)
        pygame.display.gl_set_attribute(pygame.GL_CONTEXT_PROFILE_MASK,
                                        pygame.GL_CONTEXT_PROFILE_CORE)
        pygame.display.set_mode((self.width, self.height), DOUBLEBUF | OPENGL | RESIZABLE)

        self.fps_clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("monospace", 14)

        import OpenGL.GL as gl

        self.renderer = WMORenderer(self.parser, self.blp_paths, self.blp_convert)
        self.renderer.init_gl()

        gl.glClearColor(0.12, 0.12, 0.18, 1.0)

        while self.running:
            self.fps_clock.tick(60)

            for event in pygame.event.get():
                if event.type == QUIT:
                    self.running = False
                elif event.type == VIDEORESIZE:
                    self.width, self.height = event.w, event.h
                    pygame.display.set_mode((self.width, self.height),
                                            DOUBLEBUF | OPENGL | RESIZABLE)
                elif event.type == KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        self.running = False
                    elif event.key == pygame.K_w:
                        self.renderer.show_wireframe = not self.renderer.show_wireframe
                elif event.type == MOUSEBUTTONDOWN:
                    if event.button == 1:
                        self._dragging = True
                        self._last_mouse = event.pos
                    elif event.button == 3:
                        self._panning = True
                        self._last_mouse = event.pos
                    elif event.button == 4:
                        self.camera.zoom(1)
                    elif event.button == 5:
                        self.camera.zoom(-1)
                elif event.type == MOUSEBUTTONUP:
                    if event.button == 1:
                        self._dragging = False
                    elif event.button == 3:
                        self._panning = False
                elif event.type == MOUSEMOTION:
                    if self._dragging:
                        dx = event.pos[0] - self._last_mouse[0]
                        dy = event.pos[1] - self._last_mouse[1]
                        self.camera.orbit(dx, dy)
                        self._last_mouse = event.pos
                    elif self._panning:
                        dx = event.pos[0] - self._last_mouse[0]
                        dy = event.pos[1] - self._last_mouse[1]
                        self.camera.pan(-dx, dy)
                        self._last_mouse = event.pos

            gl.glViewport(0, 0, self.width, self.height)
            gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)

            aspect = self.width / max(self.height, 1)
            proj = perspective(45.0, aspect, 0.1, 10000.0)
            view = self.camera.get_view_matrix()
            model_mat = np.eye(4, dtype=np.float32)
            mvp = proj @ view @ model_mat

            self.renderer.render(mvp, model_mat)

            # HUD
            self._draw_hud(pygame, gl, total_verts, total_tris)

            pygame.display.flip()

        pygame.quit()

    def _draw_hud(self, pygame, gl, total_verts, total_tris):
        if not self.font:
            return
        name = Path(self.wmo_root_path or self.group_paths[0]).name
        lines = [
            name,
            f"{len(self.parser.groups)} groups, {total_verts} verts, {total_tris} tris",
            f"{len(self.parser.materials)} materials, {len(self.parser.textures)} textures",
            f"FPS: {self.fps_clock.get_fps():.0f}",
            "",
            "LMB: orbit | RMB: pan | Scroll: zoom",
            "W: wireframe | Esc: quit",
        ]

        line_height = 18
        total_height = len(lines) * line_height + 8
        surf_width = 420
        surf = pygame.Surface((surf_width, total_height), pygame.SRCALPHA)
        surf.fill((0, 0, 0, 160))
        for i, line in enumerate(lines):
            text_surf = self.font.render(line, True, (220, 220, 240))
            surf.blit(text_surf, (6, 4 + i * line_height))

        text_data = pygame.image.tostring(surf, "RGBA", True)
        tex_id = gl.glGenTextures(1)
        gl.glBindTexture(gl.GL_TEXTURE_2D, tex_id)
        gl.glTexImage2D(gl.GL_TEXTURE_2D, 0, gl.GL_RGBA, surf_width, total_height,
                        0, gl.GL_RGBA, gl.GL_UNSIGNED_BYTE, text_data)
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MIN_FILTER, gl.GL_NEAREST)
        gl.glTexParameteri(gl.GL_TEXTURE_2D, gl.GL_TEXTURE_MAG_FILTER, gl.GL_NEAREST)

        # Blit using the same approach as M2ViewerWindow
        if not hasattr(self, '_blit_shader'):
            blit_vert = """
#version 330 core
layout(location=0) in vec2 aPos;
layout(location=1) in vec2 aUV;
out vec2 vUV;
void main() { gl_Position = vec4(aPos, 0.0, 1.0); vUV = aUV; }
"""
            blit_frag = """
#version 330 core
in vec2 vUV;
uniform sampler2D uTex;
out vec4 FragColor;
void main() { FragColor = texture(uTex, vUV); }
"""
            self._blit_shader = self.renderer._compile_program(blit_vert, blit_frag)
            self._blit_vao = gl.glGenVertexArrays(1)
            self._blit_vbo = gl.glGenBuffers(1)

        x, y, w, h = 8, self.height - total_height - 8, surf_width, total_height
        x0 = 2.0 * x / self.width - 1.0
        y0 = 2.0 * y / self.height - 1.0
        x1 = 2.0 * (x + w) / self.width - 1.0
        y1 = 2.0 * (y + h) / self.height - 1.0

        quad = np.array([
            x0, y0, 0, 0, x1, y0, 1, 0, x1, y1, 1, 1,
            x0, y0, 0, 0, x1, y1, 1, 1, x0, y1, 0, 1,
        ], dtype=np.float32)

        gl.glBindVertexArray(self._blit_vao)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self._blit_vbo)
        gl.glBufferData(gl.GL_ARRAY_BUFFER, quad.nbytes, quad, gl.GL_DYNAMIC_DRAW)
        gl.glVertexAttribPointer(0, 2, gl.GL_FLOAT, gl.GL_FALSE, 16, gl.ctypes.c_void_p(0))
        gl.glEnableVertexAttribArray(0)
        gl.glVertexAttribPointer(1, 2, gl.GL_FLOAT, gl.GL_FALSE, 16, gl.ctypes.c_void_p(8))
        gl.glEnableVertexAttribArray(1)

        gl.glDisable(gl.GL_DEPTH_TEST)
        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
        gl.glUseProgram(self._blit_shader)
        gl.glUniform1i(gl.glGetUniformLocation(self._blit_shader, "uTex"), 0)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, 6)
        gl.glBindVertexArray(0)
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glDisable(gl.GL_BLEND)
        gl.glDeleteTextures(1, [tex_id])


# ---------------------------------------------------------------------------
# Launch entry points (multiprocessing-safe)
# ---------------------------------------------------------------------------

def _viewer_main(m2_path: str, blp_paths: dict[str, str], blp_convert: str):
    """Entry point for M2 viewer subprocess."""
    viewer = M2ViewerWindow(m2_path, blp_paths, blp_convert)
    viewer.run()


def _wmo_viewer_main(wmo_root: str, group_paths: list[str],
                     blp_paths: dict[str, str], blp_convert: str):
    """Entry point for WMO viewer subprocess."""
    viewer = WMOViewerWindow(wmo_root, group_paths, blp_paths, blp_convert)
    viewer.run()


def launch_m2_viewer(m2_path: str, blp_paths: dict[str, str], blp_convert: str):
    """Launch M2 viewer in a separate process to avoid Tkinter/Pygame conflicts."""
    p = multiprocessing.Process(target=_viewer_main, args=(m2_path, blp_paths, blp_convert),
                                daemon=True)
    p.start()
    return p


def launch_wmo_viewer(wmo_root: str, group_paths: list[str],
                      blp_paths: dict[str, str], blp_convert: str):
    """Launch WMO viewer in a separate process."""
    p = multiprocessing.Process(target=_wmo_viewer_main,
                                args=(wmo_root, group_paths, blp_paths, blp_convert),
                                daemon=True)
    p.start()
    return p


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python m2_viewer.py <m2_or_wmo_file> [blp_convert_path]")
        sys.exit(1)

    file_path = sys.argv[1]
    blp_conv = sys.argv[2] if len(sys.argv) > 2 else ""

    if file_path.lower().endswith(".wmo"):
        # Detect root vs group and find all group files
        p = Path(file_path)
        name = p.name.lower()
        is_group = len(name) > 8 and name[-8:-4].isdigit() and name[-9] == "_"

        if is_group:
            # Derive root from group
            stem = p.stem
            root_stem = stem.rsplit("_", 1)[0]
            root_path = p.parent / f"{root_stem}.wmo"
            groups = sorted(p.parent.glob(f"{root_stem}_*.wmo"))
        else:
            root_path = p
            stem = p.stem
            groups = sorted(p.parent.glob(f"{stem}_*.wmo"))

        root_str = str(root_path) if root_path.exists() else ""
        group_strs = [str(g) for g in groups]
        if not group_strs and is_group:
            group_strs = [file_path]

        viewer = WMOViewerWindow(root_str, group_strs, {}, blp_conv)
        viewer.run()
    else:
        viewer = M2ViewerWindow(file_path, {}, blp_conv)
        viewer.run()
