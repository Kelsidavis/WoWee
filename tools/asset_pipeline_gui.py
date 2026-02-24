#!/usr/bin/env python3
"""WoWee Asset Pipeline GUI.

Cross-platform Tkinter app for running asset extraction and managing texture packs
that are merged into Data/override in deterministic order.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import platform
import queue
import shutil
import struct
import subprocess
import tempfile
import threading
import time
import zipfile
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from tkinter.scrolledtext import ScrolledText

try:
    from PIL import Image, ImageTk
    HAS_PILLOW = True
except ImportError:
    HAS_PILLOW = False


ROOT_DIR = Path(__file__).resolve().parents[1]
PIPELINE_DIR = ROOT_DIR / "asset_pipeline"
STATE_FILE = PIPELINE_DIR / "state.json"


def _audio_subprocess(file_path: str) -> None:
    """Play an audio file using pygame.mixer in a subprocess."""
    try:
        import pygame
        pygame.mixer.init(frequency=44100, size=-16, channels=2, buffer=2048)
        pygame.mixer.music.load(file_path)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            pygame.time.wait(100)
        pygame.mixer.quit()
    except Exception:
        pass


@dataclass
class PackInfo:
    pack_id: str
    name: str
    source: str
    installed_dir: str
    installed_at: str
    file_count: int = 0


@dataclass
class AppState:
    wow_data_dir: str = ""
    output_data_dir: str = str(ROOT_DIR / "Data")
    extractor_path: str = ""
    expansion: str = "auto"
    locale: str = "auto"
    skip_dbc: bool = False
    dbc_csv: bool = False
    verify: bool = False
    verbose: bool = False
    threads: int = 0
    packs: list[PackInfo] = field(default_factory=list)
    active_pack_ids: list[str] = field(default_factory=list)
    last_extract_at: str = ""
    last_extract_ok: bool = False
    last_extract_command: str = ""
    last_override_build_at: str = ""


class PipelineManager:
    def __init__(self) -> None:
        PIPELINE_DIR.mkdir(parents=True, exist_ok=True)
        (PIPELINE_DIR / "packs").mkdir(parents=True, exist_ok=True)
        self.state = self._load_state()

    def _default_state(self) -> AppState:
        return AppState()

    def _load_state(self) -> AppState:
        if not STATE_FILE.exists():
            return self._default_state()
        try:
            doc = json.loads(STATE_FILE.read_text(encoding="utf-8"))
            packs = [PackInfo(**item) for item in doc.get("packs", [])]
            doc["packs"] = packs
            state = AppState(**doc)
            return state
        except (OSError, ValueError, TypeError):
            return self._default_state()

    def save_state(self) -> None:
        serializable = asdict(self.state)
        STATE_FILE.write_text(json.dumps(serializable, indent=2), encoding="utf-8")

    def now_str(self) -> str:
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    def _normalize_id(self, name: str) -> str:
        raw = "".join(ch.lower() if ch.isalnum() else "-" for ch in name).strip("-")
        base = raw or "pack"
        return f"{base}-{int(time.time())}"

    def _pack_dir(self, pack_id: str) -> Path:
        return PIPELINE_DIR / "packs" / pack_id

    def _looks_like_data_root(self, path: Path) -> bool:
        markers = {"interface", "world", "character", "textures", "sound"}
        names = {p.name.lower() for p in path.iterdir() if p.is_dir()} if path.is_dir() else set()
        return bool(markers.intersection(names))

    def find_data_root(self, pack_path: Path) -> Path:
        direct_data = pack_path / "Data"
        if direct_data.is_dir():
            return direct_data

        lower_data = pack_path / "data"
        if lower_data.is_dir():
            return lower_data

        if self._looks_like_data_root(pack_path):
            return pack_path

        # Common zip layout: one wrapper directory.
        children = [p for p in pack_path.iterdir() if p.is_dir()] if pack_path.is_dir() else []
        if len(children) == 1:
            child = children[0]
            child_data = child / "Data"
            if child_data.is_dir():
                return child_data
            if self._looks_like_data_root(child):
                return child

        return pack_path

    def _count_files(self, root: Path) -> int:
        if not root.exists():
            return 0
        return sum(1 for p in root.rglob("*") if p.is_file())

    def install_pack_from_zip(self, zip_path: Path) -> PackInfo:
        pack_name = zip_path.stem
        pack_id = self._normalize_id(pack_name)
        target = self._pack_dir(pack_id)
        target.mkdir(parents=True, exist_ok=False)

        with zipfile.ZipFile(zip_path, "r") as zf:
            for member in zf.infolist():
                member_path = (target / member.filename).resolve()
                if not str(member_path).startswith(str(target.resolve()) + "/") and member_path != target.resolve():
                    raise ValueError(f"Zip slip detected: {member.filename!r} escapes target directory")
                zf.extract(member, target)

        data_root = self.find_data_root(target)
        info = PackInfo(
            pack_id=pack_id,
            name=pack_name,
            source=str(zip_path),
            installed_dir=str(target),
            installed_at=self.now_str(),
            file_count=self._count_files(data_root),
        )
        self.state.packs.append(info)
        self.save_state()
        return info

    def install_pack_from_folder(self, folder_path: Path) -> PackInfo:
        pack_name = folder_path.name
        pack_id = self._normalize_id(pack_name)
        target = self._pack_dir(pack_id)
        shutil.copytree(folder_path, target)

        data_root = self.find_data_root(target)
        info = PackInfo(
            pack_id=pack_id,
            name=pack_name,
            source=str(folder_path),
            installed_dir=str(target),
            installed_at=self.now_str(),
            file_count=self._count_files(data_root),
        )
        self.state.packs.append(info)
        self.save_state()
        return info

    def uninstall_pack(self, pack_id: str) -> None:
        self.state.packs = [p for p in self.state.packs if p.pack_id != pack_id]
        self.state.active_pack_ids = [pid for pid in self.state.active_pack_ids if pid != pack_id]
        target = self._pack_dir(pack_id)
        if target.exists():
            shutil.rmtree(target)
        self.save_state()

    def set_pack_active(self, pack_id: str, active: bool) -> None:
        if active:
            if pack_id not in self.state.active_pack_ids:
                self.state.active_pack_ids.append(pack_id)
        else:
            self.state.active_pack_ids = [pid for pid in self.state.active_pack_ids if pid != pack_id]
        self.save_state()

    def move_active_pack(self, pack_id: str, delta: int) -> None:
        ids = self.state.active_pack_ids
        if pack_id not in ids:
            return
        idx = ids.index(pack_id)
        nidx = idx + delta
        if nidx < 0 or nidx >= len(ids):
            return
        ids[idx], ids[nidx] = ids[nidx], ids[idx]
        self.state.active_pack_ids = ids
        self.save_state()

    def rebuild_override(self) -> dict[str, int]:
        out_dir = Path(self.state.output_data_dir)
        override_dir = out_dir / "override"
        if override_dir.exists():
            shutil.rmtree(override_dir)
        override_dir.mkdir(parents=True, exist_ok=True)

        copied = 0
        replaced = 0

        active_map = {p.pack_id: p for p in self.state.packs}
        for pack_id in self.state.active_pack_ids:
            info = active_map.get(pack_id)
            if info is None:
                continue
            pack_dir = Path(info.installed_dir)
            if not pack_dir.exists():
                continue

            data_root = self.find_data_root(pack_dir)
            for source in data_root.rglob("*"):
                if not source.is_file():
                    continue
                rel = source.relative_to(data_root)
                target = override_dir / rel
                target.parent.mkdir(parents=True, exist_ok=True)
                if target.exists():
                    replaced += 1
                shutil.copy2(source, target)
                copied += 1

        self.state.last_override_build_at = self.now_str()
        self.save_state()
        return {"copied": copied, "replaced": replaced}

    def _resolve_extractor(self) -> list[str] | None:
        configured = self.state.extractor_path.strip()
        if configured:
            path = Path(configured)
            if path.exists() and path.is_file():
                return [str(path)]

        is_win = platform.system().lower().startswith("win")
        ext = ".exe" if is_win else ""
        for candidate in [
            ROOT_DIR / "build" / "bin" / f"asset_extract{ext}",
            ROOT_DIR / "build" / f"asset_extract{ext}",
            ROOT_DIR / "bin" / f"asset_extract{ext}",
        ]:
            if candidate.exists():
                return [str(candidate)]

        if is_win:
            ps_script = ROOT_DIR / "extract_assets.ps1"
            if ps_script.exists():
                return ["powershell", "-ExecutionPolicy", "Bypass", "-File", str(ps_script)]
            return None

        shell_script = ROOT_DIR / "extract_assets.sh"
        if shell_script.exists():
            return ["bash", str(shell_script)]

        return None

    def build_extract_command(self) -> list[str]:
        mpq_dir = self.state.wow_data_dir.strip()
        output_dir = self.state.output_data_dir.strip()
        if not mpq_dir or not output_dir:
            raise ValueError("Both WoW Data directory and output directory are required.")

        extractor = self._resolve_extractor()
        if extractor is None:
            raise ValueError(
                "No extractor found. Build asset_extract first or set the extractor path in Configuration."
            )

        if extractor[0].endswith("extract_assets.sh") or extractor[-1].endswith("extract_assets.sh"):
            cmd = [*extractor, mpq_dir]
            if self.state.expansion and self.state.expansion != "auto":
                cmd.append(self.state.expansion)
            return cmd

        cmd = [*extractor, "--mpq-dir", mpq_dir, "--output", output_dir]
        if self.state.expansion and self.state.expansion != "auto":
            cmd.extend(["--expansion", self.state.expansion])
        if self.state.locale and self.state.locale != "auto":
            cmd.extend(["--locale", self.state.locale])
        if self.state.skip_dbc:
            cmd.append("--skip-dbc")
        if self.state.dbc_csv:
            cmd.append("--dbc-csv")
        if self.state.verify:
            cmd.append("--verify")
        if self.state.verbose:
            cmd.append("--verbose")
        if self.state.threads > 0:
            cmd.extend(["--threads", str(self.state.threads)])
        return cmd

    def summarize_state(self) -> dict[str, Any]:
        output_dir = Path(self.state.output_data_dir)
        manifest_path = output_dir / "manifest.json"
        override_dir = output_dir / "override"

        summary: dict[str, Any] = {
            "output_dir": str(output_dir),
            "output_exists": output_dir.exists(),
            "manifest_exists": manifest_path.exists(),
            "manifest_entries": 0,
            "override_exists": override_dir.exists(),
            "override_files": self._count_files(override_dir),
            "packs_installed": len(self.state.packs),
            "packs_active": len(self.state.active_pack_ids),
            "last_extract_at": self.state.last_extract_at or "never",
            "last_extract_ok": self.state.last_extract_ok,
            "last_override_build_at": self.state.last_override_build_at or "never",
        }

        if manifest_path.exists():
            try:
                doc = json.loads(manifest_path.read_text(encoding="utf-8"))
                entries = doc.get("entries", {})
                if isinstance(entries, dict):
                    summary["manifest_entries"] = len(entries)
            except (OSError, ValueError, TypeError):
                summary["manifest_entries"] = -1

        return summary


class AssetPipelineGUI:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.manager = PipelineManager()

        self.log_queue: queue.Queue[str] = queue.Queue()
        self.proc_thread: threading.Thread | None = None
        self.proc_process: subprocess.Popen | None = None
        self.proc_running = False

        self.root.title("WoWee Asset Pipeline")
        self.root.geometry("1120x760")

        self.status_var = tk.StringVar(value="Ready")
        self._build_ui()
        self._load_vars_from_state()
        self.refresh_pack_list()
        self.refresh_state_view()
        self.root.after(120, self._poll_logs)

    def _build_ui(self) -> None:
        top = ttk.Frame(self.root, padding=10)
        top.pack(fill="both", expand=True)

        status = ttk.Label(top, textvariable=self.status_var, anchor="w")
        status.pack(fill="x", pady=(0, 8))

        self.notebook = ttk.Notebook(top)
        self.notebook.pack(fill="both", expand=True)

        self.cfg_tab = ttk.Frame(self.notebook, padding=10)
        self.packs_tab = ttk.Frame(self.notebook, padding=10)
        self.browser_tab = ttk.Frame(self.notebook, padding=4)
        self.state_tab = ttk.Frame(self.notebook, padding=10)
        self.logs_tab = ttk.Frame(self.notebook, padding=10)

        self.notebook.add(self.cfg_tab, text="Configuration")
        self.notebook.add(self.packs_tab, text="Texture Packs")
        self.notebook.add(self.browser_tab, text="Asset Browser")
        self.notebook.add(self.state_tab, text="Current State")
        self.notebook.add(self.logs_tab, text="Logs")

        self._build_config_tab()
        self._build_packs_tab()
        self._build_browser_tab()
        self._build_state_tab()
        self._build_logs_tab()

    def _build_config_tab(self) -> None:
        self.var_wow_data = tk.StringVar()
        self.var_output_data = tk.StringVar()
        self.var_extractor = tk.StringVar()
        self.var_expansion = tk.StringVar(value="auto")
        self.var_locale = tk.StringVar(value="auto")
        self.var_skip_dbc = tk.BooleanVar(value=False)
        self.var_dbc_csv = tk.BooleanVar(value=False)
        self.var_verify = tk.BooleanVar(value=False)
        self.var_verbose = tk.BooleanVar(value=False)
        self.var_threads = tk.IntVar(value=0)

        frame = self.cfg_tab

        self._path_row(frame, 0, "WoW Data (MPQ source)", self.var_wow_data, self._pick_wow_data_dir)
        self._path_row(frame, 1, "Output Data directory", self.var_output_data, self._pick_output_dir)
        self._path_row(frame, 2, "Extractor binary/script (optional)", self.var_extractor, self._pick_extractor)

        ttk.Label(frame, text="Expansion").grid(row=3, column=0, sticky="w", pady=6)
        exp_combo = ttk.Combobox(
            frame,
            textvariable=self.var_expansion,
            values=["auto", "classic", "turtle", "tbc", "wotlk"],
            state="readonly",
            width=18,
        )
        exp_combo.grid(row=3, column=1, sticky="w", pady=6)

        ttk.Label(frame, text="Locale").grid(row=3, column=2, sticky="w", pady=6)
        loc_combo = ttk.Combobox(
            frame,
            textvariable=self.var_locale,
            values=["auto", "enUS", "enGB", "deDE", "frFR", "esES", "esMX", "ruRU", "koKR", "zhCN", "zhTW"],
            state="readonly",
            width=12,
        )
        loc_combo.grid(row=3, column=3, sticky="w", pady=6)

        ttk.Label(frame, text="Threads (0 = auto)").grid(row=4, column=0, sticky="w", pady=6)
        ttk.Spinbox(frame, from_=0, to=256, textvariable=self.var_threads, width=8).grid(
            row=4, column=1, sticky="w", pady=6
        )

        opts = ttk.Frame(frame)
        opts.grid(row=5, column=0, columnspan=4, sticky="w", pady=6)
        ttk.Checkbutton(opts, text="Skip DBC extraction", variable=self.var_skip_dbc).pack(side="left", padx=(0, 12))
        ttk.Checkbutton(opts, text="Generate DBC CSV", variable=self.var_dbc_csv).pack(side="left", padx=(0, 12))
        ttk.Checkbutton(opts, text="Verify CRC", variable=self.var_verify).pack(side="left", padx=(0, 12))
        ttk.Checkbutton(opts, text="Verbose output", variable=self.var_verbose).pack(side="left", padx=(0, 12))

        buttons = ttk.Frame(frame)
        buttons.grid(row=6, column=0, columnspan=4, sticky="w", pady=12)
        ttk.Button(buttons, text="Save Configuration", command=self.save_config).pack(side="left", padx=(0, 8))
        ttk.Button(buttons, text="Run Extraction", command=self.run_extraction).pack(side="left", padx=(0, 8))
        self.cancel_btn = ttk.Button(buttons, text="Cancel Extraction", command=self.cancel_extraction, state="disabled")
        self.cancel_btn.pack(side="left", padx=(0, 8))
        ttk.Button(buttons, text="Refresh State", command=self.refresh_state_view).pack(side="left")

        tip = (
            "Texture packs are merged into <Output Data>/override in active order. "
            "Later packs override earlier packs file-by-file."
        )
        ttk.Label(frame, text=tip, foreground="#444").grid(row=7, column=0, columnspan=4, sticky="w", pady=(8, 0))

        frame.columnconfigure(1, weight=1)

    def _build_packs_tab(self) -> None:
        left = ttk.Frame(self.packs_tab)
        left.pack(side="left", fill="both", expand=True)

        right = ttk.Frame(self.packs_tab)
        right.pack(side="right", fill="y", padx=(12, 0))

        self.pack_list = tk.Listbox(left, height=22)
        self.pack_list.pack(fill="both", expand=True)
        self.pack_list.bind("<<ListboxSelect>>", lambda _evt: self._refresh_pack_detail())

        self.pack_detail = ScrolledText(left, height=10, wrap="word", state="disabled")
        self.pack_detail.pack(fill="both", expand=False, pady=(10, 0))

        ttk.Button(right, text="Install ZIP", width=22, command=self.install_zip).pack(pady=4)
        ttk.Button(right, text="Install Folder", width=22, command=self.install_folder).pack(pady=4)
        ttk.Separator(right, orient="horizontal").pack(fill="x", pady=8)
        ttk.Button(right, text="Activate", width=22, command=self.activate_selected_pack).pack(pady=4)
        ttk.Button(right, text="Deactivate", width=22, command=self.deactivate_selected_pack).pack(pady=4)
        ttk.Button(right, text="Move Up", width=22, command=lambda: self.move_selected_pack(-1)).pack(pady=4)
        ttk.Button(right, text="Move Down", width=22, command=lambda: self.move_selected_pack(1)).pack(pady=4)
        ttk.Separator(right, orient="horizontal").pack(fill="x", pady=8)
        ttk.Button(right, text="Rebuild Override", width=22, command=self.rebuild_override).pack(pady=4)
        ttk.Button(right, text="Uninstall", width=22, command=self.uninstall_selected_pack).pack(pady=4)

    # ── Asset Browser Tab ──────────────────────────────────────────────

    def _build_browser_tab(self) -> None:
        self._browser_manifest: dict[str, dict] = {}
        self._browser_manifest_lc: dict[str, str] = {}
        self._browser_manifest_list: list[str] = []
        self._browser_tree_populated: set[str] = set()
        self._browser_photo: Any = None  # prevent GC of PhotoImage
        self._browser_wireframe_verts: list[tuple[float, float, float]] = []
        self._browser_wireframe_tris: list[tuple[int, int, int]] = []
        self._browser_az = 0.0
        self._browser_el = 0.3
        self._browser_zoom = 1.0
        self._browser_drag_start: tuple[int, int] | None = None
        self._browser_dbc_rows: list[list[str]] = []
        self._browser_dbc_shown = 0

        # Top bar: search + filter
        top_bar = ttk.Frame(self.browser_tab)
        top_bar.pack(fill="x", pady=(0, 4))

        ttk.Label(top_bar, text="Search:").pack(side="left")
        self._browser_search_var = tk.StringVar()
        search_entry = ttk.Entry(top_bar, textvariable=self._browser_search_var, width=40)
        search_entry.pack(side="left", padx=(4, 8))
        search_entry.bind("<Return>", lambda _: self._browser_do_search())

        ttk.Label(top_bar, text="Type:").pack(side="left")
        self._browser_type_var = tk.StringVar(value="All")
        type_combo = ttk.Combobox(
            top_bar,
            textvariable=self._browser_type_var,
            values=["All", "BLP", "M2", "WMO", "DBC", "ADT", "Audio", "Text"],
            state="readonly",
            width=8,
        )
        type_combo.pack(side="left", padx=(4, 8))

        ttk.Button(top_bar, text="Search", command=self._browser_do_search).pack(side="left", padx=(0, 4))
        ttk.Button(top_bar, text="Reset", command=self._browser_reset_search).pack(side="left", padx=(0, 8))

        self._browser_hide_anim_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(top_bar, text="Hide .anim/.skin", variable=self._browser_hide_anim_var,
                        command=self._browser_reset_search).pack(side="left")

        self._browser_count_var = tk.StringVar(value="")
        ttk.Label(top_bar, textvariable=self._browser_count_var).pack(side="right")

        # Main paned: left tree + right preview
        paned = ttk.PanedWindow(self.browser_tab, orient="horizontal")
        paned.pack(fill="both", expand=True)

        # Left: directory tree
        left_frame = ttk.Frame(paned)
        paned.add(left_frame, weight=1)

        tree_scroll = ttk.Scrollbar(left_frame, orient="vertical")
        self._browser_tree = ttk.Treeview(left_frame, show="tree", yscrollcommand=tree_scroll.set)
        tree_scroll.config(command=self._browser_tree.yview)
        self._browser_tree.pack(side="left", fill="both", expand=True)
        tree_scroll.pack(side="right", fill="y")

        self._browser_tree.bind("<<TreeviewOpen>>", self._browser_on_expand)
        self._browser_tree.bind("<<TreeviewSelect>>", self._browser_on_select)

        # Right: preview area
        right_frame = ttk.Frame(paned)
        paned.add(right_frame, weight=3)

        self._browser_preview_frame = ttk.Frame(right_frame)
        self._browser_preview_frame.pack(fill="both", expand=True)

        # Bottom bar: file info
        self._browser_info_var = tk.StringVar(value="Select a file to preview")
        info_bar = ttk.Label(self.browser_tab, textvariable=self._browser_info_var, anchor="w", relief="sunken")
        info_bar.pack(fill="x", pady=(4, 0))

        # Load manifest
        self._browser_load_manifest()

    def _browser_load_manifest(self) -> None:
        output_dir = Path(self.manager.state.output_data_dir)
        manifest_path = output_dir / "manifest.json"
        if not manifest_path.exists():
            self._browser_count_var.set("No manifest.json found")
            return

        try:
            doc = json.loads(manifest_path.read_text(encoding="utf-8"))
            entries = doc.get("entries", {})
            if not isinstance(entries, dict):
                self._browser_count_var.set("Invalid manifest format")
                return
        except (OSError, ValueError, TypeError) as exc:
            self._browser_count_var.set(f"Manifest error: {exc}")
            return

        # Re-key manifest by the 'p' field (forward-slash paths) for tree display
        self._browser_manifest = {}
        for _key, val in entries.items():
            display_path = val.get("p", _key).replace("\\", "/")
            self._browser_manifest[display_path] = val
        self._browser_manifest_list = sorted(self._browser_manifest.keys(), key=str.lower)
        self._browser_count_var.set(f"{len(self._browser_manifest)} entries")

        # Build case-insensitive lookup: lowercase forward-slash path -> actual manifest path
        self._browser_manifest_lc: dict[str, str] = {}
        for p in self._browser_manifest:
            self._browser_manifest_lc[p.lower()] = p

        # Build directory tree indices: one full, one filtered
        # Single O(N) pass so tree operations are O(1) lookups
        _hidden_exts = {".anim", ".skin"}
        self._browser_dir_index_full = self._build_dir_index(self._browser_manifest_list)
        filtered = [p for p in self._browser_manifest_list
                    if os.path.splitext(p)[1].lower() not in _hidden_exts]
        self._browser_dir_index_filtered = self._build_dir_index(filtered)

        self._browser_populate_tree_root()

    @staticmethod
    def _build_dir_index(paths: list[str]) -> dict[str, tuple[set[str], list[str]]]:
        index: dict[str, tuple[set[str], list[str]]] = {}
        for path in paths:
            parts = path.split("/")
            for depth in range(len(parts)):
                dir_key = "/".join(parts[:depth]) if depth > 0 else ""
                if dir_key not in index:
                    index[dir_key] = (set(), [])
                entry = index[dir_key]
                if depth < len(parts) - 1:
                    entry[0].add(parts[depth])
                else:
                    entry[1].append(parts[depth])
        return index

    def _browser_active_index(self) -> dict[str, tuple[set[str], list[str]]]:
        if self._browser_hide_anim_var.get():
            return self._browser_dir_index_filtered
        return self._browser_dir_index_full

    def _browser_populate_tree_root(self) -> None:
        self._browser_tree.delete(*self._browser_tree.get_children())
        self._browser_tree_populated.clear()

        root_entry = self._browser_active_index().get("", (set(), []))
        subdirs, files = root_entry

        for name in sorted(subdirs, key=str.lower):
            node = self._browser_tree.insert("", "end", iid=name, text=name, open=False)
            self._browser_tree.insert(node, "end", iid=name + "/__dummy__", text="")

        for name in sorted(files, key=str.lower):
            self._browser_tree.insert("", "end", iid=name, text=name)

    def _browser_on_expand(self, event: Any) -> None:
        node = self._browser_tree.focus()
        if not node or node in self._browser_tree_populated:
            return
        self._browser_tree_populated.add(node)

        # Remove dummy child
        dummy = node + "/__dummy__"
        if self._browser_tree.exists(dummy):
            self._browser_tree.delete(dummy)

        dir_entry = self._browser_active_index().get(node, (set(), []))
        child_dirs, child_files = dir_entry

        for d in sorted(child_dirs, key=str.lower):
            child_id = node + "/" + d
            if not self._browser_tree.exists(child_id):
                n = self._browser_tree.insert(node, "end", iid=child_id, text=d, open=False)
                self._browser_tree.insert(n, "end", iid=child_id + "/__dummy__", text="")

        for f in sorted(child_files, key=str.lower):
            child_id = node + "/" + f
            if not self._browser_tree.exists(child_id):
                self._browser_tree.insert(node, "end", iid=child_id, text=f)

    def _browser_on_select(self, event: Any) -> None:
        sel = self._browser_tree.selection()
        if not sel:
            return
        path = sel[0]
        entry = self._browser_manifest.get(path)
        if entry is None:
            # It's a directory node
            self._browser_info_var.set(f"Directory: {path}")
            return
        self._browser_preview_file(path, entry)

    def _browser_do_search(self) -> None:
        query = self._browser_search_var.get().strip().lower()
        type_filter = self._browser_type_var.get()

        type_exts: dict[str, set[str]] = {
            "BLP": {".blp"},
            "M2": {".m2"},
            "WMO": {".wmo"},
            "DBC": {".dbc", ".csv"},
            "ADT": {".adt"},
            "Audio": {".wav", ".mp3", ".ogg"},
            "Text": {".xml", ".lua", ".json", ".html", ".toc", ".txt", ".wtf"},
        }

        hidden_exts = {".anim", ".skin"} if self._browser_hide_anim_var.get() else set()
        results: list[str] = []
        exts = type_exts.get(type_filter)
        for path in self._browser_manifest_list:
            ext = os.path.splitext(path)[1].lower()
            if ext in hidden_exts:
                continue
            if exts and ext not in exts:
                continue
            if query and query not in path.lower():
                continue
            results.append(path)

        # Repopulate tree with filtered results
        self._browser_tree.delete(*self._browser_tree.get_children())
        self._browser_tree_populated.clear()

        if len(results) > 5000:
            # Too many results — show directory structure
            self._browser_count_var.set(f"{len(results)} results (showing first 5000)")
            results = results[:5000]
        else:
            self._browser_count_var.set(f"{len(results)} results")

        # Build tree from filtered results
        dirs_added: set[str] = set()
        for path in results:
            parts = path.split("/")
            # Ensure parent directories exist
            for i in range(1, len(parts)):
                dir_id = "/".join(parts[:i])
                if dir_id not in dirs_added:
                    dirs_added.add(dir_id)
                    parent_id = "/".join(parts[:i - 1]) if i > 1 else ""
                    if not self._browser_tree.exists(dir_id):
                        self._browser_tree.insert(parent_id, "end", iid=dir_id, text=parts[i - 1], open=True)
            # Insert file
            parent_id = "/".join(parts[:-1]) if len(parts) > 1 else ""
            if not self._browser_tree.exists(path):
                self._browser_tree.insert(parent_id, "end", iid=path, text=parts[-1])
            self._browser_tree_populated.add(parent_id)

    def _browser_reset_search(self) -> None:
        self._browser_search_var.set("")
        self._browser_type_var.set("All")
        self._browser_populate_tree_root()
        self._browser_count_var.set(f"{len(self._browser_manifest)} entries")

    def _browser_clear_preview(self) -> None:
        for widget in self._browser_preview_frame.winfo_children():
            widget.destroy()
        self._browser_photo = None

    def _browser_file_ext(self, path: str) -> str:
        return os.path.splitext(path)[1].lower()

    def _browser_resolve_path(self, manifest_path: str) -> Path | None:
        entry = self._browser_manifest.get(manifest_path)
        if entry is None:
            return None
        rel = entry.get("p", manifest_path)
        output_dir = Path(self.manager.state.output_data_dir)
        full = output_dir / rel
        if full.exists():
            return full
        return None

    def _browser_preview_file(self, path: str, entry: dict) -> None:
        self._browser_clear_preview()

        size = entry.get("s", 0)
        crc = entry.get("h", "")
        ext = self._browser_file_ext(path)

        self._browser_info_var.set(f"{path}  |  Size: {self._format_size(size)}  |  CRC: {crc}")

        if ext == ".blp":
            self._browser_preview_blp(path, entry)
        elif ext == ".m2":
            self._browser_preview_m2(path, entry)
        elif ext == ".wmo":
            self._browser_preview_wmo(path, entry)
        elif ext in (".csv",):
            self._browser_preview_dbc(path, entry)
        elif ext == ".adt":
            self._browser_preview_adt(path, entry)
        elif ext in (".xml", ".lua", ".json", ".html", ".toc", ".txt", ".wtf", ".ini"):
            self._browser_preview_text(path, entry)
        elif ext in (".wav", ".mp3", ".ogg"):
            self._browser_preview_audio(path, entry)
        else:
            self._browser_preview_hex(path, entry)

    def _format_size(self, size: int) -> str:
        if size < 1024:
            return f"{size} B"
        elif size < 1024 * 1024:
            return f"{size / 1024:.1f} KB"
        else:
            return f"{size / (1024 * 1024):.1f} MB"

    # ── BLP Preview ──

    def _browser_preview_blp(self, path: str, entry: dict) -> None:
        if not HAS_PILLOW:
            lbl = ttk.Label(self._browser_preview_frame, text="Install Pillow for image preview:\n  pip install Pillow", anchor="center")
            lbl.pack(expand=True)
            return

        file_path = self._browser_resolve_path(path)
        if file_path is None:
            ttk.Label(self._browser_preview_frame, text="File not found on disk").pack(expand=True)
            return

        # Check for blp_convert
        blp_convert = ROOT_DIR / "build" / "bin" / "blp_convert"
        if not blp_convert.exists():
            ttk.Label(self._browser_preview_frame, text="blp_convert not found in build/bin/\nBuild the project first.").pack(expand=True)
            return

        # Cache directory
        cache_dir = PIPELINE_DIR / "preview_cache"
        cache_dir.mkdir(parents=True, exist_ok=True)

        cache_key = hashlib.md5(f"{path}:{entry.get('s', 0)}".encode()).hexdigest()
        cached_png = cache_dir / f"{cache_key}.png"

        if not cached_png.exists():
            # blp_convert outputs PNG alongside source: foo.blp -> foo.png
            try:
                result = subprocess.run(
                    [str(blp_convert), "--to-png", str(file_path)],
                    capture_output=True, text=True, timeout=10
                )
                output_png = file_path.with_suffix(".png")
                if result.returncode != 0 or not output_png.exists():
                    ttk.Label(self._browser_preview_frame, text=f"blp_convert failed:\n{result.stderr[:500]}").pack(expand=True)
                    return
                shutil.move(str(output_png), cached_png)
            except Exception as exc:
                ttk.Label(self._browser_preview_frame, text=f"Conversion error: {exc}").pack(expand=True)
                return

        # Load and display
        try:
            img = Image.open(cached_png)
            orig_w, orig_h = img.size

            # Fit to preview area
            max_w = self._browser_preview_frame.winfo_width() or 600
            max_h = self._browser_preview_frame.winfo_height() or 500
            max_w = max(max_w - 20, 200)
            max_h = max(max_h - 40, 200)

            scale = min(max_w / orig_w, max_h / orig_h, 1.0)
            if scale < 1.0:
                new_w = int(orig_w * scale)
                new_h = int(orig_h * scale)
                img = img.resize((new_w, new_h), Image.LANCZOS)

            self._browser_photo = ImageTk.PhotoImage(img)
            info_text = f"{orig_w} x {orig_h}"
            ttk.Label(self._browser_preview_frame, text=info_text).pack(pady=(4, 2))
            lbl = ttk.Label(self._browser_preview_frame, image=self._browser_photo)
            lbl.pack(expand=True)
        except Exception as exc:
            ttk.Label(self._browser_preview_frame, text=f"Image load error: {exc}").pack(expand=True)

    # ── M2 Preview (wireframe + textures + animations) ──

    # Common animation ID names
    _ANIM_NAMES: dict[int, str] = {
        0: "Stand", 1: "Death", 2: "Spell", 3: "Stop", 4: "Walk", 5: "Run",
        6: "Dead", 7: "Rise", 8: "StandWound", 9: "CombatWound", 10: "CombatCritical",
        11: "ShuffleLeft", 12: "ShuffleRight", 13: "Walkbackwards", 14: "Stun",
        15: "HandsClosed", 16: "AttackUnarmed", 17: "Attack1H", 18: "Attack2H",
        19: "Attack2HL", 20: "ParryUnarmed", 21: "Parry1H", 22: "Parry2H",
        23: "Parry2HL", 24: "ShieldBlock", 25: "ReadyUnarmed", 26: "Ready1H",
        27: "Ready2H", 28: "Ready2HL", 29: "ReadyBow", 30: "Dodge",
        31: "SpellPrecast", 32: "SpellCast", 33: "SpellCastArea",
        34: "NPCWelcome", 35: "NPCGoodbye", 36: "Block", 37: "JumpStart",
        38: "Jump", 39: "JumpEnd", 40: "Fall", 41: "SwimIdle", 42: "Swim",
        43: "SwimLeft", 44: "SwimRight", 45: "SwimBackwards",
        60: "SpellChannelDirected", 61: "SpellChannelOmni",
        69: "CombatAbility", 70: "CombatAbility2H",
        94: "Kneel", 113: "Loot",
        135: "ReadyRifle", 138: "Fly", 143: "CustomSpell01",
        157: "EmoteTalk", 185: "FlyIdle",
    }

    # Texture type names for non-filename textures
    _TEX_TYPE_NAMES: dict[int, str] = {
        0: "Filename", 1: "Body/Skin", 2: "Object Skin", 3: "Weapon Blade",
        4: "Weapon Handle", 5: "Environment", 6: "Hair", 7: "Facial Hair",
        8: "Skin Extra", 9: "UI Skin", 10: "Tauren Mane", 11: "Monster Skin 1",
        12: "Monster Skin 2", 13: "Monster Skin 3", 14: "Item Icon",
    }

    def _browser_parse_m2_textures(self, data: bytes, version: int) -> list[dict]:
        """Parse M2 texture definitions. Returns list of {type, flags, filename}."""
        if version <= 256:
            ofs = 92
        else:
            ofs = 80

        if len(data) < ofs + 8:
            return []

        n_tex, ofs_tex = struct.unpack_from("<II", data, ofs)
        if n_tex == 0 or n_tex > 1000 or ofs_tex + n_tex * 16 > len(data):
            return []

        textures = []
        for i in range(n_tex):
            base = ofs_tex + i * 16
            tex_type, tex_flags = struct.unpack_from("<II", data, base)
            name_len, name_ofs = struct.unpack_from("<II", data, base + 8)
            filename = ""
            if tex_type == 0 and name_len > 1 and name_ofs + name_len <= len(data):
                raw = data[name_ofs:name_ofs + name_len]
                filename = raw.split(b"\x00", 1)[0].decode("ascii", errors="replace")
            textures.append({"type": tex_type, "flags": tex_flags, "filename": filename})
        return textures

    def _browser_parse_m2_animations(self, data: bytes, version: int) -> list[dict]:
        """Parse M2 animation sequences. Returns list of {id, variation, duration, speed, flags}."""
        if len(data) < 36:
            return []

        n_anim, ofs_anim = struct.unpack_from("<II", data, 28)
        if n_anim == 0 or n_anim > 5000:
            return []

        seq_size = 68 if version <= 256 else 64
        if ofs_anim + n_anim * seq_size > len(data):
            return []

        anims = []
        for i in range(n_anim):
            base = ofs_anim + i * seq_size
            anim_id, variation = struct.unpack_from("<HH", data, base)
            if version <= 256:
                # Vanilla: startTimestamp(4) + endTimestamp(4), duration = end - start
                start_ts, end_ts = struct.unpack_from("<II", data, base + 4)
                duration = end_ts - start_ts
                speed = struct.unpack_from("<f", data, base + 12)[0]
                flags = struct.unpack_from("<I", data, base + 16)[0]
            else:
                duration = struct.unpack_from("<I", data, base + 4)[0]
                speed = struct.unpack_from("<f", data, base + 8)[0]
                flags = struct.unpack_from("<I", data, base + 12)[0]
            anims.append({
                "id": anim_id, "variation": variation,
                "duration": duration, "speed": speed, "flags": flags,
            })
        return anims

    def _browser_resolve_blp_path(self, blp_name: str) -> Path | None:
        """Resolve a BLP filename from M2 texture to a filesystem path, case-insensitively."""
        # Normalize: backslash -> forward slash
        normalized = blp_name.replace("\\", "/")
        lc = normalized.lower()

        # Try direct manifest lookup
        actual = self._browser_manifest_lc.get(lc)
        if actual:
            return self._browser_resolve_path(actual)

        # Try without leading slash
        if lc.startswith("/"):
            actual = self._browser_manifest_lc.get(lc[1:])
            if actual:
                return self._browser_resolve_path(actual)

        return None

    def _browser_load_blp_thumbnail(self, blp_path: Path, size: int = 64) -> Any:
        """Convert BLP to PNG and return a PhotoImage thumbnail, or None."""
        if not HAS_PILLOW:
            return None

        blp_convert = ROOT_DIR / "build" / "bin" / "blp_convert"
        if not blp_convert.exists():
            return None

        cache_dir = PIPELINE_DIR / "preview_cache"
        cache_dir.mkdir(parents=True, exist_ok=True)
        cache_key = hashlib.md5(str(blp_path).encode()).hexdigest()
        cached_png = cache_dir / f"{cache_key}.png"

        if not cached_png.exists():
            try:
                result = subprocess.run(
                    [str(blp_convert), "--to-png", str(blp_path)],
                    capture_output=True, text=True, timeout=10,
                )
                output_png = blp_path.with_suffix(".png")
                if result.returncode != 0 or not output_png.exists():
                    return None
                shutil.move(str(output_png), cached_png)
            except Exception:
                return None

        try:
            img = Image.open(cached_png)
            img.thumbnail((size, size), Image.LANCZOS)
            return ImageTk.PhotoImage(img)
        except Exception:
            return None

    def _browser_preview_m2(self, path: str, entry: dict) -> None:
        file_path = self._browser_resolve_path(path)
        if file_path is None:
            ttk.Label(self._browser_preview_frame, text="File not found on disk").pack(expand=True)
            return

        try:
            data = file_path.read_bytes()
            if len(data) < 108:
                ttk.Label(self._browser_preview_frame, text="M2 file too small").pack(expand=True)
                return

            magic = data[:4]
            if magic != b"MD20":
                ttk.Label(self._browser_preview_frame, text=f"Not an M2 file (magic: {magic!r})").pack(expand=True)
                return

            version = struct.unpack_from("<I", data, 4)[0]

            # Parse vertices
            if version <= 256:
                n_verts, ofs_verts = struct.unpack_from("<II", data, 68)
            else:
                n_verts, ofs_verts = struct.unpack_from("<II", data, 60)

            if n_verts == 0 or n_verts > 500000 or ofs_verts + n_verts * 48 > len(data):
                ttk.Label(self._browser_preview_frame, text=f"M2: {n_verts} vertices (no preview)").pack(expand=True)
                return

            verts: list[tuple[float, float, float]] = []
            for i in range(n_verts):
                off = ofs_verts + i * 48
                x, y, z = struct.unpack_from("<fff", data, off)
                verts.append((x, y, z))

            # Parse skin triangles
            tris: list[tuple[int, int, int]] = []
            skin_path = file_path.with_name(file_path.stem + "00.skin")
            if skin_path.exists():
                tris = self._parse_skin_triangles(skin_path.read_bytes())
            if not tris:
                for i in range(0, len(verts) - 1, 2):
                    tris.append((i, i + 1, i + 1))

            # Parse textures and animations
            textures = self._browser_parse_m2_textures(data, version)
            animations = self._browser_parse_m2_animations(data, version)

            # --- Layout ---
            # Top bar: info label + 3D viewer button
            top_bar = ttk.Frame(self._browser_preview_frame)
            top_bar.pack(fill="x", pady=(4, 2))

            info = f"M2 v{version}  |  {n_verts} verts, {len(tris)} tris  |  {len(textures)} textures, {len(animations)} anims"
            ttk.Label(top_bar, text=info).pack(side="left", fill="x", expand=True)

            def _open_3d_viewer(fp=file_path, tex_list=textures):
                blp_convert = ROOT_DIR / "build" / "bin" / "blp_convert"
                if not blp_convert.exists():
                    messagebox.showerror("Error", "blp_convert not found in build/bin/")
                    return
                # Resolve BLP paths for type-0 textures
                blp_map: dict[str, str] = {}
                for tex in tex_list:
                    if tex["type"] == 0 and tex["filename"]:
                        fname = tex["filename"]
                        resolved = self._browser_resolve_blp_path(fname)
                        if resolved:
                            norm = fname.replace("\\", "/")
                            blp_map[norm] = str(resolved)
                            blp_map[norm.lower()] = str(resolved)
                try:
                    from tools.m2_viewer import launch_m2_viewer
                    launch_m2_viewer(str(fp), blp_map, str(blp_convert))
                except ImportError:
                    # Try relative import for when run as script
                    try:
                        from m2_viewer import launch_m2_viewer as lmv
                        lmv(str(fp), blp_map, str(blp_convert))
                    except ImportError:
                        messagebox.showerror("Error", "m2_viewer.py not found. Requires pygame, PyOpenGL, numpy, Pillow.")

            ttk.Button(top_bar, text="Open 3D Viewer", command=_open_3d_viewer).pack(side="right", padx=4)

            # Main area: wireframe (left) + sidebar (right)
            main_pane = ttk.PanedWindow(self._browser_preview_frame, orient="horizontal")
            main_pane.pack(fill="both", expand=True)

            # Left: wireframe canvas
            left_frame = ttk.Frame(main_pane)
            main_pane.add(left_frame, weight=3)

            self._browser_wireframe_verts = verts
            self._browser_wireframe_tris = tris
            self._browser_az = 0.0
            self._browser_el = 0.3
            self._browser_zoom = 1.0

            canvas = tk.Canvas(left_frame, bg="#1a1a2e", highlightthickness=0)
            canvas.pack(fill="both", expand=True)
            self._browser_canvas = canvas

            canvas.bind("<Button-1>", self._browser_wf_mouse_down)
            canvas.bind("<B1-Motion>", self._browser_wf_mouse_drag)
            canvas.bind("<MouseWheel>", self._browser_wf_scroll)
            canvas.bind("<Button-4>", lambda e: self._browser_wf_scroll_linux(e, 1))
            canvas.bind("<Button-5>", lambda e: self._browser_wf_scroll_linux(e, -1))
            canvas.bind("<Configure>", lambda e: self._browser_wf_render())
            self.root.after(50, self._browser_wf_render)

            # Right: textures + animations sidebar
            right_frame = ttk.Frame(main_pane)
            main_pane.add(right_frame, weight=1)

            # --- Textures section ---
            ttk.Label(right_frame, text="Textures", font=("", 10, "bold")).pack(anchor="w", pady=(4, 2))

            # Keep references to thumbnail PhotoImages to prevent GC
            self._browser_m2_thumbs: list[Any] = []

            if textures:
                tex_frame = ttk.Frame(right_frame)
                tex_frame.pack(fill="x", padx=2)

                for i, tex in enumerate(textures):
                    row_frame = ttk.Frame(tex_frame)
                    row_frame.pack(fill="x", pady=1)

                    tex_type = tex["type"]
                    filename = tex["filename"]

                    if tex_type == 0 and filename:
                        # Try to load BLP thumbnail
                        display_name = filename.replace("\\", "/").split("/")[-1]
                        blp_fs_path = self._browser_resolve_blp_path(filename)
                        thumb = None
                        if blp_fs_path:
                            thumb = self._browser_load_blp_thumbnail(blp_fs_path)

                        if thumb:
                            self._browser_m2_thumbs.append(thumb)
                            lbl_img = ttk.Label(row_frame, image=thumb)
                            lbl_img.pack(side="left", padx=(0, 4))

                        lbl_text = ttk.Label(row_frame, text=display_name, wraplength=180)
                        lbl_text.pack(side="left", fill="x")
                    else:
                        type_name = self._TEX_TYPE_NAMES.get(tex_type, f"Type {tex_type}")
                        lbl = ttk.Label(row_frame, text=f"[{type_name}]", foreground="#888")
                        lbl.pack(side="left")
            else:
                ttk.Label(right_frame, text="(none)", foreground="#888").pack(anchor="w")

            # --- Separator ---
            ttk.Separator(right_frame, orient="horizontal").pack(fill="x", pady=6)

            # --- Animations section ---
            ttk.Label(right_frame, text="Animations", font=("", 10, "bold")).pack(anchor="w", pady=(0, 2))

            if animations:
                anim_frame = ttk.Frame(right_frame)
                anim_frame.pack(fill="both", expand=True)

                anim_scroll = ttk.Scrollbar(anim_frame, orient="vertical")
                anim_tree = ttk.Treeview(
                    anim_frame, columns=("id", "name", "var", "dur", "spd"),
                    show="headings", height=8,
                    yscrollcommand=anim_scroll.set,
                )
                anim_scroll.config(command=anim_tree.yview)

                anim_tree.heading("id", text="ID")
                anim_tree.heading("name", text="Name")
                anim_tree.heading("var", text="Var")
                anim_tree.heading("dur", text="Dur(ms)")
                anim_tree.heading("spd", text="Speed")

                anim_tree.column("id", width=35, minwidth=30)
                anim_tree.column("name", width=90, minwidth=60)
                anim_tree.column("var", width=30, minwidth=25)
                anim_tree.column("dur", width=55, minwidth=40)
                anim_tree.column("spd", width=45, minwidth=35)

                for anim in animations:
                    aid = anim["id"]
                    name = self._ANIM_NAMES.get(aid, "")
                    anim_tree.insert("", "end", values=(
                        aid, name, anim["variation"],
                        anim["duration"], f"{anim['speed']:.1f}",
                    ))

                anim_tree.pack(side="left", fill="both", expand=True)
                anim_scroll.pack(side="right", fill="y")
            else:
                ttk.Label(right_frame, text="(none)", foreground="#888").pack(anchor="w")

        except Exception as exc:
            ttk.Label(self._browser_preview_frame, text=f"M2 parse error: {exc}").pack(expand=True)

    def _parse_skin_triangles(self, data: bytes) -> list[tuple[int, int, int]]:
        if len(data) < 48:
            return []

        # Check for SKIN magic
        off = 0
        if data[:4] == b"SKIN":
            off = 4

        n_indices, ofs_indices = struct.unpack_from("<II", data, off + 0)
        n_tris, ofs_tris = struct.unpack_from("<II", data, off + 8)

        if n_indices == 0 or n_indices > 500000:
            return []
        if n_tris == 0 or n_tris > 500000:
            return []

        # Indices are uint16 vertex lookup
        if ofs_indices + n_indices * 2 > len(data):
            return []
        indices = list(struct.unpack_from(f"<{n_indices}H", data, ofs_indices))

        # Triangles are uint16 index-into-indices
        if ofs_tris + n_tris * 2 > len(data):
            return []
        tri_idx = list(struct.unpack_from(f"<{n_tris}H", data, ofs_tris))

        tris: list[tuple[int, int, int]] = []
        for i in range(0, len(tri_idx) - 2, 3):
            a, b, c = tri_idx[i], tri_idx[i + 1], tri_idx[i + 2]
            if a < n_indices and b < n_indices and c < n_indices:
                tris.append((indices[a], indices[b], indices[c]))

        return tris

    # ── WMO Preview ──

    def _browser_preview_wmo(self, path: str, entry: dict) -> None:
        file_path = self._browser_resolve_path(path)
        if file_path is None:
            ttk.Label(self._browser_preview_frame, text="File not found on disk").pack(expand=True)
            return

        # Check if this is a root WMO or group WMO
        name = file_path.name.lower()
        # Group WMOs typically end with _NNN.wmo
        is_group = len(name) > 8 and name[-8:-4].isdigit() and name[-9] == "_"

        try:
            if is_group:
                verts, tris = self._parse_wmo_group(file_path)
            else:
                # Root WMO — try to load first group
                verts, tris = self._parse_wmo_root_first_group(file_path)

            if not verts:
                data = file_path.read_bytes()
                if len(data) >= 24 and data[:4] in (b"MVER", b"REVM"):
                    n_groups = 0
                    # Try to find nGroups in MOHD chunk
                    pos = 0
                    while pos < len(data) - 8:
                        chunk_id = data[pos:pos + 4]
                        chunk_size = struct.unpack_from("<I", data, pos + 4)[0]
                        if chunk_id in (b"MOHD", b"DHOM"):
                            if chunk_size >= 16:
                                n_groups = struct.unpack_from("<I", data, pos + 8 + 16)[0]
                            break
                        pos += 8 + chunk_size
                    ttk.Label(self._browser_preview_frame, text=f"WMO root: {n_groups} groups\nSelect a group file (_000.wmo) for wireframe.").pack(expand=True)
                else:
                    ttk.Label(self._browser_preview_frame, text="Could not parse WMO vertices").pack(expand=True)
                return

            self._browser_wireframe_verts = verts
            self._browser_wireframe_tris = tris
            self._browser_az = 0.0
            self._browser_el = 0.3
            self._browser_zoom = 1.0

            top_bar = ttk.Frame(self._browser_preview_frame)
            top_bar.pack(fill="x", pady=(4, 2))

            info = f"WMO: {len(verts)} vertices, {len(tris)} triangles"
            ttk.Label(top_bar, text=info).pack(side="left", fill="x", expand=True)

            def _open_wmo_viewer(fp=file_path, ig=is_group):
                blp_convert = ROOT_DIR / "build" / "bin" / "blp_convert"
                if not blp_convert.exists():
                    messagebox.showerror("Error", "blp_convert not found in build/bin/")
                    return
                # Determine root and group files
                if ig:
                    stem = fp.stem
                    root_stem = stem.rsplit("_", 1)[0]
                    root_path = fp.parent / f"{root_stem}.wmo"
                    groups = sorted(fp.parent.glob(f"{root_stem}_*.wmo"))
                else:
                    root_path = fp
                    groups = sorted(fp.parent.glob(f"{fp.stem}_*.wmo"))
                # Parse root for texture names, resolve BLP paths
                blp_map: dict[str, str] = {}
                if root_path.exists():
                    import struct as _st
                    rdata = root_path.read_bytes()
                    pos = 0
                    while pos + 8 <= len(rdata):
                        cid = rdata[pos:pos + 4]
                        csz = _st.unpack_from("<I", rdata, pos + 4)[0]
                        cs = pos + 8
                        ce = cs + csz
                        if ce > len(rdata):
                            break
                        tag = cid if cid[:1].isupper() else cid[::-1]
                        if tag == b"MOTX":
                            off = 0
                            while off < csz:
                                end = rdata.find(b"\x00", cs + off, ce)
                                if end < 0:
                                    break
                                s = rdata[cs + off:end].decode("ascii", errors="replace")
                                if s:
                                    resolved = self._browser_resolve_blp_path(s)
                                    if resolved:
                                        norm = s.replace("\\", "/")
                                        blp_map[norm] = str(resolved)
                                        blp_map[norm.lower()] = str(resolved)
                                    off = end - cs + 1
                                else:
                                    off += 1
                            break
                        pos = ce
                try:
                    from tools.m2_viewer import launch_wmo_viewer
                    launch_wmo_viewer(
                        str(root_path) if root_path.exists() else "",
                        [str(g) for g in groups],
                        blp_map, str(blp_convert))
                except ImportError:
                    try:
                        from m2_viewer import launch_wmo_viewer as lwv
                        lwv(str(root_path) if root_path.exists() else "",
                            [str(g) for g in groups], blp_map, str(blp_convert))
                    except ImportError:
                        messagebox.showerror("Error", "m2_viewer.py not found. Requires pygame, PyOpenGL, numpy, Pillow.")

            ttk.Button(top_bar, text="Open 3D Viewer", command=_open_wmo_viewer).pack(side="right", padx=4)

            self._browser_create_wireframe_canvas()

        except Exception as exc:
            ttk.Label(self._browser_preview_frame, text=f"WMO parse error: {exc}").pack(expand=True)

    def _parse_wmo_group(self, file_path: Path) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]:
        data = file_path.read_bytes()
        verts: list[tuple[float, float, float]] = []
        tris: list[tuple[int, int, int]] = []

        pos = 0
        while pos < len(data) - 8:
            chunk_id = data[pos:pos + 4]
            chunk_size = struct.unpack_from("<I", data, pos + 4)[0]
            chunk_data_start = pos + 8

            # Handle both normal and reversed chunk IDs
            cid = chunk_id if chunk_id[:1].isupper() else chunk_id[::-1]

            if cid == b"MOVT":
                n = chunk_size // 12
                for i in range(n):
                    off = chunk_data_start + i * 12
                    x, y, z = struct.unpack_from("<fff", data, off)
                    verts.append((x, y, z))
            elif cid == b"MOVI":
                n = chunk_size // 2
                idx_list = list(struct.unpack_from(f"<{n}H", data, chunk_data_start))
                for i in range(0, n - 2, 3):
                    tris.append((idx_list[i], idx_list[i + 1], idx_list[i + 2]))

            pos = chunk_data_start + chunk_size

        return verts, tris

    def _parse_wmo_root_first_group(self, file_path: Path) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]:
        # Try _000.wmo
        stem = file_path.stem
        group_path = file_path.parent / f"{stem}_000.wmo"
        if group_path.exists():
            return self._parse_wmo_group(group_path)
        return [], []

    # ── Wireframe Canvas (shared M2/WMO) ──

    def _browser_create_wireframe_canvas(self) -> None:
        canvas = tk.Canvas(self._browser_preview_frame, bg="#1a1a2e", highlightthickness=0)
        canvas.pack(fill="both", expand=True)
        self._browser_canvas = canvas

        canvas.bind("<Button-1>", self._browser_wf_mouse_down)
        canvas.bind("<B1-Motion>", self._browser_wf_mouse_drag)
        canvas.bind("<MouseWheel>", self._browser_wf_scroll)
        canvas.bind("<Button-4>", lambda e: self._browser_wf_scroll_linux(e, 1))
        canvas.bind("<Button-5>", lambda e: self._browser_wf_scroll_linux(e, -1))
        canvas.bind("<Configure>", lambda e: self._browser_wf_render())

        self.root.after(50, self._browser_wf_render)

    def _browser_wf_mouse_down(self, event: Any) -> None:
        self._browser_drag_start = (event.x, event.y)

    def _browser_wf_mouse_drag(self, event: Any) -> None:
        if self._browser_drag_start is None:
            return
        dx = event.x - self._browser_drag_start[0]
        dy = event.y - self._browser_drag_start[1]
        self._browser_az += dx * 0.01
        self._browser_el += dy * 0.01
        self._browser_el = max(-math.pi / 2, min(math.pi / 2, self._browser_el))
        self._browser_drag_start = (event.x, event.y)
        self._browser_wf_render()

    def _browser_wf_scroll(self, event: Any) -> None:
        if event.delta > 0:
            self._browser_zoom *= 1.1
        else:
            self._browser_zoom /= 1.1
        self._browser_wf_render()

    def _browser_wf_scroll_linux(self, event: Any, direction: int) -> None:
        if direction > 0:
            self._browser_zoom *= 1.1
        else:
            self._browser_zoom /= 1.1
        self._browser_wf_render()

    def _browser_wf_render(self) -> None:
        canvas = self._browser_canvas
        canvas.delete("all")
        w = canvas.winfo_width()
        h = canvas.winfo_height()
        if w < 10 or h < 10:
            return

        verts = self._browser_wireframe_verts
        tris = self._browser_wireframe_tris
        if not verts:
            return

        # Compute bounding box for auto-scale
        xs = [v[0] for v in verts]
        ys = [v[1] for v in verts]
        zs = [v[2] for v in verts]
        cx = (min(xs) + max(xs)) / 2
        cy = (min(ys) + max(ys)) / 2
        cz = (min(zs) + max(zs)) / 2
        extent = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs), 0.001)
        scale = min(w, h) * 0.4 / extent * self._browser_zoom

        # Rotation matrix (azimuth around Z, elevation around X)
        cos_a, sin_a = math.cos(self._browser_az), math.sin(self._browser_az)
        cos_e, sin_e = math.cos(self._browser_el), math.sin(self._browser_el)

        def project(v: tuple[float, float, float]) -> tuple[float, float, float]:
            x, y, z = v[0] - cx, v[1] - cy, v[2] - cz
            # Rotate around Z (azimuth)
            rx = x * cos_a - y * sin_a
            ry = x * sin_a + y * cos_a
            rz = z
            # Rotate around X (elevation)
            ry2 = ry * cos_e - rz * sin_e
            rz2 = ry * sin_e + rz * cos_e
            return (w / 2 + rx * scale, h / 2 - rz2 * scale, ry2)

        projected = [project(v) for v in verts]

        # Depth-sort triangles
        if tris:
            tri_depths: list[tuple[float, int]] = []
            for i, (a, b, c) in enumerate(tris):
                if a < len(projected) and b < len(projected) and c < len(projected):
                    avg_depth = (projected[a][2] + projected[b][2] + projected[c][2]) / 3
                    tri_depths.append((avg_depth, i))
            tri_depths.sort()

            # Draw max 20000 triangles for performance
            max_draw = min(len(tri_depths), 20000)
            min_d = tri_depths[0][0] if tri_depths else 0
            max_d = tri_depths[-1][0] if tri_depths else 1
            d_range = max_d - min_d if max_d != min_d else 1

            for j in range(max_draw):
                depth, idx = tri_depths[j]
                a, b, c = tris[idx]
                if a >= len(projected) or b >= len(projected) or c >= len(projected):
                    continue

                # Depth coloring: closer = brighter
                t = 1.0 - (depth - min_d) / d_range
                intensity = int(60 + t * 160)
                color = f"#{intensity:02x}{intensity:02x}{int(intensity * 1.2) & 0xff:02x}"

                p1, p2, p3 = projected[a], projected[b], projected[c]
                canvas.create_line(p1[0], p1[1], p2[0], p2[1], fill=color, width=1)
                canvas.create_line(p2[0], p2[1], p3[0], p3[1], fill=color, width=1)
                canvas.create_line(p3[0], p3[1], p1[0], p1[1], fill=color, width=1)

    # ── DBC/CSV Preview ──

    def _browser_preview_dbc(self, path: str, entry: dict) -> None:
        file_path = self._browser_resolve_path(path)
        if file_path is None:
            ttk.Label(self._browser_preview_frame, text="File not found on disk").pack(expand=True)
            return

        try:
            text = file_path.read_text(encoding="utf-8", errors="replace")
        except Exception as exc:
            ttk.Label(self._browser_preview_frame, text=f"Read error: {exc}").pack(expand=True)
            return

        lines = text.splitlines()
        if not lines:
            ttk.Label(self._browser_preview_frame, text="Empty file").pack(expand=True)
            return

        # Parse header comment if present
        header_line = ""
        data_start = 0
        if lines[0].startswith("#"):
            header_line = lines[0]
            data_start = 1

        # Split CSV
        rows: list[list[str]] = []
        for line in lines[data_start:]:
            if line.strip():
                rows.append(line.split(","))
        self._browser_dbc_rows = rows
        self._browser_dbc_shown = 0

        if not rows:
            ttk.Label(self._browser_preview_frame, text="No data rows").pack(expand=True)
            return

        n_cols = len(rows[0])

        # Try to find column names from dbc_layouts.json
        col_names: list[str] = []
        dbc_name = file_path.stem  # e.g. "Spell"
        for exp in ("wotlk", "tbc", "classic", "turtle"):
            layout_path = ROOT_DIR / "Data" / "expansions" / exp / "dbc_layouts.json"
            if layout_path.exists():
                try:
                    layouts = json.loads(layout_path.read_text(encoding="utf-8"))
                    if dbc_name in layouts:
                        mapping = layouts[dbc_name]
                        names = [""] * n_cols
                        for name, idx in mapping.items():
                            if isinstance(idx, int) and 0 <= idx < n_cols:
                                names[idx] = name
                        col_names = [n if n else f"col_{i}" for i, n in enumerate(names)]
                        break
                except (OSError, ValueError):
                    pass

        if not col_names:
            col_names = [f"col_{i}" for i in range(n_cols)]

        # Info
        info = f"{len(rows)} rows, {n_cols} columns"
        if header_line:
            info += f"  ({header_line[:80]})"
        ttk.Label(self._browser_preview_frame, text=info).pack(pady=(4, 2))

        # Table frame with scrollbars
        table_frame = ttk.Frame(self._browser_preview_frame)
        table_frame.pack(fill="both", expand=True)

        xscroll = ttk.Scrollbar(table_frame, orient="horizontal")
        yscroll = ttk.Scrollbar(table_frame, orient="vertical")

        col_ids = [f"c{i}" for i in range(n_cols)]
        tree = ttk.Treeview(
            table_frame, columns=col_ids, show="headings",
            xscrollcommand=xscroll.set, yscrollcommand=yscroll.set
        )
        xscroll.config(command=tree.xview)
        yscroll.config(command=tree.yview)

        for i, cid in enumerate(col_ids):
            name = col_names[i] if i < len(col_names) else f"col_{i}"
            tree.heading(cid, text=name)
            tree.column(cid, width=80, minwidth=40)

        tree.pack(side="left", fill="both", expand=True)
        yscroll.pack(side="right", fill="y")
        xscroll.pack(side="bottom", fill="x")

        self._browser_dbc_tree = tree
        self._browser_dbc_col_ids = col_ids
        self._browser_load_more_dbc(500)

        if len(rows) > 500:
            btn = ttk.Button(self._browser_preview_frame, text="Load more rows...", command=lambda: self._browser_load_more_dbc(500))
            btn.pack(pady=4)
            self._browser_dbc_more_btn = btn

    def _browser_load_more_dbc(self, count: int) -> None:
        rows = self._browser_dbc_rows
        start = self._browser_dbc_shown
        end = min(start + count, len(rows))

        tree = self._browser_dbc_tree
        col_ids = self._browser_dbc_col_ids
        n_cols = len(col_ids)

        for i in range(start, end):
            row = rows[i]
            values = row[:n_cols]
            while len(values) < n_cols:
                values.append("")
            tree.insert("", "end", values=values)

        self._browser_dbc_shown = end
        if end >= len(rows) and hasattr(self, "_browser_dbc_more_btn"):
            self._browser_dbc_more_btn.configure(state="disabled", text="All rows loaded")

    # ── ADT Preview ──

    def _browser_preview_adt(self, path: str, entry: dict) -> None:
        file_path = self._browser_resolve_path(path)
        if file_path is None:
            ttk.Label(self._browser_preview_frame, text="File not found on disk").pack(expand=True)
            return

        try:
            data = file_path.read_bytes()
        except Exception as exc:
            ttk.Label(self._browser_preview_frame, text=f"Read error: {exc}").pack(expand=True)
            return

        # Parse MCNK chunks for height data
        heights: list[list[float]] = []  # 16x16 chunks, each with avg height
        pos = 0
        while pos < len(data) - 8:
            chunk_id = data[pos:pos + 4]
            chunk_size = struct.unpack_from("<I", data, pos + 4)[0]

            if chunk_id in (b"MCNK", b"KNMC"):
                if chunk_size >= 120:
                    # Base height at offset 112 in MCNK body
                    base_z = struct.unpack_from("<f", data, pos + 8 + 112)[0]
                    # MCVT heights (145 floats) — scan for MCVT sub-chunk
                    mcvt_heights: list[float] = []
                    sub_pos = pos + 8 + 128  # skip MCNK header
                    sub_end = pos + 8 + chunk_size
                    while sub_pos < sub_end - 8:
                        sub_id = data[sub_pos:sub_pos + 4]
                        sub_size = struct.unpack_from("<I", data, sub_pos + 4)[0]
                        if sub_id in (b"MCVT", b"TVCM"):
                            n_h = min(sub_size // 4, 145)
                            for i in range(n_h):
                                h = struct.unpack_from("<f", data, sub_pos + 8 + i * 4)[0]
                                mcvt_heights.append(base_z + h)
                            break
                        sub_pos += 8 + sub_size

                    if mcvt_heights:
                        avg = sum(mcvt_heights) / len(mcvt_heights)
                        heights.append([avg])
                    else:
                        heights.append([base_z])

            pos += 8 + chunk_size

        if not heights:
            ttk.Label(self._browser_preview_frame, text="No MCNK chunks found in ADT").pack(expand=True)
            return

        # Arrange into 16x16 grid
        n_chunks = len(heights)
        grid_size = int(math.ceil(math.sqrt(n_chunks)))
        all_h = [h[0] for h in heights]
        min_h = min(all_h)
        max_h = max(all_h)
        h_range = max_h - min_h if max_h != min_h else 1

        ttk.Label(self._browser_preview_frame, text=f"ADT: {n_chunks} chunks, height range: {min_h:.1f} - {max_h:.1f}").pack(pady=(4, 2))

        canvas = tk.Canvas(self._browser_preview_frame, bg="#111", highlightthickness=0)
        canvas.pack(fill="both", expand=True)

        def draw_heightmap(event: Any = None) -> None:
            canvas.delete("all")
            cw = canvas.winfo_width()
            ch = canvas.winfo_height()
            if cw < 10 or ch < 10:
                return
            cell = min(cw, ch) // grid_size

            for i, h_list in enumerate(heights):
                row = i // grid_size
                col = i % grid_size
                t = (h_list[0] - min_h) / h_range
                # Green-brown colormap
                r = int(50 + t * 150)
                g = int(80 + (1 - t) * 120 + t * 50)
                b = int(30 + t * 30)
                color = f"#{r:02x}{g:02x}{b:02x}"
                x1 = col * cell
                y1 = row * cell
                canvas.create_rectangle(x1, y1, x1 + cell, y1 + cell, fill=color, outline="")

        canvas.bind("<Configure>", draw_heightmap)
        canvas.after(50, draw_heightmap)

    # ── Text Preview ──

    def _browser_preview_text(self, path: str, entry: dict) -> None:
        file_path = self._browser_resolve_path(path)
        if file_path is None:
            ttk.Label(self._browser_preview_frame, text="File not found on disk").pack(expand=True)
            return

        try:
            text = file_path.read_text(encoding="utf-8", errors="replace")
        except Exception as exc:
            ttk.Label(self._browser_preview_frame, text=f"Read error: {exc}").pack(expand=True)
            return

        st = ScrolledText(self._browser_preview_frame, wrap="none", font=("Courier", 10))
        st.pack(fill="both", expand=True)
        st.insert("1.0", text[:500000])  # Cap at 500k chars
        st.configure(state="disabled")

    # ── Audio Preview ──

    def _browser_preview_audio(self, path: str, entry: dict) -> None:
        file_path = self._browser_resolve_path(path)
        if file_path is None:
            ttk.Label(self._browser_preview_frame, text="File not found on disk").pack(expand=True)
            return

        ext = self._browser_file_ext(path)
        info_lines = [f"Audio file: {file_path.name}", f"Size: {self._format_size(entry.get('s', 0))}"]

        try:
            data = file_path.read_bytes()
            if ext == ".wav" and len(data) >= 44:
                if data[:4] == b"RIFF" and data[8:12] == b"WAVE":
                    channels = struct.unpack_from("<H", data, 22)[0]
                    sample_rate = struct.unpack_from("<I", data, 24)[0]
                    bits = struct.unpack_from("<H", data, 34)[0]
                    data_size = struct.unpack_from("<I", data, 40)[0]
                    duration = data_size / (sample_rate * channels * bits // 8) if (sample_rate * channels * bits) else 0
                    info_lines.extend([
                        f"Format: WAV (RIFF)",
                        f"Channels: {channels}",
                        f"Sample rate: {sample_rate} Hz",
                        f"Bit depth: {bits}",
                        f"Duration: {duration:.1f}s",
                    ])
            elif ext == ".mp3" and len(data) >= 4:
                info_lines.append("Format: MP3")
                if data[:3] == b"ID3":
                    info_lines.append("Has ID3 tag")
            elif ext == ".ogg" and len(data) >= 4:
                if data[:4] == b"OggS":
                    info_lines.append("Format: Ogg Vorbis")
        except Exception:
            pass

        text = "\n".join(info_lines)
        lbl = ttk.Label(self._browser_preview_frame, text=text, justify="left", anchor="nw")
        lbl.pack(padx=20, pady=(20, 8))

        # Audio playback controls
        btn_frame = ttk.Frame(self._browser_preview_frame)
        btn_frame.pack(padx=20, pady=4)

        self._audio_status_var = tk.StringVar(value="Stopped")
        status_lbl = ttk.Label(self._browser_preview_frame, textvariable=self._audio_status_var)
        status_lbl.pack(padx=20, pady=(4, 0))

        def _play_audio():
            self._browser_stop_audio()
            try:
                import multiprocessing
                ctx = multiprocessing.get_context("spawn")
                self._audio_proc = ctx.Process(
                    target=_audio_subprocess, args=(str(file_path),), daemon=True)
                self._audio_proc.start()
                self._audio_status_var.set("Playing...")
            except Exception as exc:
                self._audio_status_var.set(f"Error: {exc}")

        def _stop_audio():
            self._browser_stop_audio()
            self._audio_status_var.set("Stopped")

        ttk.Button(btn_frame, text="Play", command=_play_audio).pack(side="left", padx=4)
        ttk.Button(btn_frame, text="Stop", command=_stop_audio).pack(side="left", padx=4)

    def _browser_stop_audio(self):
        proc = getattr(self, "_audio_proc", None)
        if proc and proc.is_alive():
            proc.terminate()
            proc.join(timeout=0.5)
            if proc.is_alive():
                proc.kill()
                proc.join(timeout=0.5)
        self._audio_proc = None

    # ── Hex Dump Preview ──

    def _browser_preview_hex(self, path: str, entry: dict) -> None:
        file_path = self._browser_resolve_path(path)
        if file_path is None:
            ttk.Label(self._browser_preview_frame, text="File not found on disk").pack(expand=True)
            return

        try:
            data = file_path.read_bytes()[:512]
        except Exception as exc:
            ttk.Label(self._browser_preview_frame, text=f"Read error: {exc}").pack(expand=True)
            return

        lines: list[str] = []
        for i in range(0, len(data), 16):
            chunk = data[i:i + 16]
            hex_part = " ".join(f"{b:02x}" for b in chunk)
            ascii_part = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
            lines.append(f"{i:08x}  {hex_part:<48s}  {ascii_part}")

        ttk.Label(self._browser_preview_frame, text=f"Hex dump (first {len(data)} bytes):").pack(pady=(4, 2))
        st = ScrolledText(self._browser_preview_frame, wrap="none", font=("Courier", 10))
        st.pack(fill="both", expand=True)
        st.insert("1.0", "\n".join(lines))
        st.configure(state="disabled")

    # ── End Asset Browser ──────────────────────────────────────────────

    def _build_state_tab(self) -> None:
        actions = ttk.Frame(self.state_tab)
        actions.pack(fill="x")
        ttk.Button(actions, text="Refresh", command=self.refresh_state_view).pack(side="left")

        self.state_text = ScrolledText(self.state_tab, wrap="word", state="disabled")
        self.state_text.pack(fill="both", expand=True, pady=(10, 0))

    def _build_logs_tab(self) -> None:
        actions = ttk.Frame(self.logs_tab)
        actions.pack(fill="x")
        ttk.Button(actions, text="Clear Logs", command=self.clear_logs).pack(side="left")

        self.log_text = ScrolledText(self.logs_tab, wrap="none", state="disabled")
        self.log_text.pack(fill="both", expand=True, pady=(10, 0))

    def _path_row(self, frame: ttk.Frame, row: int, label: str, variable: tk.StringVar, browse_cmd) -> None:
        ttk.Label(frame, text=label).grid(row=row, column=0, sticky="w", pady=6)
        ttk.Entry(frame, textvariable=variable).grid(row=row, column=1, columnspan=2, sticky="ew", pady=6)
        ttk.Button(frame, text="Browse", command=browse_cmd).grid(row=row, column=3, sticky="e", pady=6)

    def _pick_wow_data_dir(self) -> None:
        picked = filedialog.askdirectory(title="Select WoW Data directory")
        if picked:
            self.var_wow_data.set(picked)

    def _pick_output_dir(self) -> None:
        picked = filedialog.askdirectory(title="Select output Data directory")
        if picked:
            self.var_output_data.set(picked)

    def _pick_extractor(self) -> None:
        picked = filedialog.askopenfilename(title="Select extractor binary or script")
        if picked:
            self.var_extractor.set(picked)

    def _load_vars_from_state(self) -> None:
        st = self.manager.state
        self.var_wow_data.set(st.wow_data_dir)
        self.var_output_data.set(st.output_data_dir)
        self.var_extractor.set(st.extractor_path)
        self.var_expansion.set(st.expansion)
        self.var_locale.set(st.locale)
        self.var_skip_dbc.set(st.skip_dbc)
        self.var_dbc_csv.set(st.dbc_csv)
        self.var_verify.set(st.verify)
        self.var_verbose.set(st.verbose)
        self.var_threads.set(st.threads)

    def save_config(self) -> None:
        st = self.manager.state
        st.wow_data_dir = self.var_wow_data.get().strip()
        st.output_data_dir = self.var_output_data.get().strip()
        st.extractor_path = self.var_extractor.get().strip()
        st.expansion = self.var_expansion.get().strip() or "auto"
        st.locale = self.var_locale.get().strip() or "auto"
        st.skip_dbc = bool(self.var_skip_dbc.get())
        st.dbc_csv = bool(self.var_dbc_csv.get())
        st.verify = bool(self.var_verify.get())
        st.verbose = bool(self.var_verbose.get())
        st.threads = int(self.var_threads.get())
        self.manager.save_state()
        self.status_var.set("Configuration saved")

    def _selected_pack(self) -> PackInfo | None:
        sel = self.pack_list.curselection()
        if not sel:
            return None
        idx = int(sel[0])
        if idx < 0 or idx >= len(self.manager.state.packs):
            return None
        return self.manager.state.packs[idx]

    def refresh_pack_list(self) -> None:
        prev_sel = self.pack_list.curselection()
        active = self.manager.state.active_pack_ids
        self.pack_list.delete(0, tk.END)
        for pack in self.manager.state.packs:
            marker = ""
            if pack.pack_id in active:
                marker = f"[active #{active.index(pack.pack_id) + 1}] "
            self.pack_list.insert(tk.END, f"{marker}{pack.name}")
        # Restore previous selection if still valid.
        for idx in prev_sel:
            if 0 <= idx < self.pack_list.size():
                self.pack_list.selection_set(idx)
                self.pack_list.see(idx)
        self._refresh_pack_detail()

    def _refresh_pack_detail(self) -> None:
        pack = self._selected_pack()
        self.pack_detail.configure(state="normal")
        self.pack_detail.delete("1.0", tk.END)
        if pack is None:
            self.pack_detail.insert(tk.END, "Select a texture pack to see details.")
            self.pack_detail.configure(state="disabled")
            return

        active = "yes" if pack.pack_id in self.manager.state.active_pack_ids else "no"
        order = "-"
        if pack.pack_id in self.manager.state.active_pack_ids:
            order = str(self.manager.state.active_pack_ids.index(pack.pack_id) + 1)
        lines = [
            f"Name: {pack.name}",
            f"Active: {active}",
            f"Order: {order}",
            f"Files: {pack.file_count}",
            f"Installed at: {pack.installed_at}",
            f"Installed dir: {pack.installed_dir}",
            f"Source: {pack.source}",
        ]
        self.pack_detail.insert(tk.END, "\n".join(lines))
        self.pack_detail.configure(state="disabled")

    def install_zip(self) -> None:
        zip_path = filedialog.askopenfilename(
            title="Choose texture pack ZIP",
            filetypes=[("ZIP archives", "*.zip"), ("All files", "*.*")],
        )
        if not zip_path:
            return
        try:
            info = self.manager.install_pack_from_zip(Path(zip_path))
        except Exception as exc:  # pylint: disable=broad-except
            messagebox.showerror("Install failed", str(exc))
            return

        self.refresh_pack_list()
        self.refresh_state_view()
        self.status_var.set(f"Installed pack: {info.name}")

    def install_folder(self) -> None:
        folder = filedialog.askdirectory(title="Choose texture pack folder")
        if not folder:
            return
        try:
            info = self.manager.install_pack_from_folder(Path(folder))
        except Exception as exc:  # pylint: disable=broad-except
            messagebox.showerror("Install failed", str(exc))
            return

        self.refresh_pack_list()
        self.refresh_state_view()
        self.status_var.set(f"Installed pack: {info.name}")

    def activate_selected_pack(self) -> None:
        pack = self._selected_pack()
        if pack is None:
            return
        self.manager.set_pack_active(pack.pack_id, True)
        self.refresh_pack_list()
        self.refresh_state_view()
        self.status_var.set(f"Activated pack: {pack.name}")

    def deactivate_selected_pack(self) -> None:
        pack = self._selected_pack()
        if pack is None:
            return
        self.manager.set_pack_active(pack.pack_id, False)
        self.refresh_pack_list()
        self.refresh_state_view()
        self.status_var.set(f"Deactivated pack: {pack.name}")

    def move_selected_pack(self, delta: int) -> None:
        pack = self._selected_pack()
        if pack is None:
            return
        self.manager.move_active_pack(pack.pack_id, delta)
        self.refresh_pack_list()
        self.refresh_state_view()
        self.status_var.set(f"Reordered active pack: {pack.name}")

    def uninstall_selected_pack(self) -> None:
        pack = self._selected_pack()
        if pack is None:
            return
        ok = messagebox.askyesno("Confirm uninstall", f"Uninstall texture pack '{pack.name}'?")
        if not ok:
            return
        self.manager.uninstall_pack(pack.pack_id)
        self.refresh_pack_list()
        self.refresh_state_view()
        self.status_var.set(f"Uninstalled pack: {pack.name}")

    def rebuild_override(self) -> None:
        self.status_var.set("Rebuilding override...")
        self.log_queue.put(f"[{self.manager.now_str()}] Starting override rebuild...")

        def worker() -> None:
            try:
                report = self.manager.rebuild_override()
                msg = f"Override rebuilt: {report['copied']} files copied, {report['replaced']} replaced"
                self.log_queue.put(f"[{self.manager.now_str()}] Override rebuild complete: {report['copied']} copied, {report['replaced']} replaced")
                self.root.after(0, lambda: self.status_var.set(msg))
            except Exception as exc:  # pylint: disable=broad-except
                self.log_queue.put(f"[{self.manager.now_str()}] Override rebuild failed: {exc}")
                self.root.after(0, lambda: self.status_var.set("Override rebuild failed"))
            finally:
                self.root.after(0, self.refresh_state_view)

        threading.Thread(target=worker, daemon=True).start()

    def clear_logs(self) -> None:
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", tk.END)
        self.log_text.configure(state="disabled")

    def _append_log(self, line: str) -> None:
        self.log_text.configure(state="normal")
        self.log_text.insert(tk.END, line + "\n")
        self.log_text.see(tk.END)
        self.log_text.configure(state="disabled")

    def _poll_logs(self) -> None:
        while True:
            try:
                line = self.log_queue.get_nowait()
            except queue.Empty:
                break
            self._append_log(line)
        self.root.after(120, self._poll_logs)

    def cancel_extraction(self) -> None:
        if self.proc_process is not None:
            self.proc_process.terminate()
            self.log_queue.put(f"[{self.manager.now_str()}] Extraction cancelled by user")
            self.status_var.set("Extraction cancelled")

    def run_extraction(self) -> None:
        if self.proc_running:
            messagebox.showinfo("Extraction running", "An extraction is already running.")
            return

        self.save_config()

        try:
            cmd = self.manager.build_extract_command()
        except ValueError as exc:
            messagebox.showerror("Cannot run extraction", str(exc))
            return

        self.cancel_btn.configure(state="normal")

        def worker() -> None:
            self.proc_running = True
            started = self.manager.now_str()
            self.log_queue.put(f"[{started}] Running: {' '.join(cmd)}")
            self.root.after(0, lambda: self.status_var.set("Extraction running..."))

            ok = False
            try:
                process = subprocess.Popen(
                    cmd,
                    cwd=str(ROOT_DIR),
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                )
                self.proc_process = process
                assert process.stdout is not None
                for line in process.stdout:
                    self.log_queue.put(line.rstrip())
                rc = process.wait()
                ok = rc == 0
                if not ok:
                    self.log_queue.put(f"Extractor exited with status {rc}")
            except Exception as exc:  # pylint: disable=broad-except
                self.log_queue.put(f"Extraction error: {exc}")
            finally:
                self.proc_process = None
                self.manager.state.last_extract_at = self.manager.now_str()
                self.manager.state.last_extract_ok = ok
                self.manager.state.last_extract_command = " ".join(cmd)
                self.manager.save_state()
                self.proc_running = False
                self.root.after(0, self.refresh_state_view)
                self.root.after(0, lambda: self.cancel_btn.configure(state="disabled"))
                self.root.after(
                    0, lambda: self.status_var.set("Extraction complete" if ok else "Extraction failed")
                )

        self.proc_thread = threading.Thread(target=worker, daemon=True)
        self.proc_thread.start()

    def refresh_state_view(self) -> None:
        summary = self.manager.summarize_state()

        active_names = []
        pack_map = {p.pack_id: p.name for p in self.manager.state.packs}
        for pid in self.manager.state.active_pack_ids:
            active_names.append(pack_map.get(pid, f"<missing {pid}>"))

        lines = [
            "WoWee Asset Pipeline State",
            "",
            f"Output directory: {summary['output_dir']}",
            f"Output exists: {summary['output_exists']}",
            f"manifest.json present: {summary['manifest_exists']}",
            f"Manifest entries: {summary['manifest_entries']}",
            "",
            f"Override folder present: {summary['override_exists']}",
            f"Override file count: {summary['override_files']}",
            f"Last override build: {summary['last_override_build_at']}",
            "",
            f"Installed texture packs: {summary['packs_installed']}",
            f"Active texture packs: {summary['packs_active']}",
            "Active order:",
        ]
        if active_names:
            for i, name in enumerate(active_names, start=1):
                lines.append(f"  {i}. {name}")
        else:
            lines.append("  (none)")

        lines.extend(
            [
                "",
                f"Last extraction time: {summary['last_extract_at']}",
                f"Last extraction success: {summary['last_extract_ok']}",
                f"Last extraction command: {self.manager.state.last_extract_command or '(none)'}",
                "",
                "Pipeline files:",
                f"  State file: {STATE_FILE}",
                f"  Packs dir:  {PIPELINE_DIR / 'packs'}",
            ]
        )

        self.state_text.configure(state="normal")
        self.state_text.delete("1.0", tk.END)
        self.state_text.insert(tk.END, "\n".join(lines))
        self.state_text.configure(state="disabled")


def main() -> None:
    root = tk.Tk()
    AssetPipelineGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
