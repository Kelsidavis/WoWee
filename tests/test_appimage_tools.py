"""Exercise failed tool downloads through the real AppImage packaging script."""

import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "container/appimage/build-appimage.sh"


class AppImageTools(unittest.TestCase):
    def check_rejected_download(self, curl_status, cached=False):
        with tempfile.TemporaryDirectory(prefix="wowee-appimage-") as tmp:
            root = Path(tmp)
            build = root / "build"
            bin_dir = build / "bin"
            bin_dir.mkdir(parents=True)
            wowee = bin_dir / "wowee"
            wowee.write_text("#!/bin/sh\nexit 0\n")
            wowee.chmod(0o755)
            (bin_dir / "assets").mkdir()
            (bin_dir / "addons").mkdir()

            stubs = root / "stubs"
            stubs.mkdir()
            # Both a completed download with a wrong checksum and a partial
            # download must stay untrusted. The payload only records execution.
            curl = stubs / "curl"
            curl.write_text(
                f"#!{sys.executable}\n"
                "import os, pathlib, sys\n"
                "path = pathlib.Path(sys.argv[sys.argv.index('-o') + 1])\n"
                "path.write_text('#!/bin/sh\\necho executed > \"$TOOL_MARKER\"\\nexit 1\\n')\n"
                "with open(os.environ['DOWNLOAD_LOG'], 'a') as log:\n"
                "    log.write(str(path) + '\\n')\n"
                f"sys.exit({curl_status})\n"
            )
            curl.chmod(0o755)
            cached_tool = build / "appimage/linuxdeploy-x86_64.AppImage"
            if cached:
                cached_tool.parent.mkdir()
                cached_tool.write_text('#!/bin/sh\necho executed > "$TOOL_MARKER"\nexit 1\n')
                cached_tool.chmod(0o755)
            # macOS has shasum rather than sha256sum. Use its real SHA-256
            # checker when running these Linux packaging regressions there.
            if not shutil.which("sha256sum"):
                checker = stubs / "sha256sum"
                checker.write_text(
                    "#!/bin/sh\nexec " + shlex.quote(shutil.which("shasum")) +
                    ' -a 256 "$@"\n'
                )
                checker.chmod(0o755)

            env = os.environ.copy()
            for key in ("LINUXDEPLOY", "APPIMAGETOOL", "BASH_ENV", "SHELLOPTS"):
                env.pop(key, None)
            env.update(
                PATH=str(stubs) + os.pathsep + env["PATH"],
                ARCH="x86_64",
                VERSION="test",
                OUT_DIR=str(build / "appimage"),
                TOOL_MARKER=str(root / "executed"),
                DOWNLOAD_LOG=str(root / "downloads"),
            )
            result = subprocess.run(
                [shutil.which("bash"), str(SCRIPT), str(build)],
                env=env, text=True, capture_output=True, timeout=20,
            )
            self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertFalse((root / "executed").exists(), result.stdout + result.stderr)
            if cached:
                self.assertFalse((root / "downloads").exists())
            else:
                self.assertFalse(cached_tool.exists())
                # Failure must also stop the second download.
                self.assertEqual(len((root / "downloads").read_text().splitlines()), 1)
            if curl_status == 0:
                self.assertIn("FAILED", result.stderr)

    def test_checksum_mismatch_is_rejected(self):
        self.check_rejected_download(curl_status=0)

    def test_partial_download_is_rejected(self):
        self.check_rejected_download(curl_status=22)

    def test_invalid_cached_tool_is_rejected(self):
        self.check_rejected_download(curl_status=0, cached=True)


if __name__ == "__main__":
    unittest.main()
