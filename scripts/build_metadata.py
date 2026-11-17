#!/usr/bin/env python3
"""Record the exact source, version, and artifact identity for the rolling OTA channel."""
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
from typing import Any, Dict, Optional, Tuple

_PACKAGE_LINE_RE = re.compile(
    r"package:\s+name='([^']*)'\s+versionCode='([^']*)'\s+versionName='([^']*)'"
)


def parse_aapt2_badging(output: str) -> Tuple[Optional[str], Optional[int], Optional[str]]:
    """Extract (package_name, version_code, version_name) from `aapt2 dump badging` output."""
    for line in output.splitlines():
        match = _PACKAGE_LINE_RE.search(line)
        if match:
            pkg_name = match.group(1) or None
            raw_code = match.group(2)
            ver_code = int(raw_code) if raw_code.isdigit() else None
            ver_name = match.group(3) or None
            return pkg_name, ver_code, ver_name
    return None, None, None


def find_aapt2() -> Optional[str]:
    """Locate an executable `aapt2` binary from PATH or Android SDK build-tools."""
    direct = shutil.which("aapt2")
    if direct:
        return direct
    for env_key in ("ANDROID_HOME", "ANDROID_SDK_ROOT"):
        sdk_root = os.environ.get(env_key, "").strip()
        if not sdk_root:
            continue
        bt_dir = Path(sdk_root) / "build-tools"
        if not bt_dir.is_dir():
            continue
        candidates = sorted(bt_dir.glob("*/aapt2"), reverse=True)
        for candidate in candidates:
            if candidate.is_file() and os.access(candidate, os.X_OK):
                return str(candidate)
    return None


def extract_apk_manifest_identity(apk: Path) -> Tuple[Optional[str], Optional[int], Optional[str]]:
    """Best-effort extraction of APK package name, versionCode, and versionName."""
    aapt2_bin = find_aapt2()
    if not aapt2_bin:
        return None, None, None
    try:
        completed = subprocess.run(
            [aapt2_bin, "dump", "badging", str(apk)],
            capture_output=True,
            text=True,
            timeout=15,
            check=False,
        )
        if completed.returncode != 0:
            return None, None, None
        return parse_aapt2_badging(completed.stdout)
    except OSError:
        return None, None, None
    except subprocess.SubprocessError:
        return None, None, None


def build_metadata_dict(apk: Path) -> Dict[str, Any]:
    digest = hashlib.sha256()
    with apk.open("rb") as stream:
        if stream.read(4) != b"PK\x03\x04":
            raise ValueError("APK is not a ZIP archive")
        stream.seek(0)
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)

    pkg_name, ver_code, ver_name = extract_apk_manifest_identity(apk)
    return {
        "source_commit": os.environ.get("GITHUB_SHA"),
        "source_repository": os.environ.get("GITHUB_REPOSITORY"),
        "workflow_run_id": os.environ.get("GITHUB_RUN_ID"),
        "workflow_run_number": os.environ.get("GITHUB_RUN_NUMBER"),
        "workflow_run_attempt": os.environ.get("GITHUB_RUN_ATTEMPT"),
        "package_name": pkg_name,
        "version_code": ver_code,
        "version_name": ver_name,
        "artifact": apk.name,
        "bytes": apk.stat().st_size,
        "sha256": digest.hexdigest(),
        "metadata_generated_epoch_ms": int(time.time() * 1000),
    }


def main() -> None:
    apk = Path(sys.argv[1])
    print(json.dumps(build_metadata_dict(apk), indent=2))


if __name__ == "__main__":
    main()
