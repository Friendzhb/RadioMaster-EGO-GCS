#!/usr/bin/env python3
"""Record the exact source and artifact identity for the rolling OTA channel."""
import hashlib
import json
import os
from pathlib import Path
import sys


def main():
    apk = Path(sys.argv[1])
    digest = hashlib.sha256()
    with apk.open("rb") as stream:
        if stream.read(4) != b"PK\x03\x04":
            raise ValueError("APK is not a ZIP archive")
        stream.seek(0)
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    print(json.dumps({
        "source_commit": os.environ.get("GITHUB_SHA"),
        "source_repository": os.environ.get("GITHUB_REPOSITORY"),
        "workflow_run_id": os.environ.get("GITHUB_RUN_ID"),
        "workflow_run_attempt": os.environ.get("GITHUB_RUN_ATTEMPT"),
        "artifact": apk.name,
        "bytes": apk.stat().st_size,
        "sha256": digest.hexdigest(),
    }, indent=2))


if __name__ == "__main__":
    main()
