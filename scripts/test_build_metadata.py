import hashlib
from pathlib import Path
import tempfile
import unittest
from unittest import mock
import zipfile

import build_metadata


class BuildMetadataTest(unittest.TestCase):
    def test_parse_aapt2_badging_extracts_package_and_versions(self) -> None:
        sample = (
            "package: name='com.radiomaster.egogcs' versionCode='98765432' "
            "versionName='3.13.42' platformBuildVersionName='14'\n"
            "sdkVersion:'26'\n"
        )
        pkg, code, name = build_metadata.parse_aapt2_badging(sample)
        self.assertEqual(pkg, "com.radiomaster.egogcs")
        self.assertEqual(code, 98765432)
        self.assertEqual(name, "3.13.42")

    def test_build_metadata_dict_computes_sha256_and_includes_manifest_fields(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            apk_path = Path(tmp_dir) / "EGO-Pilot-AX12.apk"
            with zipfile.ZipFile(apk_path, "w") as zf:
                zf.writestr("AndroidManifest.xml", "stub")
            expected_sha = hashlib.sha256(apk_path.read_bytes()).hexdigest()

            with mock.patch.object(
                build_metadata,
                "extract_apk_manifest_identity",
                return_value=("com.radiomaster.egogcs", 98765432, "3.13.42"),
            ), mock.patch.dict(
                "os.environ",
                {
                    "GITHUB_SHA": "abcdef1234567890",
                    "GITHUB_RUN_NUMBER": "42",
                },
                clear=False,
            ):
                meta = build_metadata.build_metadata_dict(apk_path)

            self.assertEqual(meta["sha256"], expected_sha)
            self.assertEqual(meta["bytes"], apk_path.stat().st_size)
            self.assertEqual(meta["version_code"], 98765432)
            self.assertEqual(meta["version_name"], "3.13.42")
            self.assertEqual(meta["source_commit"], "abcdef1234567890")
            self.assertEqual(meta["workflow_run_number"], "42")

    def test_build_metadata_dict_rejects_non_zip_payload(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            bad_path = Path(tmp_dir) / "corrupt.apk"
            bad_path.write_bytes(b"<html>gateway error</html>")
            with self.assertRaises(ValueError):
                build_metadata.build_metadata_dict(bad_path)


if __name__ == "__main__":
    unittest.main()
