import io
import urllib.error

import pytest

from scripts.ci import verify_product_upload as upload


class _Response(io.BytesIO):
    def __init__(self, content, url):
        super().__init__(content)
        self._url = url

    def geturl(self):
        return self._url

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        self.close()


def _artifacts(tmp_path):
    paths = []
    for subdir in sorted(upload.EXPECTED_SUBDIRS):
        path = tmp_path / subdir / "ros-jazzy-rclcppyy-0.3.0-h_fixture_0.conda"
        path.parent.mkdir(parents=True)
        path.write_bytes(("artifact:" + subdir).encode())
        paths.append(path)
    return paths


def _opener(remote):
    def open_url(request, timeout):
        assert timeout == 120
        url = request.full_url
        if url not in remote:
            raise urllib.error.HTTPError(url, 404, "missing", {}, None)
        return _Response(remote[url], url)
    return open_url


def test_preflight_accepts_exact_bytes_and_plans_only_missing_architecture(tmp_path):
    artifacts = _artifacts(tmp_path)
    channel = "https://packages.example.invalid/channel"
    exact = next(path for path in artifacts if path.parent.name == "linux-64")
    exact_url = "%s/%s/%s" % (channel, exact.parent.name, exact.name)

    proof = upload.verify_product_artifacts(
        artifacts,
        channel_url=channel,
        tag="v0.3.0",
        require_present=False,
        opener=_opener({exact_url: exact.read_bytes()}),
    )

    assert proof["all_exact"] is False
    assert {row["subdir"]: row["status"] for row in proof["artifacts"]} == {
        "linux-64": "exact",
        "linux-aarch64": "missing",
    }


def test_postflight_requires_both_exact_published_artifacts(tmp_path):
    artifacts = _artifacts(tmp_path)
    channel = "https://packages.example.invalid/channel"
    remote = {
        "%s/%s/%s" % (channel, path.parent.name, path.name): path.read_bytes()
        for path in artifacts
    }

    proof = upload.verify_product_artifacts(
        artifacts,
        channel_url=channel,
        tag="v0.3.0",
        require_present=True,
        opener=_opener(remote),
    )

    assert proof["all_exact"] is True
    assert {row["status"] for row in proof["artifacts"]} == {"exact"}


def test_upload_gate_rejects_existing_different_bytes_or_incomplete_matrix(tmp_path):
    artifacts = _artifacts(tmp_path)
    channel = "https://packages.example.invalid/channel"
    conflict = artifacts[0]
    conflict_url = "%s/%s/%s" % (channel, conflict.parent.name, conflict.name)
    with pytest.raises(upload.UploadVerificationError, match="different bytes"):
        upload.verify_product_artifacts(
            artifacts,
            channel_url=channel,
            tag="v0.3.0",
            require_present=False,
            opener=_opener({conflict_url: b"different"}),
        )

    with pytest.raises(upload.UploadVerificationError, match="exactly"):
        upload.verify_product_artifacts(
            artifacts[:1],
            channel_url=channel,
            tag="v0.3.0",
            require_present=False,
            opener=_opener({}),
        )
