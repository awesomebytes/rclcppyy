import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
USES_RE = re.compile(r"^\s*uses:\s*([^@\s]+)@([^\s#]+)", re.MULTILINE)
EXPECTED_REVISIONS = {
    "actions/attest": "f7c74d28b9d84cb8768d0b8ca14a4bac6ef463e6",
    "actions/cache/restore": "0057852bfaa89a56745cba8c7296529d2fc39830",
    "actions/cache/save": "0057852bfaa89a56745cba8c7296529d2fc39830",
    "actions/checkout": "93cb6efe18208431cddfb8368fd83d5badbf9bfd",
    "actions/download-artifact": "634f93cb2916e3fdff6788551b99b062d0335ce0",
    "actions/upload-artifact": "ea165f8d65b6e75b540449e92b4886f43607fa02",
    "prefix-dev/setup-pixi": "a09b6247153796b190642a2b53fac4241043cf6f",
}


def test_all_external_workflow_actions_use_reviewed_immutable_revisions():
    observed = set()
    for workflow in sorted((ROOT / ".github" / "workflows").glob("*.yml")):
        for action, revision in USES_RE.findall(workflow.read_text(encoding="utf-8")):
            assert re.fullmatch(r"[0-9a-f]{40}", revision), (
                "%s uses mutable action revision %s@%s" %
                (workflow, action, revision)
            )
            assert action in EXPECTED_REVISIONS, (
                "%s uses unreviewed external action %s" % (workflow, action)
            )
            assert revision == EXPECTED_REVISIONS[action], (
                "%s does not use the reviewed revision for %s" %
                (workflow, action)
            )
            observed.add(action)

    assert observed == set(EXPECTED_REVISIONS)
