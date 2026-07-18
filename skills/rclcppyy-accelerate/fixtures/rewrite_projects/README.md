# Runnable Rewrite Evidence

These small projects exercise the optimization workflow against real ROS 2
entities. They are examples and test fixtures, not a second runtime API.

| Project | Selected tier | Bounded rewrite |
|---|---:|---|
| `transparent_relay` | 0 | Activates compatible routing without editing the application node. |
| `managed_native_worker` | 2 | Replaces explicit node/executor ownership with managed `rclcpp` objects and opts into intra-process communication. |
| `fused_native_relay` | 3 | Moves one measured transform callback into an editable fused C++ pipeline. |

Run the scanner on a project before choosing a tier:

```bash
pixi run scan-acceleration \
  skills/rclcppyy-accelerate/fixtures/rewrite_projects/transparent_relay \
  --strict --output build/transparent-relay-scan.json
```

Generate paired stock/rewrite smoke evidence for all projects:

```bash
pixi run python \
  skills/rclcppyy-accelerate/fixtures/rewrite_projects/evidence_protocol.py \
  --repetitions 1 --output build/rewrite-fixture-evidence.json
```

Every pair asserts application-visible values and checksums, backend roles,
transform-boundary crossings, and clean teardown. The artifact retains raw elapsed
samples, but always forbids performance claims: these bounded runs lack an isolated
host, interleaved statistical repetitions, CPU isolation, and frequency controls.
Use the repository benchmark methodology before making any performance statement.

Tier 1 is a cache/configuration measurement rather than a source rewrite, so it is
covered by the main cold/warm benchmark workflow instead of this behavior corpus.
Tier 4 needs a workload-specific domain library, adapter, and lifetime proof; this
generic String corpus deliberately makes no Tier 4 recommendation or claim.
