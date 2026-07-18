# Evidence Gates

## Scanner gate

Treat scan confidence as static-syntax confidence, never runtime or performance
confidence. Every recommendation carries its source evidence and blockers. Stop
before editing when the scan is incomplete, explicitly scanner-only, or lacks the
correctness and benchmark commands needed by the gates below.

## Before editing

1. Run existing correctness tests.
2. Capture stock structured output with environment, RMW, QoS, payload, rate,
   duration, commit, architecture, CPU, cache state, and every raw sample.
3. Save application-visible output or checksums for differential comparison.

## After editing

1. Run the same behavior under stock and changed configurations in isolated process
   groups and ROS domains.
2. Assert backend markers independently for publisher, subscriber, callback, and
   pipeline roles. Reject missing/mismatched markers.
3. Exercise destruction, exception, timeout, and normal shutdown paths.
4. Repeat the exact benchmark. Compare medians on the same quiet machine and retain
   all runs. Do not use hosted CI timing as a performance claim.

## Required report

- Optimization tier and files changed.
- Contract and ownership changes, including queue policy.
- Backend status and Python boundary-crossing counts.
- Correctness/API coverage and explicit exclusions.
- Before/after metrics with raw artifact paths and environment.
- Negative results and remaining bottlenecks.

## Runnable examples

`fixtures/rewrite_projects/evidence_protocol.py` exercises one bounded rewrite at
Tiers 0, 2, and 3. Use these projects to understand the evidence shape, not as
universal benchmark results. Their artifacts intentionally use smoke mode and
retain a blocker until the same cases are repeated on a controlled benchmark host.
