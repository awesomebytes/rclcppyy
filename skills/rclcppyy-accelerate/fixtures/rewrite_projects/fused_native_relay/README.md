# Fused Native Relay

The stock application performs a small annotated-String transform in a Python
subscription callback. The Tier 3 rewrite replaces that relay with an editable
native `every` pipeline. Its counters must show every input received, processed,
and published, no drops or exceptions, and zero Python crossings in the transform.

A Python sink remains intentionally present as an external application-output
oracle and is reported separately. The rewrite changes node/entity ownership and
is explicit opt-in. The fixture does not generalize its smoke timing into a gain.
