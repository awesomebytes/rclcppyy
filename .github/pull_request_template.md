## Contract and evidence

- [ ] I identified whether this changes compatible, required-C++, optimized, or
      native behavior.
- [ ] Compatibility claims and exclusions are updated in `compatibility/jazzy.json`.
- [ ] Required-C++ tests reject accidental fallback before side effects.
- [ ] Backend status or benchmark markers prove the route used by each claimed path.
- [ ] Differential, integration, teardown, and installed-package coverage match the
      ownership and behavioral risk.
- [ ] Performance statements link to versioned raw JSON from controlled repeated
      runs; smoke timing is not used as a performance claim.
- [ ] A contract-changing optimization remains opt-in, has a disable path, and has
      explicit ownership, lifetime, RMW, and multi-architecture review.
