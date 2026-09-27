---
name: rclcppyy-accelerate
description: Analyze and optimize ROS 2 Python projects with rclcppyy using transparent compatibility, managed rclcpp, editable native callbacks, fused C++ pipelines, and domain kits. Use when Codex must inspect an existing rclpy codebase, choose a safe optimization tier, expose a C++-only ROS/library capability, reduce Python hot-path crossings, or produce correctness and benchmark evidence for a proposed rewrite.
---

# Accelerate rclcppyy

Preserve behavior first. Select the smallest optimization tier supported by
backend evidence and measurements; never assume that entering C++ is faster.

## Workflow

1. Read repository instructions and establish a clean, reproducible test command.
2. From the repository root or this skill directory, run
   `pixi run scan-acceleration TARGET --strict --output build/rclcppyy-scan.json`.
   Interpret `TARGET` and the output path relative to the repository root; use
   absolute paths for targets outside it.
3. Inspect the scan, relevant source, launch/config files, and current tests. Treat
   scanner recommendations as inputs, not conclusions. Resolve every reported
   blocker before selecting an implementation tier.
4. Read `references/techniques.md` and select the lowest applicable tier:
   - Tier 0: compatible activation and status only.
   - Tier 1: explicit same-handle C++ publishing or semantics-preserving
     cache/configuration improvements.
   - Tier 2: explicit managed `rclcpp` options or native entities.
   - Tier 3: editable native callback or fused pipeline.
   - Tier 4: domain-kit/library-native data path when a callback-level use is
     measured, not merely because a native library is imported.
5. Before editing, capture stock correctness and structured benchmark evidence.
6. Implement one bounded change. Keep compatible mode contract-preserving; put
   scheduling, ownership, buffering, reuse, loaning, and fusion behind explicit
   opt-in configuration.
7. Add differential tests and backend assertions. Read
   `references/hazards.md` before changing handles, callbacks, threads, messages,
   queues, or teardown.
8. Rerun the exact baseline workload with identical ROS/RMW/QoS and compare raw
   results. Follow `references/evidence.md`.
9. Report changed semantics, backend routes, Python boundary crossings, cache
   state, negative results, and remaining stock paths.

## Hard Rules

- Keep the exact stock Node, Context, executor, messages, entities, and
  `Publisher.publish` in the compatible profile unless a reviewed promotion
  explicitly says otherwise.
- Fail before side effects when required-C++ cannot satisfy an operation.
- Do not claim an entity is C++ because only setup or serialization uses C++.
- Do not publish a gain without versioned raw data and independent backend markers.
- Do not lower a callback until behavior is covered by a stock-vs-changed test.
- Preserve raw `rclcpp` and editable C++ access; do not invent a replacement ROS API.
- Stop and report a negative result when conversion, JIT, queueing, or boundary
  overhead outweighs the proposed optimization.

## Resources

- `scripts/scan_project.py`: relocation-stable AST inventory and evidence-linked
  recommendation inputs. The Pixi task supplies its interpreter.
- `fixtures/rewrite_projects/`: runnable stock/rewrite examples for compatible
  activation, managed native ownership, and fused native callback tiers. Their
  paired artifacts prove bounded behavior and backend routes while explicitly
  forbidding performance claims from smoke timing.
- `references/techniques.md`: tier and workload decision table.
- `references/patterns.md`: activation, managed-native, callback, and pipeline forms.
- `references/hazards.md`: ABI, lifetime, concurrency, cache, and teardown hazards.
- `references/evidence.md`: correctness, backend, benchmark, and reporting gates.
