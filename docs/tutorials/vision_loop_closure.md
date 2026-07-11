# Tutorial: a visual loop-closure front-end in short Python over C++ (via cppyy)

This tutorial rebuilds the **place-recognition / loop-closure front-end** of a
visual SLAM system — the part that recognizes *"I have been here before"* — as a
single short Python ROS 2 node, driving the **real C++ libraries** (OpenCV, DBoW2,
GTSAM) through [cppyy](https://cppyy.readthedocs.io), with everything visualized
live in [Rerun](https://rerun.io). No C++ to write, no Python bindings to install;
the image data never leaves C++ between the ROS subscription and the place-recognition
query.

**What we build, and why.** The recipe is straight out of ORB-SLAM:

1. **ORB features** on every frame (Mur-Artal & Tardós, *ORB-SLAM: a Versatile and
   Accurate Monocular SLAM System*, IEEE T-RO 2015).
2. **DBoW2** bag-of-binary-words place recognition — quantize each image's binary
   descriptors into a vocabulary of "visual words" and compare images by their word
   histograms (Gálvez-López & Tardós, *Bags of Binary Words for Fast Place Recognition
   in Image Sequences*, IEEE T-RO 2012).
3. A **temporal-consistency gate** so a loop is only confirmed once a candidate
   persists over several frames (the DLoopDetector idea).
4. **(stretch)** a **GTSAM pose graph** that uses the confirmed loop to correct a
   drifting trajectory.

None of OpenCV-C++, DBoW2, or GTSAM's C++ API has a drop-in Python story for this
pipeline (OpenCV has `cv2`, but it copies at every hop; DBoW2 has *no* binding and
*no* conda package; GTSAM's C++ is boost-heavy). cppyy lets us drive all three from
Python while keeping the frame in one C++ address space.

You need only this repo and [pixi](https://pixi.sh). Every step is a `pixi run`.

---

## 0. Setup (once)

```bash
pixi install -e vision          # OpenCV 4 (C++ + cv2), rerun-sdk, gtsam, cppyy, ROS 2
pixi run -e vision build-dbow2  # clone + patch + compile DBoW2 -> build/vendor/libDBoW2.so
```

`build-dbow2` prints the two patches it applies and finishes with
`OK -> .../libDBoW2.so`. See [the DBoW2-from-source section](#dbow2-from-source) for
what those patches are and why.

> **Rerun viewer.** Every demo is **headless by default** and writes a `.rrd`
> recording under `build/vision/` that you open later with `rerun <file>`. To pop
> the live viewer instead, prefix any demo with `RCLCPPYY_RERUN_SPAWN=1`.

> **Data.** Everything runs on a **deterministic synthetic loop sequence with zero
> download** by default. The real-data path (recommended once you've seen it work)
> downloads a TUM RGB-D sequence and the real ORB vocabulary — see
> [Real data](#real-data).

---

## M1 — The spine: a zero-copy image path

```bash
pixi run -e vision demo-vision-spine
```

Expected output (abridged):

```
  frame 25 ingest=0.029 ms
SUMMARY frames=200 ingest_avg_ms=... ingest_p50_ms=0.02 ingest_max_ms=...
Rerun recording saved: build/vision/spine.rrd  (open with: rerun build/vision/spine.rrd)
```

One process runs two ROS 2 nodes: a publisher emits the sequence as
`sensor_msgs/Image`, and a subscriber — subscribing **via rclcppyy**, so its callback
receives the **C++** `sensor_msgs::msg::Image` — logs each frame to Rerun.

### The zero-copy bridge

The interesting line is `cv_kit.msg_to_mat(msg)`. It wraps the message's pixel buffer
as an OpenCV `cv::Mat` **without copying a single byte**:

```python
mat = cv_kit.msg_to_mat(msg)     # cv::Mat whose .data IS msg.data.data()
```

The `cv::Mat` *aliases* the message's `data` vector — the Mat's data pointer is
**byte-identical** to `msg.data.data()` (the test `test_msg_to_mat_zero_copy_pointer_identity`
asserts exactly this). Compare with the standard rclpy path, where the whole `Image`
message is deserialized into Python and you then `np.frombuffer(msg.data).reshape(...)`
into a fresh array — a copy whose cost grows with every pixel.

> **How much does it matter?** Be honest: at 640×480 the copy is cache-cheap and the
> win is marginal (cppyy's per-call overhead is comparable). The zero-copy pointer
> wrap is **flat** in image size, though, while the rclpy copy scales with pixels — at
> 1920×1080 it is ~**150×** cheaper (`pixi run -e vision bench-vision`). And the real
> payoff is *composition*: the frame stays a `cv::Mat` in C++ all the way through ORB
> and the DBoW2 query, with no Python round-trip. **Lifetime rule:** the Mat aliases
> the message, so use it while the message is alive (within the callback).

---

## M2 — Features: ORB on every frame

```bash
pixi run -e vision demo-vision-features
```

```
ORB backend: cv::ORB (CPU)
  frame 25: 1000 keypoints, desc 1000x32, orb=4.00 ms
SUMMARY frames=200 orb_avg_ms=~3.7 orb_fps=~270 avg_keypoints=1000 backend=CPU
```

Each zero-copy `cv::Mat` is grayed and run through **C++ `cv::ORB`**; keypoints are
logged as a Rerun `Points2D` overlay on the image. The ORB descriptor is an **N×32
`CV_8U`** matrix — N 256-bit binary descriptors — which is exactly what DBoW2 consumes
next.

> **Honest note:** `cv2.ORB` would give similar per-frame numbers. The point of doing
> it in C++ here is not the detector call; it is that the frame never leaves C++
> between the subscription, the Mat, ORB, and the DBoW2 query.

### CUDA sidebar

The demo prints `cv::ORB (CPU)` because the conda-forge OpenCV has **no CUDA build**.
`cv_kit` auto-detects this (`cuda_available()` probes for the `cudafeatures2d` module)
and cleanly uses the CPU path — no error. The CPU/GPU choice is a **single branch
point** in `cv_kit.create_orb`, so a CUDA-enabled OpenCV drops in with **no code
change**: `create_orb` then constructs `cv::cuda::ORB` instead.

A CUDA OpenCV build (matching this env's 4.13.0) is available and validated on this
machine's GPU — see **[docs/vision/CUDA_OPENCV.md](../vision/CUDA_OPENCV.md)** for the
`pixi run -e cudabuild provision-cuda-opencv` steps and the `vision-cuda` env. Measured
there: `cv::cuda::ORB` ~576 fps vs ~110 fps CPU — **~5.3× faster** through the same
cppyy path (thanks to soname shadowing, `cv_kit` needs no change to use it).

---

## M3 — Place recognition and loop closure (the heart)

```bash
pixi run -e vision demo-vision-loop
```

```
Training vocabulary on the sequence (offline pass) ...
  ...trained vocab: 9970 words, k=10 L=4
Streaming synthetic frames for loop detection ...
  LOOP  frame 181 revisits frame 1  score=0.462
  LOOP  frame 182 revisits frame 2  score=0.465
  ...
  LOOP  frame 199 revisits frame 19  score=0.474
SUMMARY frames=200 confirmed_loops=19
  synthetic loop segment (ground truth): frames [180,200) revisit [0,20)
```

The full front-end, one node. A DBoW2 **vocabulary** (trained on this sequence by
default; the real ORBvoc for real data — see below) backs an `OrbDatabase`. For each
frame we: wrap zero-copy → ORB → **add to the database** → **query** for the most
similar earlier image → run it through the **temporal-consistency gate**.

The synthetic sequence is a sliding window over a fixed textured canvas that travels a
closed circuit whose **last 20 frames retrace the first 20** — a loop closure by
construction. The detector confirms frame `180+j` revisiting frame `j`, exactly as
designed. Confirmed loops are logged to Rerun (the query frame, the matched frame, and
the score).

### The temporal-consistency gate

A single high-scoring BoW match is not trustworthy — textured scenes throw up
transient false positives. Following DLoopDetector, `loop_detector.LoopDetector`
confirms a loop only once the best candidate has **persisted, moving coherently, over
`k` consecutive frames**, ignores the most-recent database entries (a frame always
matches its neighbours), and requires a minimum BoW score. This is why the first
`k−1` frames of a revisit are not reported — a deliberate confirmation delay.

### The golden test

```bash
pixi run -e vision test-vision
```

`test_vision_loop.py` runs this whole pipeline on the deterministic synthetic sequence
and asserts the detected loop-pair set against a recorded baseline (**precision 1.0**;
the vocabulary is `srand`-seeded so training is reproducible). This is the regression
contract for the entire front-end — no download needed.

<a name="dbow2-from-source"></a>
### DBoW2 from source

DBoW2 is **not on conda-forge and has no Python binding**, so `build-dbow2` vendors it:
it clones `dorian3d/DBoW2` into `build/vendor/` (gitignored) and direct-compiles it
with the env's C++ compiler — the same recipe as `scripts/freeze/build_l2_node.py`,
sidestepping DBoW2's CMake (which pulls a DLib dependency the ORB path never needs).
Two small, documented, idempotent patches (kept as a scripted in-place edit, never a
fork):

1. **Compile only the DLib-free ORB sources** (skip `FBrief`/`FSurf64`, which need
   DVision/opencv-contrib); include the specific headers rather than the umbrella
   `DBoW2.h` that would drag them in.
2. **Add an ORB-SLAM2-style `loadFromTextFile` plus a raw binary cache** to
   `TemplatedVocabulary.h`, so we can read the canonical `ORBvoc.txt` (which stock
   DBoW2 can't) and cache it as a fast-loading binary.

`dbow_kit` then mirrors DBoW2's own API (`train_vocabulary`, `make_database`,
`add_image`, `query`), keeping only the fiddly N×32-Mat → `vector<cv::Mat>` descriptor
split in C++.

---

## M4 (stretch) — Correcting the trajectory with a pose graph

```bash
pixi run -e vision demo-vision-posegraph
```

```
confirmed loops: 19 (e.g. [(181, 1), (182, 2), (183, 3)])
mean position error vs ground truth:
  open-loop odometry : 2.188 m
  after pose-graph   : 0.143 m
```

Detecting a loop is only half the story; the payoff is *correcting the map*. This demo
builds a 2D pose graph over the synthetic circuit: the odometry is the true circuit
corrupted by an accumulating heading drift (so the open-loop trajectory spirals away),
and each confirmed loop closure adds a `BetweenFactor` tying the revisiting pose back
to the earlier one. **GTSAM's Levenberg-Marquardt** optimizer then pulls the drifted
trajectory back onto itself — ~**15× less error**. Before/after trajectories and the
loop edges are logged to Rerun as `LineStrips3D`.

> **Why gtsam's Python binding and not cppyy here?** GTSAM 4.2's C++ headers
> `#include <boost/optional.hpp>`, which is not in this env, so the (very header-heavy,
> boost-coupled) GTSAM does **not** JIT under cppyy. That is fine: pose-graph
> optimization is a one-shot *batch* step, not a hot per-frame loop, so the "keep it in
> C++" argument does not apply and a cppyy wrapper would not earn its keep. The rest of
> the pipeline (the per-frame ORB → DBoW2 path) is where staying in C++ matters, and
> that is all cppyy.

---

## M5 — The numbers

```bash
pixi run -e vision bench-vision            # add --orbvoc for the real-vocab timing
```

| Metric | Result (synthetic, CPU; shared machine — directional) |
|---|---|
| Ingest 640×480 | rclcppyy ~0.008 ms vs rclpy-copy ~0.010 ms (~1.3×) |
| Ingest 1920×1080 | rclcppyy ~0.001 ms vs rclpy-copy ~0.167 ms (**~155×**) |
| ORB (CPU) | ~270 fps (~3.7 ms/frame, 1000 keypoints) |
| Small-vocab train | ~7 s (9970 words, k=10 L=4) |
| Query latency | ~2.8 ms/frame |
| Loop precision / recall | **1.00 / 0.95** (recall < 1 only from the k-frame confirmation delay) |
| Real ORBvoc load | text parse ~2.3 s → binary cache reload ~0.37 s (~6×) |

---

<a name="real-data"></a>
## Real data: TUM RGB-D + the real ORBvoc

The synthetic sequence proves the pipeline with zero download; the real thing runs on
a genuine SLAM dataset.

```bash
pixi run -e vision dataset-tum         # freiburg3_long_office_household (~1.48 GB) -> data/
pixi run -e vision dataset-orbvoc      # the real ORBvoc.txt (~145 MB) -> data/
pixi run -e vision demo-vision-loop -- \
    --tum data/rgbd_dataset_freiburg3_long_office_household \
    --vocab data/ORBvoc.txt --ignore-recent 300 --min-score 0.045 --consistency 4
```

`freiburg3_long_office_household` is the canonical loop-closure sequence: the handheld
camera circles an office and returns to the start. With the real ORBvoc (971,814
words, k=10 L=6) the front-end detects the **genuine** revisit — around frame 2207
returning to the ~frame-78 start region — with BoW scores ~0.05–0.10 (the normal range
for a large vocabulary on real imagery). The first ORBvoc load parses the 145 MB text
(~2–3 s) and caches a binary next to it; subsequent loads take ~0.3 s.

> Dataset: Sturm et al., *A Benchmark for the Evaluation of RGB-D SLAM Systems*, IROS
> 2012 (TUM CVG), CC BY 4.0. Vocabulary: Gálvez-López & Tardós (via ORB-SLAM2).
>
> Real data needs its parameters tuned (a much larger `--ignore-recent` so "the same
> place a moment ago" is not mistaken for a loop; a low `--min-score` because BoW
> scores on real imagery are small). Robust real-world detection additionally wants
> score normalization and geometric verification — see [Gaps](#gaps).

---

## Graduation path (L0 → L1 → L2)

Everything above runs at **L0**: cppyy JIT-compiles the libraries' headers at bringup
(a one-time ~0.2 s for OpenCV; DBoW2's headers are tiny). When startup latency matters,
the rclcppyy **freeze** machinery applies unchanged:

- **L1 (freeze):** bake the kit's headers into a Cling precompiled header so bringup
  skips the header parse. See **[docs/kits/FREEZE.md](../kits/FREEZE.md)**.
- **L2 (lowering):** if a per-frame Python hop ever dominates, emit that step as native
  C++ (the pattern in `scripts/freeze/build_l2_node.py`, which `build_dbow2.py` already
  mirrors). Here the hot path (ORB, the DBoW2 query) is *already* all C++ — Python only
  orchestrates — so there is little to lower.

<a name="gaps"></a>
## Gaps and what's next

- **Robust real-world detection.** The temporal gate uses a raw BoW-score threshold;
  DLoopDetector normalizes by the expected (previous-frame) score, and a real system
  adds **geometric verification** (RANSAC on matched keypoints) before trusting a loop.
- **Relative pose from matches.** M4's loop factors use the ground-truth relative pose;
  a real system estimates it from the matched features (PnP / essential matrix).
- **Track B (loaned messages / zero-copy SHM transport)** is out of scope: the
  zero-copy here is subscription-callback → `cv::Mat`; a loaned-message intra-process
  path would also remove the DDS-level copy.
- **The full ORB-SLAM back-end** (local mapping, bundle adjustment, relocalization) is
  out of scope — this tutorial is the loop-closure *front-end*.

---

## Where the code lives

| File | What |
|---|---|
| `rclcppyy/kits/cv_kit.py` | OpenCV bringup, zero-copy `msg_to_mat`, ORB, CUDA auto-detect |
| `rclcppyy/kits/dbow_kit.py` | DBoW2 vocabulary + database (train/load/query) |
| `scripts/vision/build_dbow2.py` | clone + patch + compile DBoW2 |
| `scripts/vision/loop_detector.py` | temporal-consistency loop gate |
| `scripts/vision/demo_spine.py` / `demo_features.py` / `demo_loop.py` / `demo_posegraph.py` | M1–M4 demos |
| `scripts/vision/train_vocab.py` / `bench_vision.py` | offline vocab trainer / M5 bench |
| `scripts/datasets/synthetic_loop.py` / `dataset_publisher.py` / `download_tum_rgbd.py` / `download_orbvoc.py` | data tooling |
| `test/test_vision_kits.py` / `test/test_vision_loop.py` | kit tests + the golden test |
| `docs/vision/REPORT.md` | the spike report (probe matrix, evidence, generic lessons) |
