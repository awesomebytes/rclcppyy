#!/usr/bin/env python3
"""Provision & validate CUDA-enabled OpenCV (cv::cuda::ORB / cudafeatures2d) for
the vision tutorial's GPU path.

Job-1 finding (full evidence in docs/vision/CUDA_OPENCV.md):

  * conda-forge ships NO CUDA OpenCV. Its opencv-feedstock build.sh sets
    -DWITH_CUDA=0 -DWITH_CUBLAS=0, there is no cuda variant key in
    conda_build_config.yaml, and 0 of 6357 conda-forge `libopencv` builds carry a
    cuda tag (verified via the anaconda.org API). Requests to enable it are
    long-standing and declined (feedstock issues #74, #109).

  * A trustworthy prebuilt DOES exist: the public, Apache-2.0 **Esri** channel's
    `libopencv 4.13.0 cuda129_py313_4` for linux-64 -- the SAME OpenCV version as
    the vision env's conda-forge opencv 4.13.0, built against CUDA 12.9 (which
    supports Blackwell sm_120). It ships the full cuda module set incl.
    libopencv_cudafeatures2d.so (cv::cuda::ORB). Validated end-to-end on an
    RTX PRO 2000 Blackwell (sm_120) via PTX->sm_120 JIT.

Consumption: we extract just the C++ .so's + headers into build/vendor/opencv-cuda/
(the Esri package's gstreamer/ffmpeg/hdf5 deps don't co-solve with current
conda-forge and the ORB path needs none of them). The CUDA 12.9 runtime the libs
link against comes from the pixi `cudabuild` feature (cudart/cublas/cufft/npp).
cv_kit loads these C++ libs directly via cppyy -- no py-opencv, no python coupling.

Subcommands:
  provision          download + verify (sha256) + extract prebuilt into build/vendor/opencv-cuda/
  validate           compile & run a C++ cv::cuda::ORB smoke test + CPU-vs-CUDA fps bench
  validate-cppyy     the same check through cppyy (run from an env with cppyy, e.g. vision)
  build-from-source  FALLBACK: configure+compile OpenCV+contrib 4.13.0 with CUDA ON (sm_120)
                     if you cannot/won't use the prebuilt (documented; ~30-90 min)

Run the packaged tasks with the cudabuild env:
  pixi run -e cudabuild provision-cuda-opencv
  pixi run -e cudabuild validate-cuda-opencv
"""
from __future__ import annotations

import argparse
import hashlib
import os
import shutil
import subprocess
import sys
import tarfile
import tempfile
import urllib.request
import zipfile
from pathlib import Path

# ---- The prebuilt we consume (pin exactly; see module docstring) -------------
ESRI_VERSION = "4.13.0"
ESRI_BUILD = "cuda129_py313_4"
ESRI_FILE = f"libopencv-{ESRI_VERSION}-{ESRI_BUILD}.conda"
# Canonical conda channel URL + anaconda.org API mirror (either works).
ESRI_URLS = [
    f"https://conda.anaconda.org/Esri/linux-64/{ESRI_FILE}",
    f"https://api.anaconda.org/download/Esri/libopencv/{ESRI_VERSION}/linux-64/{ESRI_FILE}",
]
ESRI_SHA256 = "1a9a3286db27f75bc4d01e505cb8b39417f61b32121992c91466bfa97262b278"

# CUDA/GPU knobs. RTX PRO 2000 Blackwell == compute capability 12.0 == sm_120
# (verify with `nvidia-smi --query-gpu=compute_cap --format=csv`). CUDA >= 12.8
# is required to target sm_120 natively; the Esri prebuilt targets sm_50..sm_90
# SASS + compute_50 PTX, and the driver JIT-compiles that PTX to sm_120 at load.
CUDA_ARCH_BIN = "7.5;8.0;8.6;8.9;9.0;12.0"  # used only by build-from-source
OPENCV_CUDA_MODULES = [
    "core", "imgproc", "features2d", "flann",
    "cudev", "cudaarithm", "cudawarping", "cudafilters",
    "cudaimgproc", "cudafeatures2d",
]

REPO_ROOT = Path(__file__).resolve().parents[2]
VENDOR = REPO_ROOT / "build" / "vendor" / "opencv-cuda"
VENDOR_LIB = VENDOR / "lib"
VENDOR_INCLUDE = VENDOR / "include" / "opencv4"


# ---- helpers ----------------------------------------------------------------
def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _download(dest: Path) -> None:
    last = None
    for url in ESRI_URLS:
        try:
            print(f"  downloading {url}")
            with urllib.request.urlopen(url, timeout=120) as r, open(dest, "wb") as f:
                shutil.copyfileobj(r, f)
            return
        except Exception as e:  # noqa: BLE001 - try the next mirror
            last = e
            print(f"    failed: {e}")
    raise SystemExit(f"ERROR: could not download {ESRI_FILE}: {last}")


def _extract_conda(conda_path: Path, workdir: Path) -> Path:
    """A .conda is a zip holding pkg-*.tar.zst; extract the pkg payload to disk."""
    with zipfile.ZipFile(conda_path) as z:
        pkg = next(n for n in z.namelist() if n.startswith("pkg-") and n.endswith(".tar.zst"))
        z.extract(pkg, workdir)
    zst = workdir / pkg
    try:
        import zstandard  # noqa: PLC0415
        with open(zst, "rb") as fh:
            dctx = zstandard.ZstdDecompressor()
            with dctx.stream_reader(fh) as reader, tarfile.open(fileobj=reader, mode="r|") as tar:
                tar.extractall(workdir / "pkg", filter="data")
    except ImportError:
        # Fall back to the zstd CLI if the python module is unavailable.
        tar_path = workdir / "pkg.tar"
        subprocess.run(["zstd", "-d", "-q", str(zst), "-o", str(tar_path)], check=True)
        with tarfile.open(tar_path) as tar:
            tar.extractall(workdir / "pkg", filter="data")
    return workdir / "pkg"


def _cuda_runtime_dir() -> Path:
    prefix = os.environ.get("CONDA_PREFIX")
    if prefix:
        return Path(prefix) / "lib"
    raise SystemExit(
        "ERROR: no CONDA_PREFIX in the environment. Run inside the cudabuild env, e.g.\n"
        "  pixi run -e cudabuild validate-cuda-opencv"
    )


# ---- subcommands ------------------------------------------------------------
def provision(force: bool = False) -> None:
    if VENDOR_LIB.joinpath("libopencv_cudafeatures2d.so").exists() and not force:
        print(f"already provisioned at {VENDOR} (use --force to re-extract)")
        return
    VENDOR.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td)
        conda_path = tmp / ESRI_FILE
        _download(conda_path)
        got = _sha256(conda_path)
        if got != ESRI_SHA256:
            raise SystemExit(f"ERROR: sha256 mismatch\n  expected {ESRI_SHA256}\n  got      {got}")
        print(f"  sha256 OK ({got})")
        pkg = _extract_conda(conda_path, tmp)
        if VENDOR_LIB.exists():
            shutil.rmtree(VENDOR_LIB)
        if VENDOR_INCLUDE.parent.exists():
            shutil.rmtree(VENDOR_INCLUDE.parent)
        shutil.copytree(pkg / "lib", VENDOR_LIB, symlinks=True)
        shutil.copytree(pkg / "include" / "opencv4", VENDOR_INCLUDE, symlinks=True)
    n = len(list(VENDOR_LIB.glob("libopencv_cuda*.so*")))
    print(f"provisioned CUDA OpenCV {ESRI_VERSION} into {VENDOR}")
    print(f"  {n} cuda module libs; headers under {VENDOR_INCLUDE}")


_SMOKE_CPP = r"""
#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <chrono>
#include <cstdio>
#include <vector>
using clk = std::chrono::steady_clock;
static double msd(clk::time_point a, clk::time_point b){
  return std::chrono::duration<double,std::milli>(b-a).count(); }
int main(){
  printf("OpenCV %s\n", CV_VERSION);
  int n = cv::cuda::getCudaEnabledDeviceCount();
  printf("getCudaEnabledDeviceCount() = %d\n", n);
  if(n<1){ printf("FAIL: no CUDA device\n"); return 2; }
  cv::cuda::DeviceInfo di(0);
  printf("device 0: %s  compute %d.%d\n", di.name(), di.majorVersion(), di.minorVersion());
  const int W=640,H=480,N=2000;
  cv::Mat img(H,W,CV_8UC1); cv::RNG rng(12345);
  rng.fill(img, cv::RNG::UNIFORM, 0, 256);
  for(int y=0;y<H;y+=16) cv::line(img,{0,y},{W,y},cv::Scalar(255),1);
  for(int x=0;x<W;x+=16) cv::line(img,{x,0},{x,H},cv::Scalar(0),1);
  auto cpu = cv::ORB::create(N);
  std::vector<cv::KeyPoint> kc; cv::Mat dc;
  cpu->detectAndCompute(img, cv::noArray(), kc, dc);
  const int IT=50; auto t0=clk::now();
  for(int i=0;i<IT;i++){ kc.clear(); cpu->detectAndCompute(img, cv::noArray(), kc, dc); }
  double cpu_ms = msd(t0, clk::now())/IT;
  printf("CPU  cv::ORB      : %zu kp, desc %dx%d type=%d, %.2f ms, %.1f fps\n",
         kc.size(), dc.rows, dc.cols, dc.type(), cpu_ms, 1000.0/cpu_ms);
  auto gpu = cv::cuda::ORB::create(N);
  cv::cuda::Stream st; cv::cuda::GpuMat dimg, dkp, ddesc; cv::Mat dg;
  auto w0=clk::now();
  dimg.upload(img);
  gpu->detectAndComputeAsync(dimg, cv::cuda::GpuMat(), dkp, ddesc, false, st);
  st.waitForCompletion();
  printf("CUDA warmup (PTX JIT if uncached): %.1f ms\n", msd(w0, clk::now()));
  ddesc.download(dg);
  printf("CUDA cv::cuda::ORB: desc %dx%d type=%d (CV_8U=%d)\n", dg.rows, dg.cols, dg.type(), CV_8U);
  if(dg.cols!=32 || dg.type()!=CV_8U || dg.rows<1){ printf("FAIL: not Nx32 CV_8U\n"); return 3; }
  auto g0=clk::now();
  for(int i=0;i<IT;i++){
    dimg.upload(img, st);
    gpu->detectAndComputeAsync(dimg, cv::cuda::GpuMat(), dkp, ddesc, false, st);
    ddesc.download(dg, st); st.waitForCompletion();
  }
  double gpu_ms = msd(g0, clk::now())/IT;
  printf("CUDA cv::cuda::ORB: %.2f ms, %.1f fps (incl up/download)\n", gpu_ms, 1000.0/gpu_ms);
  printf("SPEEDUP = %.2fx\nPASS\n", cpu_ms/gpu_ms);
  return 0;
}
"""


def validate() -> None:
    if not VENDOR_LIB.joinpath("libopencv_cudafeatures2d.so").exists():
        raise SystemExit("ERROR: not provisioned. Run: pixi run -e cudabuild provision-cuda-opencv")
    rt = _cuda_runtime_dir()
    cxx = os.environ.get("CXX") or shutil.which("x86_64-conda-linux-gnu-g++") or shutil.which("g++")
    if not cxx:
        raise SystemExit("ERROR: no C++ compiler (CXX) found; the cudabuild env provides gxx.")
    with tempfile.TemporaryDirectory() as td:
        src = Path(td) / "smoke.cpp"
        src.write_text(_SMOKE_CPP)
        exe = Path(td) / "smoke"
        libs = [f"-lopencv_{m}" for m in OPENCV_CUDA_MODULES]
        cmd = [cxx, "-O2", "-std=c++17", str(src), "-o", str(exe),
               f"-I{VENDOR_INCLUDE}", f"-L{VENDOR_LIB}", *libs,
               f"-Wl,-rpath,{VENDOR_LIB}", f"-Wl,-rpath,{rt}"]
        print("compiling smoke test ...")
        subprocess.run(cmd, check=True)
        env = dict(os.environ, LD_LIBRARY_PATH=f"{VENDOR_LIB}:{rt}:" + os.environ.get("LD_LIBRARY_PATH", ""))
        print("running (first run may JIT PTX->sm_120; cached in ~/.nv afterward) ...")
        raise SystemExit(subprocess.run([str(exe)], env=env).returncode)


def validate_cppyy() -> None:
    """The cv_kit consumption path: cppyy loads the C++ libs + JIT-parses the CUDA
    headers. Run from an env that has cppyy (e.g. the vision env)."""
    import time
    try:
        import cppyy
    except ImportError:
        raise SystemExit("ERROR: cppyy not importable. Run from the vision env, e.g.\n"
                         "  pixi run -e vision python scripts/vision/build_opencv_cuda.py validate-cppyy")
    if not VENDOR_LIB.joinpath("libopencv_cudafeatures2d.so").exists():
        raise SystemExit("ERROR: not provisioned. Run: pixi run -e cudabuild provision-cuda-opencv")
    rt = os.environ.get("CUDA_RT_LIB") or str(_cuda_runtime_dir())
    cppyy.add_include_path(str(VENDOR_INCLUDE))
    cppyy.add_library_path(str(VENDOR_LIB))
    cppyy.add_library_path(rt)
    for m in OPENCV_CUDA_MODULES:
        assert cppyy.load_library(f"opencv_{m}"), f"failed to load opencv_{m}"
    for h in ("opencv2/core.hpp", "opencv2/imgproc.hpp",
              "opencv2/core/cuda.hpp", "opencv2/cudafeatures2d.hpp"):
        cppyy.include(h)
    cv = cppyy.gbl.cv
    n = cv.cuda.getCudaEnabledDeviceCount()
    print(f"[cppyy] getCudaEnabledDeviceCount() = {n}")
    assert n >= 1
    di = cv.cuda.DeviceInfo(0)
    print(f"[cppyy] device: {di.name()}  compute {di.majorVersion()}.{di.minorVersion()}")
    img = cv.Mat(480, 640, 0)  # CV_8UC1 == 0 (macro not exposed via cppyy)
    cv.randu(img, cv.Scalar(0.0), cv.Scalar(255.0))
    d_img, d_kp, d_desc = cv.cuda.GpuMat(), cv.cuda.GpuMat(), cv.cuda.GpuMat()
    d_img.upload(img)
    orb = cv.cuda.ORB.create(2000)
    st = cv.cuda.Stream()
    orb.detectAndComputeAsync(d_img, cv.cuda.GpuMat(), d_kp, d_desc, False, st)
    st.waitForCompletion()
    desc = cv.Mat()
    d_desc.download(desc)
    print(f"[cppyy] descriptors {desc.rows}x{desc.cols} type={desc.type()} (CV_8U=0)")
    assert desc.cols == 32 and desc.type() == 0 and desc.rows >= 1
    it = 50
    t0 = time.perf_counter()
    for _ in range(it):
        d_img.upload(img, st)
        orb.detectAndComputeAsync(d_img, cv.cuda.GpuMat(), d_kp, d_desc, False, st)
        d_desc.download(desc, st)
        st.waitForCompletion()
    gpu_ms = (time.perf_counter() - t0) / it * 1e3
    print(f"[cppyy] cv::cuda::ORB {gpu_ms:.2f} ms/frame, {1000 / gpu_ms:.1f} fps")
    print("[cppyy] PASS")


def build_from_source() -> None:
    """FALLBACK compile path. Only needed if the prebuilt is unavailable or you
    want native sm_120 SASS (no PTX-JIT warmup). Expects the cudabuild env plus
    cmake/ninja and a CUDA toolchain (cuda-nvcc, cuda-cudart-dev, libcublas-dev,
    libnpp-dev, libcufft-dev) matching CUDA 12.9. ~30-90 min on 16 cores."""
    import multiprocessing
    prefix = os.environ.get("CONDA_PREFIX")
    if not prefix:
        raise SystemExit("ERROR: run inside a conda/pixi env with the CUDA toolchain + cmake/ninja.")
    src = REPO_ROOT / "build" / "vendor" / "opencv-src"
    build = REPO_ROOT / "build" / "vendor" / "opencv-cuda-build"
    install = REPO_ROOT / "build" / "vendor" / "opencv-cuda"
    src.mkdir(parents=True, exist_ok=True)
    ver = ESRI_VERSION

    def fetch(name: str) -> Path:
        d = src / f"{name}-{ver}"
        if d.exists():
            return d
        tgz = src / f"{name}-{ver}.tar.gz"
        url = f"https://github.com/opencv/{name}/archive/refs/tags/{ver}.tar.gz"
        print(f"  fetch {url}")
        with urllib.request.urlopen(url, timeout=120) as r, open(tgz, "wb") as f:
            shutil.copyfileobj(r, f)
        with tarfile.open(tgz) as t:
            t.extractall(src)
        return d

    ocv = fetch("opencv")
    contrib = fetch("opencv_contrib")
    build.mkdir(parents=True, exist_ok=True)
    cmake = [
        "cmake", "-G", "Ninja", "-S", str(ocv), "-B", str(build),
        f"-DCMAKE_INSTALL_PREFIX={install}",
        "-DCMAKE_BUILD_TYPE=Release",
        f"-DOPENCV_EXTRA_MODULES_PATH={contrib / 'modules'}",
        "-DWITH_CUDA=ON", "-DWITH_CUBLAS=ON", "-DWITH_CUDNN=OFF",
        "-DOPENCV_DNN_CUDA=OFF", "-DENABLE_FAST_MATH=ON", "-DCUDA_FAST_MATH=ON",
        f"-DCUDA_ARCH_BIN={CUDA_ARCH_BIN}", "-DCUDA_ARCH_PTX=12.0",
        "-DBUILD_LIST=" + ",".join(m for m in OPENCV_CUDA_MODULES if not m.startswith("cudev")),
        "-DBUILD_opencv_python3=OFF", "-DBUILD_opencv_python2=OFF",
        "-DBUILD_TESTS=OFF", "-DBUILD_PERF_TESTS=OFF", "-DBUILD_EXAMPLES=OFF",
        "-DBUILD_DOCS=OFF", "-DWITH_GSTREAMER=OFF", "-DWITH_FFMPEG=OFF",
        f"-DCUDAToolkit_ROOT={prefix}",
    ]
    print("configuring (CUDA ON, sm_120 in ARCH_BIN, PTX 12.0):\n  " + " ".join(cmake))
    subprocess.run(cmake, check=True)
    jobs = str(multiprocessing.cpu_count())
    subprocess.run(["cmake", "--build", str(build), "-j", jobs], check=True)
    subprocess.run(["cmake", "--install", str(build)], check=True)
    print(f"built + installed CUDA OpenCV {ver} into {install}")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    sp = sub.add_parser("provision", help="download+verify+extract the prebuilt")
    sp.add_argument("--force", action="store_true", help="re-extract even if present")
    sub.add_parser("validate", help="compile+run C++ cv::cuda::ORB smoke test + bench")
    sub.add_parser("validate-cppyy", help="same check via cppyy (run in an env with cppyy)")
    sub.add_parser("build-from-source", help="FALLBACK: compile OpenCV+contrib with CUDA ON")
    args = p.parse_args()
    if args.cmd == "provision":
        provision(force=args.force)
    elif args.cmd == "validate":
        validate()
    elif args.cmd == "validate-cppyy":
        validate_cppyy()
    elif args.cmd == "build-from-source":
        build_from_source()


if __name__ == "__main__":
    main()
