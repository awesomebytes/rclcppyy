#!/usr/bin/env python
"""
Fetch, patch and compile DBoW2 (Galvez-Lopez & Tardos, "Bags of Binary Words")
into ``build/vendor/libDBoW2.so`` + headers, for dbow_kit.

DBoW2 is not packaged on conda-forge, so we vendor it: clone the canonical
dorian3d/DBoW2, apply two small **documented** patches, and compile the ORB path
directly with ``$CXX`` against the vision env's OpenCV 4 -- the same direct-compile
recipe as scripts/freeze/build_l2_node.py, avoiding DBoW2's CMake (which pulls an
``ExternalProject``/DLib dependency we don't need for ORB). Everything lands in the
gitignored ``build/vendor/`` dir. Idempotent: clone/patch/compile each skip if
already done (``--force`` recompiles).

    pixi run -e vision build-dbow2

Patches applied (kept as a documented, idempotent in-place edit -- never a fork):
  1. Compile only the **DLib-free** sources. DBoW2's ORB descriptor path (FORB,
     TemplatedVocabulary, TemplatedDatabase, BowVector, FeatureVector,
     QueryResults, ScoringObject) needs only OpenCV. FBrief.cpp (BRIEF) and
     FSurf64.cpp (SURF) pull DVision/opencv-contrib and are skipped, so no DLib
     clone is needed. dbow_kit includes the specific headers (FORB.h,
     TemplatedVocabulary.h, TemplatedDatabase.h) rather than the umbrella DBoW2.h,
     which would drag in FBrief.h.
  2. Add ORB-SLAM2-style ``loadFromTextFile`` + a binary cache
     (``saveToBinaryFile`` / ``loadFromBinaryFile``) to TemplatedVocabulary.h.
     Stock DBoW2 only reads its own cv::FileStorage YAML/gz; the canonical
     ORBvoc.txt (145 MB text, from the ORB-SLAM2 repo) is a different format that
     ORB-SLAM2 added a text loader for. We add the same loader, plus a raw binary
     format so the first (~tens-of-seconds) text parse can be cached to a ~40 MB
     ``.dbow2`` that reloads in ~1 s (dbow_kit does this automatically).
"""
import argparse
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
VENDOR = os.path.join(REPO, "build", "vendor")
SRC = os.path.join(VENDOR, "DBoW2")
OUT = os.path.join(VENDOR, "libDBoW2.so")
DBOW2_URL = "https://github.com/dorian3d/DBoW2"

# The DLib-free ORB-path sources (patch 1). FBrief.cpp / FSurf64.cpp are omitted.
SOURCES = ("BowVector.cpp", "FORB.cpp", "FeatureVector.cpp",
           "QueryResults.cpp", "ScoringObject.cpp")

_PATCH_MARKER = "RCLCPPYY_DBOW_PATCH"

# Patch 2: ORB-SLAM2-style text loader + a raw binary cache, injected as inline
# members of TemplatedVocabulary. loadFromTextFile mirrors ORB-SLAM2's reader
# (line 1: "k L scoring weighting"; each further line: "parent isLeaf <F::L bytes
# as ints> weight"); the binary format stores the same node list so a load rebuilds
# children/words identically but ~30x faster than parsing the text.
_VOC_METHODS = r"""
  // ---- %(marker)s: ORB-SLAM2-style text load + binary cache (dbow_kit) ----
  bool loadFromTextFile(const std::string &filename) {
    std::ifstream f(filename.c_str());
    if(!f.good()) return false;
    m_words.clear(); m_nodes.clear();
    std::string line;
    if(!std::getline(f, line)) return false;
    { std::stringstream ss(line); int n1 = -1, n2 = -1; ss >> m_k >> m_L >> n1 >> n2;
      if(m_k < 0 || m_k > 20 || m_L < 1 || m_L > 10 || n1 < 0 || n1 > 5 || n2 < 0 || n2 > 3)
        return false;
      m_scoring = (ScoringType)n1; m_weighting = (WeightingType)n2; createScoringObject(); }
    m_nodes.resize(1); m_nodes[0].id = 0;
    while(std::getline(f, line)) {
      if(line.empty()) continue;
      std::stringstream ss(line);
      int nid = (int)m_nodes.size();
      m_nodes.resize(m_nodes.size() + 1);
      m_nodes[nid].id = nid;
      int pid = 0; ss >> pid;
      m_nodes[nid].parent = pid;
      m_nodes[pid].children.push_back(nid);
      int isLeaf = 0; ss >> isLeaf;
      std::stringstream ssd;
      for(int i = 0; i < F::L; ++i) { std::string e; ss >> e; ssd << e << " "; }
      F::fromString(m_nodes[nid].descriptor, ssd.str());
      ss >> m_nodes[nid].weight;
      if(isLeaf > 0) { int wid = (int)m_words.size(); m_words.resize(wid + 1);
        m_nodes[nid].word_id = wid; m_words[wid] = &m_nodes[nid]; }
      else { m_nodes[nid].children.reserve(m_k); }
    }
    return true;
  }
  void saveToBinaryFile(const std::string &filename) const {
    std::ofstream f(filename.c_str(), std::ios::binary);
    int hdr[4] = { m_k, m_L, (int)m_scoring, (int)m_weighting };
    f.write((const char*)hdr, sizeof(hdr));
    int nnodes = (int)m_nodes.size(); f.write((const char*)&nnodes, sizeof(int));
    for(int i = 0; i < nnodes; ++i) {
      const Node &nd = m_nodes[i];
      int parent = (int)nd.parent; f.write((const char*)&parent, sizeof(int));
      unsigned char isLeaf = (i != 0 && nd.children.empty()) ? 1 : 0;
      f.write((const char*)&isLeaf, 1);
      unsigned char dlen = (unsigned char)(nd.descriptor.empty() ? 0 : nd.descriptor.cols);
      f.write((const char*)&dlen, 1);
      if(dlen) f.write((const char*)nd.descriptor.template ptr<unsigned char>(), dlen);
      double weight = nd.weight; f.write((const char*)&weight, sizeof(double));
    }
  }
  bool loadFromBinaryFile(const std::string &filename) {
    std::ifstream f(filename.c_str(), std::ios::binary);
    if(!f.good()) return false;
    m_words.clear(); m_nodes.clear();
    int hdr[4]; f.read((char*)hdr, sizeof(hdr)); if(!f.good()) return false;
    m_k = hdr[0]; m_L = hdr[1]; m_scoring = (ScoringType)hdr[2];
    m_weighting = (WeightingType)hdr[3]; createScoringObject();
    int nnodes = 0; f.read((char*)&nnodes, sizeof(int));
    if(!f.good() || nnodes < 1) return false;
    m_nodes.resize(nnodes);
    for(int i = 0; i < nnodes; ++i) {
      Node &nd = m_nodes[i]; nd.id = i;
      int parent = 0; f.read((char*)&parent, sizeof(int)); nd.parent = parent;
      unsigned char isLeaf = 0; f.read((char*)&isLeaf, 1);
      unsigned char dlen = 0; f.read((char*)&dlen, 1);
      if(dlen) { nd.descriptor.create(1, dlen, CV_8U);
        f.read((char*)nd.descriptor.template ptr<unsigned char>(), dlen); }
      double weight = 0; f.read((char*)&weight, sizeof(double)); nd.weight = weight;
      if(i > 0) m_nodes[parent].children.push_back(i);
      if(isLeaf) { int wid = (int)m_words.size(); m_words.resize(wid + 1);
        nd.word_id = wid; m_words[wid] = &m_nodes[i]; }
    }
    return true;
  }
  // ---- end %(marker)s ----
""" % {"marker": _PATCH_MARKER}


def _run(cmd, **kw):
    print("  $ " + " ".join(cmd))
    return subprocess.call(cmd, **kw)


def clone():
    if os.path.isdir(SRC):
        print("DBoW2 already cloned at %s (skip)" % SRC)
        return
    os.makedirs(VENDOR, exist_ok=True)
    print("Cloning DBoW2 from %s ..." % DBOW2_URL)
    if _run(["git", "clone", "--depth", "1", DBOW2_URL, SRC]) != 0:
        sys.exit("ERROR: git clone failed")


def patch():
    """Patch 2: inject the text/binary loaders into TemplatedVocabulary.h. Adds
    ``#include <sstream>`` (not present) and the member block after the public
    ``load(const std::string&)`` declaration. Idempotent via the marker."""
    header = os.path.join(SRC, "include", "DBoW2", "TemplatedVocabulary.h")
    with open(header) as fh:
        text = fh.read()
    if _PATCH_MARKER in text:
        print("TemplatedVocabulary.h already patched (skip)")
        return
    if "#include <sstream>" not in text:
        text = text.replace("#include <fstream>",
                            "#include <fstream>\n#include <sstream>", 1)
    anchor = "  void load(const std::string &filename);\n"
    if anchor not in text:
        sys.exit("ERROR: could not find injection anchor in TemplatedVocabulary.h "
                 "(DBoW2 upstream changed; update build_dbow2.py)")
    text = text.replace(anchor, anchor + _VOC_METHODS, 1)
    with open(header, "w") as fh:
        fh.write(text)
    print("Patched TemplatedVocabulary.h: +loadFromTextFile / +binary cache")


def compile_so(force=False):
    if os.path.isfile(OUT) and not force:
        print("libDBoW2.so already built at %s (skip; --force to rebuild)" % OUT)
        return
    conda = os.environ["CONDA_PREFIX"]
    cxx = os.environ.get("CXX") or "c++"
    inc_cv = os.path.join(conda, "include", "opencv4")
    srcs = [os.path.join(SRC, "src", s) for s in SOURCES]
    cmd = [cxx, "-shared", "-fPIC", "-std=c++17", "-O2", "-w",
           "-I", os.path.join(SRC, "include"),
           "-I", os.path.join(SRC, "include", "DBoW2"),
           "-I", inc_cv] + srcs + [
           "-o", OUT,
           "-L", os.path.join(conda, "lib"), "-lopencv_core",
           "-Wl,-rpath," + os.path.join(conda, "lib")]
    print("Compiling libDBoW2.so (ORB path, %d sources) ..." % len(srcs))
    if _run(cmd) != 0:
        sys.exit("ERROR: DBoW2 compile failed")
    print("OK  ->  %s  (%.0f KB)" % (OUT, os.path.getsize(OUT) / 1024))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--force", action="store_true", help="recompile even if present")
    args = ap.parse_args()
    clone()
    patch()
    compile_so(force=args.force)
    print("\nDBoW2 ready. Headers: %s/include ; library: %s" % (SRC, OUT))


if __name__ == "__main__":
    main()
