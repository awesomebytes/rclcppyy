"""
dbow_kit -- drive DBoW2 (Galvez-Lopez & Tardos, "Bags of Binary Words") from
Python via cppyy for ORB place recognition / loop-closure detection.

DBoW2 has no Python binding and is not packaged on conda-forge, so it is vendored
and compiled from source by ``scripts/vision/build_dbow2.py`` into
``build/vendor/libDBoW2.so`` (run ``pixi run -e vision build-dbow2`` once). This
kit brings those headers/lib up under cppyy and mirrors DBoW2's own ORB API:
``OrbVocabulary`` (train / save / load) and ``OrbDatabase`` (add / query), plus the
one fiddly bit of glue -- turning an ``Nx32 CV_8U`` descriptor Mat (as ORB hands
back) into the ``std::vector<cv::Mat>`` of single-row descriptors DBoW2 wants, kept
in C++.

Two vocabulary paths, both supported (see the microplan / tutorial):
  * **train a small vocabulary** on the current sequence's descriptors
    (``train_vocabulary``) -- self-contained, zero-download, used by the golden
    test and for instant tutorial start;
  * **load the real ORBvoc** (``load_vocabulary`` on the 145 MB ORBvoc.txt from the
    ORB-SLAM2 repo). Parsing the text is slow (~tens of seconds); the first load
    transparently writes a ``.dbow2`` **binary cache** next to it that reloads in
    ~1 s thereafter (the standard optimization; enabled by the build_dbow2 patch).

Descriptor layout: ORB descriptors are 256-bit = 32 bytes, so one image's features
are an ``Nx32 CV_8U`` ``cv::Mat`` (from ``cv_kit`` / ``cv::ORB``). DBoW2's
``FORB::TDescriptor`` is a single ``1x32`` ``cv::Mat``; a query/add takes a
``std::vector<cv::Mat>`` (one row per feature). ``descriptors_from_mat`` does that
split in C++.
"""
import os

import cppyy

from rclcppyy.kits import cppyy_kit

# The vendored DBoW2 headers we actually use. We deliberately include these three
# rather than the umbrella DBoW2.h, which pulls FBrief.h (BRIEF -> DVision/DLib);
# the ORB path needs none of that.
_DBOW_HEADERS = ("FORB.h", "TemplatedVocabulary.h", "TemplatedDatabase.h")

# The OrbVocabulary / OrbDatabase typedefs (FORB descriptors) and the Nx32-Mat ->
# vector<1x32 Mat> split, kept in C++ (building the nested std::vector from a
# Python loop is both slow and a cppyy container-construction hazard).
_CPP_GLUE = r"""
namespace rclcppyy_dbow {
  typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB> OrbVocabulary;
  typedef DBoW2::TemplatedDatabase<DBoW2::FORB::TDescriptor, DBoW2::FORB> OrbDatabase;

  // Split an Nx32 CV_8U descriptor Mat (one image's ORB features) into the
  // std::vector<cv::Mat> of 1x32 rows DBoW2 consumes. clone() so each row owns
  // its bytes (independent of the source Mat's lifetime).
  inline std::vector<cv::Mat> descriptors_from_mat(const cv::Mat& m) {
    std::vector<cv::Mat> out; out.reserve(m.rows);
    for (int i = 0; i < m.rows; ++i) out.push_back(m.row(i).clone());
    return out;
  }

  // Append one image's features to a training set (avoids constructing the nested
  // vector<vector<Mat>> from Python).
  inline void append_features(std::vector<std::vector<cv::Mat> >& all, const cv::Mat& m) {
    all.push_back(descriptors_from_mat(m));
  }
}
"""

_NS = None
_DONE = False


def _conda():
    return os.environ["CONDA_PREFIX"]


def _vendor():
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.dirname(os.path.dirname(here))
    return os.path.join(repo, "build", "vendor")


def bringup_dbow():
    """Bring up DBoW2 (ORB path) under cppyy and return the ``rclcppyy_dbow``
    namespace (``OrbVocabulary``, ``OrbDatabase``, ``descriptors_from_mat``).
    Idempotent. Requires ``build/vendor/libDBoW2.so`` -- build it once with
    ``pixi run -e vision build-dbow2``."""
    global _NS, _DONE
    if _DONE:
        return _NS
    # cv_kit brings up OpenCV (headers/libs/glue) that DBoW2 links against.
    from rclcppyy.kits import cv_kit
    cv_kit.bringup_cv()

    vendor = _vendor()
    so = os.path.join(vendor, "libDBoW2.so")
    inc = os.path.join(vendor, "DBoW2", "include")
    if not os.path.isfile(so):
        raise RuntimeError(
            "libDBoW2.so not found at %s. Build it once: "
            "pixi run -e vision build-dbow2" % so)
    cppyy.add_include_path(inc)
    cppyy.add_include_path(os.path.join(inc, "DBoW2"))
    for header in _DBOW_HEADERS:
        cppyy.include(header)
    cppyy.load_library(so)
    cppyy.cppdef(_CPP_GLUE)
    _NS = cppyy.gbl.rclcppyy_dbow
    _DONE = True
    return _NS


def descriptors_from_mat(desc_mat):
    """Turn an ``Nx32 CV_8U`` ORB descriptor Mat into the ``std::vector<cv::Mat>``
    (one 1x32 row per feature) DBoW2 wants."""
    return bringup_dbow().descriptors_from_mat(desc_mat)


def make_vocabulary(k=9, L=3, scoring=None, weighting=None):
    """Construct an empty ``OrbVocabulary`` with branching factor ``k`` and depth
    ``L`` (defaults k=9, L=3 -> up to 9^3 = 729 words, right for a small sequence;
    the real ORBvoc is k=10, L=6 -> ~1M words). Scoring/weighting default to
    DBoW2's L1_NORM / TF_IDF."""
    ns = bringup_dbow()
    if scoring is None or weighting is None:
        return ns.OrbVocabulary(k, L)
    return ns.OrbVocabulary(k, L, weighting, scoring)


def train_vocabulary(descriptor_mats, k=9, L=3):
    """Train an ``OrbVocabulary`` on a list of per-image ``Nx32 CV_8U`` descriptor
    Mats. Builds the ``std::vector<std::vector<cv::Mat>>`` training set in C++ and
    calls ``voc.create(...)``. Returns the vocabulary. This is the zero-download
    path (used by the golden test)."""
    ns = bringup_dbow()
    features = cppyy.gbl.std.vector["std::vector<cv::Mat>"]()
    for m in descriptor_mats:
        ns.append_features(features, m)
    voc = ns.OrbVocabulary(k, L)
    with cppyy_kit.first_use("dbow_kit.train_vocabulary", "(one-time; expected)"):
        voc.create(features)
    return voc


def save_vocabulary(voc, path):
    """Save a vocabulary. ``.dbow2`` -> raw binary (fast, via the build_dbow2
    patch); ``.txt`` -> ORB-SLAM2 text is not written (load-only); anything else ->
    DBoW2's native ``cv::FileStorage`` (``.yml`` / ``.yml.gz`` / ``.xml``)."""
    if path.endswith(".dbow2"):
        voc.saveToBinaryFile(path)
    else:
        voc.save(path)


def load_vocabulary(path, use_binary_cache=True):
    """Load an ORB vocabulary, auto-detecting the format:

    * ``.txt`` (ORBvoc.txt): parse the ORB-SLAM2 text format. Slow (~tens of s) --
      so if ``use_binary_cache`` (default) we transparently write/reuse a
      ``<path>.dbow2`` binary next to it: present-and-newer -> load that in ~1 s;
      else parse the text once and write the cache.
    * ``.dbow2``: raw binary (fast).
    * otherwise: DBoW2's ``cv::FileStorage`` load (``.yml`` / ``.yml.gz``).

    Returns the vocabulary.
    """
    ns = bringup_dbow()
    voc = ns.OrbVocabulary()
    if path.endswith(".dbow2"):
        if not voc.loadFromBinaryFile(path):
            raise RuntimeError("dbow_kit: failed to load binary vocabulary %s" % path)
        return voc
    if path.endswith(".txt"):
        cache = path + ".dbow2"
        if (use_binary_cache and os.path.isfile(cache)
                and os.path.getmtime(cache) >= os.path.getmtime(path)):
            if voc.loadFromBinaryFile(cache):
                return voc
        with cppyy_kit.first_use("dbow_kit.load_vocabulary(txt)",
                                 "(one-time ORBvoc.txt parse; cached to .dbow2 next)",
                                 threshold_ms=1000):
            if not voc.loadFromTextFile(path):
                raise RuntimeError("dbow_kit: failed to parse ORBvoc text %s" % path)
        if use_binary_cache:
            try:
                voc.saveToBinaryFile(cache)
            except Exception:
                pass
        return voc
    voc.load(path)
    return voc


def make_database(voc, use_direct_index=False, di_levels=0):
    """Create an ``OrbDatabase`` backed by ``voc``. ``use_direct_index`` enables the
    direct index (feature->node map) DBoW2 uses to speed up geometric
    verification; not needed for plain BoW scoring."""
    return bringup_dbow().OrbDatabase(voc, use_direct_index, di_levels)


def add_image(db, desc_mat):
    """Add one image's ``Nx32`` descriptor Mat to the database. Returns the
    integer entry id (the frame's index in the DB)."""
    return int(db.add(descriptors_from_mat(desc_mat)))


def query(db, desc_mat, max_results=4, max_id=-1):
    """Query the database with one image's descriptor Mat. Returns a list of
    ``(entry_id, score)`` tuples, best first. ``max_id`` (>=0) restricts results to
    entries with id < max_id -- use it to ignore the just-added current frame and
    a temporal window around it. Scores are DBoW2 L1 similarity in [0, 1]."""
    bringup_dbow()
    results = cppyy.gbl.DBoW2.QueryResults()
    feats = descriptors_from_mat(desc_mat)
    db.query(feats, results, int(max_results), int(max_id))
    return [(int(results[i].Id), float(results[i].Score)) for i in range(int(results.size()))]


def warmup():
    """Front-load dbow_kit's first-use JIT (template instantiation of vocab create
    / db query call wrappers) by training a tiny throwaway vocabulary. Call once
    during init so the first live query is steady-state."""
    import numpy as np
    from rclcppyy.kits import cv_kit
    bringup_dbow()

    def _exercise():
        rng = np.random.default_rng(0)
        det = cv_kit.create_orb(150)
        mats = []
        for _ in range(4):
            img = rng.integers(0, 256, (120, 160), dtype=np.uint8)
            _, desc = det.detect_and_compute(cv_kit.numpy_to_mat(img))
            mats.append(desc)
        voc = train_vocabulary(mats, k=3, L=2)
        db = make_database(voc)
        for desc in mats:
            add_image(db, desc)
        query(db, mats[0], max_results=2)

    cppyy_kit.warmup(_exercise)
