"""
loop_detector -- temporally-consistent visual loop-closure detection on top of
dbow_kit, shared by the M3 demo node and the golden test.

The place-recognition core is a DBoW2 ``OrbDatabase`` query (dbow_kit). A single
high-scoring BoW match is not enough -- textured scenes throw up transient false
positives -- so, following DLoopDetector (Galvez-Lopez & Tardos), we add a
**temporal-consistency gate**: a loop is only *confirmed* once the best candidate
match has persisted, moving coherently, for ``consistency_k`` consecutive query
frames. We also ignore the ``ignore_recent`` most-recent database entries (a frame
always matches itself and its immediate neighbours) and require the raw BoW score
to clear ``min_score``.

Usage::

    det = LoopDetector(vocabulary, min_score=0.3, consistency_k=3, ignore_recent=25)
    for idx, desc_mat in enumerate(orb_descriptors_per_frame):
        loop = det.add_and_query(desc_mat)   # -> LoopClosure or None
        if loop:
            print(loop.query_id, "revisits", loop.match_id, loop.score)
"""
from collections import namedtuple

from rclcppyy.kits import dbow_kit

# A confirmed loop closure: the current frame (query_id) revisits an earlier frame
# (match_id) with BoW similarity `score`; `candidates` is the raw top-k list.
LoopClosure = namedtuple("LoopClosure", "query_id match_id score candidates")


class LoopDetector:
    def __init__(self, vocabulary, min_score=0.3, consistency_k=3,
                 ignore_recent=25, island=8, max_results=5):
        """``min_score``: minimum BoW score for a candidate. ``consistency_k``:
        consecutive frames a coherent candidate must persist before confirming.
        ``ignore_recent``: skip matches to the last N entries (self + neighbours).
        ``island``: how far (in frame index) a candidate may move frame-to-frame
        and still count as the *same* place (the DLoopDetector "island")."""
        self.db = dbow_kit.make_database(vocabulary)
        self.min_score = min_score
        self.consistency_k = consistency_k
        self.ignore_recent = ignore_recent
        self.island = island
        self.max_results = max_results
        self.n = 0                 # next entry id / number of frames added
        self._streak = 0           # consecutive coherent candidate count
        self._last_match = None    # match id of the previous frame's candidate

    def add_and_query(self, desc_mat):
        """Add this frame's descriptors to the DB, query for a revisit, update the
        temporal-consistency streak, and return a :class:`LoopClosure` the moment a
        candidate is confirmed (else ``None``)."""
        query_id = self.n
        # max_id is inclusive in DBoW2; restrict to entries older than the ignore
        # window so a frame never matches itself or its immediate neighbours.
        max_id = query_id - self.ignore_recent - 1
        candidates = []
        if max_id >= 0:
            candidates = dbow_kit.query(self.db, desc_mat,
                                        max_results=self.max_results, max_id=max_id)
        dbow_kit.add_image(self.db, desc_mat)
        self.n += 1

        best = candidates[0] if candidates else None
        if best is None or best[1] < self.min_score:
            self._streak = 0
            self._last_match = None
            return None

        match_id, score = best
        # Coherent with the previous frame's candidate (same "island")?
        if self._last_match is not None and abs(match_id - self._last_match) <= self.island:
            self._streak += 1
        else:
            self._streak = 1
        self._last_match = match_id

        if self._streak >= self.consistency_k:
            return LoopClosure(query_id, match_id, score, candidates)
        return None
