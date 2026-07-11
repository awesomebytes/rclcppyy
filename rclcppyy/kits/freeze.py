"""
rclcppyy.kits.freeze -- L0->L1 "freeze" for cppyy kits.

A kit's bringup cost is dominated by the one-time Cling JIT-parse of the library
headers (bt_kit: ~0.83 s for ``cppyy.include("behaviortree_cpp/bt_factory.h")``).
Freezing replaces that parse with loading a *prebuilt Cling precompiled header
(PCH)* that already contains the header AST -- a few milliseconds instead of most
of a second.

Mechanism (the same one cppyy uses for its own std PCH): Cling loads the PCH named
by the ``CLING_STANDARD_PCH`` environment variable when the interpreter starts.
We build a PCH that bakes the kit's headers on top of the std ones and point
``CLING_STANDARD_PCH`` at it. Because the interpreter binds its PCH at the first
``import cppyy`` (which ``import rclcppyy`` triggers transitively), the variable
MUST be set *before* any cppyy import. The supported entry point is the launcher::

    RCLCPPYY_FROZEN=1 pixi run -e bt freeze-bt-run scripts/bt_kit_demos/t01_first_tree.py
    # or directly:
    python scripts/freeze/run_frozen.py <script.py> [args...]

which sets the variable and then exec's the target in a fresh process, so the PCH
is active before cppyy loads. See ``docs/kits/FREEZE.md`` for the full recipe and
the artifact lifecycle.

This module is import-safe *without* the rclcppyy package (it imports only stdlib
plus, lazily, ``cppyy_backend`` -- which does not initialise the interpreter), so
``scripts/freeze/run_frozen.py`` can load it by file path to resolve the artifact
location without dragging in cppyy.
"""
import os

# A header static with internal linkage that inline/template code ODR-uses. The
# AST-only PCH carries its declaration but the JIT never emits the definition, and
# the library's own copy is a non-exported local symbol -- so JIT-compiled glue
# (makePorts, getInput<T>, setOutput<T>, ...) fails to resolve it. We emit one
# strong, externally-visible definition under the exact mangled name so every JIT
# module resolves to it. Applied ONLY on the frozen path (in L0 the live-parsed
# header already defines it, and a second definition would clash). Extend this
# per-kit table if a freeze surfaces further unresolved internal-linkage symbols.
_FORCE_SYMBOLS = {
    "bt": r"""#include <typeindex>
// BT::UndefinedAnyType (safe_any.hpp): `static std::type_index = typeid(nullptr)`.
std::type_index __rclcppyy_frozen_UndefinedAnyType
    asm("_ZN2BTL16UndefinedAnyTypeE") = typeid(nullptr);
""",
}


def version_tag():
    """Cling-PCH compatibility tag ``<cppstd>.<backend-version>`` (e.g.
    ``17.6.32.8``), matching cppyy's own std-PCH naming so a stale artifact built
    against a different cppyy-cling is obvious by filename."""
    from cppyy_backend._get_cppflags import get_cppversion
    from cppyy_backend._version import __version__
    return "%s.%s" % (get_cppversion(), __version__)


def _default_build_dir():
    # Resolved from the current working directory (not __file__), so the source
    # and the colcon-installed copy of this module agree on the artifact location.
    # All freeze tasks run from the repo root (pixi sets cwd = manifest dir), so
    # this is <repo>/build/freeze; override with $RCLCPPYY_FREEZE_DIR otherwise.
    return os.path.join(os.getcwd(), "build", "freeze")


def artifact_path(kit="bt", build_dir=None):
    """Path of the frozen PCH artifact for ``kit``. Lives under a gitignored build
    dir (``$RCLCPPYY_FREEZE_DIR`` or ``<cwd>/build/freeze``, i.e. repo-root
    ``build/freeze`` when run via pixi). The name is env-version-tagged; a
    mismatch means "rebuild" (see build script)."""
    build_dir = build_dir or os.environ.get("RCLCPPYY_FREEZE_DIR") or _default_build_dir()
    return os.path.join(build_dir, "%s_kit.pch.native.%s" % (kit, version_tag()))


def active(kit="bt"):
    """True iff the interpreter's active std PCH is a frozen ``<kit>_kit`` PCH.

    Gated on the *filename* (not an exact path) so both the launcher and a
    hand-set ``CLING_STANDARD_PCH`` count, while the default cppyy std PCH does
    not -- this is what tells the kit's bringup to apply the force-symbol glue."""
    base = os.path.basename(os.environ.get("CLING_STANDARD_PCH", ""))
    return base.startswith("%s_kit.pch" % kit)


def force_symbol_glue(kit="bt"):
    """C++ source that must be ``cppyy.cppdef``'d on the frozen path before the
    kit compiles its own glue, or ``None``. See ``_FORCE_SYMBOLS``."""
    return _FORCE_SYMBOLS.get(kit)


def apply_force_symbols(kit="bt"):
    """On the frozen path, emit the kit's force-symbol definitions. No-op if not
    frozen or the kit has none. Idempotent per process."""
    if not active(kit):
        return False
    glue = force_symbol_glue(kit)
    if not glue:
        return False
    flag = "_rclcppyy_frozen_forced_%s" % kit
    if getattr(apply_force_symbols, flag, False):
        return True
    import cppyy
    cppyy.cppdef(glue)
    setattr(apply_force_symbols, flag, True)
    return True
