"""Small, explicit policy surface for compatibility routing."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class AccelerationPolicy:
    name: str
    require_cpp: bool
    allow_contract_changes: bool
    use_cpp_publisher: bool
    use_cpp_message_facade: bool
    use_direct_cpp: bool
    warn_fallback: bool = False


_PROFILES = {
    "compatible": AccelerationPolicy(
        name="compatible",
        require_cpp=False,
        allow_contract_changes=False,
        use_cpp_publisher=False,
        use_cpp_message_facade=False,
        use_direct_cpp=False,
    ),
    "publisher_cpp": AccelerationPolicy(
        name="publisher_cpp",
        require_cpp=False,
        allow_contract_changes=False,
        use_cpp_publisher=True,
        use_cpp_message_facade=False,
        use_direct_cpp=False,
    ),
    "required_cpp": AccelerationPolicy(
        name="required_cpp",
        require_cpp=True,
        allow_contract_changes=False,
        use_cpp_publisher=True,
        use_cpp_message_facade=False,
        use_direct_cpp=False,
    ),
    "optimized": AccelerationPolicy(
        name="optimized",
        require_cpp=False,
        allow_contract_changes=True,
        use_cpp_publisher=False,
        use_cpp_message_facade=False,
        use_direct_cpp=False,
    ),
    "message_facade": AccelerationPolicy(
        name="message_facade",
        require_cpp=False,
        allow_contract_changes=False,
        use_cpp_publisher=True,
        use_cpp_message_facade=True,
        use_direct_cpp=False,
    ),
    "direct_cpp": AccelerationPolicy(
        name="direct_cpp",
        require_cpp=True,
        allow_contract_changes=True,
        use_cpp_publisher=True,
        use_cpp_message_facade=False,
        use_direct_cpp=True,
    ),
}


class BackendUnavailableError(RuntimeError):
    """Raised when a required-C++ operation has no certified route."""


def resolve_policy(profile="compatible", *, warn_fallback=False):
    try:
        policy = _PROFILES[profile]
    except KeyError as exc:
        raise ValueError(
            "unknown acceleration profile %r; expected one of %s" % (
                profile, ", ".join(sorted(_PROFILES)))) from exc
    if warn_fallback == policy.warn_fallback:
        return policy
    return AccelerationPolicy(
        name=policy.name,
        require_cpp=policy.require_cpp,
        allow_contract_changes=policy.allow_contract_changes,
        use_cpp_publisher=policy.use_cpp_publisher,
        use_cpp_message_facade=policy.use_cpp_message_facade,
        use_direct_cpp=policy.use_direct_cpp,
        warn_fallback=bool(warn_fallback),
    )


__all__ = [
    "AccelerationPolicy",
    "BackendUnavailableError",
    "resolve_policy",
]
