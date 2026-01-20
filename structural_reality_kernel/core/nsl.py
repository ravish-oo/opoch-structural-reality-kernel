"""
core/nsl.py - Null-State Logic implementation.

Trit alphabet T = {-1, 0, +1}:
  +1 = verified (witnessed and consistent with ledger)
   0 = unknown/underdetermined (Ω)
  -1 = refuted (contradiction with ledger)

NSL is the runtime calculus for verified/unknown/refuted status.
"""

from dataclasses import dataclass, field
from typing import Callable, Dict, FrozenSet, Any, List, Optional, Set, Tuple
from enum import IntEnum
import hashlib
import json


class Trit(IntEnum):
    """Trit values: verified (+1), unknown (0), refuted (-1)."""
    REFUTED = -1
    UNKNOWN = 0
    VERIFIED = 1

    def __repr__(self) -> str:
        return {-1: "-1", 0: "0", 1: "+1"}[self.value]

    def __str__(self) -> str:
        return {-1: "REFUTED", 0: "UNKNOWN", 1: "VERIFIED"}[self.value]


@dataclass
class Distinction:
    """
    A single distinction in the index set D.
    Represents an "atom of decision" at the current stage.
    """
    distinction_id: str
    description: str  # Human-readable
    test_id: Optional[str] = None  # Associated test if any
    outcome: Optional[Any] = None  # Expected outcome if any

    def canonical(self) -> str:
        return json.dumps(
            {"id": self.distinction_id, "test": self.test_id, "outcome": str(self.outcome)},
            sort_keys=True, separators=(",", ":")
        )

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()[:16]


class NSLState:
    """
    NSL State: s ∈ T^D where D is the finite distinction index set.

    Sparse representation: most entries are 0 (unknown) until witnessed.
    Only non-zero entries are stored.
    """

    def __init__(self, distinctions: Optional[Dict[str, Distinction]] = None):
        """
        Initialize NSL state.

        Args:
            distinctions: The index set D (distinction_id -> Distinction)
        """
        self.distinctions: Dict[str, Distinction] = distinctions or {}
        self._state: Dict[str, Trit] = {}  # Sparse: only non-zero entries

    def __getitem__(self, distinction_id: str) -> Trit:
        """Get trit value for a distinction (default UNKNOWN)."""
        return self._state.get(distinction_id, Trit.UNKNOWN)

    def __setitem__(self, distinction_id: str, value: Trit) -> None:
        """Set trit value for a distinction."""
        if value == Trit.UNKNOWN:
            # Remove from sparse storage
            self._state.pop(distinction_id, None)
        else:
            self._state[distinction_id] = value

    def verify(self, distinction_id: str) -> None:
        """Mark a distinction as verified (+1)."""
        self._state[distinction_id] = Trit.VERIFIED

    def refute(self, distinction_id: str) -> None:
        """Mark a distinction as refuted (-1)."""
        self._state[distinction_id] = Trit.REFUTED

    def reset(self, distinction_id: str) -> None:
        """Reset a distinction to unknown (0)."""
        self._state.pop(distinction_id, None)

    def verified_set(self) -> Set[str]:
        """Return set of verified distinction IDs."""
        return {d for d, v in self._state.items() if v == Trit.VERIFIED}

    def refuted_set(self) -> Set[str]:
        """Return set of refuted distinction IDs."""
        return {d for d, v in self._state.items() if v == Trit.REFUTED}

    def unknown_set(self) -> Set[str]:
        """Return set of unknown distinction IDs."""
        all_ids = set(self.distinctions.keys())
        known = set(self._state.keys())
        return all_ids - known

    def is_decided(self, distinction_id: str) -> bool:
        """Check if a distinction is decided (not unknown)."""
        return distinction_id in self._state

    def count_verified(self) -> int:
        return sum(1 for v in self._state.values() if v == Trit.VERIFIED)

    def count_refuted(self) -> int:
        return sum(1 for v in self._state.values() if v == Trit.REFUTED)

    def count_unknown(self) -> int:
        return len(self.distinctions) - len(self._state)

    def canonical(self) -> str:
        """
        Canonical representation.
        Sorted by distinction_id for determinism.
        """
        # Include all distinctions, with 0 for unknown
        full_state = {}
        for d_id in sorted(self.distinctions.keys()):
            full_state[d_id] = self._state.get(d_id, Trit.UNKNOWN).value

        return json.dumps(full_state, sort_keys=True, separators=(",", ":"))

    def fingerprint(self) -> str:
        return hashlib.sha256(self.canonical().encode()).hexdigest()

    def copy(self) -> 'NSLState':
        """Create a copy of this state."""
        new_state = NSLState(self.distinctions.copy())
        new_state._state = self._state.copy()
        return new_state


class NSLClosure:
    """
    Closure operator Cl: T^D → T^D

    The NSL form of Π*. Properties:
    - Monotone: s ≤ s' implies Cl(s) ≤ Cl(s')
    - Idempotent: Cl(Cl(s)) = Cl(s)
    - Extensive: s ≤ Cl(s)

    Rule: 0 never becomes +1 without a witness test (otherwise minted).
    """

    def __init__(
        self,
        implication_rules: Optional[List[Tuple[str, str, Trit]]] = None
    ):
        """
        Initialize closure operator.

        Args:
            implication_rules: List of (from_id, to_id, implied_value)
                If distinction from_id is VERIFIED, then to_id gets implied_value.
        """
        self.implications: List[Tuple[str, str, Trit]] = implication_rules or []

    def add_implication(self, from_id: str, to_id: str, implied_value: Trit) -> None:
        """Add an implication rule."""
        self.implications.append((from_id, to_id, implied_value))

    def apply(self, state: NSLState) -> NSLState:
        """
        Apply closure operator to state.

        Returns a new state that is the closure of the input.
        Applies implications until fixed point.
        """
        result = state.copy()
        changed = True

        while changed:
            changed = False
            for from_id, to_id, implied_value in self.implications:
                # Only propagate from VERIFIED distinctions
                if result[from_id] == Trit.VERIFIED:
                    current = result[to_id]

                    # Cannot change REFUTED to VERIFIED or vice versa
                    if current == Trit.UNKNOWN:
                        result[to_id] = implied_value
                        changed = True
                    elif current != implied_value and implied_value != Trit.UNKNOWN:
                        # Contradiction: mark as REFUTED
                        if current == Trit.VERIFIED and implied_value == Trit.REFUTED:
                            # Keep as VERIFIED (already witnessed)
                            pass
                        elif current == Trit.REFUTED and implied_value == Trit.VERIFIED:
                            # Keep as REFUTED (already contradicted)
                            pass

        return result

    def is_closed(self, state: NSLState) -> bool:
        """Check if state is already closed (fixed point)."""
        closed = self.apply(state)
        return state.fingerprint() == closed.fingerprint()


@dataclass
class NSLTransition:
    """
    A single NSL state transition.
    Records the change from one state to another.
    """
    pre_state_fp: str
    post_state_fp: str
    distinction_id: str
    old_value: Trit
    new_value: Trit
    witness: Optional[Any] = None

    def canonical(self) -> str:
        return json.dumps({
            "pre": self.pre_state_fp,
            "post": self.post_state_fp,
            "distinction": self.distinction_id,
            "old": self.old_value.value,
            "new": self.new_value.value,
            "has_witness": self.witness is not None
        }, sort_keys=True, separators=(",", ":"))


class NSLEngine:
    """
    NSL Runtime Engine.

    Manages the evolution of NSL state under kernel operations.
    Enforces the rule: 0 → +1 requires witness.
    """

    def __init__(self, distinctions: Dict[str, Distinction]):
        self.state = NSLState(distinctions)
        self.closure = NSLClosure()
        self.history: List[NSLTransition] = []

    def witness_verify(
        self,
        distinction_id: str,
        witness: Any,
        verifier: Callable[[Any], bool]
    ) -> bool:
        """
        Attempt to verify a distinction with a witness.

        Args:
            distinction_id: The distinction to verify
            witness: The witness object
            verifier: Total verifier function

        Returns:
            True if verified, False otherwise
        """
        if distinction_id not in self.state.distinctions:
            return False

        pre_fp = self.state.fingerprint()
        pre_value = self.state[distinction_id]

        # Cannot re-verify or un-refute
        if pre_value == Trit.VERIFIED:
            return True
        if pre_value == Trit.REFUTED:
            return False

        # Apply verifier (must be total)
        try:
            result = verifier(witness)
        except Exception:
            result = False

        if result:
            self.state.verify(distinction_id)
            post_fp = self.state.fingerprint()

            self.history.append(NSLTransition(
                pre_state_fp=pre_fp,
                post_state_fp=post_fp,
                distinction_id=distinction_id,
                old_value=pre_value,
                new_value=Trit.VERIFIED,
                witness=witness
            ))

            # Apply closure
            self.state = self.closure.apply(self.state)
            return True

        return False

    def refute(self, distinction_id: str, reason: str) -> bool:
        """
        Mark a distinction as refuted.

        Args:
            distinction_id: The distinction to refute
            reason: Reason for refutation

        Returns:
            True if refuted, False if already verified
        """
        if distinction_id not in self.state.distinctions:
            return False

        pre_fp = self.state.fingerprint()
        pre_value = self.state[distinction_id]

        # Cannot refute something already verified
        if pre_value == Trit.VERIFIED:
            return False
        if pre_value == Trit.REFUTED:
            return True

        self.state.refute(distinction_id)
        post_fp = self.state.fingerprint()

        self.history.append(NSLTransition(
            pre_state_fp=pre_fp,
            post_state_fp=post_fp,
            distinction_id=distinction_id,
            old_value=pre_value,
            new_value=Trit.REFUTED,
            witness=reason
        ))

        # Apply closure
        self.state = self.closure.apply(self.state)
        return True

    def get_omega_frontier(self) -> Dict[str, Any]:
        """
        Get the Ω frontier: unknown distinctions.

        Returns dict with frontier info.
        """
        unknown = self.state.unknown_set()
        return {
            "status": "OMEGA",
            "unknown_count": len(unknown),
            "unknown_ids": sorted(unknown),
            "verified_count": self.state.count_verified(),
            "refuted_count": self.state.count_refuted(),
            "state_fingerprint": self.state.fingerprint()
        }

    def is_fully_decided(self) -> bool:
        """Check if all distinctions are decided (no unknowns)."""
        return self.state.count_unknown() == 0


# Type alias for callable verifiers
Verifier = Callable[[Any], bool]
