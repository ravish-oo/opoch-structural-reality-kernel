"""
time_entropy_energy/entropy.py - Entropy as log survivors.

S(L) = log|W(L)|

Entropy is the log-count of remaining consistent possibilities.
The identity ΔT = -ΔS connects time and entropy.
"""

from dataclasses import dataclass, field
from typing import Any, Dict, FrozenSet, List, Optional, Tuple
import math
import hashlib

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from core.receipts import CanonicalJSON
from .survivors import SurvivorTransition
from .time_module import TimeIncrement


@dataclass
class Entropy:
    """
    S(L) = log|W(L)|

    Stored as |W| integer for receipts.
    """
    w_size: int

    @property
    def entropy(self) -> float:
        """
        Compute S = log₂|W|.

        For display only.
        """
        if self.w_size <= 0:
            return float('-inf')
        return math.log2(self.w_size)

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "ENTROPY",
            "w_size": self.w_size
        })

    def to_receipt(self) -> Dict[str, Any]:
        # Use string for display to avoid floats
        s_display = str(round(self.entropy, 6)) if self.w_size > 0 else "-INF"
        return {
            "type": "ENTROPY",
            "w_size": self.w_size,
            "entropy_display": s_display,
            "result": "PASS"
        }


@dataclass
class EntropyChange:
    """
    ΔS = S_post - S_pre = log|W'| - log|W|

    Since |W'| ≤ |W|, we have ΔS ≤ 0.
    """
    w_pre: int
    w_post: int
    event_index: int

    @property
    def s_pre(self) -> float:
        """S_pre = log|W|"""
        if self.w_pre <= 0:
            return float('-inf')
        return math.log2(self.w_pre)

    @property
    def s_post(self) -> float:
        """S_post = log|W'|"""
        if self.w_post <= 0:
            return float('-inf')
        return math.log2(self.w_post)

    @property
    def delta_s(self) -> float:
        """ΔS = S_post - S_pre ≤ 0"""
        if self.w_post <= 0:
            return float('-inf')
        if self.w_pre <= 0:
            return float('inf')
        return self.s_post - self.s_pre

    @property
    def is_nonpositive(self) -> bool:
        """Check ΔS ≤ 0 (entropy decreases or stays same)."""
        return self.w_post <= self.w_pre

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "ENTROPY_CHANGE",
            "event_index": self.event_index,
            "w_pre": self.w_pre,
            "w_post": self.w_post
        })

    def to_receipt(self) -> Dict[str, Any]:
        # Use strings for display to avoid floats
        s_pre_d = str(round(self.s_pre, 6)) if self.w_pre > 0 else "-INF"
        s_post_d = str(round(self.s_post, 6)) if self.w_post > 0 else "-INF"
        delta_s_d = str(round(self.delta_s, 6)) if self.w_post > 0 and self.w_pre > 0 else "N/A"

        return {
            "type": "ENTROPY_CHANGE",
            "event_index": self.event_index,
            "w_pre": self.w_pre,
            "w_post": self.w_post,
            "s_pre_display": s_pre_d,
            "s_post_display": s_post_d,
            "delta_s_display": delta_s_d,
            "is_nonpositive": self.is_nonpositive,
            "result": "PASS" if self.is_nonpositive else "FAIL"
        }


@dataclass
class EntropyTimeEquivalence:
    """
    Verification of the identity ΔT = -ΔS.

    This is the deepest connection: time = entropy removed.
    """
    time_increment: TimeIncrement
    entropy_change: EntropyChange

    @property
    def delta_t(self) -> float:
        """ΔT from time increment."""
        return self.time_increment.delta_t

    @property
    def neg_delta_s(self) -> float:
        """-ΔS from entropy change."""
        return -self.entropy_change.delta_s

    def verify_identity(self, tolerance: float = 1e-10) -> bool:
        """
        Verify ΔT = -ΔS.

        Returns True if identity holds within tolerance.
        """
        if self.time_increment.w_post == 0:
            return True  # Both infinite

        return abs(self.delta_t - self.neg_delta_s) < tolerance

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "ENTROPY_TIME_EQUIVALENCE",
            "event_index": self.time_increment.event_index,
            "w_pre": self.time_increment.w_pre,
            "w_post": self.time_increment.w_post
        })

    def to_receipt(self) -> Dict[str, Any]:
        holds = self.verify_identity()
        # Use strings for display to avoid floats
        dt_d = str(round(self.delta_t, 6)) if self.time_increment.w_post > 0 else "INF"
        nds_d = str(round(self.neg_delta_s, 6)) if self.entropy_change.w_post > 0 else "INF"

        return {
            "type": "ENTROPY_TIME_EQUIVALENCE",
            "event_index": self.time_increment.event_index,
            "delta_t_display": dt_d,
            "neg_delta_s_display": nds_d,
            "identity_holds": holds,
            "result": "PASS" if holds else "FAIL"
        }


def compute_entropy(w_size: int) -> Entropy:
    """Compute entropy from survivor count."""
    return Entropy(w_size=w_size)


def compute_entropy_change(transition: SurvivorTransition, event_index: int) -> EntropyChange:
    """Compute entropy change from transition."""
    return EntropyChange(
        w_pre=transition.w_pre,
        w_post=transition.w_post,
        event_index=event_index
    )


def verify_entropy_time_equivalence(
    time_inc: TimeIncrement,
    entropy_change: EntropyChange
) -> EntropyTimeEquivalence:
    """Verify the ΔT = -ΔS identity."""
    return EntropyTimeEquivalence(
        time_increment=time_inc,
        entropy_change=entropy_change
    )
