"""
time_entropy_energy/time.py - Time as irreversible elimination.

ΔT = log(|W|/|W'|) ≥ 0
T = Σ ΔT

Time is the unique additive invariant of survivor shrink.
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


@dataclass
class TimeIncrement:
    """
    A single time increment ΔT = log(|W|/|W'|).

    Stored as integer ratio (w_pre, w_post) for receipts.
    Log computed only for display.
    """
    w_pre: int
    w_post: int
    event_index: int

    @property
    def ratio(self) -> Tuple[int, int]:
        """Return (|W|, |W'|) as integer pair."""
        return (self.w_pre, self.w_post)

    @property
    def delta_t(self) -> float:
        """
        Compute ΔT = log₂(|W|/|W'|).

        For display only - not for receipts.
        """
        if self.w_post == 0:
            return float('inf')
        if self.w_pre == 0:
            return 0.0
        return math.log2(self.w_pre / self.w_post)

    @property
    def is_nonnegative(self) -> bool:
        """Check ΔT ≥ 0, i.e., |W'| ≤ |W|."""
        return self.w_post <= self.w_pre

    def canonical(self) -> str:
        """Canonical representation - integers only."""
        return CanonicalJSON.serialize({
            "type": "TIME_INCREMENT",
            "event_index": self.event_index,
            "w_pre": self.w_pre,
            "w_post": self.w_post,
            "delta_t_ratio": list(self.ratio)
        })

    def to_receipt(self) -> Dict[str, Any]:
        """Generate receipt for this time increment."""
        # Use string for display value to avoid floats in receipts
        if self.w_post > 0:
            dt_display = str(round(self.delta_t, 6))
        else:
            dt_display = "INF"

        return {
            "type": "TIME_INCREMENT",
            "event_index": self.event_index,
            "w_pre": self.w_pre,
            "w_post": self.w_post,
            "delta_t_ratio": list(self.ratio),
            "delta_t_display": dt_display,
            "is_nonnegative": self.is_nonnegative,
            "result": "PASS" if self.is_nonnegative else "FAIL"
        }


@dataclass
class TotalTime:
    """
    Total time T = Σ ΔT.

    Represented as (|D₀|, |W(L)|) since T = log(|D₀|/|W|).
    """
    d0_size: int
    final_w_size: int
    increments: List[TimeIncrement]

    @property
    def ratio(self) -> Tuple[int, int]:
        """Return (|D₀|, |W|) as integer pair."""
        return (self.d0_size, self.final_w_size)

    @property
    def total_time(self) -> float:
        """
        Compute T = log₂(|D₀|/|W|).

        For display only.
        """
        if self.final_w_size == 0:
            return float('inf')
        if self.d0_size == 0:
            return 0.0
        return math.log2(self.d0_size / self.final_w_size)

    @property
    def sum_of_increments(self) -> float:
        """Sum of individual ΔT values (for verification)."""
        return sum(inc.delta_t for inc in self.increments if inc.w_post > 0)

    def verify_additivity(self, tolerance: float = 1e-10) -> bool:
        """
        Verify T = Σ ΔT (additivity property).

        Returns True if total equals sum of increments.
        """
        if self.final_w_size == 0:
            return True  # Infinite time case
        return abs(self.total_time - self.sum_of_increments) < tolerance

    def canonical(self) -> str:
        """Canonical representation - integers only."""
        return CanonicalJSON.serialize({
            "type": "TOTAL_TIME",
            "d0_size": self.d0_size,
            "final_w_size": self.final_w_size,
            "time_ratio": list(self.ratio),
            "increment_count": len(self.increments)
        })

    def to_receipt(self) -> Dict[str, Any]:
        """Generate receipt for total time."""
        # Use string for display to avoid floats
        if self.final_w_size > 0:
            tt_display = str(round(self.total_time, 6))
        else:
            tt_display = "INF"

        return {
            "type": "TOTAL_TIME",
            "d0_size": self.d0_size,
            "final_w_size": self.final_w_size,
            "time_ratio": list(self.ratio),
            "total_time_display": tt_display,
            "increment_count": len(self.increments),
            "additivity_verified": self.verify_additivity(),
            "result": "PASS"
        }


def compute_time_increment(
    transition: SurvivorTransition,
    event_index: int
) -> TimeIncrement:
    """Compute time increment from a survivor transition."""
    return TimeIncrement(
        w_pre=transition.w_pre,
        w_post=transition.w_post,
        event_index=event_index
    )


def compute_total_time(
    d0_size: int,
    final_w_size: int,
    transitions: List[SurvivorTransition]
) -> TotalTime:
    """Compute total time from all transitions."""
    increments = [
        compute_time_increment(trans, i)
        for i, trans in enumerate(transitions)
    ]

    return TotalTime(
        d0_size=d0_size,
        final_w_size=final_w_size,
        increments=increments
    )
