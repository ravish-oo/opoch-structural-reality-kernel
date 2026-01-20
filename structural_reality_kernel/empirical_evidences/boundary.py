"""
time_entropy_energy/boundary.py - Boundary flow for open systems.

J = |W_S||W_E|/|W| ≥ 1 (join multiplicity)
T^Γ = log J ≥ 0 (boundary term)

ΔT^(U) = ΔT^(S) + ΔT^(E) + ΔT^Γ

This explains how local structure can grow by exporting irreversibility.
"""

from dataclasses import dataclass, field
from typing import Any, Dict, FrozenSet, List, Optional, Set, Tuple
import math
import hashlib

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from core.receipts import CanonicalJSON


@dataclass
class JoinMultiplicity:
    """
    J = |W_S||W_E|/|W| ≥ 1

    The join multiplicity measures how much the subsystem and environment
    are correlated (entangled) vs independent.
    """
    w_total: int  # |W| - total survivors
    w_system: int  # |W_S| - system survivors
    w_environment: int  # |W_E| - environment survivors

    @property
    def product(self) -> int:
        """|W_S| × |W_E|"""
        return self.w_system * self.w_environment

    @property
    def join_multiplicity(self) -> float:
        """J = |W_S||W_E|/|W|"""
        if self.w_total == 0:
            return float('inf')
        return self.product / self.w_total

    @property
    def is_valid(self) -> bool:
        """Check J ≥ 1 (forced by counting)."""
        return self.join_multiplicity >= 1.0 - 1e-10

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "JOIN_MULTIPLICITY",
            "w_total": self.w_total,
            "w_system": self.w_system,
            "w_environment": self.w_environment
        })

    def to_receipt(self) -> Dict[str, Any]:
        # Use string for display to avoid floats in receipts
        j_display = str(round(self.join_multiplicity, 6)) if self.w_total > 0 else "INF"
        return {
            "type": "JOIN_MULTIPLICITY",
            "w_total": self.w_total,
            "w_system": self.w_system,
            "w_environment": self.w_environment,
            "product": self.product,
            "join_multiplicity_display": j_display,
            "j_geq_1": self.is_valid,
            "result": "PASS" if self.is_valid else "FAIL"
        }


@dataclass
class BoundaryFlow:
    """
    T^Γ = log J ≥ 0

    The boundary term in the time accounting.

    ΔT^(U) = ΔT^(S) + ΔT^(E) + ΔT^Γ
    """
    join: JoinMultiplicity

    @property
    def boundary_term(self) -> float:
        """T^Γ = log J"""
        j = self.join.join_multiplicity
        if j <= 0:
            return float('-inf')
        return math.log2(j)

    @property
    def is_nonnegative(self) -> bool:
        """Check T^Γ ≥ 0."""
        return self.boundary_term >= -1e-10

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "BOUNDARY_FLOW",
            "w_total": self.join.w_total,
            "w_system": self.join.w_system,
            "w_environment": self.join.w_environment
        })

    def to_receipt(self) -> Dict[str, Any]:
        # Use strings for display to avoid floats in receipts
        j_display = str(round(self.join.join_multiplicity, 6)) if self.join.w_total > 0 else "INF"
        bt_display = str(round(self.boundary_term, 6)) if self.join.join_multiplicity > 0 else "-INF"
        return {
            "type": "BOUNDARY_FLOW",
            "w_total": self.join.w_total,
            "w_system": self.join.w_system,
            "w_environment": self.join.w_environment,
            "join_multiplicity_display": j_display,
            "boundary_term_display": bt_display,
            "t_gamma_geq_0": self.is_nonnegative,
            "result": "PASS" if self.is_nonnegative else "FAIL"
        }


@dataclass
class TimeAccounting:
    """
    Complete time accounting for a subsystem cut.

    ΔT^(U) = ΔT^(S) + ΔT^(E) + ΔT^Γ

    Where:
    - ΔT^(U) = time in universe
    - ΔT^(S) = time in system
    - ΔT^(E) = time in environment
    - ΔT^Γ = boundary flow term
    """
    delta_t_universe: float
    delta_t_system: float
    delta_t_environment: float
    delta_t_boundary: float

    @property
    def sum_of_parts(self) -> float:
        """ΔT^(S) + ΔT^(E) + ΔT^Γ"""
        return self.delta_t_system + self.delta_t_environment + self.delta_t_boundary

    def verify_accounting(self, tolerance: float = 1e-10) -> bool:
        """
        Verify ΔT^(U) = ΔT^(S) + ΔT^(E) + ΔT^Γ.
        """
        return abs(self.delta_t_universe - self.sum_of_parts) < tolerance

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "TIME_ACCOUNTING",
            "delta_t_universe": round(self.delta_t_universe, 10),
            "delta_t_system": round(self.delta_t_system, 10),
            "delta_t_environment": round(self.delta_t_environment, 10),
            "delta_t_boundary": round(self.delta_t_boundary, 10)
        })

    def to_receipt(self) -> Dict[str, Any]:
        # Use strings for display to avoid floats in receipts
        return {
            "type": "TIME_ACCOUNTING",
            "delta_t_universe_display": str(round(self.delta_t_universe, 6)),
            "delta_t_system_display": str(round(self.delta_t_system, 6)),
            "delta_t_environment_display": str(round(self.delta_t_environment, 6)),
            "delta_t_boundary_display": str(round(self.delta_t_boundary, 6)),
            "sum_of_parts_display": str(round(self.sum_of_parts, 6)),
            "accounting_verified": self.verify_accounting(),
            "result": "PASS" if self.verify_accounting() else "FAIL"
        }


def compute_boundary_term(
    w_total: int,
    w_system: int,
    w_environment: int
) -> BoundaryFlow:
    """Compute boundary flow from survivor counts."""
    join = JoinMultiplicity(
        w_total=w_total,
        w_system=w_system,
        w_environment=w_environment
    )
    return BoundaryFlow(join=join)


def compute_time_accounting(
    w_total_pre: int, w_total_post: int,
    w_system_pre: int, w_system_post: int,
    w_env_pre: int, w_env_post: int,
    boundary_flow: BoundaryFlow
) -> TimeAccounting:
    """Compute complete time accounting for a cut."""
    import math

    def safe_log_ratio(pre: int, post: int) -> float:
        if post == 0:
            return float('inf')
        if pre == 0:
            return 0.0
        return math.log2(pre / post)

    return TimeAccounting(
        delta_t_universe=safe_log_ratio(w_total_pre, w_total_post),
        delta_t_system=safe_log_ratio(w_system_pre, w_system_post),
        delta_t_environment=safe_log_ratio(w_env_pre, w_env_post),
        delta_t_boundary=boundary_flow.boundary_term
    )
