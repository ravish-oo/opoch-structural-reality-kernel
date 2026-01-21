"""
time_entropy_energy/energy.py - Energy as cost dual of observation.

ΔE = c(τ) (cost of test)
E = Σ ΔE (total energy)
ε = ΔE/ΔT (energy per observed bit)

Energy is the cost ledger for making observation/time happen.
"""

from dataclasses import dataclass, field
from typing import Any, Dict, FrozenSet, List, Optional, Tuple
import hashlib

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from core.kernel import Test, Record
from core.receipts import CanonicalJSON
from .time_module import TimeIncrement


@dataclass
class EnergyIncrement:
    """
    A single energy increment ΔE = c(τ).

    Stored as integer cost units.
    """
    test_id: str
    cost: int
    event_index: int

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "ENERGY_INCREMENT",
            "test_id": self.test_id,
            "cost": self.cost,
            "event_index": self.event_index
        })

    def to_receipt(self) -> Dict[str, Any]:
        return {
            "type": "ENERGY_INCREMENT",
            "event_index": self.event_index,
            "test_id": self.test_id,
            "cost": self.cost,
            "result": "PASS"
        }


@dataclass
class EnergyLedger:
    """
    Complete energy accounting: E = Σ c(τ).

    All values are integers.
    """
    increments: List[EnergyIncrement] = field(default_factory=list)

    @property
    def total_energy(self) -> int:
        """E = Σ ΔE = Σ c(τ)"""
        return sum(inc.cost for inc in self.increments)

    def record_cost(self, test_id: str, cost: int) -> EnergyIncrement:
        """Record energy cost of a test."""
        inc = EnergyIncrement(
            test_id=test_id,
            cost=cost,
            event_index=len(self.increments)
        )
        self.increments.append(inc)
        return inc

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "ENERGY_LEDGER",
            "total_energy": self.total_energy,
            "event_count": len(self.increments)
        })

    def to_receipt(self) -> Dict[str, Any]:
        events = [
            {"test_id": inc.test_id, "cost": inc.cost}
            for inc in self.increments
        ]
        return {
            "type": "ENERGY_LEDGER",
            "events": events,
            "total_energy": self.total_energy,
            "event_count": len(self.increments),
            "result": "PASS"
        }


@dataclass
class EnergyCoupling:
    """
    Energy-time coupling: ε = ΔE/ΔT.

    Energy per observed bit (per unit time).
    """
    delta_e: int  # Energy increment (integer)
    delta_t_ratio: Tuple[int, int]  # (|W|, |W'|) for ΔT = log(|W|/|W'|)
    event_index: int

    @property
    def delta_t(self) -> float:
        """ΔT = log₂(|W|/|W'|)"""
        w_pre, w_post = self.delta_t_ratio
        if w_post == 0:
            return float('inf')
        if w_pre == 0:
            return 0.0
        import math
        return math.log2(w_pre / w_post)

    @property
    def epsilon(self) -> float:
        """ε = ΔE/ΔT (energy per bit)"""
        dt = self.delta_t
        if dt == 0:
            return float('inf') if self.delta_e > 0 else 0.0
        if dt == float('inf'):
            return 0.0
        return self.delta_e / dt

    def canonical(self) -> str:
        return CanonicalJSON.serialize({
            "type": "ENERGY_COUPLING",
            "event_index": self.event_index,
            "delta_e": self.delta_e,
            "delta_t_ratio": list(self.delta_t_ratio)
        })

    def to_receipt(self) -> Dict[str, Any]:
        # Use strings for display to avoid floats in receipts
        dt_d = str(round(self.delta_t, 6)) if self.delta_t_ratio[1] > 0 else "INF"
        eps_d = str(round(self.epsilon, 6)) if self.delta_t > 0 and self.delta_t != float('inf') else "N/A"
        return {
            "type": "ENERGY_COUPLING",
            "event_index": self.event_index,
            "delta_e": self.delta_e,
            "delta_t_ratio": list(self.delta_t_ratio),
            "delta_t_display": dt_d,
            "epsilon_display": eps_d,
            "result": "PASS"
        }


def compute_energy(tests: Dict[str, Test], records: List[Record]) -> EnergyLedger:
    """Compute energy ledger from tests and records."""
    ledger = EnergyLedger()

    for record in records:
        test = tests.get(record.test_id)
        if test:
            ledger.record_cost(record.test_id, test.cost)

    return ledger


def compute_coupling(
    time_inc: TimeIncrement,
    energy_inc: EnergyIncrement
) -> EnergyCoupling:
    """Compute energy-time coupling."""
    return EnergyCoupling(
        delta_e=energy_inc.cost,
        delta_t_ratio=time_inc.ratio,
        event_index=time_inc.event_index
    )
