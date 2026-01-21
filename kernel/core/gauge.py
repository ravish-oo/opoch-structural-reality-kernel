"""
core/gauge.py - Gauge invariance and canonicalization.

Gauge groupoid G_L: all transformations invisible to feasible tests,
including:
- Renaming tests/outcomes preserving induced partitions
- Recoding D0 preserving separability under Δ(L)
- Reslicing independent event orders

Physical content: PhysOut = RawOut / G_L
"""

from dataclasses import dataclass
from typing import Any, Callable, Dict, FrozenSet, List, Optional, Set, Tuple
import hashlib
import json

from .kernel import (
    Test, Record, Ledger, Survivors, PiStar, KernelState,
    compute_kernel_state
)


class Canonicalizer:
    """
    Canonical form generator for various objects.

    Ensures deterministic, gauge-invariant representations.
    """

    @staticmethod
    def canonical_json(obj: Any) -> str:
        """
        Convert object to canonical JSON string.

        Rules:
        - Keys sorted lexicographically
        - No whitespace
        - Numbers as decimal integers (no leading zeros)
        - Arrays preserve order
        - Strings minimally escaped
        """
        return json.dumps(obj, sort_keys=True, separators=(",", ":"))

    @staticmethod
    def fingerprint(obj: Any) -> str:
        """SHA-256 fingerprint of canonical JSON."""
        canonical = Canonicalizer.canonical_json(obj)
        return hashlib.sha256(canonical.encode()).hexdigest()

    @staticmethod
    def canonical_multiset(items: List[Any]) -> List[Any]:
        """
        Canonical representation of a multiset.
        Sorted by canonical JSON of elements.
        """
        return sorted(items, key=lambda x: Canonicalizer.canonical_json(x))

    @staticmethod
    def canonical_partition(
        partition: List[Set[Any]]
    ) -> List[Tuple[int, str]]:
        """
        Canonical representation of a partition.

        Returns list of (size, representative_fingerprint) sorted by size desc,
        then by fingerprint.
        """
        result = []
        for part in partition:
            size = len(part)
            # Use sorted elements for deterministic representative
            sorted_part = sorted(part, key=lambda x: str(x))
            rep = sorted_part[0] if sorted_part else None
            fp = Canonicalizer.fingerprint(rep)
            result.append((size, fp))

        return sorted(result, key=lambda x: (-x[0], x[1]))

    @staticmethod
    def pi_invariant_fingerprint(pi_star: PiStar) -> str:
        """
        Compute Π-invariant fingerprint of a quotient.

        Uses class-size multiset (completely gauge-invariant).
        """
        sizes = sorted([c.size for c in pi_star.partition], reverse=True)
        return Canonicalizer.fingerprint({"sizes": sizes})


@dataclass
class GaugeTransformation:
    """Represents a gauge transformation."""
    name: str
    transformation_type: str  # "recoding", "renaming", "reslicing"
    mapping: Dict[Any, Any]
    inverse_mapping: Dict[Any, Any]

    def apply_to_d0(self, d0: FrozenSet[Any]) -> FrozenSet[Any]:
        """Apply recoding to D0."""
        return frozenset(self.mapping.get(x, x) for x in d0)

    def apply_inverse_to_element(self, y: Any) -> Any:
        """Apply inverse to recover original element."""
        return self.inverse_mapping.get(y, y)


class GaugeChecker:
    """
    Verifies gauge invariance properties.

    Checks that Π* class-size multiset is unchanged under
    gauge transformations.
    """

    def __init__(self):
        self.checks: List[Dict[str, Any]] = []

    def check_recoding_invariance(
        self,
        d0: FrozenSet[Any],
        ledger: Ledger,
        tests: Dict[str, Test],
        transformation: GaugeTransformation,
        alpha: int = 1
    ) -> Dict[str, Any]:
        """
        Check that Π* is invariant under recoding.

        A recoding is a bijection on D0 labels. The class-size
        multiset should be identical.
        """
        # Original Π*
        state1 = compute_kernel_state(d0, ledger, tests, alpha)
        pi1 = state1.pi_star
        sizes1 = sorted(pi1.class_sizes(), reverse=True)

        # Recoded D0
        recoded_d0 = transformation.apply_to_d0(d0)

        # Recode tests to work on recoded domain
        recoded_tests = {}
        for tid, test in tests.items():
            inv = transformation.inverse_mapping

            def make_recoded_eval(orig_test, inverse_map):
                def recoded_eval(x):
                    orig_x = inverse_map.get(x, x)
                    return orig_test(orig_x)
                return recoded_eval

            recoded_tests[tid] = Test(
                test_id=tid,
                evaluator=make_recoded_eval(test, inv),
                cost=test.cost,
                outcome_space=test.outcome_space
            )

        # Recoded Π*
        state2 = compute_kernel_state(recoded_d0, ledger, recoded_tests, alpha)
        pi2 = state2.pi_star
        sizes2 = sorted(pi2.class_sizes(), reverse=True)

        passed = sizes1 == sizes2

        result = {
            "check_type": "recoding_invariance",
            "transformation": transformation.name,
            "original_sizes": sizes1,
            "recoded_sizes": sizes2,
            "passed": passed,
            "pi_fp_original": pi1.canonical_fingerprint(),
            "pi_fp_recoded": pi2.canonical_fingerprint()
        }
        self.checks.append(result)
        return result

    def check_order_invariance(
        self,
        d0: FrozenSet[Any],
        records: List[Record],
        tests: Dict[str, Test],
        alpha: int = 1
    ) -> Dict[str, Any]:
        """
        Check Π* diamond property: order of records doesn't matter.

        Π*(L ∪ {r,s}) = Π*(L ∪ {s,r})
        """
        if len(records) < 2:
            result = {
                "check_type": "order_invariance",
                "passed": True,
                "reason": "fewer than 2 records"
            }
            self.checks.append(result)
            return result

        # Build ledger in original order
        ledger1 = Ledger(records)
        state1 = compute_kernel_state(d0, ledger1, tests, alpha)
        pi1_fp = state1.pi_star.canonical_fingerprint()

        # Build ledger in reversed order
        ledger2 = Ledger(list(reversed(records)))
        state2 = compute_kernel_state(d0, ledger2, tests, alpha)
        pi2_fp = state2.pi_star.canonical_fingerprint()

        # Build ledger with shuffled order (deterministic shuffle)
        import random
        rng = random.Random(42)
        shuffled = records.copy()
        rng.shuffle(shuffled)
        ledger3 = Ledger(shuffled)
        state3 = compute_kernel_state(d0, ledger3, tests, alpha)
        pi3_fp = state3.pi_star.canonical_fingerprint()

        passed = (pi1_fp == pi2_fp == pi3_fp)

        result = {
            "check_type": "order_invariance",
            "passed": passed,
            "pi_fp_original": pi1_fp,
            "pi_fp_reversed": pi2_fp,
            "pi_fp_shuffled": pi3_fp,
            "record_count": len(records)
        }
        self.checks.append(result)
        return result

    def check_renaming_invariance(
        self,
        d0: FrozenSet[Any],
        ledger: Ledger,
        tests: Dict[str, Test],
        outcome_renaming: Dict[Any, Any],
        alpha: int = 1
    ) -> Dict[str, Any]:
        """
        Check invariance under outcome renaming.

        If we rename outcomes consistently (bijection), the partition
        structure should be preserved.
        """
        # Original Π*
        state1 = compute_kernel_state(d0, ledger, tests, alpha)
        sizes1 = sorted(state1.pi_star.class_sizes(), reverse=True)

        # Rename outcomes in tests and ledger
        renamed_tests = {}
        for tid, test in tests.items():
            renamed_outcomes = frozenset(
                outcome_renaming.get(o, o) for o in test.outcome_space
            )

            def make_renamed_eval(orig_test, rename_map):
                def renamed_eval(x):
                    result = orig_test(x)
                    return rename_map.get(result, result)
                return renamed_eval

            renamed_tests[tid] = Test(
                test_id=tid,
                evaluator=make_renamed_eval(test, outcome_renaming),
                cost=test.cost,
                outcome_space=renamed_outcomes
            )

        # Rename outcomes in ledger
        renamed_records = []
        for record, count in ledger:
            renamed_outcome = outcome_renaming.get(record.outcome, record.outcome)
            renamed_records.extend([
                Record(test_id=record.test_id, outcome=renamed_outcome)
            ] * count)
        renamed_ledger = Ledger(renamed_records)

        # Renamed Π*
        state2 = compute_kernel_state(d0, renamed_ledger, renamed_tests, alpha)
        sizes2 = sorted(state2.pi_star.class_sizes(), reverse=True)

        passed = sizes1 == sizes2

        result = {
            "check_type": "renaming_invariance",
            "passed": passed,
            "original_sizes": sizes1,
            "renamed_sizes": sizes2
        }
        self.checks.append(result)
        return result

    def all_passed(self) -> bool:
        """Check if all gauge checks passed."""
        return all(c.get("passed", False) for c in self.checks)

    def summary(self) -> Dict[str, Any]:
        """Summary of all checks."""
        return {
            "total_checks": len(self.checks),
            "passed": sum(1 for c in self.checks if c.get("passed")),
            "failed": sum(1 for c in self.checks if not c.get("passed")),
            "all_passed": self.all_passed()
        }


def create_random_recoding(
    d0: FrozenSet[Any],
    seed: int = 42
) -> GaugeTransformation:
    """
    Create a random bijection on D0 for testing gauge invariance.
    """
    import random
    rng = random.Random(seed)

    elements = list(d0)
    shuffled = elements.copy()
    rng.shuffle(shuffled)

    mapping = dict(zip(elements, shuffled))
    inverse = dict(zip(shuffled, elements))

    return GaugeTransformation(
        name=f"random_recoding_seed_{seed}",
        transformation_type="recoding",
        mapping=mapping,
        inverse_mapping=inverse
    )
