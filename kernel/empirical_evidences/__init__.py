"""
Empirical Evidences - Structural Reality Kernel

Complete implementations organized by domain:

Submodules:
    physics/       - Physical laws derived from kernel
    logic/         - Logic, computation, and complexity theory
    mind/          - Consciousness and intelligence
    applications/  - Real-world applications
    verification/  - Verification test suites

Note: MAPF (Multi-Agent Path Finding) has been moved to its own
top-level module: kernel.mapf

All modules provide:
- Mathematical derivations as code
- Full verification suites
- Proof bundle generation
- Canonical receipts
"""

from . import physics
from . import logic
from . import mind
from . import applications
from . import verification

__all__ = [
    "physics",
    "logic",
    "mind",
    "applications",
    "verification",
]
