"""
Structural Reality Kernel - Demo Modules

Four demonstration domains:
1. np_sat: NP witness search (SAT solving)
2. mapf: Multi-Agent Path Finding
3. quantum_gns: Finite noncommutative algebra + GNS construction
4. gravity_cost_geometry: Separator-cost distance and holonomy
"""

from .np_sat import NPSatDemo, run_np_sat_demo
from .mapf import MAPFDemo, run_mapf_demo
from .quantum_gns import QuantumGNSDemo, run_quantum_gns_demo
from .gravity_cost_geometry import GravityCostDemo, run_gravity_demo

__all__ = [
    "NPSatDemo", "run_np_sat_demo",
    "MAPFDemo", "run_mapf_demo",
    "QuantumGNSDemo", "run_quantum_gns_demo",
    "GravityCostDemo", "run_gravity_demo"
]
