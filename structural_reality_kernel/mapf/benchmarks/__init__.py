"""
MAPF Benchmarks Module

Provides benchmark framework for MAPF solvers including:
- MovingAI benchmark support
- LoRR competition format
- External benchmark integration
- Performance metrics collection
"""

from .benchmarks import (
    BenchmarkResult,
    MAPFBenchmarkSuite,
    run_mandatory_benchmarks,
    run_all_benchmarks,
)

from .movingai import MovingAIMap

__all__ = [
    "BenchmarkResult",
    "MAPFBenchmarkSuite",
    "run_mandatory_benchmarks",
    "run_all_benchmarks",
    "MovingAIMap",
]
