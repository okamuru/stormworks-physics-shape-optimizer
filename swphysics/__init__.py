"""Stormworks Physics Shape analysis prototype."""

from .definitions import DefinitionCatalog
from .partition import Box, PartitionResult, partition_cubes_exact, partition_cubes_greedy
from .vehicle import Vehicle, load_vehicle

__all__ = [
    "Box",
    "DefinitionCatalog",
    "PartitionResult",
    "Vehicle",
    "load_vehicle",
    "partition_cubes_exact",
    "partition_cubes_greedy",
]

__version__ = "1.2.1 Alpha"
__author__ = "IrisNuiYaMa_164"
