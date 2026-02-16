"""
PIEC Analysis Package

A comprehensive Python package for generating all graphs, tables, and statistical 
analyses mentioned in the PIEC thesis and paper.
"""

from .data_loader import DataLoader
from .pinn_analysis import PINNAnalyzer
from .navigation_analysis import NavigationAnalyzer
from .localization_analysis import LocalizationAnalyzer
from .ablation_analysis import AblationAnalyzer
from .statistical_tests import StatisticalTests
from .visualization import Visualizer
from .table_generator import TableGenerator

__version__ = '0.1.0'
__all__ = [
    'DataLoader',
    'PINNAnalyzer',
    'NavigationAnalyzer',
    'LocalizationAnalyzer',
    'AblationAnalyzer',
    'StatisticalTests',
    'Visualizer',
    'TableGenerator',
]
