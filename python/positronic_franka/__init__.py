"""Python shim for the positronic_franka extension.

This package exposes the compiled pybind11 module as `positronic_franka._franka`
and re-exports its symbols at the package top-level for convenience.
"""

from ._franka import *  # re-export extension symbols
# Keep a private alias for tools that expect a module attribute
from . import _franka as _core  # noqa: F401

