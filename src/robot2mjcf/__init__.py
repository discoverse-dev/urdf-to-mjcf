__version__ = "0.0.2"

from .cli.convert import convert_urdf_to_mjcf as run

__all__ = ["run", "__version__"]
