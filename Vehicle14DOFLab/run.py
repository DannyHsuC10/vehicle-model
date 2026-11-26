"""Start the lab from any location, including a renamed project folder."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import sys


if __name__ == "__main__":
    folder = Path(__file__).resolve().parent
    spec = spec_from_file_location("Vehicle14DOFLab", folder / "__init__.py",
                                   submodule_search_locations=[str(folder)])
    package = module_from_spec(spec)
    sys.modules[spec.name] = package
    spec.loader.exec_module(package)
    from Vehicle14DOFLab.__main__ import main

    raise SystemExit(main())
