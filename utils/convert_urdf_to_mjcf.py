"""Mini Cheetah model conversion helper.

The supported Mini Cheetah MJCF is already checked in at
MiniCheetah_description/mjcf/mini_cheetah.xml. Keep this file as the place to
add a future URDF-to-MJCF workflow if a Mini Cheetah URDF source is introduced.
"""

from pathlib import Path


MJCF_PATH = Path(__file__).resolve().parents[1] / "MiniCheetah_description" / "mjcf" / "mini_cheetah.xml"


if __name__ == "__main__":
    if not MJCF_PATH.is_file():
        raise FileNotFoundError(f"Mini Cheetah MJCF not found: {MJCF_PATH}")
    print(f"Mini Cheetah MJCF is already available: {MJCF_PATH}")
