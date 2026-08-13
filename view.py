import argparse
from io import StringIO
import time

import numpy as np
import viser
from viser.extras import ViserUrdf
import yourdfpy

from urdf_skeleton import URDFSkeleton


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("path", help="Path to URDF file.")
    parser.add_argument(
        "-s", "--skeleton", action="store_true", help="Convert URDF to a skeleton."
    )
    args = parser.parse_args()

    if args.skeleton:
        urdf = URDFSkeleton.from_file(args.path)
        urdf = yourdfpy.URDF.load(StringIO(urdf.to_string()))
    else:
        urdf = yourdfpy.URDF.load(args.path)

    server = viser.ViserServer()
    urdf = ViserUrdf(server, urdf)

    sliders = []
    with server.gui.add_folder("Joints"):
        for name, (lo, hi) in urdf.get_actuated_joint_limits().items():
            lo = lo if lo is not None else -np.pi
            hi = hi if hi is not None else np.pi
            slider = server.gui.add_slider(name, lo, hi, (hi - lo) / 100, 0.0)
            slider.on_update(
                lambda _: urdf.update_cfg(np.array([s.value for s in sliders]))
            )
            sliders.append(slider)

    urdf.update_cfg(np.array([s.value for s in sliders]))
    while True:
        time.sleep(10.0)


if __name__ == "__main__":
    main()
