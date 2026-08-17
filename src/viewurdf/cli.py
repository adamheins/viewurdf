import argparse
from io import StringIO
import time

import numpy as np
import viser
from viser.extras import ViserUrdf
import yourdfpy

from .skeleton import URDFSkeleton


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("path", help="Path to URDF file.")
    parser.add_argument(
        "-s", "--skeleton", action="store_true", help="Start directly in skeleton mode."
    )
    args = parser.parse_args()

    # skeleton URDF
    skeleton = URDFSkeleton.from_file(args.path)
    skeleton = yourdfpy.URDF.load(StringIO(skeleton.to_string()))

    # normal URDF
    robot = yourdfpy.URDF.load(args.path)

    server = viser.ViserServer()
    robot_base = server.scene.add_frame(
        "/robot", show_axes=True, axes_length=0.5, axes_radius=0.005
    )
    skeleton_base = server.scene.add_frame(
        "/skeleton", show_axes=True, axes_length=0.5, axes_radius=0.005
    )

    robot = ViserUrdf(server, robot, root_node_name="/robot")
    skeleton = ViserUrdf(server, skeleton, root_node_name="/skeleton")

    skeleton_base.visible = False

    sliders = []
    with server.gui.add_folder("Joints"):

        def update_cfg(_):
            robot.update_cfg(np.array([s.value for s in sliders]))
            skeleton.update_cfg(np.array([s.value for s in sliders]))

        for name, (lo, hi) in robot.get_actuated_joint_limits().items():
            lo = lo if lo is not None else -np.pi
            hi = hi if hi is not None else np.pi
            slider = server.gui.add_slider(name, lo, hi, (hi - lo) / 100, 0.0)
            slider.on_update(update_cfg)
            sliders.append(slider)

    # TODO: I want to add parameters for the skeleton geometry
    with server.gui.add_folder("Skeleton Mode"):
        checkbox = server.gui.add_checkbox(label="Enable", initial_value=skeleton_base.visible)

        def update_vis(_):
            skeleton_base.visible = checkbox.value
            robot_base.visible = not checkbox.value

        checkbox.on_update(update_vis)

    robot.update_cfg(np.array([s.value for s in sliders]))
    skeleton.update_cfg(np.array([s.value for s in sliders]))

    # spin
    while True:
        time.sleep(10.0)


if __name__ == "__main__":
    main()
