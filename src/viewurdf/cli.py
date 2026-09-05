import argparse
import importlib
from io import StringIO
import time
import webbrowser

import numpy as np
from robot_descriptions._xacro import get_urdf_path
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
    parser.add_argument(
        "-r",
        "--robot-description",
        action="store_true",
        help="Use a model from robot_descriptions.py rather than a local URDF.",
    )
    parser.add_argument("--ip", default="0.0.0.0", help="IP address to serve visualizer on.")
    parser.add_argument("-p", "--port", default="8080", help="Port to serve visualizer on.")
    parser.add_argument("--no-open", action="store_true", help="Do not automatically open the visualizer in a browser tab.")
    args = parser.parse_args()

    if args.robot_description:
        module = importlib.import_module(f"robot_descriptions.{args.path}_description")
        path = get_urdf_path(module)
    else:
        path = args.path

    # skeleton URDF
    skeleton = URDFSkeleton.from_file(path)
    skeleton = yourdfpy.URDF.load(StringIO(skeleton.to_string()))

    # normal URDF
    robot = yourdfpy.URDF.load(path)

    server = viser.ViserServer(host=args.ip, port=args.port)
    server.gui.configure_theme(control_width="large")

    if not args.no_open:
        url = f"http://{args.ip}:{args.port}"
        webbrowser.open_new_tab(url)

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

    with server.gui.add_folder("Skeleton Mode"):
        checkbox = server.gui.add_checkbox(
            label="Enable", initial_value=skeleton_base.visible
        )

        def update_vis(_):
            skeleton_base.visible = checkbox.value
            robot_base.visible = not checkbox.value

        checkbox.on_update(update_vis)

        link_radius_slider = server.gui.add_slider(
            "Link radius", 0.001, 0.01, (0.01 - 0.001) / 100, 0.003
        )
        joint_radius_slider = server.gui.add_slider(
            "Joint radius", 0.005, 0.05, (0.05 - 0.005) / 100, 0.01
        )
        joint_length_slider = server.gui.add_slider(
            "Joint length", 0.01, 0.1, (0.1 - 0.01) / 100, 0.03
        )

        def update_scale(_):
            nonlocal skeleton
            skeleton = URDFSkeleton.from_file(
                path,
                link_radius=link_radius_slider.value,
                joint_radius=joint_radius_slider.value,
                joint_length=joint_length_slider.value,
            )
            skeleton = yourdfpy.URDF.load(StringIO(skeleton.to_string()))
            skeleton = ViserUrdf(server, skeleton, root_node_name="/skeleton")

        link_radius_slider.on_update(update_scale)
        joint_radius_slider.on_update(update_scale)
        joint_length_slider.on_update(update_scale)

    robot.update_cfg(np.array([s.value for s in sliders]))
    skeleton.update_cfg(np.array([s.value for s in sliders]))

    # spin
    while True:
        time.sleep(10.0)


if __name__ == "__main__":
    main()
