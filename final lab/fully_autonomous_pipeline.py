import argparse
import os
import subprocess
import sys
import time
from typing import List


def run_step(name: str, command: List[str]) -> None:
    print(f"\n========== {name} ==========")
    print(" ".join(command))
    result = subprocess.run(command, check=False)
    if result.returncode != 0:
        raise RuntimeError(f"{name} failed with code {result.returncode}")


def start_background(name: str, command: List[str]) -> subprocess.Popen:
    print(f"\n========== {name} (background) ==========")
    print(" ".join(command))
    process = subprocess.Popen(command)
    time.sleep(3.0)
    if process.poll() is not None:
        raise RuntimeError(f"{name} exited early with code {process.returncode}")
    return process


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Fully autonomous pipeline: SLAM + exploration + rearrangement"
    )
    parser.add_argument(
        "--semi-script",
        default=os.path.join(os.path.dirname(__file__), "semi_autonomous_pipeline.py"),
        help="Path to semi-autonomous rearrangement script",
    )
    parser.add_argument(
        "--config",
        default=os.path.join(os.path.dirname(__file__), "config", "regions.yaml"),
        help="Scenario config for rearrangement",
    )
    parser.add_argument(
        "--skip-exploration",
        action="store_true",
        help="Skip SLAM/exploration stages and run rearrangement only",
    )
    args = parser.parse_args()

    # Uses external ROS packages that are outside Lab1-4.
    # 1) SLAM Toolbox for map building
    # 2) Frontier exploration package for autonomous unknown-space exploration
    # 3) Nav2 map_server / lifecycle manager for map serving after exploration
    if not args.skip_exploration:
        slam_proc = start_background(
            "Start SLAM Toolbox",
            ["ros2", "launch", "slam_toolbox", "online_async_launch.py"],
        )
        exploration_proc = start_background(
            "Run frontier exploration",
            ["ros2", "launch", "nav2_wavefront_frontier_exploration", "exploration.launch.py"],
        )
        input(
            "\nExploration is running. Press Enter after the robot mapped the space "
            "and all target objects have been discovered."
        )
        exploration_proc.terminate()
        run_step("Save map", ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", "final_lab_map"])
        slam_proc.terminate()
        print(
            "\nNOTE: Map files were written as final_lab_map.yaml / final_lab_map.pgm (typical map_saver_cli output). "
            "You must run Nav2 + localization against this map (map_server + AMCL or fused localization) before "
            "the semi-autonomous script can use map-frame goals and live TF map->camera. "
            "This repo script does not auto-launch that stack.\n"
        )

    run_step(
        "Run semi-autonomous rearrangement",
        [sys.executable, args.semi_script, "--config", args.config],
    )


if __name__ == "__main__":
    main()
