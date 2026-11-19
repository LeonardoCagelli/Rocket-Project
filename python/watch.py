"""
watch.py - Very simple viewer for the rocket training
======================================================

Opens a window that shows the rocket trying to hit the balloon, driven by
a saved SAC model. While train.py is running in another terminal, the
viewer automatically reloads the model file whenever it changes, so you
can watch the rocket improve during training.

Usage (from the same folder as train.py):

    python watch.py                              # watches models/best/best_model.zip
    python watch.py --model ../models/checkpoints/sac_rocket_100000_steps.zip

If the model file doesn't exist yet (e.g. training just started), the
rocket uses random actions until the first model is saved.

Requires matplotlib:  pip install matplotlib
"""

import argparse
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle, Rectangle
from stable_baselines3 import SAC

from rocket_env import (
    BALLOON_RADIUS,
    ROCKET_HEIGHT,
    ROCKET_WIDTH,
    WORLD_HEIGHT,
    WORLD_WIDTH,
    RocketEnv,
)

# Same project layout as train.py
PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_MODEL = PROJECT_ROOT / "models" / "best" / "best_model.zip"

# Delay between frames in milliseconds (16 ms is about 60 frames/s).
FRAME_INTERVAL_MS = 16


class Viewer:
    def __init__(self, model_path):
        self.model_path = Path(model_path)
        self.model = None
        self.model_mtime = None
        self.model_label = "random actions (no model yet)"

        # A separate simulator instance: it does not affect the training.
        self.env = RocketEnv()
        self.obs, _ = self.env.reset()

        # Statistics shown on screen
        self.episode = 1
        self.finished_episodes = 0
        self.hits = 0
        self.steps = 0
        self.episode_reward = 0.0
        self.last_result = "-"

        self.load_model_if_changed()

        # ---------------- Figure ----------------
        self.fig, self.ax = plt.subplots(figsize=(8, 6.5))
        self.fig.canvas.manager.set_window_title("Rocket training viewer")

        self.ax.set_xlim(0, WORLD_WIDTH)
        self.ax.set_ylim(WORLD_HEIGHT, 0)  # y points down, as in the game
        self.ax.set_aspect("equal")
        self.ax.set_xticks([])
        self.ax.set_yticks([])

        self.balloon_patch = Circle((0, 0), BALLOON_RADIUS, color="blue")
        self.rocket_patch = Rectangle(
            (0, 0), ROCKET_WIDTH, ROCKET_HEIGHT, color="red"
        )
        (self.nose,) = self.ax.plot([], [], "ko", markersize=4)

        self.ax.add_patch(self.balloon_patch)
        self.ax.add_patch(self.rocket_patch)

        self.text = self.ax.text(
            0.01,
            0.99,
            "",
            transform=self.ax.transAxes,
            va="top",
            ha="left",
            fontsize=9,
            family="monospace",
            bbox=dict(facecolor="white", alpha=0.8, edgecolor="gray"),
        )

        self.draw(self.env.sim.get_state())

        # Keep a reference, otherwise the animation is garbage collected.
        self.anim = FuncAnimation(
            self.fig,
            self.update,
            interval=FRAME_INTERVAL_MS,
            blit=False,
            cache_frame_data=False,
        )

    # ---------------- Model handling ----------------

    def load_model_if_changed(self):
        """Load the model file if it exists and changed since last time."""
        if not self.model_path.exists():
            return

        mtime = self.model_path.stat().st_mtime
        if mtime == self.model_mtime:
            return

        try:
            self.model = SAC.load(str(self.model_path), device="cpu")
        except Exception:
            # The file may be half-written by train.py: retry next episode.
            return

        self.model_mtime = mtime
        self.model_label = (
            f"{self.model_path.name} (saved {time.strftime('%H:%M:%S', time.localtime(mtime))})"
        )
        print(f"Loaded model: {self.model_label}")

    # ---------------- Drawing ----------------

    def draw(self, state):
        theta = state.orientation
        c, s = np.cos(theta), np.sin(theta)

        # Lower-left corner of the rotated rectangle (Rectangle rotates
        # around this corner, not around the center).
        half_w, half_h = ROCKET_WIDTH / 2.0, ROCKET_HEIGHT / 2.0
        corner_x = state.rocket_x - half_w * c + half_h * s
        corner_y = state.rocket_y - half_w * s - half_h * c

        self.rocket_patch.set_xy((corner_x, corner_y))
        self.rocket_patch.set_angle(np.degrees(theta))

        # Black dot on the nose of the rocket
        self.nose.set_data(
            [state.rocket_x + half_w * c], [state.rocket_y + half_w * s]
        )

        self.balloon_patch.set_center((state.target_x, state.target_y))

        self.text.set_text(
            f"Model:    {self.model_label}\n"
            f"Episode:  {self.episode}   step {self.steps}\n"
            f"Hits:     {self.hits} / {self.finished_episodes}\n"
            f"Reward:   {self.episode_reward:.1f}\n"
            f"Last:     {self.last_result}"
        )

    # ---------------- Animation step ----------------

    def update(self, _frame):
        if self.model is None:
            action = self.env.action_space.sample()
        else:
            action, _ = self.model.predict(self.obs, deterministic=True)

        self.obs, reward, terminated, truncated, info = self.env.step(action)
        self.steps += 1
        self.episode_reward += reward

        self.draw(self.env.sim.get_state())

        if terminated or truncated:
            self.finished_episodes += 1

            if terminated and info["success"]:
                self.hits += 1
                self.last_result = "HIT"
            elif terminated:
                self.last_result = "hit the boundary"
            else:
                self.last_result = "time limit"

            # New episode (and pick up a newer model if there is one).
            self.load_model_if_changed()
            self.obs, _ = self.env.reset()
            self.episode += 1
            self.steps = 0
            self.episode_reward = 0.0


def main():
    parser = argparse.ArgumentParser(
        description="Watch the rocket controlled by a saved SAC model."
    )
    parser.add_argument(
        "--model",
        type=str,
        default=str(DEFAULT_MODEL),
        help="Path to a saved model .zip (default: models/best/best_model.zip).",
    )
    args = parser.parse_args()

    Viewer(args.model)
    plt.show()


if __name__ == "__main__":
    main()