
"""
Test whether the exported DQN ONNX model produces the same
discrete actions as the original Stable-Baselines3 DQN model.
"""

from pathlib import Path

import numpy as np
import onnxruntime as ort
from stable_baselines3 import DQN


# Project paths
PROJECT_ROOT = Path(__file__).resolve().parent.parent

MODEL_PATH = PROJECT_ROOT / "models" / "dqn_best" / "best_model.zip"
ONNX_PATH = PROJECT_ROOT / "models" / "dqn_rocket_actor.onnx"


def main():

    # Check that both models exist
    if not MODEL_PATH.is_file():
        raise FileNotFoundError(
            f"Trained DQN model not found: {MODEL_PATH}"
        )

    if not ONNX_PATH.is_file():
        raise FileNotFoundError(
            f"ONNX model not found: {ONNX_PATH}"
        )

    # Load the original DQN model
    model = DQN.load(str(MODEL_PATH), device="cpu")

    # Confirm that the model matches our environment
    if model.observation_space.shape != (15,):
        raise ValueError(
            f"Expected 15 observations, got {model.observation_space.shape}"
        )

    if model.action_space.n != 9:
        raise ValueError(
            f"Expected 9 discrete actions, got {model.action_space.n}"
        )

    # Load the exported ONNX model
    session = ort.InferenceSession(
        str(ONNX_PATH),
        providers=["CPUExecutionProvider"],
    )

    # Test several observations with the correct shape and dtype
    rng = np.random.default_rng(42)

    observations = rng.uniform(
        low=-1.0,
        high=1.0,
        size=(5, model.observation_space.shape[0]),
    ).astype(np.float32)

    for i, observation in enumerate(observations):

        # Original DQN deterministic action
        action_sb3, _ = model.predict(
            observation,
            deterministic=True,
        )

        action_sb3 = int(np.asarray(action_sb3).reshape(-1)[0])

        # ONNX deterministic action
        action_onnx = session.run(
            ["action"],
            {
                "observation": observation.reshape(1, -1)
            },
        )[0]

        action_onnx = int(np.asarray(action_onnx).reshape(-1)[0])

        print(f"\nTest {i + 1}")
        print("SB3 action: ", action_sb3)
        print("ONNX action:", action_onnx)

        # Confirm that both actions are valid
        assert 0 <= action_sb3 <= 8, (
            f"Invalid SB3 action: {action_sb3}"
        )

        assert 0 <= action_onnx <= 8, (
            f"Invalid ONNX action: {action_onnx}"
        )

        # Both models must select exactly the same action
        np.testing.assert_equal(
            action_onnx,
            action_sb3,
        )

    print("\nSUCCESS: ONNX actions match the original DQN model!")


if __name__ == "__main__":
    main()
