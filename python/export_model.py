
"""
Export the trained Stable-Baselines3 DQN policy to ONNX.

Input:
    A normalized observation with shape (1, 15).

Output:
    An integer in range [0, 8], corresponding to one of the nine
    possible discrete actions.
"""

from pathlib import Path

import onnx
import torch
from stable_baselines3 import DQN


# Project paths
PROJECT_ROOT = Path(__file__).resolve().parent.parent

MODEL_PATH = PROJECT_ROOT / "models" / "dqn_best" / "best_model.zip"
OUTPUT_PATH = PROJECT_ROOT / "models" / "dqn_rocket_actor.onnx"


class OnnxableDQNPolicy(torch.nn.Module):
    """Export the deterministic DQN policy as an ONNX model."""

    def __init__(self, q_network):
        super().__init__()
        self.q_network = q_network

    def forward(self, observation):
        # Calculate the Q-value for each of the nine possible actions.
        q_values = self.q_network(observation)

        # Select the action with the highest Q-value.
        action = torch.argmax(q_values, dim=1)

        return action


def main():

    if not MODEL_PATH.is_file():
        raise FileNotFoundError(
            f"Trained model not found: {MODEL_PATH}"
        )

    # Load the trained DQN model on CPU for export.
    model = DQN.load(str(MODEL_PATH), device="cpu")
    model.policy.eval()

    print(f"Loaded model from: {MODEL_PATH}")

    # Confirm that this model matches our environment.
    observation_size = model.observation_space.shape
    action_size = model.action_space.n

    if observation_size != (15,):
        raise ValueError(
            f"Expected 15 observations, got {observation_size}"
        )

    if action_size != 9:
        raise ValueError(
            f"Expected 9 discrete actions, got {action_size}"
        )

    # Extract the DQN Q-network.
    dqn_network = model.policy.q_net

    # Wrap the network to export the greedy action instead of Q-values.
    policy = OnnxableDQNPolicy(dqn_network).eval()

    example_observation = torch.zeros(
        (1, model.observation_space.shape[0]),
        dtype=torch.float32,
    )

    # Create the output directory if it does not already exist.
    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)

    with torch.no_grad():
        torch.onnx.export(
            policy,
            example_observation,
            str(OUTPUT_PATH),
            input_names=["observation"],
            output_names=["action"],
            opset_version=17,
            dynamo=False,
        )

    print(f"ONNX model exported to: {OUTPUT_PATH}")

    # Check that the exported ONNX file is structurally valid.
    onnx_model = onnx.load(str(OUTPUT_PATH))
    onnx.checker.check_model(onnx_model)

    print(f"Export successful: {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
