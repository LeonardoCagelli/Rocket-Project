"""
Rocket Interception - DQN Training Pipeline

Expected environment:
- Observation: 15-dimensional float32 vector.
- Action: one integer in [0, 8], selecting one of nine control changes.

"""

import argparse
from pathlib import Path

from stable_baselines3 import DQN
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.monitor import Monitor

from rocket_env import RocketEnv

from collections import Counter

PROJECT_ROOT = Path(__file__).resolve().parent.parent

MODELS_DIR = PROJECT_ROOT / "models"
CHECKPOINTS_DIR = MODELS_DIR / "dqn_checkpoints"
BEST_MODEL_DIR = MODELS_DIR / "dqn_best"

LOGS_DIR = PROJECT_ROOT / "logs"
TENSORBOARD_DIR = LOGS_DIR / "dqn_rocket"
EVALUATION_LOG_DIR = LOGS_DIR / "dqn_evaluation"




# This function helps to evaluate the performance of the model
def evaluate_policy(model, env, n_episodes=100):
    """
    Evaluate a trained DQN policy over complete episodes.

    Uses deterministic=True to evaluate the learned policy
    without epsilon-greedy exploration.
    """

    results = Counter()

    episode_rewards = []
    episode_lengths = []

    successful_steps = 0
    number_successful_steps = 0

    print("\n")
    print("=" * 55)
    print("       FINAL POLICY EVALUATION")
    print("=" * 55)

    for episode in range(n_episodes):

        observation, info = env.reset()

        done = False
        episode_reward = 0.0
        episode_length = 0

        terminal_reason = "unknown"
        success = False

        while not done:

            # Select action using the trained policy.
            action, _ = model.predict(
                observation,
                deterministic=True
            )

            observation, reward, terminated, truncated, info = (
                env.step(action)
            )

            episode_reward += float(reward)
            episode_length += 1

            done = terminated or truncated

            if done:

                success = bool(info.get("success", False))

                terminal_reason = info.get(
                    "termination reason",
                    "unknown"
                )

                # Fallback classification if the reason is missing.
                if success:
                    terminal_reason = "success"
                    successful_steps += episode_length
                    number_successful_steps+=1

                elif truncated and terminal_reason in ("none", "unknown"):
                    terminal_reason = "timeout"

                elif terminated and terminal_reason in ("none", "unknown"):
                    if info.get("boundary_failure", False):
                        terminal_reason = "boundary"
                    else:
                        terminal_reason = "other"

        # Store episode statistics.
        results[terminal_reason] += 1

        if success:
            results["success"] += 0  # Success already counted by reason.

        episode_rewards.append(episode_reward)
        episode_lengths.append(episode_length)

        print(
            f"Episode {episode + 1:3d}/{n_episodes} | "
            f"Reward: {episode_reward:8.2f} | "
            f"Steps: {episode_length:4d} | "
            f"Reason: {terminal_reason}"
        )

    # --------------------------------------------------
    # Summary
    # --------------------------------------------------

    successful_episodes = results["success"]

    print("\n")
    print("=" * 55)
    print("             EVALUATION SUMMARY")
    print("=" * 55)

    print(f"Total episodes:       {n_episodes}")

    print(
        f"Successful episodes:  {successful_episodes} "
        f"({100 * successful_episodes / n_episodes:.2f}%)"
    )

    print("\nTermination reasons:")

    for reason in ("success", "boundary", "other", "timeout", "unknown"):
        count = results[reason]

        print(
            f"  {reason:12s}: {count:3d} "
            f"({100 * count / n_episodes:.2f}%)"
        )

    print("\nPerformance:")

    print(
        f"Average reward:       "
        f"{sum(episode_rewards) / n_episodes:.2f}"
    )

    print(
        f"Average episode len: "
        f"{sum(episode_lengths) / n_episodes:.2f}"
    )

    print(
        f"Average steps for successful episode: "
        f"{successful_steps / number_successful_steps:.2f}"
    )

    print("=" * 55)

    return results





# ===================================
#               MAIN
# ===================================


def main():
    parser = argparse.ArgumentParser(
        description="Train a rocket interception controller using DQN."
    )
    parser.add_argument(
        "--timesteps",
        type=int,
        default=200_000,
        help="Total environment steps (default: 200000).",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Random seed for reproducibility.",
    )
    args = parser.parse_args()

    if args.timesteps <= 0:
        parser.error("--timesteps must be greater than zero.")

    for directory in (
        MODELS_DIR,
        # CHECKPOINTS_DIR,
        BEST_MODEL_DIR,
        TENSORBOARD_DIR,
        EVALUATION_LOG_DIR,
    ):
        directory.mkdir(parents=True, exist_ok=True)

    print("=" * 60)
    print(" Rocket Interception - DQN Training")
    print("=" * 60)
    print(f"Project root:   {PROJECT_ROOT}")
    print(f"Training steps: {args.timesteps}")
    print(f"Random seed:    {args.seed}")
    print("Action space:   Discrete(9)")
    print()

    print("Checking the Gymnasium environment...")
    check_env(RocketEnv(), warn=True)
    print("Environment check passed.")
    print()

    # These keys must exist in the final info dictionary of each episode.
    # Change the strings here if you rename the corresponding keys in rocket_env.py.
    monitor_info = ("success", "termination reason")

    train_env = Monitor(RocketEnv(), info_keywords=monitor_info)
    eval_env = Monitor(RocketEnv(), info_keywords=monitor_info)

    checkpoint_frequency = 50000 
    evaluation_frequency = 25000 

    checkpoint_callback = CheckpointCallback(
        save_freq=checkpoint_frequency,
        save_path=str(CHECKPOINTS_DIR),
        name_prefix="dqn_rocket",
        save_replay_buffer=True,
        save_vecnormalize=False,
    )

    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=str(BEST_MODEL_DIR),
        log_path=str(EVALUATION_LOG_DIR),
        eval_freq=evaluation_frequency,
        n_eval_episodes=15,
        deterministic=True,
        render=False,
        verbose=1,  # =1 means that it print the log 
    )

    model = DQN(
        policy="MlpPolicy",  
        env=train_env,        
        learning_rate=1e-4,   
        buffer_size=100_000,  
        learning_starts=500,    # Number of environment steps collected before the first neural network update.
        batch_size=128,       
        tau=1.0,                # Amount of target-network weight updating. With 1.0, target-network updates are complete replacements rather than gradual updates.
        gamma=0.995,             # Discount factor for future rewards. A value close to 1 makes the agent consider long-term consequences of its actions.
        train_freq=4,           # The agent performs learning updates every 4 environment steps.
        gradient_steps=1,       # Number of neural network gradient updates performed each time training is triggered. Combined with train_freq=4, this gives approximately one update per 4 environment steps.
        target_update_interval=1_000, # Updates the target Q-network every 1,000 environment steps.
        exploration_fraction=0.50,    # Fraction of the total training duration over which the exploration rate decreases.
        exploration_initial_eps=1.0,
        exploration_final_eps=0.10,
        policy_kwargs=dict(net_arch=[256, 256]),
        # policy_kwargs=dict(net_arch=[128, 128]), # two hidden layers of 128 neurons
        seed=args.seed,        
        device="auto",         # Automatically selects the available computation platform (CPU or GPU)
        verbose=1,             # Displays training information in the terminal.
        tensorboard_log=str(TENSORBOARD_DIR),
                            # Directory where TensorBoard training metrics
                            # are saved, allowing you to monitor learning.
    )

    print("Starting DQN training...")
    print(f"Checkpoints: {CHECKPOINTS_DIR}")
    print(f"Best model:  {BEST_MODEL_DIR}")
    print(f"TensorBoard: {TENSORBOARD_DIR}")
    print()

    try:
        model.learn(
            total_timesteps=args.timesteps,
            callback=[eval_callback],       # i removed checkpoints callback
            log_interval=1000,
            tb_log_name="DQN_Rocket",
        )

        final_model_path = MODELS_DIR / "dqn_rocket_final"
        model.save(str(final_model_path))
        # End of training process 


        # --------------------------------------------------
        # Final evaluation
        # --------------------------------------------------

        evaluation_env = RocketEnv()
        evaluate_policy(model=model, env=evaluation_env, n_episodes=500)
        evaluation_env.close()


        print()
        print("=" * 60)
        print(" Training finished")
        print("=" * 60)
        print(f"Final model:       {final_model_path}.zip")
        print(f"Best model folder: {BEST_MODEL_DIR}")
        print(f"Checkpoints:       {CHECKPOINTS_DIR}")
        print(f"TensorBoard logs:  {TENSORBOARD_DIR}")
        print(f"Evaluation logs:   {EVALUATION_LOG_DIR}")

        # print("\n\n\n- - - - - - - - - - - - - - - - - - -")
        # print("Training Env")
        # train_env.print_total_steps_count()
        # print("\nEvalutation Env")
        # eval_env.print_total_steps_count()

    finally:

        train_env.close()
        eval_env.close()


if __name__ == "__main__":
    main()
