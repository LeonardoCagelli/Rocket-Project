
import math

import gymnasium as gym
import numpy as np
from gymnasium import spaces

import sys
from pathlib import Path


# These commands helps to import rocket_sim
PROJECT_ROOT = Path(__file__).resolve().parent.parent
BUILD_DIR = PROJECT_ROOT / "build-rl"

sys.path.insert(0, str(BUILD_DIR))

import rocket_sim

import random # used to print reward log


# --------------------------------------------------
# Physical parameters from src/config.h
# --------------------------------------------------

WORLD_WIDTH = 1000.0
WORLD_HEIGHT = 800.0

ROCKET_WIDTH = 80.0
ROCKET_HEIGHT = 20.0

BALLOON_RADIUS = 15.0

MAX_THRUST = 5000.0
MAX_GIMBAL_ANGLE = 0.0005

SIMULATION_FREQUENCY = 60.0
DT = 1.0 / SIMULATION_FREQUENCY

MAX_EPISODE_STEPS = 900 # 25 seconds

# --------------------------------------------------
# Normalisation parameters
# --------------------------------------------------

# Velocity normalisation 
VEL_NORMALISATION = 100.0
OMEGA_NORMALISATION = 10.0


# Discrete Reinfocement Learning Algorithm parameters

INITIAL_THRUST = 0.0
INITIAL_GIMBAL_ANGLE = 0.0

DELTA_THRUST_PERCENTAGE = 0.01
DELTA_GIMBAL_PERCENTAGE = 0.05
DELTA_THRUST = MAX_THRUST * DELTA_THRUST_PERCENTAGE
DELTA_GIMBAL_ANGLE = MAX_GIMBAL_ANGLE * DELTA_GIMBAL_PERCENTAGE



class RocketEnv(gym.Env):
    """
    Gymnasium environment for rocket interception.

    C++:
        Physics, dynamics, collision detection and simulator state.

    Python:
        Observations, action mapping, rewards and episode limits.

    Intended RL algorithm:
        Stable-Baselines3 SAC (continuous actions).
    """

    metadata = {"render_modes": []}

    def __init__(self, max_episode_steps=MAX_EPISODE_STEPS):
        super().__init__()

        self.sim = rocket_sim.Simulator()
        self.max_episode_steps = max_episode_steps

        # --------------------------------------------------
        # Action space
        # --------------------------------------------------
        # Discrete outputs a 9-dimensional space which represents one of the nine possible actions which are:
			# 0 increase thrust and gimbal angle
			# 1 increase thrust and no gimbal angle variation
			# 2 increase thrust and decrease gimbal angle 
			# 3 no thrust variation and increase gimbal angle
			# 4 no thrust variation and no gimbal angle variation
			# 5 no thrust variation and decrease gimbal angle 
			# 6 decrease thrust and increase gimbal angle
			# 7 decrease thrust and no gimbal angle variation
			# 8 decrease thrust and gimbal angle 

        self.ACTION_TABLE = [
                (+1, +1),  # 0: increase thrust, increase gimbal
                (+1,  0),  # 1: increase thrust, no gimbal change
                (+1, -1),  # 2: increase thrust, decrease gimbal
                ( 0, +1),  # 3: no thrust change, increase gimbal
                ( 0,  0),  # 4: no change to either
                ( 0, -1),  # 5: no thrust change, decrease gimbal
                (-1, +1),  # 6: decrease thrust, increase gimbal
                (-1,  0),  # 7: decrease thrust, no gimbal change
                (-1, -1),  # 8: decrease thrust, decrease gimbal
            ]
                    
        # The model outputs an integer from 0 to 8 representing one of the nine possible actions 
        self.action_space = spaces.Discrete(9)

        # --------------------------------------------------
        # Observation space: 15 values
        # --------------------------------------------------
        # 0: target-relative x position, normalized
        # 1: target-relative y position, normalized
        # 2: squared distance, normalized
        # 3: target-relative x velocity, normalized
        # 4: target-relative y velocity, normalized
        # 5: rocket x velocity, normalized
        # 6: rocket y velocity, normalized
        # 7: target x velocity, normalized
        # 8: target y velocity, normalized
        # 9: sine of rocket orientation
        # 10: cosine of rocket orientation
        # 11: rocket angular velocity, normalized
        # 12: rocket thrust 
        # 13: sine of rocket gimbal angle
        # 14: cosine of rocket gimbal angle


        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(15,),
            dtype=np.float32,
        )

        self.steps = 0

        # preious distance and previous speed are used to calculate the reward 
        self.previous_distance = 0.0
        self.previous_alignment = 0.0

        # Peculiar parameters 
        self.initial_thrust = INITIAL_THRUST
        self.initial_gimbal_angle = INITIAL_GIMBAL_ANGLE
        self.delta_thrust = DELTA_THRUST
        self.delta_gimbal_angle = DELTA_GIMBAL_ANGLE
        self.current_thrust = INITIAL_THRUST
        self.current_gimbal_angle = INITIAL_GIMBAL_ANGLE

        self.map = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.total_episode = 1

        # REWARD parameters 
        self.distance_normalisation = 5.0
        self.time_pressure = 0.08
        self.omega_coeff = 0.01
        self.alignment_coeff = 0.01


        self.total_steps_counts = 0

        self.print_reward_parameters()



    def print_reward_parameters(self):
            print("\n\n==================================")
            print("===== REWARD PARAMETERS:")
            print(f"===== Distance normalisation: {self.distance_normalisation}")
            print(f"===== Time pressure: {self.time_pressure}")
            print(f"===== Omega coefficient: {self.omega_coeff}")
            print(f"===== Alignment coefficient: {self.alignment_coeff}")
            print("==================================\n\n")



    def print_total_steps_count(self):
        print(f"   {self.total_steps_counts}   ")

        
    # --------------------------------------------------
    # Observation
    # --------------------------------------------------

    # Observation is a 15 - dimensional space
    def _get_observation(self, state):
        """
        Convert the C++ State into the observation vector
        consumed by the SAC policy.

        The C++ State itself is not modified.
        """

        relative_x = state.target_x - state.rocket_x
        relative_y = state.target_y - state.rocket_y

        relative_vx = state.target_vx - state.rocket_vx
        relative_vy = state.target_vy - state.rocket_vy

        observation = np.array(
            [
                relative_x / WORLD_WIDTH,
                relative_y / WORLD_HEIGHT,

                state.square_distance
                / (WORLD_WIDTH**2 + WORLD_HEIGHT**2),

                relative_vx / 100.0,
                relative_vy / 100.0,

                state.rocket_vx / 100.0,
                state.rocket_vy / 100.0,

                state.target_vx / 100.0,
                state.target_vy / 100.0,

                math.sin(state.orientation),
                math.cos(state.orientation),

                state.omega / 10.0,
                state.thrust / MAX_THRUST,
                math.sin(state.gimbal_angle / MAX_GIMBAL_ANGLE),
                math.cos(state.gimbal_angle / MAX_GIMBAL_ANGLE),
            ],
            dtype=np.float32,
        )

        return observation

    # --------------------------------------------------
    # Helper functions
    # --------------------------------------------------

    # This function is no longer used since it has been replaced with the below function which uses the poisiton 
    # of the head instead of the center of the rocket
    def _distance_to_target(self, state):
        """Euclidean distance between the object centers."""

        dx = state.target_x - state.rocket_x
        dy = state.target_y - state.rocket_y

        return math.hypot(dx, dy)


    def normalisation_function(self, x):
        return abs((1 - math.exp(-x)) / (1 + math.exp(-x)))


    def compute_sped(self, state):
        return math.hypot(state.rocket_vx, state.rocket_vy)
        
    

    def _distance_head_to_target(self, state):
        lever_arm = ROCKET_WIDTH / 2.0

        head_x = (
            state.rocket_x
            + lever_arm * math.cos(state.orientation)
        )
        head_y = (
            state.rocket_y
            + lever_arm * math.sin(state.orientation)
        )

        return math.hypot(
            state.target_x - head_x,
            state.target_y - head_y,
        )

    def _check_interception(self, state):
        """
        Reproduce the interception geometry in Simulator.cpp.

        The rocket is approximated by a circle, and its front
        end is used as the collision reference point.
        """

        lever_arm = ROCKET_WIDTH / 2.0

        head_x = (
            state.rocket_x
            + lever_arm * math.cos(state.orientation)
        )

        head_y = (
            state.rocket_y
            + lever_arm * math.sin(state.orientation)
        )

        dx = head_x - state.target_x
        dy = head_y - state.target_y

        distance = math.hypot(dx, dy)

        collision_distance = (
            (ROCKET_WIDTH / 2.0) + BALLOON_RADIUS
        )

        return distance < collision_distance

    def _check_boundary_failure(self, state):
        """Reproduce the rotated-rocket boundary test in C++."""

        half_width = ROCKET_WIDTH / 2.0
        half_height = ROCKET_HEIGHT / 2.0
        theta = state.orientation

        extent_x = (
            abs(half_width * math.cos(theta))
            + abs(half_height * math.sin(theta))
        )

        extent_y = (
            abs(half_height * math.cos(theta))
            + abs(half_width * math.sin(theta))
        )

        return (
            state.rocket_x < extent_x
            or state.rocket_y < extent_y
            or state.rocket_x > WORLD_WIDTH - extent_x
            or state.rocket_y > WORLD_HEIGHT - extent_y
        )

    # --------------------------------------------------
    # Reset
    # --------------------------------------------------

    def reset(self, seed=None, options=None):
        """Start a new episode."""

        super().reset(seed=seed)

        # print("\n\n===================================================")
        # print(f"==== Episode END (reset function call) => number {self.total_episode} ====")
        # print(f"Number of steps: {self.steps}")

        # print(f"==== SELECTED ACTIONS: ====")
        # print(f"==== 0: {self.map[0]}")
        # print(f"==== 1: {self.map[1]}")
        # print(f"==== 2: {self.map[2]}")
        # print(f"==== 3: {self.map[3]}")
        # print(f"==== 4: {self.map[4]}")
        # print(f"==== 5: {self.map[5]}")
        # print(f"==== 6: {self.map[6]}")
        # print(f"==== 7: {self.map[7]}")
        # print(f"==== 8: {self.map[8]}")
        # print(f"\nFinal thrust: {self.current_thrust} | Final gimabl angle: {self.current_gimbal_angle}")
        # print("===================================================\n")

        self.sim.reset()
        self.steps = 0
        self.map = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.total_episode += 1


        self.current_thrust = self.initial_thrust
        self.current_gimbal_angle = self.initial_gimbal_angle

        state = self.sim.get_state()

        self.previous_distance = self._distance_head_to_target(state)

        observation = self._get_observation(state)

        info = {
            "distance": self.previous_distance,
            "success": False,
        }
    
        return observation, info



    # Function to compute the alignment between the head of the rocket and the target
    def _alignment_penalty(self, state):
        """Per-step cost in [-alignment_coeff, 0]; zero only when the nose points at the balloon."""
        lever_arm = ROCKET_WIDTH / 2.0
        head_x = state.rocket_x + lever_arm * math.cos(state.orientation)
        head_y = state.rocket_y + lever_arm * math.sin(state.orientation)

        desired_heading = math.atan2(state.target_y - head_y, state.target_x - head_x)
        angle_error = desired_heading - state.orientation
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        return -self.alignment_coeff * (1.0 - math.cos(angle_error)) / 2.0



    # --------------------------------------------------
    # Step                                              
    # --------------------------------------------------



    def step(self, action):
        """Apply an action and advance the C++ simulator."""
        
        self.steps += 1

        # Core of the new reinforcement learning agent ==> use of action table to select the action 
        action = int(action)

        self.map[action] += 1 # take track of the episode selected actions
        thrust_direction, gimbal_direction = self.ACTION_TABLE[action]

        # print(f"thrust and gimbal angle directions: {thrust_direction} , {gimbal_direction}")

        # Update the commanded controls incrementally.
        self.current_thrust += (
            thrust_direction * self.delta_thrust
        )

        self.current_gimbal_angle += (
            gimbal_direction * self.delta_gimbal_angle
        )

        # Keep controls inside their physical limits.
        self.current_thrust = float(
            np.clip(self.current_thrust, 0.0, MAX_THRUST)
        )

        self.current_gimbal_angle = float(
            np.clip(
                self.current_gimbal_angle,
                -MAX_GIMBAL_ANGLE,
                MAX_GIMBAL_ANGLE,
            )
        )

        # Send the current physical control values to C++.
        cpp_action = rocket_sim.Action(
            self.current_thrust,
            self.current_gimbal_angle,
        )

        result = self.sim.step(cpp_action, DT)

        state = result.state

        distance = self._distance_head_to_target(state)

        # --------------------------------------------------
        # Reward
        # --------------------------------------------------
        reward = 0.0

        # 1. Progress toward the balloon
        distance_reward = (self.previous_distance - distance) / self.distance_normalisation
        reward += distance_reward

        # print(f"\n\nSPEED NORMALISATION: {reward}\n\n")

        # 2. Time pressure
        reward -= self.time_pressure

        # 3. Penalise high angular velocity
        # state.omega is expressed in rad/s.

        # if state.omega > 1:
        #     print(f"ATTENZIONE ATTENZIONE ATTENZIONE: {state.omega}")

        omega_normalised = self.normalisation_function(state.omega)
        omega_reward = self.omega_coeff * omega_normalised
        # reward -= omega_reward

        # 4. Reward heading alignment with the balloon

        alignment_reward = self._alignment_penalty(state)
        # reward += alignment_reward



        # if random.random() < 0.0001:
        #     print("\n\n===================================")
        #     print("===== PARTIAL REWARD:")
        #     print(f"===== Distance      : {round(distance_reward, 5):<10} ")
        #     print(f"===== Time pressure : {round(self.time_pressure, 5):<10} ")
        #     #print(f"===== Omega         : {round(omega_reward, 5):<10} - {state.omega}")
        #     print(f"===== Alignment     : {round(alignment_reward, 5):<10} ")
        #     #print(f"===== Speed         : {round(speed_reward, 5):<10} ")
        #     print("===================================\n\n")

        # --------------------------------------------------
        # Terminal outcome
        # --------------------------------------------------

        success = self._check_interception(state)
        boundary_failure = self._check_boundary_failure(state)

        terminated = bool(result.terminated)
        truncated = False
        terminal_reason = "none"

        if terminated or self.steps >= self.max_episode_steps:
            if success:
                time_left = 1.0 - (self.steps / self.max_episode_steps)
                reward += 100.0 + 300.0 * time_left
                terminal_reason = "success"

            elif boundary_failure:
                reward -= 0.0
                terminal_reason = "boundary"

            elif self.steps >= self.max_episode_steps:
                truncated = True
                reward -= 0.0
                terminal_reason = "timeout"

            else:
                reward -= 50.0
                terminal_reason = "other"

        # --------------------------------------------------
 
        self.previous_distance = distance

        observation = self._get_observation(state)

        info = {
            "distance": distance,
            "success": success,
            "boundary_failure": boundary_failure,
            "thrust": self.current_thrust,
            "gimbal_angle": self.current_gimbal_angle,
            "episode_steps": self.steps,
            "termination reason": terminal_reason,
        }

        # if self.steps % 100 == 0:
        #     print(
        #         f"Step: {self.steps} | "
        #         f"Action ID: {action} | "
        #         f"Thrust direction: {thrust_direction} | "
        #         f"Gimbal direction: {gimbal_direction} | "
        #         f"Thrust: {self.current_thrust:.1f} | "
        #         f"Gimbal: {self.current_gimbal_angle:.5f}",
        #         flush=True
        #     )

        self.total_steps_counts += 1

        return (
            observation,
            float(reward),
            terminated,
            truncated,
            info,
        )



