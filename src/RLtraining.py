import gymnasium as gym
import numpy as np
import torch as th
from stable_baselines3 import PPO
from dataclasses import dataclass

# Constants matching boat.h and constants.h
M = 5.0
L = 1.0
I = M * L * L * (1.0/12.0)  # Moment of inertia
F_MAX = 5.0
T_MAX = 2.0
DAMPING_V = 1.8
DAMPING_W = 1.2 * L
I_MOTOR = 0.05
DAMPING_PHI = 1.5

@dataclass
class State:
    """Exactly matching the C++ State struct in boat.h"""
    x: float = 0.0    # X position
    vx: float = 0.0   # X velocity
    y: float = 0.0    # Y position
    vy: float = 0.0   # Y velocity
    theta: float = 0.0  # Hull orientation (radians)
    omega: float = 0.0  # Angular velocity of hull
    phi: float = 0.0    # Motor angle relative to hull (radians)
    dphi: float = 0.0   # Angular velocity of motor

class BoatEnv(gym.Env):
    def __init__(self):
        super().__init__()
        # Action: [F_normalized, Tau_normalized] in [-1, 1]
        self.action_space = gym.spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)
        
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(10,), dtype=np.float32)        
        
        self.state = State()
        self.dt = 0.05
        self.steps_in_episode = 0
        self.max_steps = 400
        
        # Path definition (e.g., a straight line along x-axis from -50 to 50)
        self.path_start = np.array([-50.0, 0.0])
        self.path_end = np.array([50.0, 0.0])
        self.path_direction = self.path_end - self.path_start
        self.path_length = np.linalg.norm(self.path_direction)
        self.path_direction = self.path_direction / self.path_length
        self.path_normal = np.array([-self.path_direction[1], self.path_direction[0]])

    def f(self, s: State, F: float, tau: float) -> State:
            """MATCHING C++: F*sin for vx, F*cos for vy"""
            return State(
                x=s.vx,
                vx=(F * np.sin(s.theta + s.phi) - DAMPING_V * s.vx) / M, # FIXED
                y=s.vy,
                vy=(F * np.cos(s.theta + s.phi) - DAMPING_V * s.vy) / M, # FIXED
                theta=s.omega,
                omega=(L * F * np.sin(s.phi) - DAMPING_W * s.omega) / I,
                phi=s.dphi,
                dphi=(tau - DAMPING_PHI * s.dphi) / I_MOTOR
            )

    def add_state(self, s: State, k: State, factor: float) -> State:
        """Helper for RK4 - matches C++ add lambda"""
        return State(
            x=s.x + k.x * factor,
            vx=s.vx + k.vx * factor,
            y=s.y + k.y * factor,
            vy=s.vy + k.vy * factor,
            theta=s.theta + k.theta * factor,
            omega=s.omega + k.omega * factor,
            phi=s.phi + k.phi * factor,
            dphi=s.dphi + k.dphi * factor
        )

    def update(self, F: float, tau: float):
        """Exact RK4 implementation matching boat.cpp update()"""
        # RK4 Integration Step
        k1 = self.f(self.state, F, tau)
        k2 = self.f(self.add_state(self.state, k1, self.dt/2.0), F, tau)
        k3 = self.f(self.add_state(self.state, k2, self.dt/2.0), F, tau)
        k4 = self.f(self.add_state(self.state, k3, self.dt), F, tau)

        # Final state update
        self.state.x += (self.dt/6.0) * (k1.x + 2*k2.x + 2*k3.x + k4.x)
        self.state.vx += (self.dt/6.0) * (k1.vx + 2*k2.vx + 2*k3.vx + k4.vx)
        self.state.y += (self.dt/6.0) * (k1.y + 2*k2.y + 2*k3.y + k4.y)
        self.state.vy += (self.dt/6.0) * (k1.vy + 2*k2.vy + 2*k3.vy + k4.vy)
        self.state.theta += (self.dt/6.0) * (k1.theta + 2*k2.theta + 2*k3.theta + k4.theta)
        self.state.omega += (self.dt/6.0) * (k1.omega + 2*k2.omega + 2*k3.omega + k4.omega)
        self.state.phi += (self.dt/6.0) * (k1.phi + 2*k2.phi + 2*k3.phi + k4.phi)
        self.state.dphi += (self.dt/6.0) * (k1.dphi + 2*k2.dphi + 2*k3.dphi + k4.dphi)

    def compute_path_errors(self):
        """Calculate cross-track and heading errors relative to path"""
        # Position relative to path start
        rel_pos = np.array([self.state.x, self.state.y]) - self.path_start
        
        # Along-track distance (projection onto path)
        along_track = np.dot(rel_pos, self.path_direction)
        along_track = np.clip(along_track, 0, self.path_length)
        
        # Cross-track error (perpendicular distance)
        cross_track = np.dot(rel_pos, self.path_normal)
        
        # Target point on path
        target_point = self.path_start + along_track * self.path_direction
        
        # Desired heading (direction to next point on path)
        if along_track < self.path_length - 1.0:
                    next_point = self.path_start + (along_track + 1.0) * self.path_direction
                    desired_heading = np.arctan2(next_point[0] - target_point[0], 
                                                next_point[1] - target_point[1])
        else:
            desired_heading = np.arctan2(self.path_end[0] - target_point[0],
                                        self.path_end[1] - target_point[1])

        # Heading error
        heading_error = desired_heading - self.state.theta
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))
        
        return cross_track, heading_error, along_track

    def _get_obs(self):
            """NEW 10-D VECTOR: Removed x, y, theta. Added sin/cos error."""
            cross_track, heading_error, along_track = self.compute_path_errors()
            
            # Project global velocities into path-relative velocities
            v_along = (self.state.vx * self.path_direction[0] + 
                    self.state.vy * self.path_direction[1])
            v_cross = (self.state.vx * self.path_normal[0] + 
                    self.state.vy * self.path_normal[1])

            return np.array([
                np.sin(heading_error),    # 1: Stable heading
                np.cos(heading_error),    # 2: Stable heading
                v_along,            # 3
                v_cross,            # 4
                self.state.omega,         # 5
                self.state.phi,           # 6
                self.state.dphi,          # 7
                cross_track,              # 8
                along_track / self.path_length, # 9: Normalized Progress (0-1)
                self.path_length - along_track  # 10: Distance to end
            ], dtype=np.float32)

    def reset(self, seed=None, options=None):
            super().reset(seed=seed)
            
            # --- NEW: RANDOMIZE PATH DIRECTION ---
            # Instead of always [-50, 0] to [50, 0], pick a random angle
            angle = np.random.uniform(0, 2 * np.pi)
            direction = np.array([np.sin(angle), np.cos(angle)])
            
            self.path_start = np.array([0.0, 0.0]) # Start at origin for simplicity
            self.path_end = direction * 50.0       # 50 units long in random direction
            
            self.path_direction = direction
            self.path_length = 50.0
            self.path_normal = np.array([-self.path_direction[1], self.path_direction[0]])

            # Random starting position relative to the NEW path
            # Start the boat somewhere near the beginning of the random path
            self.state = State(
                x=np.random.uniform(-2, 2),
                y=np.random.uniform(-2, 2),
                theta=angle + np.random.uniform(-0.5, 0.5), # Pointing roughly along path
                vx=0.0, vy=0.0, omega=0.0, phi=0.0, dphi=0.0
            )
            
            self.steps_in_episode = 0
            return self._get_obs(), {}

    def step(self, action):
            # ... keep action scaling and update ...
            F = np.clip(action[0], -1, 1) * F_MAX
            tau = np.clip(action[1], -1, 1) * T_MAX
            self.update(F, tau)
            
            cross_track, heading_error, along_track = self.compute_path_errors()
            
            # --- NEW STABILIZED REWARD LOGIC ---
            # Use exponential decay for cross_track so it stays between 0 and 1
            # This rewards being "close" rather than just punishing being "far"
            dist_reward = np.exp(-(cross_track**2) / 4.0) 
            
            # Alignment reward (1.0 if perfectly aligned, 0.0 if perpendicular)
            align_reward = np.cos(heading_error) 

            # Forward velocity reward (normalized by F_MAX/M)
            # FIX 2: Reward velocity ALONG the path, not strictly global X-axis
            along_track_vel = (self.state.vx * self.path_direction[0] + 
                            self.state.vy * self.path_direction[1])
            vel_reward = along_track_vel / (F_MAX / M)

            # Total step reward (now bounded roughly between -1 and +2)
            reward = (0.5 * dist_reward) + (0.5 * align_reward) + (4.0 * vel_reward)
            
            # Add the constant Step/Time Penalty (Don't waste time!)
            reward -= 0.5

            # Penalize Reversing (Action 0 is Thrust)
            # Reversing should hurt the score to force continuous forward motion
            if action[0] < 0:
                reward -= 2.0 * abs(action[0])

            # Penalize a crooked motor (phi)
            # This teaches the boat to straighten the motor when it isn't actively turning
            reward -= 0.2 * abs(self.state.phi)

            # Penalize excessive spinning/motor usage slightly
            reward -= 0.05 * (self.state.omega**2 + action[1]**2)

            self.steps_in_episode += 1
            
            terminated = False
            # If way off track, give a standardized penalty and stop
            if abs(cross_track) > 5.0: 
                terminated = True
                reward = -20.0 # Standardized penalty
            
            # If reached the end
            elif along_track >= self.path_length:
                terminated = True
                reward = 100.0 # A clear, meaningful "Win" signal
            
            if self.steps_in_episode >= self.max_steps:
                terminated = True
                
            return self._get_obs(), float(reward), terminated, False, {}

# Training code
env = BoatEnv()
# Increased n_steps for more stable gradient estimates
model = PPO("MlpPolicy", env, verbose=1, 
            learning_rate=1e-4, # Lower learning rate for stability
            n_steps=4096,       # Doubled from 2048
            batch_size=128, 
            gae_lambda=0.95, 
            gamma=0.99)
model.learn(total_timesteps=500000)
model.save("boat_policy")

# Export to ONNX
class OnnxWrapper(th.nn.Module):
    def __init__(self, policy):
        super().__init__()
        self.policy = policy
    def forward(self, observation):
        return self.policy(observation)[0]

onnx_model = OnnxWrapper(model.policy)
dummy_input = th.randn(1, 10)  # Match observation space
th.onnx.export(onnx_model, dummy_input, "boat_policy.onnx",
               input_names=['input'], output_names=['output'])