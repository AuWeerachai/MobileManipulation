import os
import torch
import numpy as np
import gymnasium as gym
import pybullet as p
import imageio
from rl import TouchEnv

# Load imitation policy
policy = torch.load('imitation_policy.pt', weights_only=False)
policy.eval()

env = gym.make('TouchEnv')

frames = []
n_episodes = 3

for ep in range(n_episodes):
    obs, info = env.reset(seed=np.random.randint(1000000))
    terminated = False
    truncated = False
    step_count = 0

    while not terminated and not truncated:
        width, height = 640, 480
        view_matrix = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=[-0.5, 0, 0.8],
            distance=1.5,
            yaw=45,
            pitch=-30,
            roll=0,
            upAxisIndex=2
        )
        proj_matrix = p.computeProjectionMatrixFOV(
            fov=60, aspect=width/height, nearVal=0.1, farVal=100
        )
        _, _, rgb, _, _ = p.getCameraImage(width, height, view_matrix, proj_matrix)
        frame = np.array(rgb[:, :, :3], dtype=np.uint8)
        frames.append(frame)

        with torch.no_grad():
            action = policy(torch.Tensor(obs))

        obs, reward, terminated, truncated, info = env.step(action)
        step_count += 1

    print(f'Episode {ep+1}/{n_episodes}, steps: {step_count}')

output_path = 'il_policy_video.mp4'
imageio.mimsave(output_path, frames, fps=20)
print(f'Video saved to {output_path} ({len(frames)} frames)')
