import os
import torch
import numpy as np
import gymnasium as gym
import pybullet as p
import imageio
from rl import TouchEnv
from tianshou.env import DummyVectorEnv
from tianshou.data import Collector, CollectStats

log_dir = 'log'
all_folders = [os.path.join(log_dir, f) for f in os.listdir(log_dir) if f.startswith('PPOExperiment')]
latest_folder = max(all_folders, key=os.path.getmtime)
subfolders = [f for f in os.listdir(latest_folder) if os.path.isdir(os.path.join(latest_folder, f))]
seed_folder = subfolders[0]
checkpoint_path = os.path.join(latest_folder, seed_folder, 'policy.pt')
print(f'Loading policy from: {checkpoint_path}')

# Setup env (no render_mode='human', we capture frames manually)
env = gym.make('TouchEnv')
data = torch.load(checkpoint_path, weights_only=False)
policy = data.policy if hasattr(data, 'policy') else data
policy.eval()

frames = []
n_episodes = 3

for ep in range(n_episodes):
    obs, info = env.reset(seed=np.random.randint(1000000))
    terminated = False
    truncated = False
    step_count = 0

    while not terminated and not truncated:
        # Capture frame from pybullet
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
            obs_tensor = torch.FloatTensor(obs).unsqueeze(0)
            if next(policy.parameters()).is_cuda:
                obs_tensor = obs_tensor.cuda()
            from tianshou.data import Batch
            result = policy(Batch(obs=obs_tensor, info={}))
            action = result.act.cpu().numpy()[0]

        obs, reward, terminated, truncated, info = env.step(action)
        step_count += 1

    print(f'Episode {ep+1}/{n_episodes}, steps: {step_count}')

# Save video
output_path = 'rl_policy_video.mp4'
imageio.mimsave(output_path, frames, fps=20)
print(f'Video saved to {output_path} ({len(frames)} frames)')
