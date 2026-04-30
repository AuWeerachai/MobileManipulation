import pickle
import torch
import torch.nn as nn

with open('demos.pkl', 'rb') as f:
    data = pickle.load(f)
X = torch.FloatTensor(data['X'])
y = torch.FloatTensor(data['y'])

obs_dim = X.shape[1]
act_dim = y.shape[1]

policy = nn.Sequential(
    nn.Linear(obs_dim, 64),
    nn.ReLU(),
    nn.Linear(64, 64),
    nn.ReLU(),
    nn.Linear(64, act_dim),
)

optimizer = torch.optim.Adam(policy.parameters(), lr=0.001)
loss_fn = nn.MSELoss()

for epoch in range(500):
    loss = loss_fn(policy(X), y)
    optimizer.zero_grad()
    loss.backward()
    optimizer.step()
    if (epoch + 1) % 50 == 0:
        print(f'Epoch {epoch+1}, Loss: {loss.item():.6f}')

torch.save(policy, 'imitation_policy.pt')
