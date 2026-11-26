# Step 1: Add modules to provide access to specific libraries and functions
import os  # Module provides functions to handle file paths, directories, environment variables
import sys  # Module provides access to Python-specific system parameters and functions
import random
import numpy as np
import matplotlib.pyplot as plt  # Visualization

# Step 1.1 : (Additional) Imports for Deep Q-Learning...................J_Edit_11
# Deep Learning imports (PyTorch + TorchVision)
import torch
import torch.nn as nn
import torch.optim as optim
import torchvision.transforms as T
import torchvision.models as models

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# Step 2: Establish path to SUMO (SUMO_HOME)
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

# Step 3: Add Traci module to provide access to specific libraries and functions
import traci  # Static network information (such as reading and analyzing network files)

# Step 4: Define Sumo configuration
Sumo_config = [
    'sumo',
    '-c', 'RL.sumocfg',                # Make changes to file pointing. use r"...."
    '--step-length', '0.10',
    '--delay', '1000',
    '--lateral-resolution', '0'
]

# Step 5: Open connection between SUMO and Traci
traci.start(Sumo_config)
# traci.gui.setSchema("View #0", "real world")

# -------------------------
# Step 6: Define Variables
# -------------------------

# Variables for RL State (queue lengths from detectors and current phase)
q_EB_0 = 0
q_EB_1 = 0
q_EB_2 = 0
q_SB_0 = 0
q_SB_1 = 0
q_SB_2 = 0
current_phase = 0

# ---- Reinforcement Learning Hyperparameters ----
TOTAL_STEPS = 10000    # The total number of simulation steps for continuous (online) training.

ALPHA = 0.1            # Learning rate (α) between[0, 1]
GAMMA = 0.9            # Discount factor (γ) between[0, 1]
EPSILON = 0.1          # Exploration rate (ε) between[0, 1]

ACTIONS = [0, 1]       # The discrete action space (0 = keep phase, 1 = switch phase)

# ---- Additional Stability Parameters ----
MIN_GREEN_STEPS = 100
last_switch_step = -MIN_GREEN_STEPS

# -------------------------
# Step 7: Define Functions & PyTorch Model
# -------------------------

class TrafficDQN(nn.Module):
    """
    Feedforward neural network that approximates Q-values in PyTorch.
    """
    def __init__(self, state_size, action_size):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_size, 24),
            nn.ReLU(),
            nn.Linear(24, 24),
            nn.ReLU(),
            nn.Linear(24, action_size)
        )

    def forward(self, x):
        return self.net(x)


def to_tensor(state_tuple):
    """
    Convert the state tuple into a 2D PyTorch tensor (shape: [1, state_size]) on `device`.
    """
    arr = np.array(state_tuple, dtype=np.float32).reshape((1, -1))
    return torch.tensor(arr, dtype=torch.float32, device=device)


def predict_q_values(state_tuple):
    """
    Run forward pass without gradient tracking and return 1D NumPy array of Q-values.
    """
    dqn_model.eval()
    with torch.no_grad():
        state_t = to_tensor(state_tuple)
        q_vals = dqn_model(state_t).squeeze(0).cpu().numpy()
    return q_vals


# Create the PyTorch DQN model, optimizer, and loss function
state_size = 7   # (q_EB_0, q_EB_1, q_EB_2, q_SB_0, q_SB_1, q_SB_2, current_phase)
action_size = len(ACTIONS)
dqn_model = TrafficDQN(state_size, action_size).to(device)
optimizer = optim.Adam(dqn_model.parameters(), lr=0.001)
criterion = nn.MSELoss()


def get_max_Q_value_of_state(s):  # 1. Objective Function
    Q_values = predict_q_values(s)
    return float(np.max(Q_values))


def get_reward(state):  # 2. Constraint 2
    """
    Simple reward function:
    Negative of total queue length to encourage shorter queues.
    """
    total_queue = sum(state[:-1])  # Exclude the current_phase element
    reward = -float(total_queue)
    return reward


def get_state():  # 3&4. Constraint 3 & 4
    global q_EB_0, q_EB_1, q_EB_2, q_SB_0, q_SB_1, q_SB_2, current_phase

    # Detector IDs for Node1-2-EB
    detector_Node1_2_EB_0 = "Node1_2_EB_0"
    detector_Node1_2_EB_1 = "Node1_2_EB_1"
    detector_Node1_2_EB_2 = "Node1_2_EB_2"

    # Detector IDs for Node2-7-SB
    detector_Node2_7_SB_0 = "Node2_7_SB_0"
    detector_Node2_7_SB_1 = "Node2_7_SB_1"
    detector_Node2_7_SB_2 = "Node2_7_SB_2"

    # Traffic light ID
    traffic_light_id = "Node2"

    # Get queue lengths from each detector
    q_EB_0 = get_queue_length(detector_Node1_2_EB_0)
    q_EB_1 = get_queue_length(detector_Node1_2_EB_1)
    q_EB_2 = get_queue_length(detector_Node1_2_EB_2)

    q_SB_0 = get_queue_length(detector_Node2_7_SB_0)
    q_SB_1 = get_queue_length(detector_Node2_7_SB_1)
    q_SB_2 = get_queue_length(detector_Node2_7_SB_2)

    # Get current phase index
    current_phase = get_current_phase(traffic_light_id)

    return (q_EB_0, q_EB_1, q_EB_2, q_SB_0, q_SB_1, q_SB_2, current_phase)


def apply_action(action, tls_id="Node2"):  # 5. Constraint 5
    """
    Executes the chosen action on the traffic light, combining:
    - Min Green Time check
    - Switching to the next phase if allowed
    """
    global last_switch_step

    if action == 0:
        return
    elif action == 1:
        if current_simulation_step - last_switch_step >= MIN_GREEN_STEPS:
            program = traci.trafficlight.getAllProgramLogics(tls_id)[0]
            num_phases = len(program.phases)
            next_phase = (get_current_phase(tls_id) + 1) % num_phases
            traci.trafficlight.setPhase(tls_id, next_phase)
            last_switch_step = current_simulation_step


def update_Q_table(old_state, action, reward, new_state):  # 6. Constraint 6
    """
    Single-step gradient update in PyTorch approximating the Q-value target.
    """
    Q_values_old = predict_q_values(old_state).copy()
    Q_values_new = predict_q_values(new_state)
    best_future_q = float(np.max(Q_values_new))

    target_q_values = Q_values_old.copy()
    target_q_values[action] = (
        Q_values_old[action]
        + ALPHA * (reward + GAMMA * best_future_q - Q_values_old[action])
    )

    dqn_model.train()
    state_t = to_tensor(old_state)
    target_t = torch.tensor(target_q_values.reshape((1, -1)), dtype=torch.float32, device=device)

    optimizer.zero_grad()
    pred_t = dqn_model(state_t)
    loss = criterion(pred_t, target_t)
    loss.backward()
    optimizer.step()


def get_action_from_policy(state):  # 7. Constraint 7
    """
    Epsilon-greedy strategy using the PyTorch DQN's predicted Q-values.
    """
    if random.random() < EPSILON:
        return random.choice(ACTIONS)
    Q_values = predict_q_values(state)
    return int(np.argmax(Q_values))


def get_queue_length(detector_id):  # 8. Constraint 8
    return traci.lanearea.getLastStepVehicleNumber(detector_id)


def get_current_phase(tls_id):  # 8. Constraint 8
    return traci.trafficlight.getPhase(tls_id)


# -------------------------
# Step 8: Fully Online Continuous Learning Loop
# -------------------------

step_history = []
reward_history = []
queue_history = []

cumulative_reward = 0.0

print("\n=== Starting Fully Online Continuous Learning (PyTorch DQN) ===")
for step in range(TOTAL_STEPS):
    current_simulation_step = step

    state = get_state()
    action = get_action_from_policy(state)
    apply_action(action)

    traci.simulationStep()

    new_state = get_state()
    reward = get_reward(new_state)
    cumulative_reward += reward

    update_Q_table(state, action, reward, new_state)

    if step % 1 == 0:
        updated_q_vals = predict_q_values(state)
        print(
            f"Step {step}, Current_State: {state}, Action: {action}, "
            f"New_State: {new_state}, Reward: {reward:.2f}, "
            f"Cumulative Reward: {cumulative_reward:.2f}, "
            f"Q-values(current_state): {updated_q_vals}"
        )
        step_history.append(step)
        reward_history.append(cumulative_reward)
        queue_history.append(sum(new_state[:-1]))

# -------------------------
# Step 9: Close connection between SUMO and Traci
# -------------------------
traci.close()

print("\nOnline Training completed.")
print("PyTorch DQN Architecture:")
print(dqn_model)

# -------------------------
# Visualization of Results
# -------------------------

plt.figure(figsize=(10, 6))
plt.plot(step_history, reward_history, marker='o', linestyle='-', label="Cumulative Reward")
plt.xlabel("Simulation Step")
plt.ylabel("Cumulative Reward")
plt.title("RL Training (PyTorch DQN): Cumulative Reward over Steps")
plt.legend()
plt.grid(True)
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(step_history, queue_history, marker='o', linestyle='-', label="Total Queue Length")
plt.xlabel("Simulation Step")
plt.ylabel("Total Queue Length")
plt.title("RL Training (PyTorch DQN): Queue Length over Steps")
plt.legend()
plt.grid(True)
plt.show()