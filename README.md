## Police–Thief Reinforcement Learning (Unity ML-Agents)

This project trains two driving agents in Unity using ML-Agents: a Runner that navigates toward targets and a Police agent that chases the Runner. It supports transfer learning to initialize the Police with the Runner's driving skills for faster convergence.

---

**Verified Versions**

- Unity: 6000.0.41f1
- Python: 3.10.9

---

**Key Features**

- Two behaviors: RunnerAgent (navigation) and PoliceAgent (pursuit)
- Discrete actions: steering and throttle (3×3 = 9 actions)
- Stacked vector observations (4 frames)
- Transfer learning from Runner → Police for faster training
- Ready-to-train with Unity Editor or a Windows build

---

**Repository Structure**

- training/configs/car_agent.yaml: ML-Agents training configuration
- unity_env/Police-Thief-RL: Unity project (agents, scenes, models)
- build/: Optional Windows build for headless training
- results/: Training outputs (configs, logs, models)
- docs/: Design notes and transfer learning details
- analysis/: Notebooks and evaluation utilities
- scripts/: Helper scripts (placeholders for future utilities)

See also: docs/transfer_learning_setup.md and docs/observation_space_comparison.md

---

**Requirements**

- Windows with PowerShell
- Unity Editor 6000.0.41f1 (or compatible)
- Python 3.10.x

---

**Setup**

1) Create and activate a virtual environment

```powershell
python -m venv .venv_ml
. .venv_ml\Scripts\Activate.ps1
```

If running on locked-down machines, allow scripts for this session:

```powershell
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope Process
```

2) Install dependencies (known-good pins shown; `requirements.txt` is also provided)

```powershell
pip install --upgrade pip
pip install mlagents==1.1.0 mlagents-envs==1.1.0
pip install torch==1.13.1+cpu torchvision==0.14.1+cpu torchaudio==0.13.1 --index-url https://download.pytorch.org/whl/cpu
pip install onnx==1.15.0 protobuf==3.20.3 numpy
# Or: pip install -r requirements.txt
```

---

**How To Train**

You can train from the Unity Editor or using the prebuilt executable under build/.

Option A — Unity Editor

```powershell
mlagents-learn training\configs\car_agent.yaml --run-id run_editor --train
# Then press Play in the Unity Editor for the scene with agents
```

Option B — Windows Build (headless friendly)

```powershell
mlagents-learn training/configs/car_agent.yaml --run-id run_build --train --no-graphics --env ".\build\My project.exe"
```

Resuming a run

```powershell
mlagents-learn training/configs/car_agent.yaml --run-id run_build --resume --no-graphics --env ".\build\My project.exe"
```

Outputs are written to results/<run-id>.

---

**Transfer Learning (Runner → Police)**

The Police can start from the Runner's trained weights to accelerate training. Summary:

- Match observation space to 17 (13 base + 3 target direction + 1 target distance)
- Ensure the Unity scene VectorObservationSize is 17; with 4-stack total input is 68
- Set `init_path` for `PoliceAgent` to a compatible Runner model

Edit training/configs/car_agent.yaml (example):

```yaml
behaviors:
	PoliceAgent:
		# ...hyperparameters...
		init_path: ../unity_env/Police-Thief-RL/Assets/Scripts/Models/RunnerAgent22_lastTriesForMultiObjective.onnx
```

Full details, rationale, and troubleshooting: docs/transfer_learning_setup.md and docs/observation_space_comparison.md

---

**Monitoring with TensorBoard**

```powershell
pip install tensorboard
tensorboard --logdir results --port 6006
```

Key metrics to watch: Cumulative Reward, Episode Length, DifficultyLevel (if using curriculum), and SuccessRate.

---

**Results and Models**

- Checkpoints and logs are under results/<run-id>/
- Behavior-specific folders contain saved models, e.g., results/run12/PoliceAgent/
- For deployment in Unity, use the .onnx exported model in the Unity project’s Models folder

Note: Current helper scripts under scripts/ are placeholders; training/export is handled by ML-Agents.

---

**Troubleshooting**

- Observation mismatch (e.g., expected 68, got 76/92): Rebuild the Unity scene with VectorObservationSize = 17 and ensure 4-frame stacking; delete old builds if needed
- Training won’t start: Verify `init_path` exists, scene has correct observation size, and the correct behavior names are present
- Random behavior after transfer: Transfer loaded, but reward shaping/curriculum may need tuning; check SuccessRate and curriculum progression
- Port in use: Adjust `base_port` in config or close other Unity instances
- PowerShell activation: Use `Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope Process`

More details: docs/transfer_learning_setup.md

---

**Quick Commands**

```powershell
# Fresh training in Editor
mlagents-learn training\configs\car_agent.yaml --run-id run1 --train

# Train with Windows build (no graphics)
mlagents-learn training/configs/car_agent.yaml --run-id run5 --train --no-graphics --env ".\build\My project.exe"

# Resume a run
mlagents-learn training/configs/car_agent.yaml --run-id run5 --resume --no-graphics --env ".\build\My project.exe"
```

---

**License**

See LICENSE for details.
