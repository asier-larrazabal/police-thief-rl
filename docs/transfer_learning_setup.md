# Transfer Learning Setup - Runner to Police

## Overview
We're using **transfer learning** to initialize the PoliceAgent with the RunnerAgent's trained driving skills. Since both agents need similar driving mechanics (steering, throttle control, navigation), the runner's knowledge can accelerate police training significantly.

## Rationale
The RunnerAgent has already learned:
- How to steer and control throttle effectively
- How to navigate using position and velocity
- How to approach a target (waypoint cube)

The PoliceAgent needs the same skills but with a different target (the runner instead of a cube).

## Changes Made

### 1. **Observation Space Alignment** 
Modified [PoliceAgent.cs](../unity_env/Police-Thief-RL/Assets/Scripts/PoliceAgent.cs) to match RunnerAgent's 17 observations:

**Previous (23 observations):**
- 13 base: position(3), rotation(4), velocity(3), angular_velocity(3)
- 3: runner relative position
- 3: runner velocity
- 1: waypoint distance
- 3: waypoint direction

**New (17 observations - matches RunnerAgent):**
- 13 base: position(3), rotation(4), velocity(3), angular_velocity(3)
- 3: target direction (normalized)
- 1: target distance

This structure is **identical** to how RunnerAgent tracks its waypoint cube.

### 2. **Training Configuration**
Updated [car_agent.yaml](../training/configs/car_agent.yaml) to include:

```yaml
PoliceAgent:
  init_path: ../unity_env/Police-Thief-RL/Assets/Scripts/Models/RunnerAgent22_lastTriesForMultiObjective.onnx
```

This tells ML-Agents to:
1. Load the runner's trained neural network weights
2. Use them as the starting point for police training
3. Continue training with police-specific rewards

### 3. **Unity Scene Configuration**
Changed VectorObservationSize in [ciudad.unity](../unity_env/Police-Thief-RL/Assets/Scenes/ciudad.unity):
- From: 23 (with waypoint observations)
- To: 17 (matching runner)

With NumStackedVectorObservations=4, total input: 17 × 4 = **68 observations**

## Action Space
Both agents use the **same action space**:
- Discrete[3] steering: -1 (left), 0 (straight), +1 (right)
- Discrete[3] throttle: -1 (reverse), 0 (brake), +1 (forward)

This compatibility is essential for transfer learning.

## Expected Benefits

### Faster Convergence
- The police starts with driving knowledge instead of random behavior
- Should reach stable rewards in ~200k-300k steps vs ~800k+ from scratch

### Better Initial Performance
- Won't need to learn basic steering/throttle coordination
- Can focus learning on chasing strategy

### More Sample Efficient
- Fewer episodes needed to learn pursuit behavior
- Runner knowledge transfers directly

## Training Command

```powershell
cd training
mlagents-learn configs/car_agent.yaml --run-id run12_transfer --force
```

Then start the Unity build.

## Monitoring

Watch these TensorBoard metrics:
- **Cumulative Reward**: Should start higher than previous runs (~20-30 instead of 0-5)
- **Episode Length**: Should show intelligent pursuit earlier
- **DifficultyLevel**: Curriculum should progress faster
- **SuccessRate**: Should reach 60%+ sooner

## Comparison to Previous Runs

| Run | Observations | Init Method | Result |
|-----|-------------|-------------|--------|
| run9 | 19 (no waypoints) | Random | Stable 40-55 rewards, ~760k steps |
| run10 | 23 (with waypoints) | Random | Unstable 11-82, high variance |
| run11 | 23 (with waypoints) | Random | Unstable 20-63, didn't converge |
| **run12** | **17 (transfer)** | **Runner model** | **Expected: faster convergence** |

## Troubleshooting

### If training fails to start:
- Check init_path points to correct .onnx file
- Verify observation space is exactly 17 (not 19 or 23)
- Confirm Unity scene has VectorObservationSize: 17

### If police behaves randomly:
- Transfer learning loaded successfully but rewards need adjustment
- Check that curriculum is working (SetDifficultyLevel bug was fixed)
- Monitor SuccessRate - should increase faster than previous runs

### If observation mismatch error:
```
Shape mismatch: expected [68] but got [76] or [92]
```
- Rebuild Unity scene to pick up VectorObservationSize: 17 change
- Delete old builds and create fresh training build

## Next Steps

1. **Build Unity** with updated scene (VectorObservationSize: 17)
2. **Start training** with run12_transfer
3. **Monitor TensorBoard** - expect better initial performance
4. **Evaluate at 300k steps** - should already show good pursuit
5. **Compare with run9** - transfer learning should converge faster

## Fallback Plan

If transfer learning doesn't improve results:
- Use **run9 model** (PoliceAgent-999939.onnx) - it works well
- Consider environment changes (minimum runner speed, waypoint rotation)
- Try different reward shaping for pursuit behavior
