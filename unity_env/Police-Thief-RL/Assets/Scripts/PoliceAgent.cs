using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using VehicleBehaviour;

public class PoliceAgent : Agent
{
    [Header("Referencia al fugitivo (runner)")]
    [SerializeField] private RunnerAgent runnerAgent;
    private WheelVehicle wheelVehicle;
    private Rigidbody rb;
    private Vector3 initialPosition;
    private Quaternion initialRotation;

    private float prevDistanceToRunner;
    private float bestDistanceToRunner;
    private bool hasCollided = false;

    [Header("Curriculum Learning")]
    private float currentDifficultyLevel = 0f;
    private int successfulCatches = 0;
    private int episodeCount = 0;
    private float successRate = 0f;
    private const int EPISODES_TO_CHECK = 50; // Cada 50 episodios revisa progreso

    public override void Initialize()
    {
        wheelVehicle = GetComponent<WheelVehicle>();
        rb = GetComponent<Rigidbody>();
        initialPosition = transform.localPosition;
        initialRotation = transform.localRotation;
        if (wheelVehicle != null)
            wheelVehicle.IsPlayer = false;
    }

    public override void OnEpisodeBegin()
    {
        transform.localPosition = initialPosition + new Vector3(Random.Range(-3f, 3f), 0, Random.Range(-3f, 3f));
        float randomAngle = Random.Range(-90f, 90f);
        transform.localRotation = initialRotation * Quaternion.Euler(0, randomAngle, 0);

        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        if (wheelVehicle != null)
        {
            wheelVehicle.Steering = 0f;
            wheelVehicle.Throttle = 0f;
        }

        if (runnerAgent != null)
        {
            prevDistanceToRunner = Vector3.Distance(transform.position, runnerAgent.transform.position);
            bestDistanceToRunner = prevDistanceToRunner;
        }

        // Actualizar curriculum cada 50 episodios
        episodeCount++;
        if (episodeCount % EPISODES_TO_CHECK == 0)
        {
            UpdateCurriculumLevel();
        }

        hasCollided = false;
    }

    private void UpdateCurriculumLevel()
    {
        successRate = (float)successfulCatches / EPISODES_TO_CHECK;

        Academy.Instance.StatsRecorder.Add("PoliceAgent/SuccessRate", successRate);
        Academy.Instance.StatsRecorder.Add("PoliceAgent/DifficultyLevel", currentDifficultyLevel);

        Debug.Log($"Success rate: {successRate * 100f}% in last {EPISODES_TO_CHECK} episodes. Current difficulty: {currentDifficultyLevel}");

        // Aumentar dificultad si tiene >60% éxito
        if (successRate > 0.6f && currentDifficultyLevel < 3f)
        {
            currentDifficultyLevel += 1f;
            Debug.Log($"Increasing difficulty to level {currentDifficultyLevel}");
            if (runnerAgent != null)
                runnerAgent.SetDifficultyLevel(currentDifficultyLevel);
        }
        // Reducir dificultad si tiene <20% éxito (excepto en nivel 0)
        else if (successRate < 0.2f && currentDifficultyLevel > 0f)
        {
            currentDifficultyLevel -= 1f;
            Debug.Log($"Decreasing difficulty to level {currentDifficultyLevel}");
            if (runnerAgent != null)
                runnerAgent.SetDifficultyLevel(currentDifficultyLevel);
        }

        successfulCatches = 0; // Reset contador
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // TRANSFER LEARNING: Use same observation structure as RunnerAgent (17 obs)
        // This allows loading runner's trained model as initialization
        
        // Base observations (same as RunnerAgent): 13 obs
        sensor.AddObservation(transform.localPosition);      // 3 obs
        sensor.AddObservation(transform.localRotation);      // 4 obs
        sensor.AddObservation(rb.linearVelocity);            // 3 obs
        sensor.AddObservation(rb.angularVelocity);           // 3 obs
        
        if (runnerAgent != null)
        {
            // Target tracking (same structure as RunnerAgent tracking cube): 4 obs
            Vector3 relPos = runnerAgent.transform.localPosition - transform.localPosition;
            sensor.AddObservation(relPos.normalized);        // 3 obs (direction to target)
            sensor.AddObservation(relPos.magnitude);         // 1 obs (distance to target)
            
            // Total: 17 observations (matches RunnerAgent exactly)
            
            // DEBUG: Log cada 100 frames
            if (Time.frameCount % 100 == 0)
            {
                Debug.Log($"Police observations - Runner dist: {relPos.magnitude:F2}, Runner dir: {relPos.normalized}");
            }
        }
        else
        {
            sensor.AddObservation(Vector3.zero);             // 3 obs
            sensor.AddObservation(0f);                       // 1 obs
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        int steeringAction = actions.DiscreteActions[0];
        int throttleAction = actions.DiscreteActions[1];

        float steering = 0f;
        switch (steeringAction)
        {
            case 0: steering = -1f; break;
            case 1: steering = 0f; break;
            case 2: steering = 1f; break;
        }
        wheelVehicle.Steering = Mathf.Clamp(steering, -1f, 1f);

        float throttle = 0f;
        switch (throttleAction)
        {
            case 0: throttle = -1f; break;
            case 1: throttle = 0f; break;
            case 2: throttle = 1f; break;
        }
        wheelVehicle.Throttle = Mathf.Clamp(throttle, -1f, 1f);

        if (runnerAgent == null)
        {
            return;
        }

        // RECOMPENSAS SIMPLES
        float dist = Vector3.Distance(transform.position, runnerAgent.transform.position);

        // 1. Recompensa por acercarse (solo si mejora el récord)
        if (dist < bestDistanceToRunner)
        {
            float reward = (bestDistanceToRunner - dist) * 0.5f;
            AddReward(reward);
            bestDistanceToRunner = dist;
        }

        // 2. Pequeña recompensa por ir rápido HACIA el runner
        Vector3 toRunner = (runnerAgent.transform.position - transform.position).normalized;
        float velocityTowardsRunner = Vector3.Dot(rb.linearVelocity.normalized, toRunner);
        if (velocityTowardsRunner > 0)
        {
            AddReward(velocityTowardsRunner * 0.01f);
        }

        // 3. Captura exitosa
        if (dist < 6.5f)
        {
            AddReward(50f);
            successfulCatches++;
            if (runnerAgent != null)
                runnerAgent.AddReward(-50f);
            EndEpisode();
            if (runnerAgent != null)
                runnerAgent.EndEpisode();
            Debug.Log($"¡Runner capturado! Nivel dificultad: {currentDifficultyLevel}");
            return;
        }

        prevDistanceToRunner = dist;

        // 4. Penalización por colisión con pared
        if (hasCollided)
        {
            AddReward(-5f);
            EndEpisode();
            if (runnerAgent != null)
                runnerAgent.EndEpisode();
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Wall") || collision.gameObject.CompareTag("Walls"))
        {
            hasCollided = true;
        }
        if (collision.gameObject.CompareTag("Runner"))
        {
            AddReward(50f);
            successfulCatches++;
            if (runnerAgent != null)
                runnerAgent.AddReward(-50f);
            EndEpisode();
            if (runnerAgent != null)
                runnerAgent.EndEpisode();
            Debug.Log($"¡Runner capturado por colisión! Nivel dificultad: {currentDifficultyLevel}");
        }
    }

    private void OnCollisionExit(Collision collision)
    {
        if (collision.gameObject.CompareTag("Wall") || collision.gameObject.CompareTag("Walls"))
        {
            hasCollided = false;
        }
    }



    public void OnRunnerReachedGoal()
    {
        AddReward(-20f);
        EndEpisode();
    }

    public void OnRunnerCollided()
    {
        EndEpisode();
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var discreteActionsOut = actionsOut.DiscreteActions;
        float h = Input.GetAxis("Horizontal");
        discreteActionsOut[0] = (h < -0.1f) ? 0 : (h > 0.1f) ? 2 : 1;
        float v = Input.GetAxis("Vertical");
        discreteActionsOut[1] = (v < -0.1f) ? 0 : (v > 0.1f) ? 2 : 1;
    }

    public Vector3 GetVelocity()
    {
        return rb != null ? rb.linearVelocity : Vector3.zero;
    }
}
