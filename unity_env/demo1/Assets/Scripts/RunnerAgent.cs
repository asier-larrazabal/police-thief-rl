using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using VehicleBehaviour;

public class RunnerAgent : Agent
{
    [Header("Referencia al policía")]
    [SerializeField] private PoliceAgent policeAgent;
    private WheelVehicle wheelVehicle;
    private Rigidbody rb;
    private Vector3 initialPosition;
    private Quaternion initialRotation;

    [Header("Objetivo y configuración de entrenamiento")]
    [SerializeField] private Vector3[] targetPositions;  // Array de posiciones posibles del objetivo
    [SerializeField] private Transform targetTransform;   // Referencia al transform que se moverá
    [SerializeField] private float goalThreshold = 5.4f;  // Umbral para considerar objetivo alcanzado

    [SerializeField] private float maxSteeringAngle = 45f;  // Valor máximo de giro en grados

    private bool hasCollided = false;
    private float bestDistanceToGoal;

    // Recompensa acumulada en el episodio actual
    private float totalEpisodeReward;

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
        bestDistanceToGoal = float.MaxValue;
        hasCollided = false;
        totalEpisodeReward = 0f;

        // Elegir aleatoriamente la posición del objetivo
        if (targetPositions != null && targetPositions.Length > 0 && targetTransform != null)
        {
            int index = Random.Range(0, targetPositions.Length);
            targetTransform.localPosition = targetPositions[index];
        }

        // Resetear agente
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
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(transform.localPosition);
        sensor.AddObservation(transform.localRotation);
        sensor.AddObservation(rb.linearVelocity);
        sensor.AddObservation(rb.angularVelocity);

        if (targetTransform != null)
        {
            Vector3 relativePosition = targetTransform.position - transform.position;
            sensor.AddObservation(relativePosition.normalized);
            sensor.AddObservation(Vector3.Distance(transform.position, targetTransform.position));
        }
        else
        {
            sensor.AddObservation(Vector3.zero);
            sensor.AddObservation(0f);
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        int steeringAction = actions.DiscreteActions[0];
        int throttleAction = actions.DiscreteActions[1];

        float steeringAngle = 0f;
        switch (steeringAction)
        {
            case 0: steeringAngle = -maxSteeringAngle; break;  // Izquierda
            case 1: steeringAngle = 0f; break;                 // Recto
            case 2: steeringAngle = maxSteeringAngle; break;   // Derecha
        }

        float steeringNormalized = Mathf.Clamp(steeringAngle / maxSteeringAngle, -1f, 1f);
        wheelVehicle.Steering = steeringNormalized;

        float throttle = 0f;
        switch (throttleAction)
        {
            case 0: throttle = -1f; break;   // Reversa
            case 1: throttle = 0f; break;    // Parado
            case 2: throttle = 1f; break;    // Adelante
        }
        wheelVehicle.Throttle = throttle;

        if (targetTransform != null)
        {
            float distanceToGoal = Vector3.Distance(transform.position, targetTransform.position);
            // Recompensa solo si mejora la mejor distancia alcanzada
            if (distanceToGoal < bestDistanceToGoal)
            {
                bestDistanceToGoal = distanceToGoal;

                float alpha = 0.25f;
                float beta = 0.05f;
                float reward = alpha * Mathf.Exp(-beta * distanceToGoal);
                AddReward(reward);
                totalEpisodeReward += reward;

                Debug.Log($"Distancia mejorada: {distanceToGoal}");
                Debug.Log($"Recompensa por mejora: {reward}");
            }

            if (distanceToGoal < goalThreshold)
            {
                AddReward(100f);
                totalEpisodeReward += 100f;

                Debug.Log("Objetivo alcanzado, reiniciando episodio");
                Debug.Log($"[EPISODIO] Recompensa total acumulada: {totalEpisodeReward}");
                EndEpisode();
            }
        }

        if (hasCollided)
        {
            AddReward(-30f);
            totalEpisodeReward += -30f;

            Debug.Log("Colisión detectada, reiniciando episodio");
            Debug.Log($"[EPISODIO] Recompensa total acumulada: {totalEpisodeReward}");
            EndEpisode();
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Wall") || collision.gameObject.CompareTag("Walls"))
        {
            hasCollided = true;
        }
    }

    private void OnCollisionExit(Collision collision)
    {
        if (collision.gameObject.CompareTag("Wall") || collision.gameObject.CompareTag("Walls"))
        {
            hasCollided = false;
        }
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
