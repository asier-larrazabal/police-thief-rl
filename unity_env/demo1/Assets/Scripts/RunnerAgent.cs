using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using VehicleBehaviour;

public class RunnerAgent : Agent
{
    [Header("Referencia al policía")]
    public PoliceAgent policeAgent;
    private WheelVehicle wheelVehicle;
    private Rigidbody rb;
    private Vector3 initialPosition;
    private Quaternion initialRotation;

    [Header("Objetivo y configuración de entrenamiento")]
    public Transform targetTransform;     // Posición del objetivo
    public float goalThreshold = 5f;      // Umbral para considerar objetivo alcanzado

    private float collisionCheckDistance = 1.5f;

    public float maxSteeringAngle = 45f;  // Valor máximo de giro en grados

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
        // Reinicia la posición del agente cerca de la inicial para diversidad
        transform.localPosition = initialPosition + new Vector3(Random.Range(-3f, 3f), 0, Random.Range(-3f, 3f));
        transform.localRotation = initialRotation;

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
            sensor.AddObservation(relativePosition.normalized);          // Dirección al objetivo
            sensor.AddObservation(Vector3.Distance(transform.position, targetTransform.position)); // Distancia al objetivo
        }
        else
        {
            sensor.AddObservation(Vector3.zero);
            sensor.AddObservation(0f);
        }
        // Si usas RayPerceptionSensor 3D elimina raycasts manuales aquí para evitar duplicados
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

        float steeringNormalized = (steeringAngle / maxSteeringAngle) * 1.5f;
        steeringNormalized = Mathf.Clamp(steeringNormalized, -1f, 1f);
        wheelVehicle.Steering = steeringNormalized;

        float throttle = 0f;
        switch (throttleAction)
        {
            case 0: throttle = -1f; break;    // Reversa
            case 1: throttle = 0f; break;     // Parado
            case 2: throttle = 1f; break;     // Adelante
        }
        wheelVehicle.Throttle = throttle;

        if (targetTransform != null)
        {
            float distanceToGoal = Vector3.Distance(transform.position, targetTransform.position);
            Debug.Log($"Distancia al objetivo: {distanceToGoal}");
            // Penalización leve por paso de tiempo para incentivar rapidez
            AddReward(-0.01f);

            // Recompensa por acercarse al objetivo (inversa de la distancia, suavizado)
            AddReward(0.01f * (1f / (distanceToGoal + 1f)));

            // Recompensa mayor y termina episodio cuando llegue al objetivo
            if (distanceToGoal < goalThreshold)
            {
                AddReward(10.0f);
                Debug.Log("Objetivo alcanzado, reiniciando episodio");
                EndEpisode();
            }
        }

        if (CheckCollision())
        {
            // Penalización fuerte por colisión y finaliza episodio
            AddReward(-1.0f);
            EndEpisode();
        }
    }

    bool CheckCollision()
    {
        Vector3 origin = transform.position + Vector3.up * 0.5f;
        return Physics.Raycast(origin, transform.forward, collisionCheckDistance);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var discreteActionsOut = actionsOut.DiscreteActions;

        float h = Input.GetAxis("Horizontal");
        if (h < -0.1f) discreteActionsOut[0] = 0;
        else if (h > 0.1f) discreteActionsOut[0] = 2;
        else discreteActionsOut[0] = 1;

        float v = Input.GetAxis("Vertical");
        if (v < -0.1f) discreteActionsOut[1] = 0;
        else if (v > 0.1f) discreteActionsOut[1] = 2;
        else discreteActionsOut[1] = 1;
    }

    public Vector3 GetVelocity() => rb ? rb.linearVelocity : Vector3.zero;
}
