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
    [SerializeField] private Transform targetTransform;     // Posición del objetivo
    [SerializeField] private float goalThreshold = 5f;      // Umbral para considerar objetivo alcanzado

    [SerializeField] private float maxSteeringAngle = 45f;  // Valor máximo de giro en grados

    // Flags para detectar colisión
    private bool hasCollided = false;

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
        hasCollided = false;
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
            case 0: throttle = -1f; break;    // Reversa
            case 1: throttle = 0f; break;     // Parado
            case 2: throttle = 1f; break;     // Adelante
        }
        wheelVehicle.Throttle = throttle;

        AddReward(-0.001f); // Penalización por tiempo

        float maxDistance = 50f;
        float reward = 0f;

        if (targetTransform != null)
        {
            float distanceToGoal = Vector3.Distance(transform.position, targetTransform.position);
            reward = 0.05f * (1f - (distanceToGoal / maxDistance));
            AddReward(reward);
            Debug.Log($"Distancia al objetivo: {distanceToGoal}");
            Debug.Log($"Recompensa por distancia: {reward}");

            if (distanceToGoal < goalThreshold)
            {
                AddReward(10.0f);
                Debug.Log("Objetivo alcanzado, reiniciando episodio");
                EndEpisode();
            }
        }

        if (hasCollided)
        {
            AddReward(-10.0f);
            Debug.Log("Colisión detectada, reiniciando episodio");
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
