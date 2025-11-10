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

    private float collisionCheckDistance = 1.5f;

    public float maxSteeringAngle = 45f; // Puedes ajustar este valor por inspector

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
        transform.localPosition = initialPosition + new Vector3(Random.Range(-3f,3f), 0, Random.Range(-3f,3f));
        transform.localRotation = initialRotation;
        if (rb != null)
            rb.linearVelocity = rb.angularVelocity = Vector3.zero;
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
        sensor.AddObservation(policeAgent.transform.localPosition - transform.localPosition);
        sensor.AddObservation(policeAgent.GetVelocity());
        float[] rayAngles = { 0f, -30f, 30f, -60f, 60f };
        foreach (float angle in rayAngles)
        {
            Vector3 dir = Quaternion.Euler(0, angle, 0) * transform.forward;
            sensor.AddObservation(Physics.Raycast(transform.position + Vector3.up, dir, 10f) ? 1f : 0f);
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // Obtener acciones discretas
        int steeringAction = actions.DiscreteActions[0];
        int throttleAction = actions.DiscreteActions[1];

        // Mapear acción de giro a ángulo fijo de ±45 grados
        float steeringAngle = 0f;
        if (steeringAction == 0) steeringAngle = -maxSteeringAngle; // Izquierda
        else if (steeringAction == 1) steeringAngle = 0f;           // Recto
        else if (steeringAction == 2) steeringAngle = maxSteeringAngle; // Derecha

        // Aplicar el ángulo normalizado esperado por WheelVehicle (entre -1 y 1)
        wheelVehicle.Steering = steeringAngle / maxSteeringAngle;

        // Mapear acción de aceleración
        float throttle = 0f;
        if (throttleAction == 0) throttle = -1f; // Marcha atrás
        else if (throttleAction == 1) throttle = 0f; // Parado
        else if (throttleAction == 2) throttle = 1f; // Aceleración

        wheelVehicle.Throttle = throttle;

        // Ejemplo de cálculo de distancia para recompensas
        float dist = Vector3.Distance(transform.position, policeAgent.transform.position);

        // Recompensa que incentiva alejarse
        AddReward(0.05f * Mathf.Clamp(dist, 0, 20));

        // Penalización y fin de episodio si es atrapado
        if (dist < 4f)
        {
            AddReward(-1.0f);
            policeAgent.AddReward(1.0f);
            policeAgent.EndEpisode();
            EndEpisode();
        }

        // Penalización por colisión (ejemplo, ajusta a tus métodos)
        if (CheckCollision())
        {
            AddReward(-1.0f);
            policeAgent.AddReward(1.0f);
            policeAgent.EndEpisode();
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

        // Steering
        float h = Input.GetAxis("Horizontal");
        if (h < -0.1f) discreteActionsOut[0] = 0; // Left
        else if (h > 0.1f) discreteActionsOut[0] = 2; // Right
        else discreteActionsOut[0] = 1; // Center

        // Throttle
        float v = Input.GetAxis("Vertical");
        if (v < -0.1f) discreteActionsOut[1] = 0; // Reverse
        else if (v > 0.1f) discreteActionsOut[1] = 2; // Forward
        else discreteActionsOut[1] = 1; // Idle
    }

    public Vector3 GetVelocity() => rb != null ? rb.linearVelocity : Vector3.zero;
}
