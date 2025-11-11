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

    public float maxSteeringAngle = 45f; // valor usado para giro máximo en grados

    public override void Initialize()
    {
        wheelVehicle = GetComponent<WheelVehicle>();
        rb = GetComponent<Rigidbody>();
        initialPosition = transform.localPosition;
        initialRotation = transform.localRotation;
        if (wheelVehicle != null) wheelVehicle.IsPlayer = false;
    }

    public override void OnEpisodeBegin()
    {
        transform.localPosition = initialPosition + new Vector3(Random.Range(-3f, 3f), 0, Random.Range(-3f, 3f));
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
        // Si usas RayPerceptionSensor 3D elimina raycasts manuales aquí
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        int steeringAction = actions.DiscreteActions[0];
        int throttleAction = actions.DiscreteActions[1];

        float steeringAngle = 0f;
        if (steeringAction == 0) steeringAngle = -maxSteeringAngle;
        else if (steeringAction == 1) steeringAngle = 0f;
        else if (steeringAction == 2) steeringAngle = maxSteeringAngle;

        float steeringNormalized = (steeringAngle / maxSteeringAngle) * 1.5f;
        steeringNormalized = Mathf.Clamp(steeringNormalized, -1f, 1f);
        wheelVehicle.Steering = steeringNormalized;

        float throttle = 0f;
        if (throttleAction == 0) throttle = -1f;
        else if (throttleAction == 1) throttle = 0f;
        else if (throttleAction == 2) throttle = 1f;

        wheelVehicle.Throttle = throttle;
        Debug.Log($"[RunnerAgent] Steering input (normalizado): {steeringNormalized}");
        Debug.Log($"[RunnerAgent] Throttle input: {throttle}");

        float dist = Vector3.Distance(transform.position, policeAgent.transform.position);
        AddReward(0.05f * Mathf.Clamp(dist, 0, 20));

        if (dist < 4f)
        {
            AddReward(-1.0f);
            policeAgent.AddReward(1.0f);
            policeAgent.EndEpisode();
            EndEpisode();
        }

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

        float h = Input.GetAxis("Horizontal");
        if (h < -0.1f) discreteActionsOut[0] = 0;
        else if (h > 0.1f) discreteActionsOut[0] = 2;
        else discreteActionsOut[0] = 1;

        float v = Input.GetAxis("Vertical");
        if (v < -0.1f) discreteActionsOut[1] = 0;
        else if (v > 0.1f) discreteActionsOut[1] = 2;
        else discreteActionsOut[1] = 1;
    }

    public Vector3 GetVelocity() => rb != null ? rb.linearVelocity : Vector3.zero;
}
