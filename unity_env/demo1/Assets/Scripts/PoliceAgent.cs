using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using VehicleBehaviour;

public class PoliceAgent : Agent
{
    [Header("Referencia al fugitivo")]
    public RunnerAgent runnerAgent;
    private WheelVehicle wheelVehicle;
    private Rigidbody rb;
    private Vector3 initialPosition;
    private Quaternion initialRotation;

    private float collisionCheckDistance = 1.5f;

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
        sensor.AddObservation(runnerAgent.transform.localPosition - transform.localPosition);
        sensor.AddObservation(runnerAgent.GetVelocity());

        float[] rayAngles = { 0f, -30f, 30f, -60f, 60f };
        foreach (float angle in rayAngles)
        {
            Vector3 dir = Quaternion.Euler(0, angle, 0) * transform.forward;
            sensor.AddObservation(Physics.Raycast(transform.position + Vector3.up, dir, 10f) ? 1f : 0f);
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float steering = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float throttle = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        wheelVehicle.Steering = steering;
        wheelVehicle.Throttle = throttle;

        float dist = Vector3.Distance(transform.position, runnerAgent.transform.position);
        AddReward(-dist * 0.002f);

        CheckCollision();

        if (dist < 4f)
        {
            AddReward(1.0f);
            runnerAgent.AddReward(-1.0f);
            runnerAgent.EndEpisode();
            EndEpisode();
        }
        else
        {
            AddReward(0.01f);
        }
    }

    void CheckCollision()
    {
        Vector3 origin = transform.position + Vector3.up * 0.5f;
        if (Physics.Raycast(origin, transform.forward, collisionCheckDistance))
        {
            AddReward(-1.0f);
            runnerAgent.EndEpisode();
            EndEpisode();
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var ca = actionsOut.ContinuousActions;
        ca[0] = Input.GetAxis("Horizontal");
        ca[1] = Input.GetAxis("Vertical");
    }

    public Vector3 GetVelocity() => rb != null ? rb.linearVelocity : Vector3.zero;
}
