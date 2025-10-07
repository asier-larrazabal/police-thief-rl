using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using VehicleBehaviour;

public class PoliceAgent : Agent
{
    [Header("Referencias")]
    public RunnerAgent runnerAgent; // Asigna en el inspector
    private WheelVehicle wheelVehicle;
    private Rigidbody rb;
    private Vector3 initialPosition;
    private Quaternion initialRotation;

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
        transform.localPosition = initialPosition + new Vector3(Random.Range(-2f, 2f), 0, Random.Range(-2f, 2f));
        transform.localRotation = initialRotation;
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
        sensor.AddObservation(runnerAgent.transform.localPosition - transform.localPosition); // Posición relativa del fugitivo
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
        AddReward(-dist * 0.002f); // Acercarse al runner es positivo

        if (dist < 4f)
        {
            AddReward(1.0f); // El policía gana
            runnerAgent.AddReward(-1.0f); // El runner pierde
            runnerAgent.EndEpisode();
            EndEpisode();
        }
        else
        {
            AddReward(0.01f);
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var ca = actionsOut.ContinuousActions;
        ca[0] = Input.GetAxis("Horizontal");
        ca[1] = Input.GetAxis("Vertical");
    }

    // Permitir que RunnerAgent acceda a la velocidad para observaciones
    public Vector3 GetVelocity()
    {
        return rb != null ? rb.linearVelocity : Vector3.zero;
    }
}
