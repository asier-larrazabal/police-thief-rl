using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using VehicleBehaviour;

public class CarAgent1 : Agent
{
    [Header("Target")]
    public Transform policeTarget;
    public string policeCarName = "PoliceCar";
    
    [Header("AI Settings")]
    public float detectionRange = 30f;
    public float escapeDistance = 20f;
    
    [Header("Obstacle Detection")]
    public float raycastDistance = 15f;
    public LayerMask obstacleLayerMask = -1;
    
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
    
    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(transform.localPosition);
        sensor.AddObservation(transform.localRotation);
        sensor.AddObservation(rb != null ? rb.linearVelocity : Vector3.zero);
        sensor.AddObservation(rb != null ? rb.angularVelocity : Vector3.zero);
        
        if (policeTarget != null)
        {
            Vector3 relativePosition = transform.InverseTransformPoint(policeTarget.position);
            sensor.AddObservation(relativePosition);
        }
        else
        {
            sensor.AddObservation(Vector3.zero);
        }
        
        float[] rayAngles = { 0f, -22.5f, 22.5f, -45f, 45f };
        for (int i = 0; i < 5; i++)
        {
            Vector3 rayDirection = Quaternion.Euler(0, rayAngles[i], 0) * transform.forward;
            RaycastHit hit;
            
            if (Physics.Raycast(transform.position + Vector3.up * 0.5f, rayDirection, out hit, raycastDistance, obstacleLayerMask))
            {
                sensor.AddObservation(hit.distance / raycastDistance);
            }
            else
            {
                sensor.AddObservation(1.0f);
            }
        }
    }
    
    public override void OnActionReceived(ActionBuffers actions)
    {
        if (wheelVehicle == null) return;
        
        float steering = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float throttle = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        
        wheelVehicle.Steering = steering;
        wheelVehicle.Throttle = throttle;
        
        float forwardSpeed = Vector3.Dot(rb.linearVelocity, transform.forward);
        if (forwardSpeed > 0) AddReward(forwardSpeed * 0.1f);
        
        float totalSpeed = rb.linearVelocity.magnitude;
        if (totalSpeed < 0.5f) AddReward(-0.01f);
        
        AddReward(0.001f);
        
        Vector3 origin = transform.position + Vector3.up * 0.5f;
        if (Physics.Raycast(origin, transform.forward, 2f, obstacleLayerMask))
        {
            AddReward(-1.0f);
            EndEpisode();
        }
    }
    
    public override void OnEpisodeBegin()
    {
        transform.localPosition = initialPosition;
        transform.localRotation = initialRotation;
        
        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }
    }
    
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Horizontal");
        continuousActionsOut[1] = Input.GetAxis("Vertical");
    }
}
