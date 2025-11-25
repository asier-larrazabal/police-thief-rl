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
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(transform.localPosition);
        sensor.AddObservation(transform.localRotation);
        sensor.AddObservation(rb.linearVelocity);
        sensor.AddObservation(rb.angularVelocity);
        if (runnerAgent != null)
        {
            Vector3 relPos = runnerAgent.transform.localPosition - transform.localPosition;
            sensor.AddObservation(relPos);
            sensor.AddObservation(runnerAgent.GetVelocity());
        }
        else
        {
            sensor.AddObservation(Vector3.zero);
            sensor.AddObservation(Vector3.zero);
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
        AddReward(-0.001f);
        return;
    }

    float dist = Vector3.Distance(transform.position, runnerAgent.transform.position);
    float delta = prevDistanceToRunner - dist;

    AddReward(delta * 0.1f);
    prevDistanceToRunner = dist;

    AddReward(-0.001f);


    /*if (dist < 6.5f)
    {
        AddReward(100f);
        runnerAgent.AddReward(-100f);
        EndEpisode();
        runnerAgent.EndEpisode();
        Debug.Log("¡Runner capturado!");
        return;
    }*/

    // Aquí chequea colisión sin terminar episodio, solo penaliza
    if (hasCollided)
    {
        EndEpisode();
        runnerAgent.EndEpisode();
    }
}

    // Privado método para chequear colisión delante del coche
    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Wall") || collision.gameObject.CompareTag("Walls"))
        {
            hasCollided = true;
        }
        if (collision.gameObject.CompareTag("Runner"))
        {
            AddReward(100f);
            runnerAgent.AddReward(-100f);
            EndEpisode();
            runnerAgent.EndEpisode();
            Debug.Log("¡Runner capturado por colisión!");
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
