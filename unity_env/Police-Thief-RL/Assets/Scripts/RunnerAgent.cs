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
    [SerializeField] private Vector3[] targetPositions;
    [SerializeField] private Transform targetTransform;
    [SerializeField] private float goalThreshold = 5.4f;
    [SerializeField] private float maxSteeringAngle = 45f;


    private bool hasCollided = false;
    private float bestDistanceToGoal;
    private float totalEpisodeReward;
    private float prevDistanceToGoal;


    // Multiobjetivo
    //private int objetivosRestantes;
    //private int objetivoActualIndex;


    // Antistuck y alineación
    private int stuckCounter = 0;
    private float alignmentPrev = 0f;


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
        totalEpisodeReward = 0f;
        stuckCounter = 0;


        //objetivosRestantes = 1;
        int objetivoActualIndex = Random.Range(0, targetPositions.Length);
        targetTransform.localPosition = targetPositions[objetivoActualIndex];


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


        bestDistanceToGoal = Vector3.Distance(transform.position, targetTransform.position);
        prevDistanceToGoal = bestDistanceToGoal;


        // Inicializa alineación
        Vector3 toGoal = (targetTransform.position - transform.position).normalized;
        alignmentPrev = Vector3.Dot(transform.forward, toGoal);
    }


    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(transform.localPosition);
        sensor.AddObservation(transform.localRotation);


        // Observaciones de velocidad y giro actuales
        sensor.AddObservation(rb.linearVelocity);    // 3 floats
        sensor.AddObservation(rb.angularVelocity);   // 3 floats


        if (targetTransform != null)
        {
            Vector3 relativePosition = targetTransform.position - transform.position;
            sensor.AddObservation(relativePosition.normalized);         // Dirección relativa
            sensor.AddObservation(Vector3.Distance(transform.position, targetTransform.position)); // Distancia escalar
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
            case 0: steeringAngle = -maxSteeringAngle; break;
            case 1: steeringAngle = 0f; break;
            case 2: steeringAngle = maxSteeringAngle; break;
        }
        float steeringNormalized = Mathf.Clamp(steeringAngle / maxSteeringAngle, -1f, 1f);
        wheelVehicle.Steering = steeringNormalized;


        float throttle = 0f;
        switch (throttleAction)
        {
            case 0: throttle = -1f; break;
            case 1: throttle = 0f; break;
            case 2: throttle = 1f; break;
        }
        wheelVehicle.Throttle = throttle;


        // Penalización pequeña por paso (premia avanzar y penaliza quedarse parado mucho tiempo)
        AddReward(-0.01f);


        // ANTI-STUCK: penaliza quedarse parado o no acercarse al objetivo
        if (rb.linearVelocity.magnitude < 0.5f)
        {
            stuckCounter++;
            if (stuckCounter > 30) // 30 frames seguidos atascado
            {
                AddReward(-1.0f);
                stuckCounter = 0;
            }
        }
        else
        {
            stuckCounter = 0;
        }


        // INCENTIVO ALINEACIÓN: premia orientarse hacia el target
        if (targetTransform != null)
        {
            Vector3 toGoal = (targetTransform.position - transform.position).normalized;
            float alignment = Vector3.Dot(transform.forward, toGoal);
            float alignmentDelta = alignment - alignmentPrev;
            if (alignmentDelta > 0)
            {
                AddReward(alignmentDelta * 0.05f); // Potencia avanzar mirando al objetivo
            }
            alignmentPrev = alignment;
        }


        // REWARD por acercamiento (“progreso”)
        if (targetTransform != null)
        {
            float distanceToGoal = Vector3.Distance(transform.position, targetTransform.position);
            float deltaDist = prevDistanceToGoal - distanceToGoal;
            if (deltaDist > 0)
            {
                float advanceReward = deltaDist * 0.1f;
                AddReward(advanceReward);
                totalEpisodeReward += advanceReward;
            }
            prevDistanceToGoal = distanceToGoal;


            if (distanceToGoal < goalThreshold)
            {
                /*AddReward(100f);
                totalEpisodeReward += 100f;
                objetivosRestantes--;


                Debug.Log("Objetivo alcanzado");
                Debug.Log($"Recompensa total al objetivo: {totalEpisodeReward}");


                if (objetivosRestantes > 0)
                {
                    int nuevoIndex;
                    do
                    {
                        nuevoIndex = Random.Range(0, targetPositions.Length);
                    } while (nuevoIndex == objetivoActualIndex && targetPositions.Length > 1);


                    objetivoActualIndex = nuevoIndex;
                    targetTransform.localPosition = targetPositions[objetivoActualIndex];
                    bestDistanceToGoal = Vector3.Distance(transform.position, targetTransform.position);
                    prevDistanceToGoal = bestDistanceToGoal;


                    Vector3 toGoal = (targetTransform.position - transform.position).normalized;
                    alignmentPrev = Vector3.Dot(transform.forward, toGoal);


                    Debug.Log($"Nuevo objetivo seleccionado, quedan: {objetivosRestantes}");
                }
                else
                {
                    Debug.Log("Todos los objetivos alcanzados, reiniciando episodio");
                    Debug.Log($"[EPISODIO] Recompensa total acumulada: {totalEpisodeReward}");
                    EndEpisode();
                }*/
                AddReward(100f);
                // totalEpisodeReward += 100f;   // Opcional usar o no, ML-Agents lo controla internamente

                Debug.Log("Runner ha llegado al bloque");
                NotifyPoliceRunnerReachedGoal();
                EndEpisode();
            }
        }


        if (hasCollided)
        {
            AddReward(-75f);
            totalEpisodeReward += -75f;

            Debug.Log("Colisión detectada, reiniciando episodio");
            Debug.Log($"[EPISODIO] Recompensa total acumulada: {totalEpisodeReward}");

            if (policeAgent != null)
            {
                policeAgent.OnRunnerCollided();
            }

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
    public void NotifyPoliceRunnerReachedGoal()
    {
        if (policeAgent != null)
        {
            policeAgent.OnRunnerReachedGoal();
        }
    }

    
}