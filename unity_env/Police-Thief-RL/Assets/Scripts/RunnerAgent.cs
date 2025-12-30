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

    [Header("Curriculum Learning")]
    [SerializeField] private float difficultyLevel = 3f;
    private float speedMultiplier = 0f;

    [Header("Multi-Objetivo")]
    [SerializeField] private Vector3[] targetPositions;
    [SerializeField] private Transform targetTransform;
    [SerializeField] private float goalThreshold = 5.4f;
    [SerializeField] private float maxSteeringAngle = 45f;

    private bool hasCollided = false;
    private float bestDistanceToGoal;
    private float totalEpisodeReward;
    private float prevDistanceToGoal;

    // Multi-objetivo: 2 cubos por episodio
    private int objetivosRestantes;
    private int objetivoActualIndex;
    private float[] lastDistances = new float[20];
    private int lastDistancesIndex = 0;

    // Anti-stuck y alineación
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
        UpdateSpeedMultiplier();
    }

    public override void OnEpisodeBegin()
    {
        bestDistanceToGoal = float.MaxValue;
        hasCollided = false;
        totalEpisodeReward = 0f;
        stuckCounter = 0;

        // Validación: asegurar que targetPositions está asignado
        if (targetPositions == null || targetPositions.Length == 0)
        {
            Debug.LogError("ERROR: targetPositions no está asignado en el Inspector de RunnerAgent. Asigna 3 cubos.");
            return;
        }

        if (targetTransform == null)
        {
            Debug.LogError("ERROR: targetTransform no está asignado en el Inspector de RunnerAgent.");
            return;
        }

        // Asegurar que tenemos al menos 1 cubo para los 3 objetivos
        if (targetPositions.Length < 1)
        {
            Debug.LogError("ERROR: Se requiere al menos 1 cubo en targetPositions");
            return;
        }

        // Iniciar con 2 objetivos (cubos)
        objetivosRestantes = 2;
        objetivoActualIndex = Random.Range(0, targetPositions.Length);
        targetTransform.localPosition = targetPositions[objetivoActualIndex];
        
        Debug.Log($"OnEpisodeBegin: Iniciando episodio con {objetivosRestantes} objetivos. Objetivo actual: {objetivoActualIndex}");

        // Resetear posición y rotación del agente
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

        Vector3 toGoal = (targetTransform.position - transform.position).normalized;
        alignmentPrev = Vector3.Dot(transform.forward, toGoal);

        for (int i = 0; i < lastDistances.Length; i++)
            lastDistances[i] = prevDistanceToGoal;
        lastDistancesIndex = 0;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Observations: 17 total (compatible con PoliceAgent)
        sensor.AddObservation(transform.localPosition);      // 3
        sensor.AddObservation(transform.localRotation);      // 4
        sensor.AddObservation(rb.linearVelocity);            // 3
        sensor.AddObservation(rb.angularVelocity);           // 3

        if (targetTransform != null)
        {
            Vector3 relPos = targetTransform.position - transform.position;
            sensor.AddObservation(relPos.normalized);        // 3 (direction)
            sensor.AddObservation(relPos.magnitude);         // 1 (distance)
        }
        else
        {
            sensor.AddObservation(Vector3.zero);             // 3
            sensor.AddObservation(0f);                       // 1
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
        wheelVehicle.Throttle = throttle * speedMultiplier;

        // Penalización por tiempo (reducida para dar más margen)
        AddReward(-0.002f);

        // Detección de stuck
        if (rb.linearVelocity.magnitude < 0.5f)
        {
            stuckCounter++;
            if (stuckCounter > 60)
            {
                AddReward(-0.5f);
                stuckCounter = 0;
            }
        }
        else
        {
            stuckCounter = 0;
        }

        // Recompensa por alineación
        if (targetTransform != null)
        {
            Vector3 toGoal = (targetTransform.position - transform.position).normalized;
            float alignment = Vector3.Dot(transform.forward, toGoal);
            float alignmentDelta = alignment - alignmentPrev;
            if (alignmentDelta > 0)
                AddReward(alignmentDelta * 0.025f);
            alignmentPrev = alignment;
        }

        // Recompensas por distancia
        if (targetTransform != null)
        {
            float distanceToGoal = Vector3.Distance(transform.position, targetTransform.position);

            // Exponential decay reward solo por mejora de mejor distancia
            if (distanceToGoal < bestDistanceToGoal)
            {
                bestDistanceToGoal = distanceToGoal;
                float alpha = 0.25f;
                float beta = 0.05f;
                float reward = alpha * Mathf.Exp(-beta * distanceToGoal);
                AddReward(reward);
                totalEpisodeReward += reward;
            }

            // Historial de distancias para detectar "stuck" en una ruta
            lastDistances[lastDistancesIndex] = distanceToGoal;
            lastDistancesIndex = (lastDistancesIndex + 1) % lastDistances.Length;

            float minDist = lastDistances[0], maxDist = lastDistances[0];
            for (int i = 1; i < lastDistances.Length; i++)
            {
                if (lastDistances[i] < minDist) minDist = lastDistances[i];
                if (lastDistances[i] > maxDist) maxDist = lastDistances[i];
            }
            if ((maxDist - minDist < 0.5f) && (distanceToGoal > goalThreshold + 2f))
                AddReward(-0.3f);

            // Recompensa por reducción de distancia
            float deltaDist = prevDistanceToGoal - distanceToGoal;
            if (deltaDist > 0)
            {
                float advanceReward = deltaDist * 0.07f;
                AddReward(advanceReward);
            }
            prevDistanceToGoal = distanceToGoal;

            // Objetivo alcanzado
            if (distanceToGoal < goalThreshold)
            {
                AddReward(150f);
                totalEpisodeReward += 150f;
                int objetivosAlcanzados = 2 - objetivosRestantes + 1;
                Debug.Log($"[OBJETIVO {objetivosAlcanzados}] Alcanzado! Objetivos restantes ANTES: {objetivosRestantes}");

                objetivosRestantes--;
                
                Debug.Log($"[DEBUG] objetivosRestantes DESPUÉS de decrementar: {objetivosRestantes}");

                if (objetivosRestantes > 0)
                {
                    Debug.Log($"[MULTI-OBJETIVO] Aún quedan {objetivosRestantes} objetivos. Asignando nuevo objetivo...");
                    
                    // Asignar nuevo objetivo diferente
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
                    for (int i = 0; i < lastDistances.Length; i++)
                        lastDistances[i] = prevDistanceToGoal;
                    lastDistancesIndex = 0;
                    
                    Debug.Log($"[NUEVO OBJETIVO] Asignado índice {objetivoActualIndex}. Distancia: {bestDistanceToGoal:F2}");
                }
                else
                {
                    // 2 objetivos completados, fin de episodio
                    Debug.Log($"[EPISODIO COMPLETADO] Los 2 objetivos alcanzados. Recompensa total: {totalEpisodeReward}. LLAMANDO A EndEpisode()");
                    
                    // Notificar al PoliceAgent que el runner completó su objetivo (escapó)
                    if (policeAgent != null)
                    {
                        policeAgent.OnRunnerReachedGoal();
                    }
                    
                    EndEpisode();
                }
            }
        }

        // Colisión con pared
        if (hasCollided)
        {
            AddReward(-50f);
            totalEpisodeReward += -50f;
            Debug.Log("Colisión detectada. Fin de episodio.");
            
            // Notificar al PoliceAgent que el runner colisionó
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

    private void UpdateSpeedMultiplier()
    {
        switch ((int)difficultyLevel)
        {
            case 0: speedMultiplier = 0f; break;      // Estático
            case 1: speedMultiplier = 0.3f; break;    // 30%
            case 2: speedMultiplier = 0.6f; break;    // 60%
            case 3: speedMultiplier = 1.0f; break;    // 100%
            default: speedMultiplier = Mathf.Clamp(difficultyLevel / 3f, 0f, 1.0f); break;
        }
    }

    public void SetDifficultyLevel(float level)
    {
        difficultyLevel = Mathf.Clamp(level, 0f, 3f);
        UpdateSpeedMultiplier();
    }

    public float GetDistanceToCurrentGoal()
    {
        if (targetTransform == null) return float.MaxValue;
        return Vector3.Distance(transform.position, targetTransform.position);
    }

    public Vector3 GetDirectionToGoal()
    {
        if (targetTransform == null) return Vector3.zero;
        return (targetTransform.position - transform.position).normalized;
    }

    public Vector3 GetCurrentGoalPosition()
    {
        return targetTransform != null ? targetTransform.position : Vector3.zero;
    }
}