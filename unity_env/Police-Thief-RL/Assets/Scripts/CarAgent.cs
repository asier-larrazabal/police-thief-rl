using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using VehicleBehaviour;

public class FugitiveAgent : Agent
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
    public int numRaycasts = 5;
    public float raySpread = 45f;
    public bool showDebugRays = true;
    
    [Header("Debug")]
    public bool showRewardLogs = false;
    
    private WheelVehicle wheelVehicle;
    private Rigidbody rb;
    
    // Variables para guardar la posición inicial
    private Vector3 initialPosition;
    private Quaternion initialRotation;
    
    public override void Initialize()
    {
        wheelVehicle = GetComponent<WheelVehicle>();
        rb = GetComponent<Rigidbody>();
        
        // Guardar la posición inicial del agente
        initialPosition = transform.localPosition;
        initialRotation = transform.localRotation;
        
        if (wheelVehicle == null)
        {
            Debug.LogError("FugitiveAgent: No se encontró WheelVehicle!");
        }
        
        if (rb == null)
        {
            Debug.LogError("FugitiveAgent: No se encontró Rigidbody!");
        }
        
        // Buscar policía automáticamente
        if (policeTarget == null)
        {
            GameObject policeObject = GameObject.Find(policeCarName);
            if (policeObject != null)
            {
                policeTarget = policeObject.transform;
                Debug.Log($"Policía encontrado: {policeCarName}");
            }
        }
        
        // Desactivar control del jugador para ML-Agents
        if (wheelVehicle != null)
        {
            wheelVehicle.IsPlayer = false;
        }
    }
    
    public override void CollectObservations(VectorSensor sensor)
{
    // Observaciones manuales útiles para conducción y huida/persecución:
    sensor.AddObservation(transform.localPosition); // 3
    sensor.AddObservation(transform.localRotation); // 4 (quaternion)
    sensor.AddObservation(rb.linearVelocity);       // 3
    sensor.AddObservation(rb.angularVelocity);      // 3
    sensor.AddObservation(policeTarget != null 
        ? transform.InverseTransformPoint(policeTarget.position) 
        : Vector3.zero);                           // 3
    // NO añadas raycasts manuales si usas RayPerceptionSensor 3D
}
    
    public override void OnActionReceived(ActionBuffers actions)
    {
        if (wheelVehicle == null) return;
        
        // Obtener acciones continuas
        float steering = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float throttle = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        
        // Aplicar acciones al vehículo
        wheelVehicle.Steering = steering;
        wheelVehicle.Throttle = throttle;
    
        // Sistema de recompensas
        CalculateRewards();
    }
    
    void CalculateRewards()
    {
        if (rb == null) return;
        
        // 1. RECOMPENSA FUERTE POR MOVERSE HACIA DELANTE
        float forwardSpeed = Vector3.Dot(rb.linearVelocity, transform.forward);
        if (forwardSpeed > 0)
        {
            AddReward(forwardSpeed * 0.1f); // Recompensa por velocidad hacia delante
        }
        else
        {
            AddReward(forwardSpeed * 0.05f); // Penalización leve por ir hacia atrás
        }
        
        // 2. PENALIZACIÓN POR ESTAR QUIETO
        float totalSpeed = rb.linearVelocity.magnitude;
        if (totalSpeed < 0.5f) // Si está prácticamente quieto
        {
            AddReward(-0.01f); // Penalización por no moverse
        }
        
        // 3. RECOMPENSA POR SUPERVIVENCIA
        AddReward(0.001f);
        
        // 4. VERIFICAR COLISIONES (más tolerante)
        if (CheckCollisionSoft())
        {
            AddReward(-0.1f); // Penalización suave por proximidad a obstáculos
        }
        
        // 5. COLISIÓN CRÍTICA
        if (CheckCollisionCritical())
        {
            AddReward(-1.0f);
            EndEpisode();
            return;
        }
        
        // 6. Sistema del policía (solo si existe y está activo)
        if (policeTarget != null)
        {
            float distanceToPolice = Vector3.Distance(transform.position, policeTarget.position);
            
            if (distanceToPolice > escapeDistance)
            {
                AddReward(0.05f); // Recompensa por mantenerse alejado
            }
            else if (distanceToPolice < escapeDistance * 0.5f)
            {
                AddReward(-0.02f); // Penalización por estar muy cerca
            }
            
            // Captura crítica
            if (distanceToPolice < 3f)
            {
                AddReward(-0.5f);
                EndEpisode();
            }
        }
        
        // DEBUG: Mostrar información de recompensas
        if (showRewardLogs && Time.frameCount % 60 == 0)
        {
            Debug.Log($"Velocidad total: {totalSpeed:F2}, " +
                     $"Velocidad adelante: {forwardSpeed:F2}, " +
                     $"Recompensa acumulada: {GetCumulativeReward():F3}");
        }
    }
    
    // Método auxiliar para colisión suave (detección temprana)
    bool CheckCollisionSoft()
    {
        Vector3 origin = transform.position + Vector3.up * 0.5f;
        return Physics.Raycast(origin, transform.forward, 8f, obstacleLayerMask);
    }
    
    // Método auxiliar para colisión crítica (muy cerca)
    bool CheckCollisionCritical()
    {
        Vector3 origin = transform.position + Vector3.up * 0.5f;
        return Physics.Raycast(origin, transform.forward, 2f, obstacleLayerMask);
    }
    
    public override void OnEpisodeBegin()
    {
        // Restaurar la posición inicial
        transform.localPosition = initialPosition;
        transform.localRotation = initialRotation;
        
        // Opcional: Añadir ligera variación para mejor entrenamiento
        Vector3 randomOffset = new Vector3(
            Random.Range(-1f, 1f),
            0,
            Random.Range(-1f, 1f)
        );
        transform.localPosition += randomOffset;
        
        // Reiniciar física
        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }
        
        // Reiniciar controles del vehículo
        if (wheelVehicle != null)
        {
            wheelVehicle.Steering = 0f;
            wheelVehicle.Throttle = 0f;
            wheelVehicle.boosting = false;
        }
    }
    
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // Control manual para pruebas (WASD/Flechas)
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Horizontal"); // Steering
        continuousActionsOut[1] = Input.GetAxis("Vertical");   // Throttle
    }
    
    // Métodos auxiliares para debugging y configuración
    public void SetPoliceTarget(Transform newTarget)
    {
        policeTarget = newTarget;
    }
    
    public void ToggleDebugLogs()
    {
        showRewardLogs = !showRewardLogs;
    }
    
    void OnDrawGizmos()
    {
        // Rango de detección del policía
        if (policeTarget != null)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(transform.position, detectionRange);
            
            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(transform.position, escapeDistance);
            
            Gizmos.color = Color.red;
            Gizmos.DrawLine(transform.position, policeTarget.position);
        }
        
        // Rango de detección de obstáculos
        Gizmos.color = Color.blue;
        Gizmos.DrawWireSphere(transform.position, raycastDistance);
    }
}
