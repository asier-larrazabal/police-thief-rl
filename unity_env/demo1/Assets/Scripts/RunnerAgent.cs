using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using VehicleBehaviour; // Asegúrate de que este namespace contiene tu script de WheelVehicle

public class RunnerAgent : Agent
{
    [Header("Referencias")]
    [SerializeField] private PoliceAgent policeAgent;
    [SerializeField] private WheelVehicle wheelVehicle;
    [SerializeField] private Rigidbody rb;

    [Header("Objetivo y Configuración")]
    [SerializeField] private Transform targetTransform; 
    [SerializeField] private Vector3[] targetPositions; // Array de posiciones posibles para el objetivo
    [SerializeField] private float goalThreshold = 4.0f; // Distancia mínima para completar
    [SerializeField] private float maxSteeringAngle = 45f;

    // Estado interno
    private Vector3 initialPosition;
    private Quaternion initialRotation;
    private float bestDistanceToGoal; // Récord de distancia mínima en el episodio actual
    private bool hasCollided = false;

    // Inicialización única al arrancar
    public override void Initialize()
    {
        // Obtener componentes si no están asignados
        if (wheelVehicle == null) wheelVehicle = GetComponent<WheelVehicle>();
        if (rb == null) rb = GetComponent<Rigidbody>();

        // Guardar posición inicial original
        initialPosition = transform.localPosition;
        initialRotation = transform.localRotation;

        // Configurar vehículo para IA
        if (wheelVehicle != null) 
            wheelVehicle.IsPlayer = false;
    }

    // Se llama cada vez que empieza un nuevo episodio
    public override void OnEpisodeBegin()
    {
        // 1. Resetear variables de estado
        hasCollided = false;
        bestDistanceToGoal = float.MaxValue; 

        // 2. Resetear física del coche
        if (rb != null)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }
        if (wheelVehicle != null)
        {
            wheelVehicle.Steering = 0f;
            wheelVehicle.Throttle = 0f;
            wheelVehicle.Handbrake = false; // Asegurar que no empiece frenado
        }

        // 3. Posición y Rotación Aleatoria del Agente
        // Aleatoriedad en posición (pequeña variación para no salir del área)
        transform.localPosition = initialPosition + new Vector3(Random.Range(-2f, 2f), 0, Random.Range(-2f, 2f));
        // Aleatoriedad en rotación: ¡CRUCIAL para generalizar! (360 grados)
        transform.localRotation = Quaternion.Euler(0, Random.Range(0f, 360f), 0);

        // 4. Posición Aleatoria del Objetivo
        if (targetPositions != null && targetPositions.Length > 0 && targetTransform != null)
        {
            // Elegir una posición al azar del array predefinido
            int index = Random.Range(0, targetPositions.Length);
            targetTransform.localPosition = targetPositions[index];
        }
        
        // Actualizar distancia inicial como "mejor distancia" inicial si queremos ser estrictos,
        // o dejar float.MaxValue para que el primer paso cuente como mejora.
        // Dejaremos float.MaxValue para que el primer acercamiento cuente.
    }

    // Observaciones que ve la red neuronal
    public override void CollectObservations(VectorSensor sensor)
    {
        // 1. Datos propios
        sensor.AddObservation(transform.localPosition);
        sensor.AddObservation(transform.localRotation);
        sensor.AddObservation(rb.linearVelocity);
        sensor.AddObservation(rb.angularVelocity);

        // 2. Datos del objetivo relativo
        if (targetTransform != null)
        {
            Vector3 dirToTarget = (targetTransform.position - transform.position).normalized;
            float distToTarget = Vector3.Distance(transform.position, targetTransform.position);
            
            sensor.AddObservation(dirToTarget); // Dirección hacia donde ir
            sensor.AddObservation(distToTarget); // Cuánto falta
        }
        else
        {
            sensor.AddObservation(Vector3.zero);
            sensor.AddObservation(0f);
        }

        // NOTA: El RayPerceptionSensor3D se añade automáticamente como observación 
        // si el componente está en el mismo GameObject.
    }

    // Ejecución de acciones y recompensas
    public override void OnActionReceived(ActionBuffers actions)
    {
        // Interpretación de acciones discretas
        int steeringAction = actions.DiscreteActions[0]; // 0:Izq, 1:Nada, 2:Der
        int throttleAction = actions.DiscreteActions[1]; // 0:Atrás, 1:Nada, 2:Adelante

        // Aplicar dirección
        float steering = 0f;
        if (steeringAction == 0) steering = -1f;
        else if (steeringAction == 2) steering = 1f;
        
        if (wheelVehicle != null) wheelVehicle.Steering = steering;

        // Aplicar aceleración
        float throttle = 0f;
        if (throttleAction == 0) throttle = -1f; // Reversa
        else if (throttleAction == 2) throttle = 1f; // Adelante

        if (wheelVehicle != null) wheelVehicle.Throttle = throttle;

        // --- SISTEMA DE RECOMPENSAS ---

        // 1. Penalización por tiempo (Existencia)
        // Castigo constante pequeño para incentivar velocidad
        AddReward(-0.005f); 

        if (targetTransform != null)
        {
            float currentDistance = Vector3.Distance(transform.position, targetTransform.position);

            // 2. Recompensa por MEJORA DE RÉCORD (Anti-Oscilación)
            // Solo premiamos si estamos más cerca que NUNCA ANTES en este episodio
            if (currentDistance < bestDistanceToGoal)
            {
                // Calculamos cuánto hemos batido el récord anterior
                // (usamos Mathf.Min para evitar saltos gigantes al inicio si bestDistance era MaxValue)
                float improvement = 0f;
                if(bestDistanceToGoal != float.MaxValue)
                {
                    improvement = bestDistanceToGoal - currentDistance;
                }

                // Solo damos recompensa si la mejora es razonable (evita bugs de teletransporte)
                if(improvement > 0.001f) 
                {
                    AddReward(improvement * 0.5f); 
                }

                // Actualizamos el nuevo récord
                bestDistanceToGoal = currentDistance;
            }

            // 3. Recompensa Final (Meta)
            if (currentDistance < goalThreshold)
            {
                AddReward(100.0f); // Gran premio final
                Debug.Log("Meta alcanzada!");
                EndEpisode();
            }
        }

        // 4. Penalización por Colisión
        if (hasCollided)
        {
            AddReward(-10.0f); // Castigo fuerte
            Debug.Log("Colisión!");
            EndEpisode();
        }
        
        // 5. Penalización por caerse del mapa (opcional, buena práctica)
        if (transform.localPosition.y < -5f)
        {
            AddReward(-10.0f);
            EndEpisode();
        }
    }

    // Detección de colisiones
    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Wall") || collision.gameObject.CompareTag("Walls"))
        {
            hasCollided = true;
        }
    }

    // Control manual para pruebas (Heuristic)
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var discreteActions = actionsOut.DiscreteActions;
        
        // Resetear
        discreteActions[0] = 1; 
        discreteActions[1] = 1;

        float h = Input.GetAxis("Horizontal");
        if (h < -0.1f) discreteActions[0] = 0;
        else if (h > 0.1f) discreteActions[0] = 2;

        float v = Input.GetAxis("Vertical");
        if (v < -0.1f) discreteActions[1] = 0;
        else if (v > 0.1f) discreteActions[1] = 2;
    }
    public Vector3 GetVelocity()
{
    if (rb != null)
    {
        return rb.linearVelocity; // En Unity 6 o superior usa linearVelocity
        // return rb.velocity;    // En Unity versiones anteriores a 6 usa velocity
    }
    return Vector3.zero;
}
}

