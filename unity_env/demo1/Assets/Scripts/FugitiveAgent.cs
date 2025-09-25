using UnityEngine;
using VehicleBehaviour;

public class FugitiveAgent : MonoBehaviour
{
    [Header("Target")]
    public Transform policeTarget; // El policía a evitar
    public string policeCarName = "PoliceCar"; // Nombre del objeto policía
    
    [Header("AI Settings")]
    public float detectionRange = 30f; // Distancia de detección del policía
    public float escapeDistance = 20f; // Distancia mínima para mantener
    public float updateFrequency = 0.1f; // Frecuencia de actualización de IA
    public bool allowPlayerControl = true; // Permitir control manual para pruebas
    
    [Header("Escape Behavior")]
    public float panicSpeedMultiplier = 1.2f; // Velocidad extra cuando está cerca el policía
    public float evasionAngle = 45f; // Ángulo de evasión cuando está muy cerca
    
    [Header("Obstacle Detection")]
    public float raycastDistance = 15f; // Distancia de detección de obstáculos
    public LayerMask obstacleLayerMask = -1; // Capas que se consideran obstáculos
    public int numRaycasts = 5; // Número de rayos (centro, izq, der, izq-ext, der-ext)
    public float raySpread = 45f; // Ángulo de apertura de los rayos laterales
    public bool showDebugRays = true; // Mostrar rayos en Scene view
    
    private WheelVehicle wheelVehicle; // Referencia al script de físicas
    private float lastUpdateTime;
    private bool isPlayerControlled = true;
    private bool isAvoiding = false; // Si está evitando un obstáculo
    private float avoidanceMultiplier = 0f; // Dirección de evasión (-1 izq, +1 der)
    
    void Start()
    {
        // Obtener el componente WheelVehicle (las físicas del asset)
        wheelVehicle = GetComponent<WheelVehicle>();
        
        if (wheelVehicle == null)
        {
            Debug.LogError("FugitiveAI: No se encontró WheelVehicle en este objeto!");
            return;
        }
        
        // Configurar control inicial
        wheelVehicle.IsPlayer = allowPlayerControl;
        isPlayerControlled = allowPlayerControl;
        
        // Buscar automáticamente el policía si no está asignado
        if (policeTarget == null)
        {
            GameObject policeObject = GameObject.Find(policeCarName);
            if (policeObject != null)
            {
                policeTarget = policeObject.transform;
                Debug.Log($"Policía encontrado: {policeCarName}");
            }
            else
            {
                Debug.LogWarning($"No se encontró el policía con nombre: {policeCarName}");
            }
        }
    }
    
    void Update()
    {
        // Permitir cambiar entre control manual y IA
        if (allowPlayerControl && Input.GetKeyDown(KeyCode.Tab))
        {
            ToggleControlMode();
        }
        
        // Solo ejecutar IA si no está bajo control del jugador
        if (!isPlayerControlled && Time.time - lastUpdateTime >= updateFrequency)
        {
            DetectObstacles();
            UpdateEscapeAI();
            lastUpdateTime = Time.time;
        }
    }
    
    void DetectObstacles()
    {
        isAvoiding = false;
        avoidanceMultiplier = 0f;
        
        Vector3 origin = transform.position + Vector3.up * 0.5f; // Elevar un poco el origen
        
        // Array de ángulos para los rayos: centro, izquierda, derecha, izq-extremo, der-extremo
        float[] rayAngles = { 0f, -raySpread/2f, raySpread/2f, -raySpread, raySpread };
        float[] rayWeights = { 1.0f, 0.7f, 0.7f, 0.5f, 0.5f }; // Peso de cada rayo
        
        for (int i = 0; i < numRaycasts && i < rayAngles.Length; i++)
        {
            Vector3 rayDirection = Quaternion.Euler(0, rayAngles[i], 0) * transform.forward;
            RaycastHit hit;
            
            if (Physics.Raycast(origin, rayDirection, out hit, raycastDistance, obstacleLayerMask))
            {
                // Ignorar el terreno si tiene el tag "Terrain"
                if (hit.collider.CompareTag("Terrain")) continue;
                
                isAvoiding = true;
                
                // Calcular dirección de evasión basada en el rayo que detectó
                if (i == 0) // Rayo central - usar normal de la superficie
                {
                    // Si hay obstáculo al frente, decidir dirección basada en la normal
                    float normalDirection = Vector3.Dot(hit.normal, transform.right);
                    avoidanceMultiplier += normalDirection > 0 ? 1f : -1f;
                }
                else if (rayAngles[i] < 0) // Rayos izquierdos - girar a la derecha
                {
                    avoidanceMultiplier += rayWeights[i];
                }
                else // Rayos derechos - girar a la izquierda
                {
                    avoidanceMultiplier -= rayWeights[i];
                }
                
                // Debug visual
                if (showDebugRays)
                {
                    Debug.DrawLine(origin, hit.point, Color.red, 0.1f);
                }
            }
            else if (showDebugRays)
            {
                Debug.DrawLine(origin, origin + rayDirection * raycastDistance, Color.green, 0.1f);
            }
        }
        
        // Normalizar el multiplicador de evasión
        avoidanceMultiplier = Mathf.Clamp(avoidanceMultiplier, -1f, 1f);
    }
    
    void UpdateEscapeAI()
    {
        if (wheelVehicle == null) return;
        
        float finalSteering = 0f;
        float finalThrottle = 0.8f;
        bool useBoost = false;
        
        // PRIORIDAD 1: Evitar obstáculos
        if (isAvoiding)
        {
            finalSteering = avoidanceMultiplier;
            finalThrottle = 0.6f; // Reducir velocidad al evitar obstáculos
            
            // Si hay obstáculo muy cerca, retroceder brevemente
            Vector3 frontCheck = transform.position + transform.forward * 3f;
            if (Physics.CheckSphere(frontCheck, 2f, obstacleLayerMask))
            {
                finalThrottle = -0.5f; // Marcha atrás
            }
        }
        // PRIORIDAD 2: Escapar del policía
        else if (policeTarget != null)
        {
            float distanceToPolice = Vector3.Distance(transform.position, policeTarget.position);
            
            if (distanceToPolice < detectionRange)
            {
                Vector3 directionFromPolice = (transform.position - policeTarget.position).normalized;
                
                // Si está MUY cerca, hacer maniobra evasiva
                if (distanceToPolice < escapeDistance)
                {
                    // Maniobra evasiva - girar bruscamente
                    float evasionDirection = Random.Range(-1f, 1f) > 0 ? 1f : -1f;
                    Vector3 evasionVector = Quaternion.Euler(0, evasionAngle * evasionDirection, 0) * directionFromPolice;
                    
                    float angleToEvasion = Vector3.SignedAngle(transform.forward, evasionVector, Vector3.up);
                    finalSteering = Mathf.Clamp(angleToEvasion / 45f, -1f, 1f);
                    
                    // Acelerar con pánico
                    finalThrottle = 1f * panicSpeedMultiplier;
                    useBoost = true; // Usar boost en emergencia
                }
                else
                {
                    // Escape normal - huir en dirección opuesta
                    float angleToEscape = Vector3.SignedAngle(transform.forward, directionFromPolice, Vector3.up);
                    finalSteering = Mathf.Clamp(angleToEscape / 45f, -1f, 1f);
                    
                    // Acelerar para escapar
                    finalThrottle = 1f;
                    useBoost = distanceToPolice < detectionRange * 0.7f;
                }
            }
            else
            {
                // Policía lejos - movimiento normal con ligero zigzag
                finalSteering = Random.Range(-0.1f, 0.1f);
                finalThrottle = 0.8f;
                useBoost = false;
            }
        }
        else
        {
            // Sin policía - movimiento normal
            finalSteering = Random.Range(-0.1f, 0.1f);
            finalThrottle = 0.8f;
            useBoost = false;
        }
        
        // Aplicar controles al vehículo
        wheelVehicle.Steering = finalSteering;
        wheelVehicle.Throttle = finalThrottle;
        wheelVehicle.boosting = useBoost;
    }
    
    // Cambiar entre control manual y IA
    void ToggleControlMode()
    {
        isPlayerControlled = !isPlayerControlled;
        wheelVehicle.IsPlayer = isPlayerControlled;
        
        if (!isPlayerControlled)
        {
            Debug.Log("Modo IA activado - El fugitivo escapará automáticamente");
        }
        else
        {
            Debug.Log("Modo manual activado - Controla con WASD");
        }
    }
    
    // Método para forzar modo IA
    public void SetAIMode(bool aiMode)
    {
        isPlayerControlled = !aiMode;
        wheelVehicle.IsPlayer = !aiMode;
    }
    
    // Método para cambiar el objetivo a evitar
    public void SetPoliceTarget(Transform newPolice)
    {
        policeTarget = newPolice;
    }
    
    // Método para activar/desactivar la IA
    public void SetAIActive(bool active)
    {
        enabled = active;
        if (!active && !isPlayerControlled)
        {
            wheelVehicle.Throttle = 0f;
            wheelVehicle.Steering = 0f;
            wheelVehicle.boosting = false;
        }
    }
    
    // Información visual en el editor
    void OnDrawGizmos()
    {
        if (policeTarget != null)
        {
            // Mostrar rango de detección del policía
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(transform.position, detectionRange);
            
            // Mostrar rango de escape crítico
            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(transform.position, escapeDistance);
            
            // Línea al policía
            Gizmos.color = Color.red;
            Gizmos.DrawLine(transform.position, policeTarget.position);
        }
        
        // Mostrar rango de detección de obstáculos
        Gizmos.color = Color.blue;
        Gizmos.DrawWireSphere(transform.position, raycastDistance);
        
        // Mostrar dirección de los rayos de detección
        if (Application.isPlaying && showDebugRays)
        {
            Vector3 origin = transform.position + Vector3.up * 0.5f;
            float[] rayAngles = { 0f, -raySpread/2f, raySpread/2f, -raySpread, raySpread };
            
            Gizmos.color = Color.cyan;
            for (int i = 0; i < numRaycasts && i < rayAngles.Length; i++)
            {
                Vector3 rayDirection = Quaternion.Euler(0, rayAngles[i], 0) * transform.forward;
                Gizmos.DrawLine(origin, origin + rayDirection * raycastDistance);
            }
        }
    }
}
