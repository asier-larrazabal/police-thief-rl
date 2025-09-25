using UnityEngine;
using VehicleBehaviour;

public class PoliceAgent : MonoBehaviour
{
    [Header("Target")]
    public Transform fugitiveTarget; // El runner a perseguir
    public string fugitiveCarName = "Car01";
    
    [Header("AI Settings")]
    public float detectionRange = 50f; // Distancia máxima de persecución
    public float updateFrequency = 0.1f; // Frecuencia de actualización de IA
    
    private WheelVehicle wheelVehicle; // Referencia al script de físicas
    private float lastUpdateTime;
    
    void Start()
    {
        // Obtener el componente WheelVehicle (las físicas del asset)
        wheelVehicle = GetComponent<WheelVehicle>();
        
        if (wheelVehicle == null)
        {
            Debug.LogError("PoliceAI: No se encontró WheelVehicle en este objeto!");
            return;
        }
        
        // Desactivar el control del jugador para usar IA
        wheelVehicle.IsPlayer = false;
        
        // Buscar automáticamente el fugitivo si no está asignado
        if (fugitiveTarget == null)
        {
            GameObject fugitiveObject = GameObject.Find(fugitiveCarName);
            if (fugitiveObject != null)
            {
                fugitiveTarget = fugitiveObject.transform;
                Debug.Log($"Fugitivo encontrado: {fugitiveCarName}");
            }
            else
            {
                Debug.LogWarning($"No se encontró el fugitivo con nombre: {fugitiveCarName}");
            }
        }
    }
    
    void Update()
    {
        // Actualizar la IA a intervalos regulares para mejor rendimiento
        if (Time.time - lastUpdateTime >= updateFrequency)
        {
            UpdateAI();
            lastUpdateTime = Time.time;
        }
    }
    
        void UpdateAI()
    {
        if (wheelVehicle == null || fugitiveTarget == null) return;
        
        // Calcular distancia al fugitivo
        float distanceToTarget = Vector3.Distance(transform.position, fugitiveTarget.position);
        
        if (distanceToTarget > detectionRange)
        {
            wheelVehicle.Throttle = 0f;
            wheelVehicle.Steering = 0f;
            return;
        }
        
        // Calcular dirección hacia el fugitivo
        Vector3 directionToTarget = (fugitiveTarget.position - transform.position).normalized;
        
        // Calcular ángulo de giro necesario
        float angleToTarget = Vector3.SignedAngle(transform.forward, directionToTarget, Vector3.up);
        
        // Controlar aceleración
        wheelVehicle.Throttle = 1f;
        
        // Controlar dirección - MÁS AGRESIVO
        float steeringInput = angleToTarget / 30f; // Cambiado de 45f a 30f para giros más rápidos
        wheelVehicle.Steering = Mathf.Clamp(steeringInput, -1f, 1f);
        
        // Si el ángulo es muy grande, girar más agresivamente
        if (Mathf.Abs(angleToTarget) > 90f)
        {
            wheelVehicle.Steering = angleToTarget > 0 ? 1f : -1f; // Giro a fondo
            wheelVehicle.Throttle = 0.5f; // Reducir velocidad para girar mejor
        }
        
        // Usar boost si está lejos
        if (distanceToTarget > 20f)
        {
            wheelVehicle.boosting = true;
        }
        else
        {
            wheelVehicle.boosting = false;
        }
    }
    
    // Método para cambiar el objetivo de persecución
    public void SetTarget(Transform newTarget)
    {
        fugitiveTarget = newTarget;
    }
    
    // Método para activar/desactivar la IA
    public void SetAIActive(bool active)
    {
        enabled = active;
        if (!active)
        {
            wheelVehicle.Throttle = 0f;
            wheelVehicle.Steering = 0f;
            wheelVehicle.boosting = false;
        }
    }
}
