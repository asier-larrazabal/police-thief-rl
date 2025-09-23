using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class FugitiveAgent : Agent
{
    [Header("Car Settings")]
    public float motorForce = 800f;
    public float brakeForce = 1500f;
    public float maxSteerAngle = 30f;

    [Header("Car Physics")]
    public float downForce = 100f;
    public float driftFactor = 0.95f;
    public Transform centerOfMass;

    [Header("Police - Buscar por nombre")]
    public string policeCarName = "PoliceCar";

    private Rigidbody rb;
    private Vector3 startPos;
    private Transform policeTransform;

    // Variables de control
    private float motorInput;
    private float steerInput;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        if (rb == null) rb = gameObject.AddComponent<Rigidbody>();

        // CONFIGURACIÓN PARA MEJOR GIRO
        rb.mass = 400f; // ← Más ligero = gira mejor
        rb.linearDamping = 0.08f; // ← Menos drag
        rb.angularDamping = 0.5f; // ← IMPORTANTE: Menos resistencia al giro

        // Centro de masa más bajo para estabilidad
        rb.centerOfMass = new Vector3(0, -0.5f, 0);

        // CENTRO DE MASA MÁS BAJO (hace el coche más estable)
        if (centerOfMass != null)
            rb.centerOfMass = centerOfMass.localPosition;
        else
            rb.centerOfMass = new Vector3(0, -0.5f, 0);

        startPos = transform.position;

        // BUSCAR policía
        GameObject policeObject = GameObject.Find(policeCarName);
        if (policeObject != null)
        {
            policeTransform = policeObject.transform;
            Debug.Log("Policía encontrada: " + policeCarName);
        }
        else
        {
            Debug.LogError("No se encontró un GameObject llamado: " + policeCarName);
        }
    }

    void FixedUpdate()
    {
        // APLICAR DOWNFORCE (ayuda al agarre)
        rb.AddForce(-transform.up * downForce * rb.linearVelocity.magnitude);

        // APLICAR FÍSICA DE COCHE REAL
        ApplyMotor();
        ApplySteering();
        ApplyTireFriction();
    }

    public override void OnEpisodeBegin()
    {
        transform.position = startPos;
        transform.rotation = Quaternion.identity;
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Car position and velocity (6 values)
        sensor.AddObservation(transform.localPosition);
        sensor.AddObservation(rb.linearVelocity);

        // Car forward direction (3 values)
        sensor.AddObservation(transform.forward);

        // Police information (4 values)
        if (policeTransform != null)
        {
            Vector3 toPolice = policeTransform.position - transform.position;
            sensor.AddObservation(toPolice.normalized);
            sensor.AddObservation(toPolice.magnitude / 50f);
        }
        else
        {
            sensor.AddObservation(Vector3.zero);
            sensor.AddObservation(0f);
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // OBTENER inputs del ML-Agent
        motorInput = actionBuffers.ContinuousActions[0];
        steerInput = actionBuffers.ContinuousActions[1];

        // Las fuerzas se aplican en FixedUpdate()

        // Rewards
        AddReward(0.001f);

        if (policeTransform != null)
        {
            float dist = Vector3.Distance(transform.position, policeTransform.position);
            if (dist < 3f)
            {
                AddReward(-1f);
                EndEpisode();
            }
        }
    }

    void ApplyMotor()
    {
        // FUERZA DEL MOTOR MÁS POTENTE
        Vector3 forwardForce = transform.forward * motorInput * motorForce * 2f; // ← x2 más potente
        rb.AddForce(forwardForce);
    }

    void ApplySteering()
    {
        float currentSpeed = rb.linearVelocity.magnitude;

        if (Mathf.Abs(steerInput) > 0.05f)
        {
            // REALISMO 1: Solo gira bien cuando se mueve
            bool isMoving = currentSpeed > 0.3f || Mathf.Abs(motorInput) > 0.1f;

            if (isMoving)
            {
                float steerAngle = steerInput * maxSteerAngle;

                // REALISMO 2: Steering depende de la velocidad 
                float speedFactor = Mathf.Clamp01(currentSpeed / 12f);
                float realSteerAngle = steerAngle * (1f - speedFactor * 0.3f); // ← Menos penalización

                // STEERING MÁS POTENTE - pero realista
                // Parte física para realismo (aumentada)
                rb.AddTorque(transform.up * realSteerAngle * 40f); // ← De 15f a 40f

                // Parte directa para garantizar respuesta (aumentada)
                transform.Rotate(0, steerInput * 60f * Time.fixedDeltaTime, 0); // ← De 25f a 60f
            }
            else
            {
                // A velocidad cero: giro más potente
                transform.Rotate(0, steerInput * 30f * Time.fixedDeltaTime, 0); // ← De 10f a 30f
            }
        }
    }

    void ApplyTireFriction()
    {
        // FRICCIÓN MENOS AGRESIVA para permitir más movimiento
        Vector3 forwardVelocity = Vector3.Project(rb.linearVelocity, transform.forward);
        Vector3 sidewaysVelocity = rb.linearVelocity - forwardVelocity;

        // REDUCIR fricción para más movimiento
        rb.AddForce(-sidewaysVelocity * driftFactor * 20f); // ← Reducido de 50f
        rb.AddForce(-forwardVelocity * 0.02f); // ← Reducido de 0.05f

        // MENOS resistencia del aire
        float speed = rb.linearVelocity.magnitude;
        Vector3 airResistance = -rb.linearVelocity.normalized * speed * speed * 0.005f; // ← Reducido
        rb.AddForce(airResistance);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var actions = actionsOut.ContinuousActions;
        actions[0] = Input.GetAxis("Vertical");   // Forward/Backward
        actions[1] = Input.GetAxis("Horizontal"); // Turn Left/Right
    }

    void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Wall"))
        {
            AddReward(-0.5f);
        }
    }

    // VISUALIZACIÓN para debugging
    void OnDrawGizmos()
    {
        if (rb != null)
        {
            // MOSTRAR centro de masa
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(transform.TransformPoint(rb.centerOfMass), 0.1f);

            // MOSTRAR velocidad
            Gizmos.color = Color.blue;
            Gizmos.DrawLine(transform.position, transform.position + rb.linearVelocity);
        }
    }
}
