using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class PoliceAgent : Agent
{
    [Header("Car Settings")]
    public float motorForce = 1000f; // Más potente que el fugitivo
    public float maxSteerAngle = 45f;

    [Header("Car Physics")]
    public float downForce = 150f;
    public float driftFactor = 0.98f; // Mejor agarre

    [Header("Target")]
    public Transform fugitiveTarget; // El coche a perseguir
    public string fugitiveCarName = "FugitiveCar";

    private Rigidbody rb;
    private Vector3 startPos;

    // Variables de control
    private float motorInput;
    private float steerInput;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        if (rb == null) rb = gameObject.AddComponent<Rigidbody>();

        // CONFIGURACIÓN PARA POLICÍA (más ágil)
        rb.mass = 350f; // Más ligero = más rápido
        rb.linearDamping = 0.05f;
        rb.angularDamping = 0.3f;
        rb.centerOfMass = new Vector3(0, -0.6f, 0);

        startPos = transform.position;

        // BUSCAR fugitivo automáticamente
        if (fugitiveTarget == null)
        {
            GameObject fugitiveObject = GameObject.Find(fugitiveCarName);
            if (fugitiveObject != null)
            {
                fugitiveTarget = fugitiveObject.transform;
                Debug.Log("Fugitivo encontrado: " + fugitiveCarName);
            }
            else
            {
                Debug.LogError("No se encontró un GameObject llamado: " + fugitiveCarName);
            }
        }
    }

    void FixedUpdate()
    {
        // APLICAR DOWNFORCE
        rb.AddForce(-transform.up * downForce * rb.linearVelocity.magnitude);

        // APLICAR FÍSICA
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
        // Police position and velocity (6 values)
        sensor.AddObservation(transform.localPosition);
        sensor.AddObservation(rb.linearVelocity);

        // Police forward direction (3 values)
        sensor.AddObservation(transform.forward);

        // Target information (7 values)
        if (fugitiveTarget != null)
        {
            Vector3 toTarget = fugitiveTarget.position - transform.position;
            sensor.AddObservation(toTarget.normalized);
            sensor.AddObservation(toTarget.magnitude / 50f);

            // Target velocity para predicción
            Rigidbody targetRb = fugitiveTarget.GetComponent<Rigidbody>();
            if (targetRb != null)
            {
                sensor.AddObservation(targetRb.linearVelocity);
            }
            else
            {
                sensor.AddObservation(Vector3.zero);
            }
        }
        else
        {
            sensor.AddObservation(Vector3.zero);
            sensor.AddObservation(0f);
            sensor.AddObservation(Vector3.zero);
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        motorInput = actionBuffers.ContinuousActions[0];
        steerInput = actionBuffers.ContinuousActions[1];

        // REWARDS para la policía
        AddReward(-0.001f); // Penalización por tiempo (urgencia)

        if (fugitiveTarget != null)
        {
            float distance = Vector3.Distance(transform.position, fugitiveTarget.position);

            // REWARD por acercarse
            if (distance < 5f)
            {
                AddReward(0.01f);
                if (distance < 3f)
                {
                    AddReward(0.1f); // Gran reward por estar muy cerca
                    if (distance < 1.5f)
                    {
                        AddReward(1f); // ¡ATRAPADO!
                        EndEpisode();
                    }
                }
            }

            // PENALIZACIÓN por alejarse mucho
            if (distance > 20f)
            {
                AddReward(-0.01f);
            }
        }
    }

    void ApplyMotor()
    {
        Vector3 forwardForce = transform.forward * motorInput * motorForce * 2.2f;
        rb.AddForce(forwardForce);
    }

    void ApplySteering()
    {
        float currentSpeed = rb.linearVelocity.magnitude;

        if (Mathf.Abs(steerInput) > 0.05f)
        {
            bool isMoving = currentSpeed > 0.3f || Mathf.Abs(motorInput) > 0.1f;

            if (isMoving)
            {
                float steerAngle = steerInput * maxSteerAngle;
                float speedFactor = Mathf.Clamp01(currentSpeed / 15f);
                float realSteerAngle = steerAngle * (1f - speedFactor * 0.2f);

                // STEERING POTENTE para persecución
                rb.AddTorque(transform.up * realSteerAngle * 45f);
                transform.Rotate(0, steerInput * 70f * Time.fixedDeltaTime, 0);
            }
            else
            {
                transform.Rotate(0, steerInput * 35f * Time.fixedDeltaTime, 0);
            }
        }
    }

    void ApplyTireFriction()
    {
        Vector3 forwardVelocity = Vector3.Project(rb.linearVelocity, transform.forward);
        Vector3 sidewaysVelocity = rb.linearVelocity - forwardVelocity;

        // MEJOR agarre para persecución
        rb.AddForce(-sidewaysVelocity * driftFactor * 25f);
        rb.AddForce(-forwardVelocity * 0.015f);

        float speed = rb.linearVelocity.magnitude;
        Vector3 airResistance = -rb.linearVelocity.normalized * speed * speed * 0.003f;
        rb.AddForce(airResistance);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // IA SIMPLE para testing (perseguir directamente)
        var actions = actionsOut.ContinuousActions;

        if (fugitiveTarget != null)
        {
            Vector3 toTarget = fugitiveTarget.position - transform.position;
            float angle = Vector3.SignedAngle(transform.forward, toTarget, Vector3.up);

            actions[0] = 1f; // Siempre acelerar
            actions[1] = Mathf.Clamp(angle / 45f, -1f, 1f); // Girar hacia el target
        }
        else
        {
            actions[0] = 0f;
            actions[1] = 0f;
        }
    }
void OnCollisionEnter(Collision collision)
{
    if (collision.gameObject.CompareTag("Wall"))
    {
        AddReward(-0.3f);
    }
    
    // DETECTAR colisión con fugitivo
    if (collision.gameObject.CompareTag("Player") || collision.gameObject.name.Contains("Fugitive"))
    {
        Debug.Log("¡POLICÍA ATRAPA AL FUGITIVO!");
        // Aquí puedes añadir lógica adicional si quieres
    }
}
    public void RestartPolice()
{
    transform.position = startPos;
    transform.rotation = Quaternion.identity;
    rb.linearVelocity = Vector3.zero;
    rb.angularVelocity = Vector3.zero;
    Debug.Log("Policía reiniciada");
}
}

