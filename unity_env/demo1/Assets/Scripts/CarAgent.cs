using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using VehicleBehaviour;

public class CarAgent : Agent
{
    public WheelVehicle controller; // Referencia a tu controlador Arcade Car Physics

    public override void Initialize()
    {
        // Puede usarse para inicializar algo si hace falta
    }

    public override void OnEpisodeBegin()
    {
        // Reinicia posición y estado del coche al comenzar episodio
        controller.ResetPos();
        controller.Throttle = 0f;
        controller.Steering = 0f;
        controller.Handbrake = false;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        if (controller != null)
        {
            float normalizedSpeed = controller.Speed / 100f;
            sensor.AddObservation(normalizedSpeed);
        }
        else
        {
            sensor.AddObservation(0f);
            Debug.LogWarning("Controller is not assigned in CarAgent.");
        }
    }


    public override void OnActionReceived(ActionBuffers actions)
    {
        float throttle = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float steering = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);

        controller.Throttle = throttle;
        controller.Steering = steering;

        // Recompensa simple: pequeño incentivo por ir hacia adelante sin chocar
        float reward = Mathf.Clamp(controller.Speed / 20f, 0f, 1f);
        AddReward(reward * 0.1f);

        // Penalización por ir marcha atrás o velocidad negativa
        if (controller.Speed < 0)
        {
            AddReward(-0.05f);
        }

        // Aquí podrías añadir detección de colisiones para penalizar y reiniciar episodio
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // Habilitar control manual para testeo con teclado
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Throttle") - Input.GetAxis("Brake");
        continuousActionsOut[1] = Input.GetAxis("Horizontal");
    }
}
