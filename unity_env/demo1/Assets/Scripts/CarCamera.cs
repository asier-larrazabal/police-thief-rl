using UnityEngine;

public class CarCamera : MonoBehaviour
{
    [Header("Target")]
    public Transform target;

    [Header("Camera Position")]
    public float distance = 20f;
    public float height = 10f;
    public float heightDamping = 2f;
    public float rotationDamping = 3f;

    void LateUpdate()
    {
        if (!target) return;

        // CALCULAR posición deseada detrás del coche
        float wantedRotationAngle = target.eulerAngles.y;
        float wantedHeight = target.position.y + height;

        float currentRotationAngle = transform.eulerAngles.y;
        float currentHeight = transform.position.y;

        // SUAVIZAR rotación y altura
        currentRotationAngle = Mathf.LerpAngle(currentRotationAngle, wantedRotationAngle, rotationDamping * Time.deltaTime);
        currentHeight = Mathf.Lerp(currentHeight, wantedHeight, heightDamping * Time.deltaTime);

        // CONVERTIR ángulo a posición
        Quaternion currentRotation = Quaternion.Euler(0, currentRotationAngle, 0);
        Vector3 position = target.position;
        position -= currentRotation * Vector3.forward * distance;
        position.y = currentHeight;

        // APLICAR posición
        transform.position = position;
        transform.LookAt(target.position + Vector3.up);
    }
}
