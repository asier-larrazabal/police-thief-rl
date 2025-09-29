using UnityEngine;

public class CarCamera : MonoBehaviour
{
    [Header("Target")]
    public Transform target;    

    [Header("Camera Position")]
    public float distance = 20f;
    public float height = 20f;
    public float heightDamping = 2f;
    public float rotationDamping = 3f;

    void LateUpdate()
    {
        if (!target) return;

        // CALCULAR posici�n deseada detr�s del coche
        float wantedRotationAngle = target.eulerAngles.y;
        float wantedHeight = target.position.y + height;

        float currentRotationAngle = transform.eulerAngles.y;
        float currentHeight = transform.position.y;

        // SUAVIZAR rotaci�n y altura
        currentRotationAngle = Mathf.LerpAngle(currentRotationAngle, wantedRotationAngle, rotationDamping * Time.deltaTime);
        currentHeight = Mathf.Lerp(currentHeight, wantedHeight, heightDamping * Time.deltaTime);

        // CONVERTIR �ngulo a posici�n
        Quaternion currentRotation = Quaternion.Euler(0, currentRotationAngle, 0);
        Vector3 position = target.position;
        position -= currentRotation * Vector3.forward * distance;
        position.y = currentHeight;

        // APLICAR posici�n
        transform.position = position;
        transform.LookAt(target.position + Vector3.up);
    }
}
