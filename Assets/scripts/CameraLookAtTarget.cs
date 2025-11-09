using UnityEngine;

/// <summary>
/// 指定したターゲットの方向を向き続けるカメラスクリプト。
/// MainCamera などにアタッチして使用してください。
/// </summary>
[DisallowMultipleComponent]
public class CameraLookAtTarget : MonoBehaviour
{
    [Header("ターゲット設定")]
    [Tooltip("注視する Transform。Inspector から設定してください。")]
    [SerializeField] private Transform target;

    [Tooltip("ターゲット位置に加算されるワールド空間オフセット。")]
    [SerializeField] private Vector3 worldOffset = Vector3.zero;

    [Header("回転設定")]
    [Tooltip("0 の場合は即座に向く。0 より大きい場合は毎秒何度まで回転するか。")]
    [SerializeField] private float rotationSpeedDegPerSec = 0f;

    /// <summary>
    /// スクリプトからターゲットを差し替えたい場合に呼び出してください。
    /// </summary>
    /// <param name="newTarget">注視させたい Transform。</param>
    public void SetTarget(Transform newTarget)
    {
        target = newTarget;
    }

    private void LateUpdate()
    {
        if (target == null)
        {
            return;
        }

        Vector3 lookPosition = target.position + worldOffset;
        Vector3 direction = lookPosition - transform.position;

        if (direction.sqrMagnitude <= Mathf.Epsilon)
        {
            return;
        }

        Quaternion desiredRotation = Quaternion.LookRotation(direction.normalized, Vector3.up);

        if (rotationSpeedDegPerSec <= 0f)
        {
            transform.rotation = desiredRotation;
        }
        else
        {
            float maxStep = rotationSpeedDegPerSec * Time.deltaTime;
            transform.rotation = Quaternion.RotateTowards(transform.rotation, desiredRotation, maxStep);
        }
    }
}

