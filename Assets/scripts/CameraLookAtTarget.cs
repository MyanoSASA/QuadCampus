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

    [Header("スワイプ視線オフセット")]
    [Tooltip("スワイプで視線に任意オフセットを加えられるようにするか。")]
    [SerializeField] private bool enableSwipeOffset = true;
    [Tooltip("スワイプの移動量に乗算する感度。")]
    [SerializeField] private float swipeSensitivity = 0.08f;
    [Tooltip("左右方向の最大オフセット角度（度）。0 以下で制限なし。")]
    [SerializeField] private float maxYawOffset = 75f;
    [Tooltip("上下方向の最大オフセット角度（度）。0 以下で制限なし。")]
    [SerializeField] private float maxPitchOffset = 45f;
    [Tooltip("指を離した後に中央へ戻る速度（度/秒）。0 で保持。")]
    [SerializeField] private float swipeReturnSpeedDegPerSec = 0f;
    [Tooltip("オンで左右スワイプ操作を反転。")]
    [SerializeField] private bool invertHorizontalSwipe = true;
    [Tooltip("オンで上下スワイプ操作を反転（上で見下ろす動き）。")]
    [SerializeField] private bool invertVerticalSwipe = true;

    [Header("スワイプ有効エリア")]
    [Tooltip("画面中央を基準とした左右の半幅（0-0.5）。")]
    [SerializeField, Range(0f, 0.5f)] private float swipeAreaHalfWidthNormalized = 0.2f;
    [Tooltip("スワイプを受け付ける最小の正規化 Y（0=下, 1=上）。")]
    [SerializeField, Range(0f, 1f)] private float swipeAreaMinYNormalized = 0.5f;
    [Tooltip("スワイプを受け付ける最大の正規化 Y（0=下, 1=上）。")]
    [SerializeField, Range(0f, 1f)] private float swipeAreaMaxYNormalized = 1f;

    private Vector2 swipeAngles;

    /// <summary>
    /// スクリプトからターゲットを差し替えたい場合に呼び出してください。
    /// </summary>
    /// <param name="newTarget">注視させたい Transform。</param>
    public void SetTarget(Transform newTarget)
    {
        target = newTarget;
    }

    private void Update()
    {
        if (!enableSwipeOffset)
        {
            return;
        }

        Vector2 delta = ReadSwipeDelta();

        if (delta.sqrMagnitude > 0f)
        {
            float horizontalSign = invertHorizontalSwipe ? -1f : 1f;
            float verticalSign = invertVerticalSwipe ? -1f : 1f;

            swipeAngles.x += delta.x * swipeSensitivity * horizontalSign;
            swipeAngles.y += delta.y * swipeSensitivity * verticalSign;

            if (maxYawOffset > 0f)
            {
                swipeAngles.x = Mathf.Clamp(swipeAngles.x, -maxYawOffset, maxYawOffset);
            }

            if (maxPitchOffset > 0f)
            {
                swipeAngles.y = Mathf.Clamp(swipeAngles.y, -maxPitchOffset, maxPitchOffset);
            }
        }
        else if (swipeReturnSpeedDegPerSec > 0f)
        {
            float step = swipeReturnSpeedDegPerSec * Time.deltaTime;
            swipeAngles = Vector2.MoveTowards(swipeAngles, Vector2.zero, step);
        }
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
        Quaternion offsetRotation = enableSwipeOffset
            ? Quaternion.Euler(-swipeAngles.y, swipeAngles.x, 0f)
            : Quaternion.identity;
        Quaternion finalRotation = offsetRotation * desiredRotation;

        if (rotationSpeedDegPerSec <= 0f)
        {
            transform.rotation = finalRotation;
        }
        else
        {
            float maxStep = rotationSpeedDegPerSec * Time.deltaTime;
            transform.rotation = Quaternion.RotateTowards(transform.rotation, finalRotation, maxStep);
        }
    }

    private Vector2 ReadSwipeDelta()
    {
        if (Input.touchCount == 1)
        {
            Touch touch = Input.GetTouch(0);
            if (touch.phase == TouchPhase.Moved && IsWithinSwipeArea(touch.position))
            {
                return touch.deltaPosition;
            }
        }

#if UNITY_EDITOR || UNITY_STANDALONE || UNITY_WEBGL
        if (Input.GetMouseButton(0) && IsWithinSwipeArea(Input.mousePosition))
        {
            float dx = Input.GetAxis("Mouse X");
            float dy = Input.GetAxis("Mouse Y");

            if (!Mathf.Approximately(dx, 0f) || !Mathf.Approximately(dy, 0f))
            {
                return new Vector2(dx, dy);
            }
        }
#endif

        return Vector2.zero;
    }

    private bool IsWithinSwipeArea(Vector3 screenPosition)
    {
        float normalizedX = Mathf.Approximately(Screen.width, 0f) ? 0.5f : screenPosition.x / Screen.width;
        float normalizedY = Mathf.Approximately(Screen.height, 0f) ? 0.5f : screenPosition.y / Screen.height;

        float minX = 0.5f - swipeAreaHalfWidthNormalized;
        float maxX = 0.5f + swipeAreaHalfWidthNormalized;

        return normalizedX >= minX
            && normalizedX <= maxX
            && normalizedY >= Mathf.Min(swipeAreaMinYNormalized, swipeAreaMaxYNormalized)
            && normalizedY <= Mathf.Max(swipeAreaMinYNormalized, swipeAreaMaxYNormalized);
    }
}

