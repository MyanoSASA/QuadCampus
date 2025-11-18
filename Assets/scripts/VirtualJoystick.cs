using UnityEngine;
using UnityEngine.EventSystems;

[RequireComponent(typeof(RectTransform))]
public class VirtualJoystick : MonoBehaviour, IDragHandler, IPointerDownHandler, IPointerUpHandler
{
    [Header("参照")]
    [SerializeField] private RectTransform background;
    [SerializeField] private RectTransform handle;

    [Header("設定")]
    [SerializeField, Tooltip("ハンドルが移動できる範囲 (背景サイズの半分に対する倍率)")]
    private float handleRange = 1f;
    [SerializeField, Tooltip("入力のデッドゾーン (0-1)")]
    private float deadZone = 0.05f;
    [SerializeField, Tooltip("タッチを離した際にハンドルを中央へ戻す")]
    private bool resetOnRelease = true;
    [SerializeField, Tooltip("水平入力を反転する")] private bool invertHorizontal = false;
    [SerializeField, Tooltip("垂直入力を反転する")] private bool invertVertical = false;

    Canvas _canvas;
    RectTransform _rectTransform;
    Vector2 _output;
    bool _isHeld;

    public Vector2 Value => _output;
    public bool HasInput => _isHeld || _output.sqrMagnitude > 1e-4f;
    public bool IsHeld => _isHeld;

    void Awake()
    {
        _rectTransform = background != null ? background : GetComponent<RectTransform>();
        background = _rectTransform;
        if (handle == null && transform.childCount > 0)
        {
            handle = transform.GetChild(0) as RectTransform;
        }

        _canvas = GetComponentInParent<Canvas>();
        ResetHandle();
    }

    void OnEnable()
    {
        ResetHandle();
    }

    public void OnPointerDown(PointerEventData eventData)
    {
        _isHeld = true;
        UpdateHandle(eventData);
    }

    public void OnDrag(PointerEventData eventData)
    {
        UpdateHandle(eventData);
    }

    public void OnPointerUp(PointerEventData eventData)
    {
        _isHeld = false;
        _output = Vector2.zero;
        if (resetOnRelease)
        {
            ResetHandle();
        }
    }

    public void SetValue(Vector2 value)
    {
        _output = Vector2.ClampMagnitude(value, 1f);
        ApplyHandlePosition(_output);
    }

    void UpdateHandle(PointerEventData eventData)
    {
        if (_rectTransform == null) return;

        if (!RectTransformUtility.ScreenPointToLocalPointInRectangle(_rectTransform, eventData.position, GetEventCamera(eventData), out Vector2 localPoint))
        {
            return;
        }

        Vector2 radius = _rectTransform.sizeDelta * 0.5f;
        if (radius.x < Mathf.Epsilon || radius.y < Mathf.Epsilon)
        {
            _output = Vector2.zero;
            return;
        }

        Vector2 normalized = new Vector2(localPoint.x / radius.x, localPoint.y / radius.y);
        normalized = Vector2.ClampMagnitude(normalized, 1f);

        if (invertHorizontal) normalized.x *= -1f;
        if (invertVertical) normalized.y *= -1f;

        float magnitude = normalized.magnitude;
        if (deadZone > 0f && magnitude < deadZone)
        {
            normalized = Vector2.zero;
        }

        _output = normalized;
        ApplyHandlePosition(_output);
    }

    void ApplyHandlePosition(Vector2 value)
    {
        if (handle == null || _rectTransform == null) return;

        float clampedRange = Mathf.Max(0f, handleRange);
        Vector2 radius = _rectTransform.sizeDelta * 0.5f;
        Vector2 handlePosition = new Vector2(value.x * radius.x * clampedRange, value.y * radius.y * clampedRange);
        handle.anchoredPosition = handlePosition;
    }

    void ResetHandle()
    {
        if (handle != null)
        {
            handle.anchoredPosition = Vector2.zero;
        }
        _output = Vector2.zero;
    }

    Camera GetEventCamera(PointerEventData eventData)
    {
        if (eventData != null && eventData.pressEventCamera != null)
        {
            return eventData.pressEventCamera;
        }

        if (_canvas != null && _canvas.renderMode != RenderMode.ScreenSpaceOverlay)
        {
            return _canvas.worldCamera;
        }

        return null;
    }

    void OnValidate()
    {
        deadZone = Mathf.Clamp01(deadZone);
        handleRange = Mathf.Max(0f, handleRange);
        if (background == null)
        {
            background = GetComponent<RectTransform>();
        }
    }
}

