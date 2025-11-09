using UnityEngine;

#if UNITY_EDITOR
using UnityEditor;
#endif

[DisallowMultipleComponent]
public class HoverController : MonoBehaviour
{
    [Header("General")]
    [SerializeField] bool playOnAwake = true;
    [SerializeField] bool useLocalSpace = true;
    [SerializeField] Transform movementReference;

    [Header("Noise Settings")]
    [SerializeField] Vector3 positionAmplitude = new Vector3(0.02f, 0.01f, 0.02f);
    [SerializeField, Range(0.01f, 5f)] float positionFrequency = 0.6f;
    [SerializeField] Vector3 rotationAmplitude = new Vector3(1.2f, 1.2f, 1.2f);
    [SerializeField, Range(0.01f, 5f)] float rotationFrequency = 0.6f;

    [Tooltip("速度の滑らかさ（大きいほど鈍く、小さいほど反応が鋭い）")]
    [SerializeField, Range(0.01f, 30f)] float velocitySmoothing = 10f;

    [Header("Debug")]
    [SerializeField, ReadOnly] float forwardSpeed;
    [SerializeField, ReadOnly] float lateralSpeed;

    Vector3 _prevRefPos, _smoothedVelWS, _prevVelWS;
    Vector3 _currentPos;
    Quaternion _currentRot;
    Vector3 _basePos;
    Quaternion _baseRot;
    bool _running;

    void Awake()
    {
        if (movementReference == null) movementReference = transform;
        _prevRefPos = movementReference.position;
        _smoothedVelWS = Vector3.zero;
        _prevVelWS = Vector3.zero;

        _basePos = transform.localPosition;
        _baseRot = transform.localRotation;
        _currentPos = _basePos;
        _currentRot = _baseRot;
        _running = playOnAwake;
    }

    void LateUpdate()
    {
        if (!_running) return;
        Step(Time.deltaTime, Time.time);
    }

    void Step(float dt, float t)
    {
        // === 速度を算出 ===
        Vector3 refPos = movementReference.position;
        Vector3 instVelWS = (refPos - _prevRefPos) / Mathf.Max(dt, 1e-6f);
        _prevRefPos = refPos;

        _smoothedVelWS = Vector3.Lerp(_smoothedVelWS, instVelWS, 1f - Mathf.Exp(-velocitySmoothing * dt));

        // === ローカル変換 ===
        Vector3 velLS = transform.InverseTransformDirection(_smoothedVelWS);
        forwardSpeed = velLS.z;
        lateralSpeed = velLS.x;

        // === 加速度 ===
        Vector3 accel = (_smoothedVelWS - _prevVelWS) / Mathf.Max(dt, 1e-6f);
        Vector3 accelLS = transform.InverseTransformDirection(accel);
        _prevVelWS = _smoothedVelWS;

        // === ノイズ ===
        Vector3 posNoise = new Vector3(
            (Mathf.PerlinNoise(1, t * positionFrequency) * 2f - 1f) * positionAmplitude.x,
            (Mathf.PerlinNoise(2, t * positionFrequency) * 2f - 1f) * positionAmplitude.y,
            (Mathf.PerlinNoise(3, t * positionFrequency) * 2f - 1f) * positionAmplitude.z
        );
        Vector3 rotNoise = new Vector3(
            (Mathf.PerlinNoise(4, t * rotationFrequency) * 2f - 1f) * rotationAmplitude.x,
            (Mathf.PerlinNoise(5, t * rotationFrequency) * 2f - 1f) * rotationAmplitude.y,
            (Mathf.PerlinNoise(6, t * rotationFrequency) * 2f - 1f) * rotationAmplitude.z
        );

        // === 基準 ===
        Vector3 targetPos = _basePos + posNoise;
        Quaternion targetRot = _baseRot * Quaternion.Euler(rotNoise);

        // === スムーズ追従 ===
        _currentPos = Vector3.Lerp(_currentPos, targetPos, 1f - Mathf.Exp(-8f * dt));
        _currentRot = Quaternion.Slerp(_currentRot, targetRot, 1f - Mathf.Exp(-8f * dt));

        transform.localPosition = _currentPos;
        transform.localRotation = _currentRot;
    }
}

public class ReadOnlyAttribute : PropertyAttribute {}
#if UNITY_EDITOR
[CustomPropertyDrawer(typeof(ReadOnlyAttribute))]
public class ReadOnlyDrawer : PropertyDrawer
{
    public override void OnGUI(Rect pos, SerializedProperty prop, GUIContent label)
    {
        GUI.enabled = false;
        EditorGUI.PropertyField(pos, prop, label, true);
        GUI.enabled = true;
    }
}
#endif
