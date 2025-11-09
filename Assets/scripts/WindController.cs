using UnityEngine;

[DisallowMultipleComponent]
public class WindController : MonoBehaviour
{
    [SerializeField, Tooltip("風を適用する Rigidbody。未設定なら同一オブジェクトから取得します")]
    private Rigidbody target;

    [Header("基本風")]
    [SerializeField, Tooltip("定常風の向き")] private Vector3 baseDirection = Vector3.right;
    [SerializeField, Tooltip("定常風の強さ (N)")] private float baseStrength = 4f;

    [Header("乱流・ガスト")]
    [SerializeField, Tooltip("ランダム風の強さ (N)")] private float randomStrength = 2f;
    [SerializeField, Tooltip("ランダム風の周波数 (Hz)")] private float randomFrequency = 0.35f;
    [SerializeField, Tooltip("風の有効/無効")] private bool windEnabled = false;

    float _time;

    public float BaseStrength
    {
        get => baseStrength;
        set => baseStrength = Mathf.Max(0f, value);
    }

    public float RandomStrength
    {
        get => randomStrength;
        set => randomStrength = Mathf.Max(0f, value);
    }

    public float RandomFrequency
    {
        get => randomFrequency;
        set => randomFrequency = Mathf.Clamp(value, 0.01f, 5f);
    }

    public bool WindEnabled
    {
        get => windEnabled;
        set => windEnabled = value;
    }

    void Awake()
    {
        if (target == null) TryGetComponent(out target);
    }

    void FixedUpdate()
    {
        if (!windEnabled || target == null) return;

        _time += Time.fixedDeltaTime * randomFrequency;

        Vector3 baseWind = baseDirection.sqrMagnitude > 0.001f ? baseDirection.normalized * baseStrength : Vector3.zero;
        Vector3 randomWind = SampleNoise(_time) * randomStrength;
        Vector3 totalWind = baseWind + randomWind;

        if (totalWind.sqrMagnitude > 0f)
        {
            target.AddForce(totalWind, ForceMode.Force);
        }
    }

    Vector3 SampleNoise(float t)
    {
        float x = (Mathf.PerlinNoise(t, 0.127f) - 0.5f) * 2f;
        float y = (Mathf.PerlinNoise(0.361f, t) - 0.5f) * 2f;
        float z = (Mathf.PerlinNoise(t * 0.73f, t * 1.91f) - 0.5f) * 2f;
        return new Vector3(x, y * 0.5f, z);
    }
}

