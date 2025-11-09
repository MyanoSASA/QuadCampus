using System;
using System.Collections.Generic;
using UnityEngine;

[DisallowMultipleComponent]
public class DroneScript : MonoBehaviour
{
    // ==============================
    // Navigation Lights
    // ==============================
    [Serializable]
    public class LightUnit
    {
        public string label = "Light";
        public Renderer renderer;
        public int materialIndex = 0;

        [ColorUsage(true, true)] public Color onColor = Color.white;
        [ColorUsage(true, true)] public Color offColor = Color.black;
        [Min(0f)] public float intensity = 1f;

        [Range(0f, 1f)] public float phaseOffset = 0f;
        [SerializeField] public bool enabled = true;
        public Light linkedUnityLight = null;
    }

    [Header("Navigation Lights (assign RightFront, RightRear, LeftFront, LeftRear)")]
    [SerializeField] private LightUnit rightFrontLight;
    [SerializeField] private LightUnit rightRearLight;
    [SerializeField] private LightUnit leftFrontLight;
    [SerializeField] private LightUnit leftRearLight;

    [Header("Blink Settings")]
    [SerializeField, Min(0.01f)] private float blinkPeriodSec = 1.0f;
    [SerializeField] private AnimationCurve blinkCurve =
        new AnimationCurve(new Keyframe(0f, 1f), new Keyframe(0.45f, 1f), new Keyframe(0.5f, 0f), new Keyframe(1f, 0f));
    [SerializeField] private bool navLightsMasterOn = true;
    [SerializeField] private bool autoEnableEmissionKeyword = true;

    static readonly int EmissionColorID = Shader.PropertyToID("_EmissionColor");

    class LightCache { public LightUnit unit; public MaterialPropertyBlock mpb; }
    readonly List<LightCache> _lightCaches = new();

    // ==============================
    // Rotors + Pseudo Motion Blur (RGBを元色に合わせ、αのみ可変)
    // ==============================
    [Serializable]
    public class RendererColorRef
    {
        public Renderer renderer;
        public bool originallyEnabled = true;

        public bool hasBaseColor; public Color originalBaseColor;
        public bool hasColor;     public Color originalColor;

        public Material[] originalMats;
        public Material[] tempTransparentMats; // for blur clones only
        public bool usingTempTransparent;
    }

    [Serializable]
    public class RotorUnit
    {
        [Header("Identity")]
        public string label = "Rotor";
        public Transform rotorTransform;

        [Header("Base Rotation (Original Data)")]
        [SerializeField, Tooltip("deg/sec at throttle=1.0 (use your original data as-is)")]
        public float rpmDegPerSec = 1200f;
        [SerializeField] public Vector3 localAxis = Vector3.up;
        [SerializeField, Tooltip("+1=CW, -1=CCW")] public int direction = 1;
        [SerializeField] public bool isOn = true;

        [NonSerialized] public float currentAngle;
        [NonSerialized] public Quaternion baseRotation;

        // --- Pseudo Motion Blur ---
        [Header("Pseudo Motion Blur")]
        public bool enableBlur = true;
        [Tooltip("Override global threshold (RPM). <=0 uses global.")]
        public float rpmThresholdForBlur = 0f;
        [Range(0, 128)] public int   blurMaxStepsOverride = 0;   // 0 = global
        [Range(0f,360f)] public float blurSpanDegOverride  = 0f; // 0 = global
        [Range(0f,1f)]  public float blurAlphaOverride     = -1f;// <0 = global
        public Transform blurSourceOverride;

        // runtime (blur clones)
        [NonSerialized] public List<Transform>  blurClones = new();
        [NonSerialized] public List<Renderer[]> blurRenderers = new();
        [NonSerialized] public MaterialPropertyBlock blurMPB;
        [NonSerialized] public bool blurAllocated;

        // runtime (base)
        [NonSerialized] public List<RendererColorRef> baseRenderers;
    }

    [Header("Rotors (assign RightFront, RightRear, LeftFront, LeftRear)")]
    [SerializeField] private RotorUnit rotorRF; // Right Front (CCW)
    [SerializeField] private RotorUnit rotorRR; // Right Rear  (CW)
    [SerializeField] private RotorUnit rotorLF; // Left  Front (CW)
    [SerializeField] private RotorUnit rotorLR; // Left  Rear  (CCW)

    [Header("Rotor Control")]
    [SerializeField] private bool rotorsMasterOn = true;
    [SerializeField] private bool resetAnglesOnAwake = true;

    [Header("Throttle (Linear)")]
    [SerializeField, Range(0f, 1f), Tooltip("deg/sec = rpmDegPerSec * throttle")]
    public float throttle01 = 0f;

    [Header("Global Blur Settings")]
    [SerializeField] private bool  blurGlobalEnable = true;
    [SerializeField] private bool  blurPrewarmAllocate = true;
    [SerializeField, Tooltip("RPM threshold to show blur (RPM = degPerSec/6)")]
    private float blurThresholdRPM = 200f; // 1200deg/sで発火しやすい
    [SerializeField, Range(1,128)]
    private int   blurMaxSteps = 96;       // 円に近づけるなら増やす
    [SerializeField, Range(5f,360f)]
    private float blurBaseSpanDeg = 160f;  // 低速時の扇角
    [SerializeField, Tooltip("RPM at which span ≈ 360°")]
    private float blurRingRPM = 800f;      // この辺でディスク化
    [SerializeField, Range(0f,1f)]
    private float blurMaxAlpha = 0.18f;    // ブラー全体の上限α（薄め推奨）

    [SerializeField] private AnimationCurve blurOpacityCurve =
        new AnimationCurve(new Keyframe(0f, 1f), new Keyframe(1f, 0f));

    [SerializeField] private float blurScale = 1.02f;

    [Tooltip("Optional transparent material for clones. 未指定でもコードでTransparent化します")]
    [SerializeField] private Material blurOverrideMaterial;

    // 密度補正：段数が多いほど薄く
    [SerializeField, Range(0f,2f)] private float alphaDensityCompensationExp = 0.9f;
    // 全体係数（さらに薄くしたいとき）
    [SerializeField, Range(0f,1f)] private float blurGlobalAlphaMul = 0.7f;

    [Header("Debug")]
    [SerializeField] private bool  blurForceOn = false;
    [SerializeField, Range(1,128)] private int blurForceSteps = 64;

    // ==============================
    // Gimbal
    // ==============================
    public enum Axis { X, Y, Z }

    [Serializable]
    public class GimbalConfig
    {
        public bool enabled = true;
        public Transform pitchTransform;
        public Transform rollTransform;

        [Header("Offsets (deg)")] public float pitchOffsetDeg = 0f; public float rollOffsetDeg = 0f;
        [Header("Axis Mapping")] public Axis pitchAxis = Axis.X; public Axis rollAxis = Axis.Z;
        [Header("Invert")] public bool invertPitch = false; public bool invertRoll = false;
        [Header("Smoothing")] public float followSpeedDegPerSec = 720f;
    }

    [Header("Gimbal (assign pitch/roll transforms)")]
    [SerializeField] private GimbalConfig gimbal = new GimbalConfig();
    private Quaternion _gimbalPitchBaseRot, _gimbalRollBaseRot;

    // ==============================
    // Lifecycle
    // ==============================
    void Awake()
    {
        // Light defaults + phases
        if (leftFrontLight != null) leftFrontLight.onColor = new Color(1f, 0.1f, 0.1f, 1f);
        if (leftRearLight  != null) leftRearLight.onColor  = new Color(1f, 0.1f, 0.1f, 1f);
        if (rightFrontLight!= null) rightFrontLight.onColor= new Color(0.1f, 1f, 0.1f, 1f);
        if (rightRearLight  != null) rightRearLight.onColor = new Color(0.1f, 1f, 0.1f, 1f);

        if (leftFrontLight  != null) leftFrontLight.phaseOffset  = 0.0f;
        if (leftRearLight   != null) leftRearLight.phaseOffset   = 0.0f;
        if (rightFrontLight != null) rightFrontLight.phaseOffset = 0.5f;
        if (rightRearLight  != null) rightRearLight.phaseOffset  = 0.5f;

        BuildLightCache(rightFrontLight);
        BuildLightCache(rightRearLight);
        BuildLightCache(leftFrontLight);
        BuildLightCache(leftRearLight);

        // Rotor directions（仕様）
        if (rotorLF != null) rotorLF.direction = +1;  // LF → CW
        if (rotorRF != null) rotorRF.direction = -1;  // RF → CCW
        if (rotorLR != null) rotorLR.direction = -1;  // LR → CCW
        if (rotorRR != null) rotorRR.direction = +1;  // RR → CW

        InitRotor(rotorLF);
        InitRotor(rotorRF);
        InitRotor(rotorLR);
        InitRotor(rotorRR);

        if (gimbal.pitchTransform != null) _gimbalPitchBaseRot = gimbal.pitchTransform.localRotation;
        if (gimbal.rollTransform  != null) _gimbalRollBaseRot  = gimbal.rollTransform.localRotation;

        // 事前にブラー用クローンを確保
        if (blurPrewarmAllocate)
        {
            EnsureBlurAllocated(rotorLF);
            EnsureBlurAllocated(rotorRF);
            EnsureBlurAllocated(rotorLR);
            EnsureBlurAllocated(rotorRR);
            HideBlur(rotorLF); HideBlur(rotorRF); HideBlur(rotorLR); HideBlur(rotorRR);
        }
    }

    void Update()
    {
        UpdateNavLights();
        UpdateRotors(Time.deltaTime);
    }

    void LateUpdate()
    {
        UpdateGimbal(Time.deltaTime);
    }

    // ==============================
    // Navigation Lights
    // ==============================
    void BuildLightCache(LightUnit u)
    {
        if (u == null || u.renderer == null) return;
        if (autoEnableEmissionKeyword && u.renderer.sharedMaterial != null)
            u.renderer.sharedMaterial.EnableKeyword("_EMISSION");
        var c = new LightCache { unit = u, mpb = new MaterialPropertyBlock() };
        _lightCaches.Add(c);
        ApplyLight(c, 0f);
    }

    void UpdateNavLights()
    {
        float t = Time.time;
        foreach (var c in _lightCaches)
        {
            float k = 0f;
            if (navLightsMasterOn && c.unit.enabled && blinkPeriodSec > 0.001f)
            {
                float phase = Mathf.Repeat((t / blinkPeriodSec) + c.unit.phaseOffset, 1f);
                k = Mathf.Clamp01(blinkCurve.Evaluate(phase));
            }
            ApplyLight(c, k);
        }
    }

    void ApplyLight(LightCache c, float k)
    {
        if (c.unit.renderer == null) return;
        Color em = (k > 0f) ? c.unit.onColor * (c.unit.intensity * k) : c.unit.offColor;
        c.unit.renderer.GetPropertyBlock(c.mpb, c.unit.materialIndex);
        c.mpb.SetColor(EmissionColorID, em);
        c.unit.renderer.SetPropertyBlock(c.mpb, c.unit.materialIndex);
        if (c.unit.linkedUnityLight != null)
        {
            bool on = navLightsMasterOn && c.unit.enabled && (k > 0f);
            c.unit.linkedUnityLight.enabled = on;
            if (on) c.unit.linkedUnityLight.color = c.unit.onColor;
        }
    }

    // ==============================
    // Rotor + Blur（RGB固定／αのみ可変）
    // ==============================
    void InitRotor(RotorUnit r)
    {
        if (r == null || r.rotorTransform == null) return;

        r.baseRotation = r.rotorTransform.localRotation;
        if (resetAnglesOnAwake) r.currentAngle = 0f;
        ApplyRotorAngle(r);

        r.blurMPB = new MaterialPropertyBlock();
        r.blurAllocated = false;

        // 元プロペラのレンダラ取得 & 元色保持（RGB参照用）
        r.baseRenderers = new List<RendererColorRef>();
        var rdAll = r.rotorTransform.GetComponentsInChildren<Renderer>(true);
        foreach (var rd in rdAll)
        {
            if (rd == null) continue;
            var info = new RendererColorRef
            {
                renderer = rd,
                originallyEnabled = rd.enabled,
                originalMats = rd.sharedMaterials
            };

            try
            {
                var mat = rd.sharedMaterial;
                if (mat != null)
                {
                    if (mat.HasProperty("_BaseColor")) { info.hasBaseColor = true; info.originalBaseColor = mat.GetColor("_BaseColor"); }
                    if (mat.HasProperty("_Color"))     { info.hasColor     = true; info.originalColor     = mat.GetColor("_Color"); }
                }
            }
            catch {}

            r.baseRenderers.Add(info);
        }
    }

    void UpdateRotors(float dt)
    {
        if (!rotorsMasterOn) return;
        StepRotor(rotorLF, dt);
        StepRotor(rotorRF, dt);
        StepRotor(rotorLR, dt);
        StepRotor(rotorRR, dt);
    }

    void StepRotor(RotorUnit r, float dt)
    {
        if (r == null || r.rotorTransform == null || !r.isOn) return;

        float degPerSec = Mathf.Max(0f, r.rpmDegPerSec) * Mathf.Clamp01(throttle01);
        float delta = degPerSec * dt * (r.direction == 0 ? 1 : Mathf.Sign(r.direction));
        r.currentAngle = (r.currentAngle + delta) % 360f;

        ApplyRotorAngle(r);

        if (!blurGlobalEnable || !r.enableBlur) { HideBlur(r); return; }

        float rpm = degPerSec / 6f;
        float threshold = (r.rpmThresholdForBlur > 0f) ? r.rpmThresholdForBlur : blurThresholdRPM;

        if (blurForceOn || rpm >= Mathf.Max(0f, threshold))
        {
            EnsureBlurAllocated(r);

            int visible = blurForceOn ? Mathf.Clamp(blurForceSteps, 1, GetMaxSteps(r))
                                      : ComputeVisibleSteps(rpm, threshold, GetMaxSteps(r));
            float spanDeg = ComputeDynamicSpan(r, rpm, threshold);

            UpdateBlurClonesCircleApproach_RGBFixed_AlphaOnly(r, visible, spanDeg);
        }
        else
        {
            HideBlur(r);
        }
    }

    void ApplyRotorAngle(RotorUnit r)
    {
        Vector3 axis = (r.localAxis.sqrMagnitude < 1e-6f) ? Vector3.up : r.localAxis.normalized;
        Quaternion q = r.baseRotation * Quaternion.AngleAxis(r.currentAngle, axis);
        r.rotorTransform.localRotation = q;
    }

    int GetMaxSteps(RotorUnit r)
    {
        int ov = Mathf.Clamp(r.blurMaxStepsOverride, 0, 128);
        return (ov > 0) ? ov : blurMaxSteps;
    }

    int ComputeVisibleSteps(float rpm, float threshold, int maxSteps)
    {
        // threshold … 2*threshold で 1→maxSteps
        float t = Mathf.InverseLerp(threshold, threshold * 2f, rpm);
        return Mathf.Clamp(Mathf.RoundToInt(Mathf.Lerp(1f, maxSteps, t)), 1, maxSteps);
    }

    float ComputeDynamicSpan(RotorUnit r, float rpm, float threshold)
    {
        float baseSpan = (r.blurSpanDegOverride > 0f) ? r.blurSpanDegOverride : blurBaseSpanDeg;
        float spanT = Mathf.InverseLerp(Mathf.Max(1f, threshold), Mathf.Max(1f, blurRingRPM), rpm);
        return Mathf.Lerp(baseSpan, 360f, Mathf.Clamp01(spanT));
    }

    void EnsureBlurAllocated(RotorUnit r)
    {
        if (r == null || r.rotorTransform == null) return;
        if (r.blurAllocated) return;

        Transform src = r.blurSourceOverride != null ? r.blurSourceOverride : r.rotorTransform;

        int max = GetMaxSteps(r);
        r.blurClones.Capacity = max;
        r.blurRenderers.Capacity = max;

        for (int i = 0; i < max; i++)
        {
            var go = Instantiate(src.gameObject, src.parent);
            go.name = (src.gameObject.name + $"_Blur_{i+1}");

            // スクリプト・物理削除
            foreach (var comp in go.GetComponentsInChildren<MonoBehaviour>(true))
                if (!(comp is Transform)) Destroy(comp);
            foreach (var col in go.GetComponentsInChildren<Collider>(true)) Destroy(col);
            foreach (var rb in go.GetComponentsInChildren<Rigidbody>(true)) Destroy(rb);

            // レンダラ列挙：マテリアルをTransparentに強制
            var rends = go.GetComponentsInChildren<Renderer>(true);
            if (rends != null && rends.Length > 0)
            {
                foreach (var rd in rends)
                {
                    if (blurOverrideMaterial != null)
                    {
                        rd.material = new Material(blurOverrideMaterial);
                    }
                    else
                    {
                        CloneAndForceTransparent(rd);
                    }
                }
            }

            var t = go.transform;
            t.localPosition = src.localPosition;
            t.localRotation = src.localRotation;
            t.localScale    = src.localScale * Mathf.Max(0.01f, blurScale);

            r.blurClones.Add(t);
            r.blurRenderers.Add(rends ?? Array.Empty<Renderer>());
            go.SetActive(false);
        }

        r.blurAllocated = true;
    }

    // ★RGBは元色を維持、αのみ可変
    void UpdateBlurClonesCircleApproach_RGBFixed_AlphaOnly(RotorUnit r, int visibleSteps, float spanDeg)
    {
        if (!r.blurAllocated) return;

        float sign = (r.direction == 0 ? 1f : Mathf.Sign(r.direction));
        Vector3 axis = (r.localAxis.sqrMagnitude < 1e-6f) ? Vector3.up : r.localAxis.normalized;

        // 元ローターの代表色（_BaseColor/_Color）を取得（なければ白）
        Color baseRGB = SampleBaseRGB(r);

        float maxA   = (r.blurAlphaOverride >= 0f ? r.blurAlphaOverride : blurMaxAlpha);
        float alphaBase = maxA * blurGlobalAlphaMul;

        // ON/OFF
        for (int i = 0; i < r.blurClones.Count; i++)
        {
            bool on = i < visibleSteps;
            if (r.blurClones[i] != null) r.blurClones[i].gameObject.SetActive(on);
        }
        if (visibleSteps <= 0) return;

        float step  = (visibleSteps <= 1) ? 0f : (spanDeg / (visibleSteps - 1));
        float start = -sign * spanDeg;

        for (int i = 0; i < visibleSteps; i++)
        {
            float offset = (visibleSteps <= 1) ? -sign * spanDeg : start + step * i;
            float angle  = r.currentAngle + offset;

            var t = r.blurClones[i];
            if (t == null) continue;

            t.localPosition = r.rotorTransform.localPosition;
            t.localRotation = r.baseRotation * Quaternion.AngleAxis(angle, axis);

            // 端ほど薄く：カーブで α を落とす
            float u = (visibleSteps <= 1) ? 1f : (float)i / (float)(visibleSteps - 1); // 0..1
            float a = Mathf.Clamp01(blurOpacityCurve.Evaluate(u)) * alphaBase;

            // 段数が多い時は相対的にさらに薄く
            if (alphaDensityCompensationExp > 0f && visibleSteps > 1)
            {
                float densityScale = Mathf.Pow(visibleSteps, alphaDensityCompensationExp);
                a /= densityScale;
            }

            if (r.blurMPB == null) r.blurMPB = new MaterialPropertyBlock();
            r.blurMPB.Clear();

            // RGBは元色、Aのみ可変
            Color c = new Color(baseRGB.r, baseRGB.g, baseRGB.b, a);
            r.blurMPB.SetColor("_BaseColor", c);
            r.blurMPB.SetColor("_Color",     c);

            var rends = r.blurRenderers[i];
            if (rends != null)
            {
                foreach (var rd in rends)
                {
                    if (rd == null) continue;
                    EnsureRendererTransparent(rd);
                    rd.SetPropertyBlock(r.blurMPB);
                }
            }
        }
    }

    Color SampleBaseRGB(RotorUnit r)
    {
        // 優先：_BaseColor → _Color → なければ白
        if (r.baseRenderers != null)
        {
            foreach (var info in r.baseRenderers)
            {
                if (info.renderer == null) continue;

                // まず保存済みの色（shared）を参照
                if (info.hasBaseColor) return new Color(info.originalBaseColor.r, info.originalBaseColor.g, info.originalBaseColor.b, 1f);
                if (info.hasColor)     return new Color(info.originalColor.r,     info.originalColor.g,     info.originalColor.b,     1f);

                // ダメなら現在マテリアルから読む
                try
                {
                    var mats = info.renderer.sharedMaterials;
                    if (mats != null && mats.Length > 0 && mats[0] != null)
                    {
                        var m = mats[0];
                        if (m.HasProperty("_BaseColor")) { var c = m.GetColor("_BaseColor"); return new Color(c.r, c.g, c.b, 1f); }
                        if (m.HasProperty("_Color"))     { var c = m.GetColor("_Color");     return new Color(c.r, c.g, c.b, 1f); }
                    }
                }
                catch {}
            }
        }
        return Color.white;
    }

    void HideBlur(RotorUnit r)
    {
        if (r == null) return;
        if (r.blurAllocated)
        {
            for (int i = 0; i < r.blurClones.Count; i++)
                if (r.blurClones[i] != null) r.blurClones[i].gameObject.SetActive(false);
        }
        // 基本方針：元ローターは一切いじらないので復元不要
    }

    // ==============================
    // Material helpers (force Transparent for clones)
    // ==============================
    void CloneAndForceTransparent(Renderer rd)
    {
        if (rd == null) return;
        var shared = rd.sharedMaterials;
        if (shared == null || shared.Length == 0) return;
        rd.materials = CloneAndForceTransparent(shared);
    }

    Material[] CloneAndForceTransparent(Material[] src)
    {
        var arr = new Material[src.Length];
        for (int i = 0; i < src.Length; i++)
        {
            var m = src[i] != null ? new Material(src[i]) : new Material(Shader.Find("Universal Render Pipeline/Lit"));
            ForceTransparentSettings(m);
            arr[i] = m;
        }
        return arr;
    }

    void EnsureRendererTransparent(Renderer rd)
    {
        if (rd == null) return;
        foreach (var m in rd.materials)
        {
            if (m == null) continue;
            ForceTransparentSettings(m);
        }
    }

    void ForceTransparentSettings(Material m)
    {
        if (m == null) return;

        var shaderName = m.shader != null ? m.shader.name : "";

        // URP Lit/Unlit
        if (shaderName.Contains("Universal Render Pipeline"))
        {
            m.SetFloat("_Surface", 1f); // Transparent
            m.EnableKeyword("_SURFACE_TYPE_TRANSPARENT");
            m.SetFloat("_Blend", 0f);
            m.SetFloat("_SrcBlend", (float)UnityEngine.Rendering.BlendMode.SrcAlpha);
            m.SetFloat("_DstBlend", (float)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
            m.SetFloat("_ZWrite", 0f);
            m.renderQueue = (int)UnityEngine.Rendering.RenderQueue.Transparent;
            m.DisableKeyword("_ALPHAPREMULTIPLY_ON");
            m.DisableKeyword("_ALPHATEST_ON");
            return;
        }

        // Built-in Standard
        if (shaderName.Contains("Standard"))
        {
            m.SetFloat("_Mode", 2f); // Fade
            m.EnableKeyword("_ALPHABLEND_ON");
            m.DisableKeyword("_ALPHATEST_ON");
            m.DisableKeyword("_ALPHAPREMULTIPLY_ON");
            m.SetFloat("_SrcBlend", (float)UnityEngine.Rendering.BlendMode.SrcAlpha);
            m.SetFloat("_DstBlend", (float)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
            m.SetFloat("_ZWrite", 0f);
            m.renderQueue = (int)UnityEngine.Rendering.RenderQueue.Transparent;
            return;
        }

        // Fallback
        m.SetFloat("_SrcBlend", (float)UnityEngine.Rendering.BlendMode.SrcAlpha);
        m.SetFloat("_DstBlend", (float)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
        m.SetFloat("_ZWrite", 0f);
        m.renderQueue = (int)UnityEngine.Rendering.RenderQueue.Transparent;
    }

    // ==============================
    // Gimbal
    // ==============================
    void UpdateGimbal(float dt)
    {
        if (gimbal == null || !gimbal.enabled) return;
        if (gimbal.pitchTransform == null && gimbal.rollTransform == null) return;

        Quaternion worldRot = transform.rotation;

        Vector3 fwd = transform.forward;
        Vector3 fwdXZ = new Vector3(fwd.x, 0f, fwd.z);
        if (fwdXZ.sqrMagnitude < 1e-6f) fwdXZ = Vector3.forward;
        Quaternion yawOnly = Quaternion.LookRotation(fwdXZ.normalized, Vector3.up);

        Quaternion tilt = Quaternion.Inverse(yawOnly) * worldRot;
        Vector3 tiltEuler = tilt.eulerAngles;

        float pitchDeg = Normalize180(tiltEuler.x);
        float rollDeg  = Normalize180(tiltEuler.z);

        float targetPitch = -pitchDeg + gimbal.pitchOffsetDeg;
        float targetRoll  = -rollDeg  + gimbal.rollOffsetDeg;

        if (gimbal.invertPitch) targetPitch = -targetPitch;
        if (gimbal.invertRoll)  targetRoll  = -targetRoll;

        float speed = Mathf.Max(0f, gimbal.followSpeedDegPerSec);

        if (gimbal.rollTransform != null)
        {
            Vector3 axis = AxisToVector(gimbal.rollAxis);
            Quaternion goal = _gimbalRollBaseRot * Quaternion.AngleAxis(targetRoll, axis);
            gimbal.rollTransform.localRotation =
                Quaternion.RotateTowards(gimbal.rollTransform.localRotation, goal, speed * dt);
        }

        if (gimbal.pitchTransform != null)
        {
            Vector3 axis = AxisToVector(gimbal.pitchAxis);
            Quaternion goal = _gimbalPitchBaseRot * Quaternion.AngleAxis(targetPitch, axis);
            gimbal.pitchTransform.localRotation =
                Quaternion.RotateTowards(gimbal.pitchTransform.localRotation, goal, speed * dt);
        }
    }

    static Vector3 AxisToVector(Axis a)
    {
        switch (a)
        {
            case Axis.X: return Vector3.right;
            case Axis.Y: return Vector3.up;
            case Axis.Z: return Vector3.forward;
        }
        return Vector3.right;
    }

    static float Normalize180(float deg) => Mathf.Repeat(deg + 180f, 360f) - 180f;
}
