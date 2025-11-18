using UnityEngine;

#if UNITY_EDITOR
using UnityEditor;
#endif

public enum DroneCourseMode
{
    Rectangular,
    FigureEight
}

/// <summary>
/// 指定した寸法に基づいてドローン試験用コースのPlaneとパイロンを生成するスクリプト。
/// 本コンポーネントを空のGameObjectに追加し、インスペクターのボタンからレイアウトを構築してください。
/// </summary>
[ExecuteAlways]
public class DroneCourseLayoutGenerator : MonoBehaviour
{
    private const string GeneratedRootName = "_GeneratedCourse";

    [Header("モード設定")]
    public DroneCourseMode courseMode = DroneCourseMode.Rectangular;

    [Header("コース寸法 (m)")]
    [Tooltip("最外周の幅。図の21mに対応。")]
    public float outerWidth = 21f;

    [Tooltip("最外周の奥行き。図の13mに対応。")]
    public float outerDepth = 13f;

    [Tooltip("減点区画を含む内側矩形の幅。図の18mに対応。")]
    public float middleWidth = 18f;

    [Tooltip("減点区画を含む内側矩形の奥行き。図の10mに対応。")]
    public float middleDepth = 10f;

    [Tooltip("飛行経路矩形の幅。図の16mに対応。")]
    public float flightWidth = 16f;

    [Tooltip("飛行経路矩形の奥行き。図の8mに対応。")]
    public float flightDepth = 8f;

    [Tooltip("減点区画の幅（飛行経路からの逸脱距離）。図の1.5mに対応。")]
    public float deductionZoneWidth = 1.5f;

    [Tooltip("不合格区画の幅（飛行経路からの逸脱距離）。図の2.5mに対応。")]
    public float disqualificationZoneWidth = 2.5f;

    [Tooltip("内側減点区画の幅（ランディングパッド周囲）。図の10mに対応。")]
    public float innerDeductionZoneWidth = 10f;

    [Tooltip("内側減点区画の奥行き（ランディングパッド周囲）。図の2mに対応。")]
    public float innerDeductionZoneDepth = 2f;

    [Header("ランディングパッド")]
    [Tooltip("中央ランディングパッドの直径。図の2mに対応。")]
    public float landingPadDiameter = 2f;

    [Tooltip("ランディングパッドの高さ。")]
    public float landingPadHeight = 0.03f;

    [Header("パイロン設定")]
    [Tooltip("パイロンの底辺サイズ。")]
    public float pylonSide = 0.4f;

    [Tooltip("パイロンの高さ。")]
    public float pylonHeight = 2.5f;

    [Tooltip("パイロンの色。既存マテリアルが未設定の場合に使用。")]
    public Color pylonColor = new Color(0.85f, 0.1f, 0.1f);

    [Header("マテリアル (任意)")]
    public Material outerAreaMaterial;
    public Material middleAreaMaterial;
    public Material flightAreaMaterial;
    public Material landingPadMaterial;
    public Material pylonMaterial;
    public Material deductionZoneMaterial; // 減点区画用マテリアル
    public Material disqualificationZoneMaterial; // 不合格区画用マテリアル

    [Header("色 (マテリアル未設定時)")]
    public Color outerAreaColor = new Color(0.5f, 0.5f, 0.5f); // 不合格区画（灰色）
    public Color middleAreaColor = new Color(1f, 0f, 0f); // 減点区画（赤色）
    public Color flightAreaColor = new Color(1f, 1f, 1f); // 飛行可能区画（白色）

    [Header("八の字コース寸法 (m)")]
    [Tooltip("八の字モード時の最外周幅 (図の18m)")]
    public float figureOuterWidth = 18f;

    [Tooltip("八の字モード時の最外周奥行き (図の13m)")]
    public float figureOuterDepth = 13f;

    [Tooltip("八の字モード時の内側矩形幅 (図の15m)")]
    public float figureMiddleWidth = 15f;

    [Tooltip("八の字モード時の内側矩形奥行き (図の10m)")]
    public float figureMiddleDepth = 10f;

    [Tooltip("八の字モード時の飛行エリア矩形幅 (縮小後 12m)")]
    public float figureFlightRectWidth = 12f;

    [Tooltip("八の字モード時の飛行エリア矩形奥行き (縮小後 7m)")]
    public float figureFlightRectDepth = 7f;

    [Tooltip("八の字円の直径 (図の5m)")]
    public float figureCircleDiameter = 5f;

    [Tooltip("八の字円の厚み")]
    public float figureCircleThickness = 0.02f;

    [Tooltip("八の字円センター間距離 (左右の円中心がX=-2.5, X=2.5になるように設定)")]
    public float figureCircleCenterSpacing = 5f;

    [Tooltip("八の字内側円の直径 (図の1m)")]
    public float figureInnerCircleDiameter = 1f;

    [Tooltip("八の字内側円の厚み")]
    public float figureInnerCircleThickness = 0.02f;

    [Tooltip("八の字重なり部の直径 (図の2m)")]
    public float figureOverlapCircleDiameter = 2f;

    [Tooltip("八の字重なり部の厚み")]
    public float figureOverlapCircleThickness = 0.02f;

    [Tooltip("八の字スタートマーカーまでの距離 (図の1.5m)")]
    public float figureStartDistanceFromCircle = 1.5f;

    [Tooltip("八の字ディスクのベース高さオフセット")]
    public float figureDiscBaseHeight = 0.02f;

    [Header("八の字カラー (マテリアル未設定時)")]
    public Color figureCircleColor = new Color(0.2f, 0.2f, 0.2f);
    public Color figureInnerCircleColor = new Color(1f, 0.92f, 0.016f); // 内側減点区画（黄色）
    public Color figureOverlapCircleColor = new Color(0.6f, 0.9f, 0.6f);
    public Color figureDeductionZoneColor = new Color(1f, 0.92f, 0.016f); // 外側減点区画（黄色）
    public Color figureDisqualificationZoneColor = new Color(0.5f, 0.5f, 0.5f); // 不合格区画（灰色）

    [Header("レイヤー高さ設定")]
    [Tooltip("各レイヤー間のYオフセット")]
    public float layerHeightOffset = 0.01f;

    /// <summary>
    /// レイアウト生成。
    /// </summary>
    public void BuildLayout()
    {
        BuildLayout(courseMode);
    }

    /// <summary>
    /// 指定モードでレイアウト生成 (動的呼び出し用)。
    /// </summary>
    public Transform BuildLayout(DroneCourseMode mode)
    {
        courseMode = mode;
        var root = ResetGeneratedRoot();

        switch (mode)
        {
            case DroneCourseMode.Rectangular:
                BuildRectangularLayout(root);
                break;
            case DroneCourseMode.FigureEight:
                BuildFigureEightLayout(root);
                break;
        }

        RemoveAllColliders(root);

        return root;
    }

    /// <summary>
    /// 生成済みのレイアウトを削除。
    /// </summary>
    public void ClearLayout()
    {
        RemoveGeneratedRoot();
    }

    private void BuildRectangularLayout(Transform root)
    {
        // レイヤー順序（下から上）：
        // 1. 不合格区画（最下層）
        // 2. 外側減点区画（飛行経路から1.5m外側の帯）
        // 3. 飛行可能区画（白い帯）
        // 4. 内側減点区画（ランディングパッド直下 10m x 2m）
        // 5. ランディングパッド（最上層）

        float currentY = 0f;

        // 1. 不合格区画（最下層）- 灰色
        CreatePlane("DisqualificationZone", outerWidth, outerDepth, currentY,
            disqualificationZoneMaterial, figureDisqualificationZoneColor, root);
        currentY += layerHeightOffset;

        // 2. 外側減点区画（斜線エリア全体）
        CreatePlane("OuterDeductionZone", middleWidth, middleDepth, currentY,
            deductionZoneMaterial, middleAreaColor, root);
        currentY += layerHeightOffset;

        // 3. 飛行可能区画（白いエリア）
        CreatePlane("FlightArea", flightWidth, flightDepth, currentY,
            flightAreaMaterial, flightAreaColor, root);
        currentY += layerHeightOffset;

        // 4. 内側減点区画（ランディングパッド直下）
        CreatePlane("InnerDeductionZone", innerDeductionZoneWidth, innerDeductionZoneDepth, currentY,
            deductionZoneMaterial, middleAreaColor, root);
        currentY += layerHeightOffset;

        // 5. ランディングパッド（最上層）
        CreateLandingPad(root, currentY);

        float halfFlightWidth = flightWidth * 0.5f;
        float halfFlightDepth = flightDepth * 0.5f;
        float flightLayerY = layerHeightOffset * 2f; // フライトゾーンのY高さ

        // パイロンはフライトゾーン境界から1.5m内側（= 減点区画幅）に配置
        float pylonX = Mathf.Max(halfFlightWidth - deductionZoneWidth, 0f);
        float pylonZ = Mathf.Max(halfFlightDepth - deductionZoneWidth, 0f);
        var pylonData = new (string name, Vector3 position)[]
        {
            ("A", new Vector3(0f, flightLayerY, -pylonZ)),
            ("B", new Vector3(pylonX, flightLayerY, -pylonZ)),
            ("C", new Vector3(pylonX, flightLayerY, pylonZ)),
            ("D", new Vector3(-pylonX, flightLayerY, pylonZ)),
            ("E", new Vector3(-pylonX, flightLayerY, -pylonZ))
        };

        CreatePylons(root, pylonData);
    }

    private void BuildFigureEightLayout(Transform root)
    {
        // レイヤー順序（下から上）：
        // 1. 不合格区画（最下層）
        // 2. 外側減点区画（旧フライトエリア寸法 15m x 10m の黄色帯）
        // 3. 飛行可能区画（白いエリア 12m x 7m）
        // 4. 内側減点区画（円形・黄色）
        // 5. ランディングパッド（最上層）

        float currentY = 0f;

        // 1. 不合格区画（最下層）- 灰色
        // 最外周：15m x 10m
        CreatePlane("DisqualificationZone", figureOuterWidth, figureOuterDepth, currentY,
            disqualificationZoneMaterial, figureDisqualificationZoneColor, root);
        currentY += layerHeightOffset;

        // 2. 外側減点区画（旧フライトエリアと同寸法）
        CreatePlane("OuterDeductionZone", figureMiddleWidth, figureMiddleDepth, currentY,
            deductionZoneMaterial, figureDeductionZoneColor, root);
        currentY += layerHeightOffset;

        // 3. 飛行可能区画（白いエリア）
        CreatePlane("FigureFlightRect", figureFlightRectWidth, figureFlightRectDepth, currentY,
            flightAreaMaterial, flightAreaColor, root);
        currentY += layerHeightOffset;

        float halfSpacing = figureCircleCenterSpacing * 0.5f;
        float baseHeight = currentY + figureDiscBaseHeight;
        float circleThickness = Mathf.Max(figureCircleThickness, 0f);

        if (figureCircleDiameter > 0f)
        {
            float circleCenterY = baseHeight + circleThickness * 0.5f;
            CreateDisc("FlightCircle_Left", figureCircleDiameter, figureCircleThickness, new Vector3(-halfSpacing, circleCenterY, 0f), flightAreaMaterial, figureCircleColor, root);
            CreateDisc("FlightCircle_Right", figureCircleDiameter, figureCircleThickness, new Vector3(halfSpacing, circleCenterY, 0f), flightAreaMaterial, figureCircleColor, root);
        }

        if (figureOverlapCircleDiameter > 0f)
        {
            float overlapCenterY = baseHeight + Mathf.Max(figureOverlapCircleThickness, 0f) * 0.5f;
            CreateDisc("FigureOverlap", figureOverlapCircleDiameter, figureOverlapCircleThickness, new Vector3(0f, overlapCenterY, 0f), flightAreaMaterial, figureOverlapCircleColor, root);
        }

        if (figureInnerCircleDiameter > 0f)
        {
            float innerThickness = Mathf.Max(figureInnerCircleThickness, 0f);
            float innerCenterY = baseHeight + circleThickness + layerHeightOffset + innerThickness * 0.5f;
            CreateDisc("InnerDeductionZone_Left", figureInnerCircleDiameter, figureInnerCircleThickness, new Vector3(-halfSpacing, innerCenterY, 0f), deductionZoneMaterial, figureInnerCircleColor, root);
            CreateDisc("InnerDeductionZone_Right", figureInnerCircleDiameter, figureInnerCircleThickness, new Vector3(halfSpacing, innerCenterY, 0f), deductionZoneMaterial, figureInnerCircleColor, root);
        }

        // 5. ランディングパッド（最上層）
        CreateLandingPad(root, baseHeight);

        float radius = Mathf.Max(figureCircleDiameter * 0.5f, 0f);
        var pylons = new (string name, Vector3 position)[]
        {
            ("Left", new Vector3(-halfSpacing, baseHeight, 0f)),
            ("Right", new Vector3(halfSpacing, baseHeight, 0f))
        };

        CreatePylons(root, pylons);
    }

    private Transform ResetGeneratedRoot()
    {
        RemoveGeneratedRoot();

        var rootObject = new GameObject(GeneratedRootName);
        rootObject.transform.SetParent(transform, false);

#if UNITY_EDITOR
        if (!Application.isPlaying)
        {
            Undo.RegisterCreatedObjectUndo(rootObject, "Create Drone Course Layout Root");
        }
#endif

        return rootObject.transform;
    }

    private void RemoveGeneratedRoot()
    {
        var existing = transform.Find(GeneratedRootName);
        if (existing == null) return;

#if UNITY_EDITOR
        if (!Application.isPlaying)
        {
            Undo.DestroyObjectImmediate(existing.gameObject);
            return;
        }
#endif
        Destroy(existing.gameObject);
    }

    private void CreatePlane(string name, float width, float depth, float yOffset, Material material, Color fallbackColor, Transform parent, Vector3? positionOffset = null)
    {
        var plane = GameObject.CreatePrimitive(PrimitiveType.Plane);
        plane.name = name;
        plane.transform.SetParent(parent, false);
        Vector3 pos = positionOffset ?? Vector3.zero;
        plane.transform.localPosition = new Vector3(pos.x, yOffset, pos.z);
        plane.transform.localScale = new Vector3(width / 10f, 1f, depth / 10f);

#if UNITY_EDITOR
        if (!Application.isPlaying)
        {
            Undo.RegisterCreatedObjectUndo(plane, $"Create {name}");
        }
#endif

        ConfigureRenderer(plane, material, fallbackColor);
        RemoveColliderComponent(plane);
    }

    private GameObject CreateLandingPad(Transform parent, float heightOffset = 0f)
    {
        var pad = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        pad.name = "LandingPad";
        pad.transform.SetParent(parent, false);
        float clampedThickness = Mathf.Max(landingPadHeight, 0f);
        float scaleY = Mathf.Max(clampedThickness * 0.5f, 0.001f);
        pad.transform.localPosition = new Vector3(0f, heightOffset + clampedThickness * 0.5f, 0f);
        pad.transform.localScale = new Vector3(landingPadDiameter, scaleY, landingPadDiameter);

#if UNITY_EDITOR
        if (!Application.isPlaying)
        {
            Undo.RegisterCreatedObjectUndo(pad, "Create LandingPad");
        }
#endif

        ConfigureRenderer(pad, landingPadMaterial, new Color(0.85f, 0.85f, 0.85f));
        RemoveColliderComponent(pad);
        return pad;
    }

    private GameObject CreateDisc(string name, float diameter, float thickness, Vector3 centerPosition, Material material, Color fallbackColor, Transform parent)
    {
        var disc = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        disc.name = name;
        disc.transform.SetParent(parent, false);

        float clampedThickness = Mathf.Max(thickness, 0f);
        float scaleY = Mathf.Max(clampedThickness * 0.5f, 0.001f);
        disc.transform.localPosition = new Vector3(centerPosition.x, centerPosition.y, centerPosition.z);
        disc.transform.localScale = new Vector3(diameter, scaleY, diameter);

#if UNITY_EDITOR
        if (!Application.isPlaying)
        {
            Undo.RegisterCreatedObjectUndo(disc, $"Create {name}");
        }
#endif

        ConfigureRenderer(disc, material, fallbackColor);
        RemoveColliderComponent(disc);
        return disc;
    }

    private void CreatePylons(Transform parent, (string name, Vector3 position)[] pylonData)
    {
        foreach (var (name, position) in pylonData)
        {
            var pylon = GameObject.CreatePrimitive(PrimitiveType.Cube);
            pylon.name = $"Pylon_{name}";
            pylon.transform.SetParent(parent, false);
            float height = name.Equals("A", System.StringComparison.OrdinalIgnoreCase)
                ? Mathf.Max(pylonHeight / 3f, 0.001f)
                : Mathf.Max(pylonHeight, 0.001f);
            pylon.transform.localPosition = position + Vector3.up * (height * 0.5f);
            pylon.transform.localScale = new Vector3(pylonSide, height, pylonSide);

#if UNITY_EDITOR
            if (!Application.isPlaying)
            {
                Undo.RegisterCreatedObjectUndo(pylon, $"Create Pylon {name}");
            }
#endif

            ConfigureRenderer(pylon, pylonMaterial, pylonColor);
            RemoveColliderComponent(pylon);
        }
    }

    private void ConfigureRenderer(GameObject target, Material materialOverride, Color? fallbackColor = null)
    {
        var renderer = target.GetComponent<Renderer>();
        if (renderer == null) return;

        if (materialOverride != null)
        {
#if UNITY_EDITOR
            if (!Application.isPlaying)
            {
                Undo.RecordObject(renderer, $"Assign material to {target.name}");
            }
#endif
            renderer.sharedMaterial = materialOverride;
        }
        else if (fallbackColor.HasValue)
        {
#if UNITY_EDITOR
            if (!Application.isPlaying)
            {
                Undo.RecordObject(renderer, $"Colorize {target.name}");
            }
#endif

            // Standardシェーダー用にインスタンス化されたマテリアルを使用
            var tempMaterial = new Material(Shader.Find("Standard"))
            {
                color = fallbackColor.Value
            };
            renderer.sharedMaterial = tempMaterial;
        }
    }

    private void RemoveColliderComponent(GameObject target)
    {
        var colliders = target.GetComponents<Collider>();
        if (colliders == null || colliders.Length == 0) return;

        foreach (var collider in colliders)
        {
            DestroyCollider(collider);
        }
    }

    private void RemoveAllColliders(Transform root)
    {
        if (root == null) return;

        var colliders = root.GetComponentsInChildren<Collider>(true);
        if (colliders == null || colliders.Length == 0) return;

        foreach (var collider in colliders)
        {
            DestroyCollider(collider);
        }
    }

    private void DestroyCollider(Collider collider)
    {
        if (collider == null) return;

#if UNITY_EDITOR
        if (!Application.isPlaying)
        {
            Undo.DestroyObjectImmediate(collider);
            return;
        }
#endif

        Destroy(collider);
    }

#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        Gizmos.color = new Color(1f, 0f, 0f, 0.4f);
        Gizmos.matrix = transform.localToWorldMatrix;

        if (courseMode == DroneCourseMode.Rectangular)
        {
            float halfFlightWidth = flightWidth * 0.5f;
            float halfFlightDepth = flightDepth * 0.5f;

            Vector3[] corners =
            {
                new Vector3(-halfFlightWidth, 0f, -halfFlightDepth),
                new Vector3(halfFlightWidth, 0f, -halfFlightDepth),
                new Vector3(halfFlightWidth, 0f, halfFlightDepth),
                new Vector3(-halfFlightWidth, 0f, halfFlightDepth)
            };

            for (int i = 0; i < corners.Length; i++)
            {
                var next = corners[(i + 1) % corners.Length];
                Gizmos.DrawLine(corners[i], next);
            }
        }
        else
        {
            float radius = Mathf.Max(figureCircleDiameter * 0.5f, 0f);
            if (radius > 0f)
            {
                float halfSpacing = figureCircleCenterSpacing * 0.5f;
                Gizmos.DrawWireSphere(new Vector3(-halfSpacing, 0f, 0f), radius);
                Gizmos.DrawWireSphere(new Vector3(halfSpacing, 0f, 0f), radius);
            }
        }

        Gizmos.matrix = Matrix4x4.identity;
    }
#endif
}

#if UNITY_EDITOR
[CustomEditor(typeof(DroneCourseLayoutGenerator))]
public class DroneCourseLayoutGeneratorEditor : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        EditorGUILayout.Space();
        using (new EditorGUILayout.HorizontalScope())
        {
            if (GUILayout.Button("レイアウト生成"))
            {
                foreach (var target in targets)
                {
                    var generator = (DroneCourseLayoutGenerator)target;
                    generator.BuildLayout();
                    EditorUtility.SetDirty(generator);
                }
            }

            if (GUILayout.Button("生成物を削除"))
            {
                foreach (var target in targets)
                {
                    var generator = (DroneCourseLayoutGenerator)target;
                    generator.ClearLayout();
                    EditorUtility.SetDirty(generator);
                }
            }
        }
    }
}
#endif

