using UnityEngine;

[DisallowMultipleComponent]
public class ObjectFollower : MonoBehaviour
{
    [Header("Target Settings")]
    [Tooltip("追従するターゲットオブジェクト")]
    [SerializeField] Transform target;

    [Header("Follow Settings")]
    [Tooltip("位置を追従するか")]
    [SerializeField] bool followPosition = true;
    
    [Tooltip("追従速度（大きいほど速く追従）")]
    [SerializeField, Range(0.1f, 50f)] float followSpeed = 5f;
    
    [Tooltip("スムーズな追従を使用するか（falseの場合は即座に追従）")]
    [SerializeField] bool smoothFollow = true;

    [Header("Offset")]
    [Tooltip("ターゲットからの位置オフセット（初期位置から自動設定）")]
    [SerializeField] Vector3 positionOffset = Vector3.zero;

    [Header("Constraints")]
    [Tooltip("X軸の追従を無効化")]
    [SerializeField] bool freezeX = false;
    
    [Tooltip("Y軸の追従を無効化")]
    [SerializeField] bool freezeY = false;
    
    [Tooltip("Z軸の追従を無効化")]
    [SerializeField] bool freezeZ = false;

    [Header("Debug")]
    [SerializeField, ReadOnly] float distanceToTarget;

    bool _offsetInitialized = false;

    void Awake()
    {
        InitializeOffset();
    }

    void Start()
    {
        // Startでも初期化（Awakeでターゲットが設定されていない場合に備える）
        if (!_offsetInitialized)
        {
            InitializeOffset();
        }
    }

    void InitializeOffset()
    {
        if (target == null) return;

        // 初期位置からオフセットを自動計算
        positionOffset = transform.position - target.position;
        
        _offsetInitialized = true;
    }

    void LateUpdate()
    {
        if (target == null) return;

        if (followPosition)
        {
            UpdatePosition();
        }

        // デバッグ情報の更新
        distanceToTarget = Vector3.Distance(transform.position, target.position);
    }

    void UpdatePosition()
    {
        Vector3 targetPosition = target.position + positionOffset;
        
        // 軸の制約を適用
        if (freezeX) targetPosition.x = transform.position.x;
        if (freezeY) targetPosition.y = transform.position.y;
        if (freezeZ) targetPosition.z = transform.position.z;

        if (smoothFollow)
        {
            // スムーズな追従（指数関数的減衰）
            float t = 1f - Mathf.Exp(-followSpeed * Time.deltaTime);
            transform.position = Vector3.Lerp(transform.position, targetPosition, t);
        }
        else
        {
            // 即座に追従
            transform.position = targetPosition;
        }
    }

    /// <summary>
    /// ターゲットを設定します
    /// </summary>
    public void SetTarget(Transform newTarget)
    {
        target = newTarget;
        // ターゲット変更時にオフセットを再計算
        if (target != null)
        {
            InitializeOffset();
        }
    }

    /// <summary>
    /// 現在のターゲットを取得します
    /// </summary>
    public Transform GetTarget()
    {
        return target;
    }

    /// <summary>
    /// 追従を一時停止/再開します
    /// </summary>
    public void SetFollowEnabled(bool enabled)
    {
        followPosition = enabled;
    }
}

