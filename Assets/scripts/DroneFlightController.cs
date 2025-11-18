using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

[RequireComponent(typeof(Rigidbody))]
public class DroneFlightController : MonoBehaviour
{
    public enum FlightMode
    {
        Position,
        Attitude
    }

    public enum ControlStickMode
    {
        Mode1,
        Mode2
    }

    [Header("初期モード")]
    [SerializeField] private FlightMode initialMode = FlightMode.Position;

    [Header("スラスト設定")]
    [SerializeField, Tooltip("合計推力の上限 (N)")] private float maxThrust = 80f;
    [SerializeField, Tooltip("推力の変化速度 (N/秒)")] private float thrustSlewRate = 480f;
    [SerializeField, Tooltip("ホバリング時の推力倍率 (1=重力と同等)")] private float hoverThrustMultiplier = 1.0f;

    [Header("姿勢制御共通")]
    [SerializeField, Tooltip("ヨー入力に対する回転速度 (度/秒)")] private float yawRateDegPerSec = 90f;
    [SerializeField, Tooltip("姿勢誤差に対するトルク係数")] private float attitudeResponse = 10f;
    [SerializeField, Tooltip("角速度減衰量")] private float angularDamping = 4f;
    [SerializeField, Tooltip("姿勢補正に用いる最大角度 (度)")] private float attitudeCorrectionAngle = 45f;
    [SerializeField, Tooltip("姿勢補正トルクの上限 (N·m)")] private float attitudeTorqueLimit = 25f;
    [SerializeField, Tooltip("P/Aモードで姿勢を直接補正する")] private bool snapAutoRotation = true;
    [SerializeField, Tooltip("P/Aモードでの姿勢スナップ速度 (度/秒)")] private float autoRotationSnapSpeedDegPerSec = 120f;

    [Header("Pモード: 位置保持")]
    [SerializeField, Tooltip("前後左右入力によるターゲット位置の移動速度 (m/s)")] private float positionCommandSpeed = 5.5f;
    [SerializeField, Tooltip("上下入力によるターゲット高度の移動速度 (m/s)")] private float altitudeCommandSpeed = 3.5f;
    [SerializeField, Tooltip("水平位置のバネ係数")] private float planarSpring = 12f;
    [SerializeField, Tooltip("水平位置の減衰係数")] private float planarDamping = 8f;
    [SerializeField, Tooltip("垂直位置のバネ係数")] private float verticalSpring = 18f;
    [SerializeField, Tooltip("垂直位置の減衰係数")] private float verticalDamping = 10f;
    [SerializeField, Tooltip("水平位置補正の応答倍率")] private float planarResponseGain = 1.8f;
    [SerializeField, Tooltip("垂直位置補正の応答倍率")] private float verticalResponseGain = 1.4f;
    [SerializeField, Tooltip("水平面での最大指令加速度 (m/s^2)")] private float maxPlanarAcceleration = 10f;
    [SerializeField, Tooltip("垂直方向の最大指令加速度 (m/s^2)")] private float maxVerticalAcceleration = 9f;
    [SerializeField, Tooltip("水平速度の最大許容量 (m/s)")] private float maxPlanarVelocity = 6f;
    [SerializeField, Tooltip("上下速度の最大許容量 (m/s)")] private float maxVerticalVelocity = 5f;

    [Header("Aモード: 姿勢保持")]
    [SerializeField, Tooltip("最大傾斜角 (度)")] private float maxTiltAngle = 35f;
    [SerializeField, Tooltip("高度保持のバネ係数")] private float altitudeSpring = 14f;
    [SerializeField, Tooltip("高度保持の減衰係数")] private float altitudeDamping = 7f;


    [Header("入力設定")]
    [SerializeField, Tooltip("横移動用 Input Axis 名 (空欄で未使用)")] private string horizontalAxis = "Horizontal";
    [SerializeField, Tooltip("前後移動用 Input Axis 名 (空欄で未使用)")] private string forwardAxis = "Vertical";
    [SerializeField, Tooltip("上昇下降用 Input Axis 名 (空欄で未使用)")] private string ascentAxis = "";
    [SerializeField, Tooltip("ヨー操作用 Input Axis 名 (空欄で未使用)")] private string yawAxis = "";
    [SerializeField, Tooltip("入力のデッドゾーン")] private float inputDeadZone = 0.1f;
    [SerializeField, Tooltip("1/2/3 キーでモード変更を有効にする")] private bool enableNumberKeyModeSwitch = true;
    [SerializeField, Tooltip("左スティック横軸の Input Axis 名 (空欄で未使用)")] private string leftStickHorizontalAxis = "Horizontal";
    [SerializeField, Tooltip("左スティック縦軸の Input Axis 名 (空欄で未使用)")] private string leftStickVerticalAxis = "Vertical";
    [SerializeField, Tooltip("右スティック横軸の Input Axis 名 (空欄で未使用)")] private string rightStickHorizontalAxis = "";
    [SerializeField, Tooltip("右スティック縦軸の Input Axis 名 (空欄で未使用)")] private string rightStickVerticalAxis = "";
    [SerializeField, Tooltip("操縦モード切替キー")] private KeyCode stickModeToggleKey = KeyCode.Tab;
    [SerializeField, Tooltip("操縦モード (Mode1/Mode2)")] private ControlStickMode controlStickMode = ControlStickMode.Mode2;
    [SerializeField, Tooltip("ピッチ入力の感度倍率")] private float pitchInputSensitivity = 1f;
    [SerializeField, Tooltip("ロール入力の感度倍率")] private float rollInputSensitivity = 1f;
    [SerializeField, Tooltip("スロットル入力の感度倍率")] private float throttleInputSensitivity = 1f;
    [SerializeField, Tooltip("ヨー入力の感度倍率")] private float yawInputSensitivity = 1f;

    [Header("バーチャルジョイスティック")]
    [SerializeField, Tooltip("左スティックに対応するバーチャルジョイスティック")] private VirtualJoystick leftVirtualJoystick;
    [SerializeField, Tooltip("右スティックに対応するバーチャルジョイスティック")] private VirtualJoystick rightVirtualJoystick;
    [SerializeField, Tooltip("バーチャルジョイスティックを優先的に使用する")] private bool preferVirtualJoysticks = true;

    [Header("オートホールド設定")]
    [SerializeField, Tooltip("入力が無いとみなす閾値")] private float autoHoldInputThreshold = 0.05f;

    [Header("UI")]
    [SerializeField, Tooltip("現在のモードを表示するテキスト")] private Text modeLabel;
    [SerializeField] private Dropdown flightModeDropdown;
    [SerializeField] private Dropdown stickModeDropdown;
    [SerializeField] private Toggle snapAutoRotationToggle;
    [SerializeField] private FloatSliderBinding pitchSensitivitySlider = new FloatSliderBinding(0.1f, 3f, 2, "ピッチ感度");
    [SerializeField] private FloatSliderBinding rollSensitivitySlider = new FloatSliderBinding(0.1f, 3f, 2, "ロール感度");
    [SerializeField] private FloatSliderBinding throttleSensitivitySlider = new FloatSliderBinding(0.1f, 3f, 2, "スロットル感度");
    [SerializeField] private FloatSliderBinding yawSensitivitySlider = new FloatSliderBinding(0.1f, 3f, 2, "ヨー感度");
    [SerializeField] private FloatSliderBinding maxPlanarVelocitySlider = new FloatSliderBinding(0f, 12f, 1, "最大水平速度");
    [SerializeField] private FloatSliderBinding maxVerticalVelocitySlider = new FloatSliderBinding(0f, 10f, 1, "最大垂直速度");

    [Header("ランディング管理")]
    [SerializeField, Tooltip("着地判定に使用するレイの長さ (m)")] private float landingCheckDistance = 0.35f;
    [SerializeField, Tooltip("着地判定に使用するレイヤーマスク")] private LayerMask landingLayerMask = ~0;
    [SerializeField, Tooltip("着地判定とみなす垂直速度の上限 (m/s)")] private float landingVerticalSpeedThreshold = 0.6f;
    [SerializeField, Tooltip("着地判定とみなす水平速度の上限 (m/s)")] private float landingPlanarSpeedThreshold = 0.7f;
    [SerializeField, Tooltip("着地判定スフィアの半径 (m)")] private float landingProbeRadius = 0.18f;
    [SerializeField, Tooltip("着地判定スフィアの下向きオフセット (m)")] private float landingProbeOffset = 0.05f;
    [SerializeField, Tooltip("着地完了とみなす地面との距離 (m)")] private float landingHeightTolerance = 0.08f;
    [SerializeField, Tooltip("ディスアーム判定に必要な下向き入力")] private float disarmInputThreshold = -0.8f;
    [SerializeField, Tooltip("ディスアーム判定に必要な連続時間 (秒)")] private float disarmHoldDuration = 0.5f;
    [SerializeField, Tooltip("アーム判定に使用するスティック入力閾値")] private float armGestureThreshold = 0.8f;
    [SerializeField, Tooltip("アーム判定に必要な連続時間 (秒)")] private float armHoldDuration = 0.6f;
    [SerializeField, Tooltip("モーター音などに使用する AudioSource")] private AudioSource rotorAudioSource;
    [SerializeField, Tooltip("着陸時にRigidbodyの位置を固定する")] private bool freezePositionWhenLanded = true;
    [SerializeField, Tooltip("位置固定時に回転も固定する")] private bool freezeRotationWhenLanded = false;

    [Header("アーム/ディスアーム遷移")]
    [SerializeField, Tooltip("アーム時に強制スロットルを目標値まで滑らかに上げる時間 (秒)")] private float armThrottleRampDuration = 0.8f;
    [SerializeField, Tooltip("ディスアーム時に強制スロットルを0まで滑らかに下げる時間 (秒)")] private float disarmThrottleRampDuration = 0.6f;
    [SerializeField, Tooltip("モーター音の最小ピッチ")] private float rotorPitchMin = 0.6f;
    [SerializeField, Tooltip("モーター音の最大ピッチ")] private float rotorPitchMax = 1.15f;
    [SerializeField, Tooltip("アーム直後（アイドル時）のピッチ")] private float rotorPitchIdle = 0.72f;
    [SerializeField, Tooltip("ホバリング時の基準ピッチ")] private float rotorPitchHover = 0.98f;
    [SerializeField, Tooltip("上昇時に加算するピッチ幅")] private float rotorPitchAscentBoost = 0.1f;
    [SerializeField, Tooltip("上昇として扱う垂直速度基準 (m/s)")] private float rotorPitchAscentReferenceSpeed = 3f;
    [SerializeField, Tooltip("移動時に加算するピッチ幅")] private float rotorPitchMoveBoost = 0.04f;
    [SerializeField, Tooltip("移動として扱う水平速度基準 (m/s)")] private float rotorPitchMoveReferenceSpeed = 5f;
    [SerializeField, Tooltip("ピッチ変化の補間速度 (1/秒)")] private float rotorPitchSlewRate = 3.5f;

    [Header("ステータス表示")]
    [SerializeField, Tooltip("高度を表示するテキスト")]
    private Text altitudeStatusLabel;
    [SerializeField, Tooltip("速度を表示するテキスト")]
    private Text speedStatusLabel;

    Rigidbody _rb;
    DroneScript _droneScript;
    bool _controllerConnected;
    FlightMode _currentMode;
    Vector3 _hoverTargetPosition;
    float _targetYawDeg;
    float _targetAltitude;
    float _currentThrust;
    float _baseMaxPlanarVelocity;
    float _baseMaxVerticalVelocity;

    Vector2 _planarInput;
    float _verticalInput;
    float _yawInput;
    Vector2 _leftStickRaw;
    Vector2 _rightStickRaw;
    bool _motorsArmed = false;
    bool _isLanded;
    bool _wasLanded;
    float _disarmHoldTimer;
    float _armHoldTimer;
    float _forcedRotorThrottle = -1f;
    float _forcedRotorThrottleTarget = -1f;
    Vector2 _planarControlInput;
    float _verticalControlInput;
    float _yawControlInput;
    bool _liftOffRequested;
    readonly RaycastHit[] _landingHits = new RaycastHit[4];
    bool _rotorAudioStopPending;
    bool _rotorDeactivatePending;
    float _currentRotorPitch;
    RigidbodyConstraints _initialConstraints;
    bool _constraintsFrozen;

    void Awake()
    {
        _rb = GetComponent<Rigidbody>();
        TryGetComponent(out _droneScript);
        if (_rb != null)
        {
            _initialConstraints = _rb.constraints;
        }
        _constraintsFrozen = false;
        _baseMaxPlanarVelocity = Mathf.Max(0.01f, maxPlanarVelocity);
        _baseMaxVerticalVelocity = Mathf.Max(0.01f, maxVerticalVelocity);
        float minPitch = Mathf.Min(rotorPitchMin, rotorPitchMax);
        float maxPitch = Mathf.Max(rotorPitchMin, rotorPitchMax);
        _currentRotorPitch = Mathf.Clamp(rotorPitchIdle, minPitch, maxPitch);
        InitializeUI();
    }

    void OnEnable()
    {
        SetFlightMode(initialMode, true);
        DisarmMotors();
    }

    void Update()
    {
        UpdateControllerPresence();
        ReadInput();
        if (Input.GetKeyDown(stickModeToggleKey)) ToggleStickMode();

        if (enableNumberKeyModeSwitch)
        {
            if (Input.GetKeyDown(KeyCode.Alpha1)) SetFlightMode(FlightMode.Position);
            if (Input.GetKeyDown(KeyCode.Alpha2)) SetFlightMode(FlightMode.Attitude);
        }

        UpdateStatusUI();
    }

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        UpdateLandingState(dt);

        switch (_currentMode)
        {
            case FlightMode.Position:
                SimulatePositionMode(dt);
                break;
            case FlightMode.Attitude:
                SimulateAttitudeMode(dt);
                break;
        }
    }

    void ReadInput()
    {
        Vector2 controllerLeft = new Vector2(ReadAxis(leftStickHorizontalAxis), ReadAxis(leftStickVerticalAxis));
        Vector2 keyboardLeft = new Vector2(AxisFromKeys(KeyCode.D, KeyCode.A), AxisFromKeys(KeyCode.W, KeyCode.S));
        Vector2 leftStick = SelectInputVector(controllerLeft, keyboardLeft, leftVirtualJoystick);

        Vector2 controllerRight = new Vector2(ReadAxis(rightStickHorizontalAxis), ReadAxis(rightStickVerticalAxis));
        Vector2 keyboardRight = new Vector2(AxisFromKeys(KeyCode.RightArrow, KeyCode.LeftArrow), AxisFromKeys(KeyCode.UpArrow, KeyCode.DownArrow));
        Vector2 rightStick = SelectInputVector(controllerRight, keyboardRight, rightVirtualJoystick);

        _leftStickRaw = leftStick;
        _rightStickRaw = rightStick;

        float throttleInput = 0f;
        float yawInput = 0f;
        float pitchInput = 0f;
        float rollInput = 0f;

        switch (controlStickMode)
        {
            case ControlStickMode.Mode1:
                throttleInput = rightStick.y;
                yawInput = leftStick.x;
                pitchInput = leftStick.y;
                rollInput = rightStick.x;
                break;
            case ControlStickMode.Mode2:
            default:
                throttleInput = leftStick.y;
                yawInput = leftStick.x;
                pitchInput = rightStick.y;
                rollInput = rightStick.x;
                break;
        }

        throttleInput = ApplyDeadZone(throttleInput);
        yawInput = ApplyDeadZone(yawInput);
        pitchInput = ApplyDeadZone(pitchInput);
        rollInput = ApplyDeadZone(rollInput);

        float throttleSensitivity = Mathf.Max(0f, throttleInputSensitivity);
        float yawSensitivity = Mathf.Max(0f, yawInputSensitivity);
        float rollSensitivity = Mathf.Max(0f, rollInputSensitivity);
        float pitchSensitivity = Mathf.Max(0f, pitchInputSensitivity);

        float verticalRange = Mathf.Max(1f, throttleSensitivity);
        float yawRange = Mathf.Max(1f, yawSensitivity);

        _verticalInput = Mathf.Clamp(throttleInput * throttleSensitivity, -verticalRange, verticalRange);
        _yawInput = Mathf.Clamp(yawInput * yawSensitivity, -yawRange, yawRange);
        _planarInput = new Vector2(
            Mathf.Clamp(rollInput * rollSensitivity, -rollSensitivity, rollSensitivity),
            Mathf.Clamp(pitchInput * pitchSensitivity, -pitchSensitivity, pitchSensitivity));

        _verticalControlInput = _verticalInput;
        _yawControlInput = _yawInput;
        _planarControlInput = _planarInput;
    }

    float ReadAxis(string axisName)
    {
        if (string.IsNullOrEmpty(axisName)) return 0f;
        try
        {
            return Mathf.Clamp(Input.GetAxis(axisName), -1f, 1f);
        }
        catch
        {
            return 0f;
        }
    }

    float AxisFromKeys(KeyCode positive, KeyCode negative)
    {
        float value = 0f;
        if (Input.GetKey(positive)) value += 1f;
        if (Input.GetKey(negative)) value -= 1f;
        return value;
    }

    Vector2 SelectInputVector(Vector2 controllerValue, Vector2 keyboardValue, VirtualJoystick virtualJoystick)
    {
        Vector2 controllerClamped = ClampInput(controllerValue);
        Vector2 keyboardClamped = ClampInput(keyboardValue);

        if (virtualJoystick != null)
        {
            Vector2 virtualValue = ClampInput(virtualJoystick.Value);
            if (preferVirtualJoysticks || virtualJoystick.HasInput)
            {
                return virtualValue;
            }
        }

        if (_controllerConnected && controllerClamped.magnitude > inputDeadZone)
        {
            return controllerClamped;
        }

        return keyboardClamped;
    }

    Vector2 ClampInput(Vector2 value) => new Vector2(Mathf.Clamp(value.x, -1f, 1f), Mathf.Clamp(value.y, -1f, 1f));

    float ApplyDeadZone(float value) => Mathf.Abs(value) < inputDeadZone ? 0f : value;

    void SimulatePositionMode(float dt)
    {
        UpdateYawTarget(dt);
        UpdateHoverTarget(dt);

        Vector3 positionError = _hoverTargetPosition - _rb.position;
        Vector3 planarError = new Vector3(positionError.x, 0f, positionError.z);
        Vector3 planarVelocity = new Vector3(_rb.velocity.x, 0f, _rb.velocity.z);

        Vector3 planarAccelWorld = (planarError * planarSpring) - (planarVelocity * planarDamping);
        planarAccelWorld *= planarResponseGain;
        planarAccelWorld = Vector3.ClampMagnitude(planarAccelWorld, maxPlanarAcceleration);

        float verticalCommand = (positionError.y * verticalSpring) - (_rb.velocity.y * verticalDamping);
        verticalCommand *= verticalResponseGain;
        float verticalAccel = Mathf.Clamp(verticalCommand, -maxVerticalAcceleration, maxVerticalAcceleration);

        Vector3 desiredAccelWorld = new Vector3(planarAccelWorld.x, verticalAccel, planarAccelWorld.z);

        Vector3 desiredForce = desiredAccelWorld - Physics.gravity;
        float gravityMag = Mathf.Max(0.1f, Physics.gravity.magnitude);
        float maxTiltRad = Mathf.Deg2Rad * Mathf.Clamp(maxTiltAngle, 0.1f, 80f);

        float verticalForce = Mathf.Max(desiredForce.y, gravityMag * 0.2f);
        Vector3 planarForce = Vector3.ProjectOnPlane(desiredForce, Vector3.up);
        float planarForceMag = planarForce.magnitude;
        float maxPlanarForce = Mathf.Tan(maxTiltRad) * verticalForce;
        if (planarForceMag > maxPlanarForce && planarForceMag > 1e-4f)
        {
            float excess = planarForceMag - maxPlanarForce;
            float limited = maxPlanarForce + excess * 0.25f;
            planarForce = planarForce.normalized * limited;
        }

        desiredForce = planarForce + Vector3.up * verticalForce;
        Vector3 desiredUp = desiredForce.sqrMagnitude > 1e-4f ? desiredForce.normalized : Vector3.up;

        Quaternion desiredRotation = BuildDesiredRotation(desiredUp, _targetYawDeg);
        ApplyOrientation(desiredRotation, dt, true);

        float forceMagnitude = desiredForce.magnitude;
        float targetThrust = Mathf.Clamp(_rb.mass * forceMagnitude, 0f, maxThrust);
        ApplyThrust(targetThrust, dt);
    }

    void SimulateAttitudeMode(float dt)
    {
        UpdateYawTarget(dt);
        UpdateAltitudeTarget(dt);

        float pitchDeg = Mathf.Clamp(_planarControlInput.y, -1f, 1f) * maxTiltAngle;
        float rollDeg = Mathf.Clamp(-_planarControlInput.x, -1f, 1f) * maxTiltAngle;

        Quaternion yawRotation = Quaternion.AngleAxis(_targetYawDeg, Vector3.up);
        Quaternion tiltRotation = Quaternion.Euler(pitchDeg, 0f, rollDeg);
        Quaternion desiredRotation = yawRotation * tiltRotation;
        Vector3 desiredUp = desiredRotation * Vector3.up;

        float altitudeError = _targetAltitude - _rb.position.y;
        float verticalAccel = Mathf.Clamp(altitudeError * altitudeSpring - _rb.velocity.y * altitudeDamping, -maxVerticalAcceleration, maxVerticalAcceleration);
        Vector3 desiredAccel = new Vector3(0f, verticalAccel, 0f);

        ApplyOrientation(desiredRotation, dt, true);

        float targetThrust = Mathf.Clamp(_rb.mass * Vector3.Dot(desiredAccel - Physics.gravity, desiredUp), 0f, maxThrust);
        ApplyThrust(targetThrust, dt);
    }


    void ApplyOrientation(Quaternion desiredRotation, float dt, bool snap)
    {
        if (snap && snapAutoRotation)
        {
            float snapSpeed = Mathf.Clamp(autoRotationSnapSpeedDegPerSec, 30f, 720f);
            float maxDegrees = snapSpeed * dt;
            Quaternion nextRot = Quaternion.RotateTowards(_rb.rotation, desiredRotation, maxDegrees <= 0f ? snapSpeed : maxDegrees);
            _rb.MoveRotation(nextRot);
            _rb.angularVelocity = Vector3.zero;
            return;
        }

        Quaternion rotationError = desiredRotation * Quaternion.Inverse(_rb.rotation);
        rotationError.ToAngleAxis(out float angleDeg, out Vector3 axis);

        if (float.IsNaN(axis.x) || float.IsNaN(axis.y) || float.IsNaN(axis.z)) return;
        if (axis.sqrMagnitude < 1e-6f) axis = Vector3.up;

        angleDeg = Mathf.DeltaAngle(0f, angleDeg);
        if (Mathf.Abs(angleDeg) > 0.001f)
        {
            float clampedAngle = Mathf.Clamp(angleDeg, -attitudeCorrectionAngle, attitudeCorrectionAngle);
            Vector3 torque = axis.normalized * (clampedAngle * Mathf.Deg2Rad * attitudeResponse);
            torque = Vector3.ClampMagnitude(torque, attitudeTorqueLimit);
            _rb.AddTorque(torque, ForceMode.Acceleration);
        }

        _rb.AddTorque(-_rb.angularVelocity * angularDamping, ForceMode.Acceleration);
    }

    void ApplyThrust(float targetThrust, float dt)
    {
        UpdateForcedRotorThrottle(dt);

        if (!_motorsArmed)
        {
            _currentThrust = Mathf.MoveTowards(_currentThrust, 0f, thrustSlewRate * dt);

            float idleRotorLevel = _forcedRotorThrottle >= 0f ? Mathf.Clamp01(_forcedRotorThrottle) : 0f;

            if (_droneScript != null)
            {
                _droneScript.throttle01 = idleRotorLevel;
            }

            UpdateRotorAudio(idleRotorLevel, dt);
            return;
        }

        bool throttleLocked = _isLanded && !_liftOffRequested;

        if (throttleLocked)
        {
            float hoverThrust = Mathf.Clamp(_rb.mass * Physics.gravity.magnitude * hoverThrustMultiplier, 0f, maxThrust);
            _currentThrust = Mathf.MoveTowards(_currentThrust, hoverThrust, thrustSlewRate * dt);
        }
        else
        {
            _currentThrust = Mathf.MoveTowards(_currentThrust, targetThrust, thrustSlewRate * dt);
        }

        Vector3 thrust = transform.up * _currentThrust;
        _rb.AddForce(thrust, ForceMode.Force);

        float rotorLevel;
        float rotorOutput;

        if (_forcedRotorThrottle >= 0f)
        {
            rotorLevel = Mathf.Clamp01(_forcedRotorThrottle);
            rotorOutput = rotorLevel;
        }
        else
        {
            float normalized = maxThrust > 0f ? Mathf.Clamp01(_currentThrust / maxThrust) : 0f;
            rotorLevel = normalized;
            rotorOutput = Mathf.Lerp(0.6f, 1f, normalized);
        }

        if (_droneScript != null)
        {
            _droneScript.throttle01 = rotorOutput;
        }

        UpdateRotorAudio(rotorLevel, dt);
    }

    void UpdateForcedRotorThrottle(float dt)
    {
        if (_forcedRotorThrottleTarget < 0f)
        {
            return;
        }

        float target = Mathf.Clamp01(_forcedRotorThrottleTarget);
        float current = _forcedRotorThrottle >= 0f ? Mathf.Clamp01(_forcedRotorThrottle) : 0f;
        float duration = target > current
            ? Mathf.Max(0.05f, armThrottleRampDuration)
            : Mathf.Max(0.05f, disarmThrottleRampDuration);

        if (duration <= Mathf.Epsilon)
        {
            _forcedRotorThrottle = target;
        }
        else
        {
            float step = dt / duration;
            _forcedRotorThrottle = Mathf.MoveTowards(current, target, step);
        }
    }

    void UpdateRotorAudio(float rotorLevel, float dt)
    {
        if (rotorAudioSource == null) return;

        rotorLevel = Mathf.Clamp01(rotorLevel);

        float minPitch = Mathf.Min(rotorPitchMin, rotorPitchMax);
        float maxPitch = Mathf.Max(rotorPitchMin, rotorPitchMax);
        float idlePitch = Mathf.Clamp(rotorPitchIdle, minPitch, maxPitch);
        float hoverPitch = Mathf.Clamp(rotorPitchHover, minPitch, maxPitch);

        float thrustNormalized = maxThrust > 0f ? Mathf.Clamp01(_currentThrust / maxThrust) : 0f;
        float forcedLevel = _forcedRotorThrottle >= 0f ? Mathf.Clamp01(_forcedRotorThrottle) : rotorLevel;
        float baseLevel = Mathf.Max(thrustNormalized, forcedLevel * 0.15f);

        float hoverThrust = _rb != null ? _rb.mass * Physics.gravity.magnitude * hoverThrustMultiplier : 0f;
        float hoverLevel = Mathf.Clamp01(hoverThrust / Mathf.Max(0.1f, maxThrust));

        float targetPitch;
        if (_isLanded && !_liftOffRequested)
        {
            float landedBlend = Mathf.Clamp01(forcedLevel);
            targetPitch = Mathf.Lerp(minPitch, idlePitch, landedBlend);
        }
        else
        {
            float hoverFactor = hoverLevel > 1e-4f ? Mathf.Clamp01(baseLevel / hoverLevel) : baseLevel;
            targetPitch = Mathf.Lerp(idlePitch, hoverPitch, hoverFactor);

            float verticalVelocity = _rb != null ? Vector3.Dot(_rb.velocity, Vector3.up) : 0f;
            float ascentVelocityFactor = rotorPitchAscentReferenceSpeed > 0.01f
                ? Mathf.Clamp01(Mathf.Max(0f, verticalVelocity) / rotorPitchAscentReferenceSpeed)
                : 0f;
            float ascentInputFactor = Mathf.Clamp01(Mathf.Max(0f, _verticalControlInput));
            float ascentFactor = Mathf.Clamp01(Mathf.Max(ascentVelocityFactor, ascentInputFactor));
            targetPitch += rotorPitchAscentBoost * ascentFactor;

            Vector3 planarVelocity = _rb != null ? new Vector3(_rb.velocity.x, 0f, _rb.velocity.z) : Vector3.zero;
            float moveFactor = Mathf.Clamp01(planarVelocity.magnitude / Mathf.Max(0.1f, rotorPitchMoveReferenceSpeed));
            targetPitch += rotorPitchMoveBoost * moveFactor;
        }

        targetPitch = Mathf.Clamp(targetPitch, minPitch, maxPitch);

        float slew = Mathf.Max(0.1f, rotorPitchSlewRate);
        _currentRotorPitch = Mathf.MoveTowards(_currentRotorPitch, targetPitch, slew * Mathf.Max(0f, dt));
        rotorAudioSource.pitch = _currentRotorPitch;

        bool shouldPlay = _motorsArmed || _forcedRotorThrottleTarget > 0f || forcedLevel > 0.01f || targetPitch > minPitch + 0.01f;
        if (shouldPlay)
        {
            if (!rotorAudioSource.isPlaying)
            {
                rotorAudioSource.Play();
            }
            _rotorAudioStopPending = false;
        }
        else if (!_motorsArmed)
        {
            _rotorAudioStopPending = true;
        }

        if (_rotorDeactivatePending && forcedLevel <= 0.001f && baseLevel <= 0.001f)
        {
            if (_droneScript != null)
            {
                _droneScript.SetRotorsActive(false);
            }
            _rotorDeactivatePending = false;
        }

        if (_rotorAudioStopPending && !_motorsArmed && forcedLevel <= 0.001f && baseLevel <= 0.001f && Mathf.Abs(_currentRotorPitch - minPitch) < 0.01f)
        {
            rotorAudioSource.Stop();
            _rotorAudioStopPending = false;
        }
    }

    void UpdateYawTarget(float dt)
    {
        if (Mathf.Abs(_yawControlInput) < Mathf.Epsilon) return;
        _targetYawDeg = NormalizeAngle(_targetYawDeg + _yawControlInput * yawRateDegPerSec * dt);
    }

    void UpdateHoverTarget(float dt)
    {
        bool hasPlanarInput = _planarControlInput.magnitude > autoHoldInputThreshold;

        if (hasPlanarInput)
        {
            Vector3 planar = new Vector3(_planarControlInput.x, 0f, _planarControlInput.y);
            float planarMagnitude = planar.magnitude;
            float maxPlanarInput = Mathf.Max(1f, rollInputSensitivity, pitchInputSensitivity);
            float clampedMagnitude = Mathf.Clamp(planarMagnitude, 0f, maxPlanarInput);

            if (planarMagnitude > 1e-4f)
            {
                Vector3 worldDirection = Quaternion.Euler(0f, _targetYawDeg, 0f) * (planar / planarMagnitude);
                float planarScaleFactor = _baseMaxPlanarVelocity > 1e-4f ? maxPlanarVelocity / _baseMaxPlanarVelocity : 1f;
                Vector3 desiredVelocity = worldDirection * positionCommandSpeed * planarScaleFactor * clampedMagnitude;
                desiredVelocity = Vector3.ClampMagnitude(desiredVelocity, maxPlanarVelocity);
                _hoverTargetPosition += desiredVelocity * dt;
            }
        }
        else
        {
            _hoverTargetPosition.x = _rb.position.x;
            _hoverTargetPosition.z = _rb.position.z;
        }

        if (Mathf.Abs(_verticalControlInput) > autoHoldInputThreshold)
        {
            float verticalScaleFactor = _baseMaxVerticalVelocity > 1e-4f ? maxVerticalVelocity / _baseMaxVerticalVelocity : 1f;
            float desiredVerticalVelocity = Mathf.Clamp(_verticalControlInput * altitudeCommandSpeed * verticalScaleFactor, -maxVerticalVelocity, maxVerticalVelocity);
            _hoverTargetPosition += Vector3.up * (desiredVerticalVelocity * dt);
        }
        else
        {
            _hoverTargetPosition.y = _rb.position.y;
        }

        _targetAltitude = _hoverTargetPosition.y;
    }

    void UpdateAltitudeTarget(float dt)
    {
        if (Mathf.Abs(_verticalControlInput) <= autoHoldInputThreshold)
        {
            _targetAltitude = _rb.position.y;
            return;
        }

        float verticalScaleFactor = _baseMaxVerticalVelocity > 1e-4f ? maxVerticalVelocity / _baseMaxVerticalVelocity : 1f;
        float desiredVerticalVelocity = Mathf.Clamp(_verticalControlInput * altitudeCommandSpeed * verticalScaleFactor, -maxVerticalVelocity, maxVerticalVelocity);
        _targetAltitude += desiredVerticalVelocity * dt;
    }

    void UpdateLandingState(float dt)
    {
        if (_rb == null) return;

        _wasLanded = _isLanded;
        _isLanded = EvaluateLanding();

        if (_isLanded)
        {
            Vector3 planarVelocity = new Vector3(_rb.velocity.x, 0f, _rb.velocity.z);
            bool withinVertical = Mathf.Abs(_rb.velocity.y) <= landingVerticalSpeedThreshold;
            bool withinPlanar = planarVelocity.magnitude <= landingPlanarSpeedThreshold;
            if (!(withinVertical && withinPlanar))
            {
                _isLanded = false;
            }
        }

        if (!_isLanded)
        {
            _disarmHoldTimer = 0f;
            _armHoldTimer = 0f;
            _planarControlInput = _planarInput;
            _yawControlInput = _yawInput;
            _verticalControlInput = _verticalInput;
            _liftOffRequested = false;
            _rotorAudioStopPending = false;
            _rotorDeactivatePending = false;
            if (_motorsArmed && (_forcedRotorThrottle >= 0f || _forcedRotorThrottleTarget >= 0f))
            {
                _forcedRotorThrottle = -1f;
                _forcedRotorThrottleTarget = -1f;
            }
            RestoreLandingConstraints();
            return;
        }

        float holdDt = Mathf.Max(0f, dt);
        float disarmDuration = Mathf.Max(0.05f, disarmHoldDuration);
        float armDuration = Mathf.Max(0.05f, armHoldDuration);

        if (_verticalInput <= disarmInputThreshold)
        {
            _disarmHoldTimer += holdDt;
            if (_disarmHoldTimer >= disarmDuration)
            {
                DisarmMotors();
            }
        }
        else
        {
            _disarmHoldTimer = 0f;
        }

        if (!_motorsArmed && DetectArmGesture())
        {
            _armHoldTimer += holdDt;
            if (_armHoldTimer >= armDuration)
            {
                ArmMotors();
            }
        }
        else
        {
            _armHoldTimer = 0f;
        }

        _planarControlInput = Vector2.zero;
        _yawControlInput = 0f;
        _verticalControlInput = Mathf.Max(0f, _verticalInput);

        if (_verticalInput > autoHoldInputThreshold)
        {
            _liftOffRequested = true;
        }
        else if (!_motorsArmed)
        {
            _liftOffRequested = false;
        }

        if (_liftOffRequested)
        {
            RestoreLandingConstraints();
        }
        else
        {
            ApplyLandingConstraints();
        }
    }

    void ApplyLandingConstraints()
    {
        if (_rb == null) return;
        if (!freezePositionWhenLanded && !freezeRotationWhenLanded) return;
        if (_constraintsFrozen) return;

        RigidbodyConstraints constraints = _initialConstraints;

        if (freezePositionWhenLanded)
        {
            constraints |= RigidbodyConstraints.FreezePositionX
                         | RigidbodyConstraints.FreezePositionZ;
        }

        if (freezeRotationWhenLanded)
        {
            constraints |= RigidbodyConstraints.FreezeRotationX
                         | RigidbodyConstraints.FreezeRotationY
                         | RigidbodyConstraints.FreezeRotationZ;
        }

        _rb.constraints = constraints;
        _rb.velocity = Vector3.zero;
        _rb.angularVelocity = Vector3.zero;
        _constraintsFrozen = true;
    }

    void RestoreLandingConstraints()
    {
        if (_rb == null) return;
        if (!_constraintsFrozen) return;

        _rb.constraints = _initialConstraints;
        _constraintsFrozen = false;
    }

    bool EvaluateLanding()
    {
        if (_rb == null) return false;

        float rayDistance = Mathf.Max(0.05f, landingCheckDistance);
        float probeRadius = Mathf.Clamp(landingProbeRadius, 0.01f, 2f);
        float offset = Mathf.Max(0f, landingProbeOffset);
        Vector3 origin = _rb.worldCenterOfMass + Vector3.down * offset;
        int mask = landingLayerMask.value;
        if (mask == 0) mask = Physics.DefaultRaycastLayers;

        int hitCount = Physics.SphereCastNonAlloc(origin, probeRadius, Vector3.down, _landingHits, rayDistance, mask, QueryTriggerInteraction.Ignore);
        bool groundDetected = false;
        float closestDistance = float.PositiveInfinity;

        for (int i = 0; i < hitCount; i++)
        {
            var hit = _landingHits[i];
            if (hit.collider == null) continue;
            if (hit.collider.attachedRigidbody == _rb) continue;
            if (hit.collider.transform.IsChildOf(transform)) continue;
            groundDetected = true;
            closestDistance = Mathf.Min(closestDistance, hit.distance);
        }

        if (!groundDetected) return false;

        float tolerance = Mathf.Max(0.005f, landingHeightTolerance);
        return closestDistance <= tolerance;
    }

    bool DetectArmGesture()
    {
        float threshold = Mathf.Clamp01(Mathf.Abs(armGestureThreshold));
        if (threshold <= 0f) return false;

        return _leftStickRaw.x <= -threshold &&
               _leftStickRaw.y <= -threshold &&
               _rightStickRaw.x >= threshold &&
               _rightStickRaw.y <= -threshold;
    }

    void DisarmMotors()
    {
        if (!_motorsArmed && Mathf.Abs(_forcedRotorThrottle) < 1e-4f && Mathf.Abs(_forcedRotorThrottleTarget) < 1e-4f) return;

        float currentNormalized = maxThrust > 0f ? Mathf.Clamp01(_currentThrust / maxThrust) : 0f;
        _motorsArmed = false;
        _forcedRotorThrottle = _forcedRotorThrottle >= 0f ? Mathf.Clamp01(_forcedRotorThrottle) : currentNormalized;
        _forcedRotorThrottleTarget = 0f;
        _currentThrust = 0f;
        _disarmHoldTimer = 0f;
        _armHoldTimer = 0f;
        _liftOffRequested = false;

        _rotorAudioStopPending = rotorAudioSource != null;
        _rotorDeactivatePending = _droneScript != null;
    }

    void ArmMotors()
    {
        if (_motorsArmed && (_forcedRotorThrottleTarget >= 0.99f || _forcedRotorThrottle >= 0.99f)) return;

        _motorsArmed = true;
        _forcedRotorThrottle = Mathf.Max(0f, _forcedRotorThrottle >= 0f ? _forcedRotorThrottle : 0f);
        _forcedRotorThrottleTarget = 1f;
        _currentThrust = Mathf.Clamp(_currentThrust, 0f, maxThrust);
        _armHoldTimer = 0f;
        _disarmHoldTimer = 0f;
        _liftOffRequested = false;
        _rotorAudioStopPending = false;
        _rotorDeactivatePending = false;

        if (rotorAudioSource != null && !rotorAudioSource.isPlaying)
        {
            rotorAudioSource.pitch = Mathf.Min(rotorPitchMin, rotorPitchMax);
            rotorAudioSource.Play();
        }

        if (_droneScript != null)
        {
            _droneScript.SetRotorsActive(true);
        }
    }

    public void SetFlightMode(FlightMode mode, bool force = false)
    {
        if (!force && _currentMode == mode) return;

        _currentMode = mode;
        _hoverTargetPosition = _rb.position;
        _targetAltitude = _rb.position.y;
        _targetYawDeg = transform.eulerAngles.y;
        _currentThrust = Mathf.Clamp(_rb.mass * Physics.gravity.magnitude * hoverThrustMultiplier, 0f, maxThrust);
        RefreshModeLabel();
        SyncUI();
        UpdateStatusUI();
    }

    public void CycleFlightMode(int direction = 1)
    {
        int count = Enum.GetValues(typeof(FlightMode)).Length;
        int current = (int)_currentMode;
        current = (current + direction + count) % count;
        SetFlightMode((FlightMode)current);
    }

    Quaternion BuildDesiredRotation(Vector3 desiredUp, float desiredYawDeg)
    {
        desiredUp = desiredUp.sqrMagnitude < 1e-6f ? Vector3.up : desiredUp.normalized;

        Vector3 yawForward = Quaternion.Euler(0f, desiredYawDeg, 0f) * Vector3.forward;
        Vector3 planarForward = Vector3.ProjectOnPlane(yawForward, desiredUp);
        if (planarForward.sqrMagnitude < 1e-4f)
        {
            planarForward = Vector3.ProjectOnPlane(transform.forward, desiredUp);
            if (planarForward.sqrMagnitude < 1e-4f)
            {
                planarForward = Vector3.ProjectOnPlane(Vector3.forward, desiredUp);
            }
        }

        planarForward.Normalize();
        return Quaternion.LookRotation(planarForward, desiredUp);
    }

    float NormalizeAngle(float deg)
    {
        return Mathf.Repeat(deg + 180f, 360f) - 180f;
    }

    public FlightMode CurrentMode => _currentMode;

    void UpdateControllerPresence()
    {
        _controllerConnected = false;
        var names = Input.GetJoystickNames();
        if (names == null || names.Length == 0) return;
        for (int i = 0; i < names.Length; i++)
        {
            if (!string.IsNullOrEmpty(names[i]))
            {
                _controllerConnected = true;
                break;
            }
        }
    }

    public void RefreshModeLabel()
    {
        if (modeLabel == null) return;
        string flightModeText = _currentMode switch
        {
            FlightMode.Position => "MODE : P (Position Hold)",
            FlightMode.Attitude => "MODE : A (Attitude)",
            _ => $"MODE : {_currentMode}"
        };
        modeLabel.text = $"{flightModeText} | Stick : {controlStickMode}";
    }

    void ToggleStickMode()
    {
        SetControlStickMode(controlStickMode == ControlStickMode.Mode1 ? ControlStickMode.Mode2 : ControlStickMode.Mode1);
    }

    public void SetControlStickMode(ControlStickMode mode)
    {
        controlStickMode = mode;
        RefreshModeLabel();
        SyncUI();
    }

    public ControlStickMode CurrentStickMode => controlStickMode;

    public float PlanarSpring
    {
        get => planarSpring;
        set => planarSpring = Mathf.Max(0f, value);
    }

    public float PlanarDamping
    {
        get => planarDamping;
        set => planarDamping = Mathf.Max(0f, value);
    }

    public float VerticalSpring
    {
        get => verticalSpring;
        set => verticalSpring = Mathf.Max(0f, value);
    }

    public float VerticalDamping
    {
        get => verticalDamping;
        set => verticalDamping = Mathf.Max(0f, value);
    }

    public float PlanarResponseGain
    {
        get => planarResponseGain;
        set => planarResponseGain = Mathf.Max(0.01f, value);
    }

    public float VerticalResponseGain
    {
        get => verticalResponseGain;
        set => verticalResponseGain = Mathf.Max(0.01f, value);
    }

    public float AutoRotationSnapSpeedDegPerSec
    {
        get => autoRotationSnapSpeedDegPerSec;
        set => autoRotationSnapSpeedDegPerSec = Mathf.Clamp(value, 30f, 720f);
    }

    public float PositionCommandSpeed
    {
        get => positionCommandSpeed;
        set => positionCommandSpeed = Mathf.Max(0f, value);
    }

    public float AltitudeCommandSpeed
    {
        get => altitudeCommandSpeed;
        set => altitudeCommandSpeed = Mathf.Max(0f, value);
    }

    public float MaxPlanarAcceleration
    {
        get => maxPlanarAcceleration;
        set => maxPlanarAcceleration = Mathf.Max(0f, value);
    }

    public float MaxVerticalAcceleration
    {
        get => maxVerticalAcceleration;
        set => maxVerticalAcceleration = Mathf.Max(0f, value);
    }

    public float PitchInputSensitivity
    {
        get => pitchInputSensitivity;
        set => pitchInputSensitivity = Mathf.Clamp(value, 0.1f, 3f);
    }

    public float RollInputSensitivity
    {
        get => rollInputSensitivity;
        set => rollInputSensitivity = Mathf.Clamp(value, 0.1f, 3f);
    }

    public float ThrottleInputSensitivity
    {
        get => throttleInputSensitivity;
        set => throttleInputSensitivity = Mathf.Clamp(value, 0.1f, 3f);
    }

    public float YawInputSensitivity
    {
        get => yawInputSensitivity;
        set => yawInputSensitivity = Mathf.Clamp(value, 0.1f, 3f);
    }

    [Serializable]
    class FloatSliderBinding
    {
        public Slider slider;
        public Text valueLabel;
        public string labelPrefix;
        public bool overrideRange = false;
        public float min = 0f;
        public float max = 1f;
        public int decimals = 1;

        public FloatSliderBinding() { }

        public FloatSliderBinding(float min, float max, int decimals)
        {
            overrideRange = true;
            this.min = min;
            this.max = max;
            this.decimals = decimals;
        }

        public FloatSliderBinding(float min, float max, int decimals, string labelPrefix)
        {
            overrideRange = true;
            this.min = min;
            this.max = max;
            this.decimals = decimals;
            this.labelPrefix = labelPrefix;
        }
    }

    void InitializeUI()
    {
        if (flightModeDropdown != null)
        {
            var modes = Enum.GetNames(typeof(FlightMode));
            flightModeDropdown.ClearOptions();
            flightModeDropdown.AddOptions(new List<string>(modes));
            flightModeDropdown.onValueChanged.RemoveAllListeners();
            flightModeDropdown.SetValueWithoutNotify((int)_currentMode);
            flightModeDropdown.onValueChanged.AddListener(i => SetFlightMode((FlightMode)Mathf.Clamp(i, 0, modes.Length - 1)));
            AdjustDropdownTemplateHeight(flightModeDropdown);
        }

        if (stickModeDropdown != null)
        {
            var modes = Enum.GetNames(typeof(ControlStickMode));
            stickModeDropdown.ClearOptions();
            stickModeDropdown.AddOptions(new List<string>(modes));
            stickModeDropdown.onValueChanged.RemoveAllListeners();
            stickModeDropdown.SetValueWithoutNotify((int)controlStickMode);
            stickModeDropdown.onValueChanged.AddListener(i => SetControlStickMode((ControlStickMode)Mathf.Clamp(i, 0, modes.Length - 1)));
            AdjustDropdownTemplateHeight(stickModeDropdown);
        }

        if (snapAutoRotationToggle != null)
        {
            snapAutoRotationToggle.onValueChanged.RemoveAllListeners();
            snapAutoRotationToggle.SetIsOnWithoutNotify(snapAutoRotation);
            snapAutoRotationToggle.onValueChanged.AddListener(v => snapAutoRotation = v);
        }

        ConfigureSlider(pitchSensitivitySlider, PitchInputSensitivity, v => PitchInputSensitivity = v);
        ConfigureSlider(rollSensitivitySlider, RollInputSensitivity, v => RollInputSensitivity = v);
        ConfigureSlider(throttleSensitivitySlider, ThrottleInputSensitivity, v => ThrottleInputSensitivity = v);
        ConfigureSlider(yawSensitivitySlider, YawInputSensitivity, v => YawInputSensitivity = v);
        ConfigureSlider(maxPlanarVelocitySlider, maxPlanarVelocity, v => maxPlanarVelocity = Mathf.Max(0f, v));
        ConfigureSlider(maxVerticalVelocitySlider, maxVerticalVelocity, v => maxVerticalVelocity = Mathf.Max(0f, v));
    }

    void ConfigureSlider(FloatSliderBinding binding, float currentValue, Action<float> setter)
    {
        if (binding == null || binding.slider == null) return;

        if (string.IsNullOrEmpty(binding.labelPrefix) && binding.valueLabel != null)
        {
            binding.labelPrefix = binding.valueLabel.text;
        }

        float min = binding.overrideRange ? binding.min : binding.slider.minValue;
        float max = binding.overrideRange ? binding.max : binding.slider.maxValue;
        string format = GetFormat(binding);

        binding.slider.onValueChanged.RemoveAllListeners();
        binding.slider.minValue = min;
        binding.slider.maxValue = max;
        SetSliderValue(binding, currentValue, format);

        binding.slider.onValueChanged.AddListener(v =>
        {
            setter.Invoke(v);
            SetSliderValue(binding, v, format);
        });
    }

    void AdjustDropdownTemplateHeight(Dropdown dropdown)
    {
        if (dropdown == null) return;

        RectTransform template = dropdown.template;
        if (template == null) return;

        bool templateWasActive = template.gameObject.activeSelf;
        if (!templateWasActive) template.gameObject.SetActive(true);

        try
        {
            ScrollRect scrollRect = template.GetComponent<ScrollRect>();
            RectTransform contentRect = scrollRect != null ? scrollRect.content : null;
            if (contentRect == null)
            {
                var layoutGroup = template.GetComponentInChildren<VerticalLayoutGroup>(true);
                contentRect = layoutGroup != null ? layoutGroup.GetComponent<RectTransform>() : null;
            }

            Toggle itemToggle = template.GetComponentInChildren<Toggle>(true);
            RectTransform itemRect = itemToggle != null ? itemToggle.GetComponent<RectTransform>() : null;

            float itemHeight = itemRect != null ? Mathf.Abs(itemRect.rect.height) : 0f;
            if (itemHeight < 1e-3f && itemToggle != null && itemToggle.targetGraphic != null)
            {
                itemHeight = Mathf.Abs(itemToggle.targetGraphic.rectTransform.rect.height);
            }
            if (itemHeight < 1e-3f)
            {
                itemHeight = 20f;
            }

            int optionCount = Mathf.Max(1, dropdown.options?.Count ?? 0);

            VerticalLayoutGroup layout = contentRect != null ? contentRect.GetComponent<VerticalLayoutGroup>() : null;
            float spacing = layout != null ? layout.spacing : 0f;
            float padding = layout != null ? layout.padding.top + layout.padding.bottom : 0f;

            float totalHeight = optionCount * itemHeight + spacing * (optionCount - 1) + padding;
            float desiredHeight = Mathf.Max(template.rect.height, totalHeight);

            template.SetSizeWithCurrentAnchors(RectTransform.Axis.Vertical, desiredHeight);

            RectTransform viewportRect = scrollRect != null ? scrollRect.viewport : null;
            if (viewportRect != null)
            {
                viewportRect.SetSizeWithCurrentAnchors(RectTransform.Axis.Vertical, desiredHeight);
            }

            if (contentRect != null)
            {
                contentRect.SetSizeWithCurrentAnchors(RectTransform.Axis.Vertical, totalHeight);
                LayoutRebuilder.ForceRebuildLayoutImmediate(contentRect);
            }
            else
            {
                LayoutRebuilder.ForceRebuildLayoutImmediate(template);
            }
        }
        finally
        {
            if (!templateWasActive)
            {
                template.gameObject.SetActive(false);
            }
        }
    }

    static void UpdateSliderLabel(FloatSliderBinding binding, float value, string format)
    {
        if (binding.valueLabel != null)
        {
            string prefix = binding.labelPrefix;
            if (string.IsNullOrEmpty(prefix) && binding.slider != null)
            {
                prefix = binding.slider.name;
            }

            string formattedValue = value.ToString(format);
            binding.valueLabel.text = string.IsNullOrEmpty(prefix)
                ? formattedValue
                : $"{prefix}({formattedValue})";
        }
    }

    static string GetFormat(FloatSliderBinding binding) => binding.decimals <= 0 ? "0" : $"F{binding.decimals}";

    static void SetSliderValue(FloatSliderBinding binding, float value, string format = null)
    {
        if (binding == null || binding.slider == null) return;
        float min = binding.overrideRange ? binding.min : binding.slider.minValue;
        float max = binding.overrideRange ? binding.max : binding.slider.maxValue;
        float clamped = Mathf.Clamp(value, min, max);
        binding.slider.SetValueWithoutNotify(clamped);
        UpdateSliderLabel(binding, clamped, format ?? GetFormat(binding));
    }

    void SyncUI()
    {
        if (flightModeDropdown != null)
            flightModeDropdown.SetValueWithoutNotify((int)_currentMode);
        if (stickModeDropdown != null)
            stickModeDropdown.SetValueWithoutNotify((int)controlStickMode);
        if (snapAutoRotationToggle != null)
            snapAutoRotationToggle.SetIsOnWithoutNotify(snapAutoRotation);

        SetSliderValue(pitchSensitivitySlider, PitchInputSensitivity);
        SetSliderValue(rollSensitivitySlider, RollInputSensitivity);
        SetSliderValue(throttleSensitivitySlider, ThrottleInputSensitivity);
        SetSliderValue(yawSensitivitySlider, YawInputSensitivity);
        SetSliderValue(maxPlanarVelocitySlider, maxPlanarVelocity);
        SetSliderValue(maxVerticalVelocitySlider, maxVerticalVelocity);
    }

    void UpdateStatusUI()
    {
        if (_rb == null) return;

        if (altitudeStatusLabel != null)
        {
            float altitudeMeters = _rb.position.y;
            altitudeStatusLabel.text = $"高度:{altitudeMeters:0.0} m";
        }

        if (speedStatusLabel != null)
        {
            float speedMetersPerSecond = _rb.velocity.magnitude;
            speedStatusLabel.text = $"速度:{speedMetersPerSecond:0.0} m/s";
        }
    }
}

