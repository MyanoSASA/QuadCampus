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
        Attitude,
        Manual
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

    [Header("Mモード: 手動")]
    [SerializeField, Tooltip("ピッチ操作の相対トルク量")] private float manualPitchAuthority = 6f;
    [SerializeField, Tooltip("ロール操作の相対トルク量")] private float manualRollAuthority = 6f;
    [SerializeField, Tooltip("ヨー操作の相対トルク量")] private float manualYawAuthority = 4f;
    [SerializeField, Tooltip("上下入力が追加できる推力 (N)")] private float manualCollectiveAuthority = 35f;
    [SerializeField, Tooltip("手動時の水平速度減衰量")] private float manualVelocityDamping = 2f;

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

    Rigidbody _rb;
    DroneScript _droneScript;
    bool _controllerConnected;
    FlightMode _currentMode;
    Vector3 _hoverTargetPosition;
    float _targetYawDeg;
    float _targetAltitude;
    float _currentThrust;

    Vector2 _planarInput;
    float _verticalInput;
    float _yawInput;

    void Awake()
    {
        _rb = GetComponent<Rigidbody>();
        TryGetComponent(out _droneScript);
        InitializeUI();
    }

    void OnEnable()
    {
        SetFlightMode(initialMode, true);
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
    }

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        switch (_currentMode)
        {
            case FlightMode.Position:
                SimulatePositionMode(dt);
                break;
            case FlightMode.Attitude:
                SimulateAttitudeMode(dt);
                break;
            case FlightMode.Manual:
                SimulateManualMode(dt);
                break;
        }
    }

    void ReadInput()
    {
        Vector2 controllerLeft = new Vector2(ReadAxis(leftStickHorizontalAxis), ReadAxis(leftStickVerticalAxis));
        Vector2 keyboardLeft = new Vector2(AxisFromKeys(KeyCode.D, KeyCode.A), AxisFromKeys(KeyCode.W, KeyCode.S));
        Vector2 leftStick = SelectInputVector(controllerLeft, keyboardLeft);

        Vector2 controllerRight = new Vector2(ReadAxis(rightStickHorizontalAxis), ReadAxis(rightStickVerticalAxis));
        Vector2 keyboardRight = new Vector2(AxisFromKeys(KeyCode.RightArrow, KeyCode.LeftArrow), AxisFromKeys(KeyCode.UpArrow, KeyCode.DownArrow));
        Vector2 rightStick = SelectInputVector(controllerRight, keyboardRight);

        float throttleInput = 0f;
        float yawInput = 0f;
        float pitchInput = 0f;
        float rollInput = 0f;

        switch (controlStickMode)
        {
            case ControlStickMode.Mode1:
                throttleInput = rightStick.y;
                yawInput = rightStick.x;
                pitchInput = leftStick.y;
                rollInput = leftStick.x;
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

        _verticalInput = Mathf.Clamp(throttleInput * Mathf.Max(0f, throttleInputSensitivity), -1f, 1f);
        _yawInput = Mathf.Clamp(yawInput * Mathf.Max(0f, yawInputSensitivity), -1f, 1f);
        float rollSensitivity = Mathf.Max(0f, rollInputSensitivity);
        float pitchSensitivity = Mathf.Max(0f, pitchInputSensitivity);
        _planarInput = new Vector2(
            Mathf.Clamp(rollInput * rollSensitivity, -1f, 1f),
            Mathf.Clamp(pitchInput * pitchSensitivity, -1f, 1f));
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

    Vector2 SelectInputVector(Vector2 controllerValue, Vector2 keyboardValue)
    {
        Vector2 chosen = controllerValue;
        if (!_controllerConnected || chosen.magnitude < inputDeadZone)
        {
            chosen = keyboardValue;
        }
        return new Vector2(Mathf.Clamp(chosen.x, -1f, 1f), Mathf.Clamp(chosen.y, -1f, 1f));
    }

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

        float pitchDeg = Mathf.Clamp(_planarInput.y, -1f, 1f) * maxTiltAngle;
        float rollDeg = Mathf.Clamp(-_planarInput.x, -1f, 1f) * maxTiltAngle;

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

    void SimulateManualMode(float dt)
    {
        float hoverForce = _rb.mass * Physics.gravity.magnitude * hoverThrustMultiplier;
        float targetThrust = Mathf.Clamp(hoverForce + _verticalInput * manualCollectiveAuthority, 0f, maxThrust);

        Vector3 planarVelocity = new Vector3(_rb.velocity.x, 0f, _rb.velocity.z);
        _rb.AddForce(-planarVelocity * manualVelocityDamping, ForceMode.Acceleration);

        Vector3 torque = new Vector3(_planarInput.y * manualPitchAuthority, _yawInput * manualYawAuthority, -_planarInput.x * manualRollAuthority);
        _rb.AddRelativeTorque(torque, ForceMode.Acceleration);
        _rb.AddTorque(-_rb.angularVelocity * angularDamping, ForceMode.Acceleration);

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
        _currentThrust = Mathf.MoveTowards(_currentThrust, targetThrust, thrustSlewRate * dt);
        Vector3 thrust = transform.up * _currentThrust;
        _rb.AddForce(thrust, ForceMode.Force);

        if (_droneScript != null)
        {
            float normalized = maxThrust > 0f ? Mathf.Clamp01(_currentThrust / maxThrust) : 0f;
            float rotorThrottle = Mathf.Lerp(0.6f, 1f, normalized);
            _droneScript.throttle01 = rotorThrottle;
        }
    }

    void UpdateYawTarget(float dt)
    {
        if (Mathf.Abs(_yawInput) < Mathf.Epsilon) return;
        _targetYawDeg = NormalizeAngle(_targetYawDeg + _yawInput * yawRateDegPerSec * dt);
    }

    void UpdateHoverTarget(float dt)
    {
        bool hasPlanarInput = _planarInput.magnitude > autoHoldInputThreshold;

        if (hasPlanarInput)
        {
            Vector3 planar = new Vector3(_planarInput.x, 0f, _planarInput.y);
            float magnitude = Mathf.Clamp(planar.magnitude, 0f, 1f);
            Vector3 worldDirection = Quaternion.Euler(0f, _targetYawDeg, 0f) * planar.normalized;
            Vector3 desiredVelocity = worldDirection * positionCommandSpeed * magnitude;
            desiredVelocity = Vector3.ClampMagnitude(desiredVelocity, maxPlanarVelocity);
            _hoverTargetPosition += desiredVelocity * dt;
        }
        else
        {
            _hoverTargetPosition.x = _rb.position.x;
            _hoverTargetPosition.z = _rb.position.z;
        }

        if (Mathf.Abs(_verticalInput) > autoHoldInputThreshold)
        {
            float desiredVerticalVelocity = Mathf.Clamp(_verticalInput * altitudeCommandSpeed, -maxVerticalVelocity, maxVerticalVelocity);
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
        if (Mathf.Abs(_verticalInput) <= autoHoldInputThreshold)
        {
            _targetAltitude = _rb.position.y;
            return;
        }

        float desiredVerticalVelocity = Mathf.Clamp(_verticalInput * altitudeCommandSpeed, -maxVerticalVelocity, maxVerticalVelocity);
        _targetAltitude += desiredVerticalVelocity * dt;
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
            FlightMode.Manual => "MODE : M (Manual)",
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
        }

        if (stickModeDropdown != null)
        {
            var modes = Enum.GetNames(typeof(ControlStickMode));
            stickModeDropdown.ClearOptions();
            stickModeDropdown.AddOptions(new List<string>(modes));
            stickModeDropdown.onValueChanged.RemoveAllListeners();
            stickModeDropdown.SetValueWithoutNotify((int)controlStickMode);
            stickModeDropdown.onValueChanged.AddListener(i => SetControlStickMode((ControlStickMode)Mathf.Clamp(i, 0, modes.Length - 1)));
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
}

