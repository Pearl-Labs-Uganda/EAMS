using UnityEngine;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using UnityEngine.InputSystem;

/// <summary>
/// WONDER MODE agent: drives forward and avoids obstacles, with NO target.
///
/// Forked from DifferentialCarAgent. The point of this fork is a sim-to-real
/// transfer test, so the observation vector contains ONLY quantities the
/// physical rover can actually measure. Everything the real car cannot sense is
/// gone from the observations -- though rewards still use privileged simulator
/// state freely, because rewards do not exist at deployment.
///
/// OBSERVATION VECTOR = 11. Set "Vector Observation Space Size" to 11 in the
/// Behavior Parameters component or the model will silently mismatch.
///
///   idx  quantity                     real source on the car
///   ---  ---------------------------  -------------------------------------
///   0    yaw rate, normalised          MPU6050 gz  (i2c bus 7)
///   1    ultrasonic front, 0-1         HC-SR04 front
///   2    ultrasonic rear, 0-1          HC-SR04 rear
///   3    IR front-left, 0/1            LM393
///   4    IR front-right, 0/1           LM393
///   5    IR rear-left, 0/1             LM393
///   6    IR rear-right, 0/1            LM393
///   7    IR left, 0/1                  LM393
///   8    IR right, 0/1                 LM393
///   9    smoothedLeft, -1..1           controller-side slew state
///   10   smoothedRight, -1..1          controller-side slew state
///
/// THIS ORDER IS A CONTRACT with policy.py on the rover. If you reorder it here,
/// reorder it there in the same commit. An index mismatch does not throw -- it
/// just drives badly, which is the hardest possible failure to diagnose.
///
/// Dropped versus the target-seeking agent, and why:
///   - target direction (3) + distance (1): the rover has no map, GPS, or
///     localisation, so it can never produce these.
///   - linear velocity (3): no odometry (see progress report open items). The
///     agent must now infer its speed from smoothedLeft/Right, which are
///     COMMANDS not measurements. That is a real blind spot, and it is the
///     car's actual blind spot, so it is better trained into than papered over.
///
/// Added versus the target-seeking agent:
///   - yaw rate is now an honest observation. It was in the old vector too, but
///     the physical IMU was reading zeros until the I2C bus fix, so a policy
///     leaning on it would not have transferred. It transfers now.
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class WonderCarAgent : Agent
{
    [Header("Procedural Furniture Placement")]
    [Tooltip("List of all furniture GameObjects to randomize in the scene.")]
    public List<Transform> furnitureList;

    [Tooltip("Clearance zone radius around the car's spawn point.")]
    public float carBuffer = 2.0f;

    [Header("Environment")]
    [Tooltip("BoxCollider on the floor defining where the car and furniture can spawn.")]
    public Collider spawnArea;

    [Tooltip("Randomize the CAR's start position each episode. In wonder mode there is no target to randomize, so this is what stops the agent memorising one corner of the room.")]
    public bool randomizeCarSpawn = true;

    [Header("Episode Limits")]
    [Tooltip("Max decision steps before the episode times out and BOOTSTRAPS (EpisodeInterrupted, not EndEpisode). In wonder mode a timeout is the GOOD outcome - it means the car survived the whole episode without hitting anything. Leave the inspector Max Step at 0 so this is the single source of truth.")]
    public int maxEpisodeSteps = 1500;

    [Header("Wheel Anchors")]
    [Tooltip("Empty GameObject positioned at the point where the Left Wheel contacts the ground. Must have a localized X-axis offset!")]
    public Transform leftWheelPowerPoint;
    [Tooltip("Empty GameObject positioned at the point where the Right Wheel contacts the ground. Must have a localized X-axis offset!")]
    public Transform rightWheelPowerPoint;

    [Header("Movement Settings")]
    [Tooltip("Forward/backward linear drive power applied directly at each wheel position.")]
    public float motorForce = 20f;

    [Tooltip("Higher = motors respond to commands faster. Lower this to mimic real motor/ESC ramp-up lag.")]
    public float motorResponseRate = 6f;

    [Header("HC-SR04 Ultrasonic Sensors")]
    public Transform ultrasonicFrontOrigin;
    public Transform ultrasonicRearOrigin;
    [Tooltip("Datasheet max range for HC-SR04 is ~4m.")]
    public float ultrasonicRange = 5.0f;

    [Header("IR Sensors (x6)")]
    public Transform irFrontLeft;
    public Transform irFrontRight;
    public Transform irRearLeft;
    public Transform irRearRight;
    public Transform irLeft;
    public Transform irRight;
    [Tooltip("Typical IR obstacle sensor detection range.")]
    public float irRange = 2.0f;

    [Tooltip("Maximum expected forward/backward velocity. No longer observed, but still used to scale the forward-speed REWARD so the reward stays roughly 0-1 per step.")]
    public float maxLinearSpeed = 3f;

    [Tooltip("Maximum expected yaw rate used to scale the angular speed observation.")]
    public float maxAngularSpeed = 6f;

    [Header("Sim-to-Real: Sensor Noise")]
    [Tooltip("Toggle to enable/disable reality-gap sensor noise injection during training.")]
    public bool simulateSensorNoise = true;

    [Tooltip("Std dev of Gaussian noise added to the normalised ultrasonic reading.")]
    [Range(0f, 0.15f)] public float ultrasonicNoiseStdDev = 0.03f;

    [Tooltip("Chance an IR reading randomly flips each physics step, mimicking real false pos/neg.")]
    [Range(0f, 0.3f)] public float irNoiseFlipChance = 0.04f;

    [Tooltip("Std dev of Gaussian noise on the normalised yaw rate. The sim gyro is exact; a real MPU6050 is not. NEW for wonder mode, because yaw rate is now a load-bearing observation rather than one of eighteen.")]
    [Range(0f, 0.15f)] public float gyroNoiseStdDev = 0.02f;

    [Tooltip("Fixed per-episode yaw-rate bias, sampled from +/- this value. Mimics MPU6050 zero-rate offset, which is constant within a run but differs between power cycles. Set 0 to disable.")]
    [Range(0f, 0.15f)] public float gyroBiasRange = 0.03f;

    [Header("Sim-to-Real: Physics Randomization")]
    [Tooltip("Toggle Domain Randomization across training episodes to build policy robust to mass/friction variances.")]
    public bool randomizePhysicsPerEpisode = true;

    [Tooltip("Min/Max scaling factors applied to the agent's Rigidbody mass.")]
    public Vector2 massMultiplierRange = new Vector2(0.85f, 1.15f);

    [Tooltip("Min/Max scaling factors applied to motor force.")]
    public Vector2 motorForceMultiplierRange = new Vector2(0.85f, 1.15f);

    [Header("Reward Shaping")]
    [Tooltip("Per-step reward for MEASURED forward velocity (not commanded). Replaces the target-distance reward. Measured, so a car pinned against a wall at full throttle earns nothing - which is exactly the failure mode a commanded-throttle reward would pay for. BUDGET: this accrues every step, so total ~= multiplier * avgNormalisedSpeed * maxEpisodeSteps. At 0.005 * 0.5 * 1500 that is ~3.75 for a clean run, against -1.0 for a collision. Keep that ratio in mind if you change either.")]
    public float forwardSpeedReward = 0.005f;

    [Tooltip("Multiplier for penalizing the car as it gets closer to obstacles via sonar.")]
    public float obstacleProximityPenalty = 0.01f;

    [Tooltip("Flat penalty applied per active short-range IR sensor trigger.")]
    public float irPenalty = 0.02f;

    [Tooltip("Penalty multiplier for rapid, high-frequency oscillations in motor commands.")]
    public float actionJitterPenalty = 0.001f;

    [Tooltip("Penalty applied per step while the mixed drive is negative (reversing). Backing out of a furniture pocket is legitimate here, so this starts lower than the target-seeking agent's 0.005.")]
    public float reversingPenalty = 0.002f;

    [Tooltip("DEFAULTS TO 0 ON PURPOSE. Per-step penalty while the car is barely moving, to break 'cowering' (sitting still scores 0, which beats risking a -1 collision). Only raise this if you actually observe cowering in TensorBoard. DANGER: idlePenalty * maxEpisodeSteps must stay well below the 1.0 collision penalty, or the agent learns that crashing early is cheaper than idling - it will drive into a wall on purpose. At 1500 steps, keep this under ~0.0004.")]
    public float idlePenalty = 0f;

    [Tooltip("Normalised forward speed below which the car counts as idle for idlePenalty.")]
    public float idleSpeedThreshold = 0.05f;

    // Component and Baseline State Caching
    private Rigidbody rb;
    private Vector3 startingPosition;
    private float baseMotorForce;
    private float baseMass;

    // Curriculum: lets the training YAML drive how many obstacles are active.
    private Unity.MLAgents.EnvironmentParameters envParams;

    // Cached so per-episode outcome logging doesn't re-fetch it every time.
    private StatsRecorder statsRecorder;

    // Tracking state for rewards and motor smoothing
    private float prevLeftAction, prevRightAction;
    private float smoothedLeft, smoothedRight;

    // Per-episode gyro zero-rate offset (sim-to-real).
    private float gyroBias;

    // Time-limit bookkeeping for the bootstrap-on-timeout path.
    private int episodeStepCount;

    // Cached once per physics step so CollectObservations and OnActionReceived
    // never raycast the same sensor twice in the same step.
    private float ultrasonicFrontReading, ultrasonicRearReading;
    private float irFLReading, irFRReading, irRLReading, irRRReading, irLReading, irRReading;

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        startingPosition = transform.position;
        baseMotorForce = motorForce;
        baseMass = rb.mass;

        envParams = Academy.Instance.EnvironmentParameters;
        statsRecorder = Academy.Instance.StatsRecorder;
    }

    public override void OnEpisodeBegin()
    {
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        smoothedLeft = smoothedRight = 0f;
        prevLeftAction = prevRightAction = 0f;
        episodeStepCount = 0;

        // One gyro offset per episode, held constant, like a real power cycle.
        gyroBias = (gyroBiasRange > 0f) ? Random.Range(-gyroBiasRange, gyroBiasRange) : 0f;

        if (randomizePhysicsPerEpisode)
            RandomizePhysics();

        int obstacleCount = (furnitureList != null) ? furnitureList.Count : 0;
        if (envParams != null)
            obstacleCount = Mathf.RoundToInt(envParams.GetWithDefault("obstacle_count", obstacleCount));

        // Furniture goes down first, then the car is placed in a clear spot.
        // Reversed relative to the target-seeking agent, where the target was
        // placed first and everything else cleared around it.
        RandomizeEnvironment(obstacleCount);
        PlaceCar();

        // Heading is always fully random. The target-seeking agent ramped
        // heading_range_deg as a curriculum because heading was defined relative
        // to the target; with no target there is no easy or hard heading.
        transform.rotation = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);

        UpdateSensorReadings();
    }

    private void RandomizePhysics()
    {
        rb.mass = baseMass * Random.Range(massMultiplierRange.x, massMultiplierRange.y);
        motorForce = baseMotorForce * Random.Range(motorForceMultiplierRange.x, motorForceMultiplierRange.y);
    }

    /// <summary>
    /// Drops the car somewhere in the spawn area with clearance from every live
    /// obstacle. Falls back to the original inspector position if it cannot find
    /// a clear spot, rather than spawning inside a sofa.
    /// </summary>
    private void PlaceCar()
    {
        if (!randomizeCarSpawn || spawnArea == null)
        {
            transform.position = startingPosition;
            return;
        }

        float carRadius = GetObjectRadius(transform);

        for (int attempt = 0; attempt < 100; attempt++)
        {
            Vector3 candidate = GetRandomPointInSpawnArea();
            bool clear = true;

            if (furnitureList != null)
            {
                foreach (var item in furnitureList)
                {
                    if (item == null || !item.gameObject.activeSelf) continue;
                    float combined = carRadius + GetObjectRadius(item) + carBuffer;
                    if (Vector3.Distance(candidate, item.position) < combined)
                    {
                        clear = false;
                        break;
                    }
                }
            }

            if (clear)
            {
                transform.position = candidate;
                return;
            }
        }

        Debug.LogWarning("WonderCarAgent: no clear car spawn found in 100 tries; " +
                         "falling back to the initial position. Reduce obstacle_count " +
                         "or enlarge the spawn area.");
        transform.position = startingPosition;
    }

    private void FixedUpdate()
    {
        UpdateSensorReadings();
    }

    private void UpdateSensorReadings()
    {
        ultrasonicFrontReading = ReadUltrasonic(ultrasonicFrontOrigin);
        ultrasonicRearReading = ReadUltrasonic(ultrasonicRearOrigin);

        irFLReading = ReadIR(irFrontLeft);
        irFRReading = ReadIR(irFrontRight);
        irRLReading = ReadIR(irRearLeft);
        irRRReading = ReadIR(irRearRight);
        irLReading = ReadIR(irLeft);
        irRReading = ReadIR(irRight);
    }

    /// <summary>
    /// Emulates an HC-SR04 using a 15-ray fan to mimic the physical cone.
    /// Returns 0 (obstacle touching) to 1 (clear).
    /// </summary>
    private float ReadUltrasonic(Transform origin)
    {
        if (origin == null) return 1f;

        float minDistance = ultrasonicRange;
        int numRays = 15;
        float totalAngle = 30f;

        for (int i = 0; i < numRays; i++)
        {
            float angle = -totalAngle / 2f + (totalAngle / (numRays - 1)) * i;
            Vector3 rayDirection = Quaternion.Euler(0f, angle, 0f) * origin.forward;
            RaycastHit hit;

            if (Physics.Raycast(origin.position, rayDirection, out hit, ultrasonicRange))
            {
                if (hit.collider.CompareTag("obstacle") || hit.collider.CompareTag("Wall"))
                {
                    if (hit.distance < minDistance) minDistance = hit.distance;
                }
            }
        }

        float normalized = minDistance / ultrasonicRange;

        if (simulateSensorNoise)
            normalized = Mathf.Clamp01(normalized + SampleGaussian(0f, ultrasonicNoiseStdDev));

        return normalized;
    }

    /// <summary>
    /// Emulates a digital IR proximity sensor with a narrow 6-ray bundle.
    /// </summary>
    private float ReadIR(Transform origin)
    {
        if (origin == null) return 0f;

        float reading = 0f;
        int numRays = 6;
        float totalAngle = 6f;

        for (int i = 0; i < numRays; i++)
        {
            float angle = -totalAngle / 2f + (totalAngle / (numRays - 1)) * i;
            Vector3 rayDirection = Quaternion.Euler(0f, angle, 0f) * origin.forward;
            RaycastHit hit;

            if (Physics.Raycast(origin.position, rayDirection, out hit, irRange))
            {
                if (hit.collider.CompareTag("obstacle") || hit.collider.CompareTag("Wall"))
                {
                    reading = 1f;
                }
            }
        }

        if (simulateSensorNoise && Random.value < irNoiseFlipChance)
            reading = 1f - reading;

        return reading;
    }

    private float SampleGaussian(float mean, float stdDev)
    {
        float u1 = 1f - Random.value;
        float u2 = 1f - Random.value;
        float randStdNormal = Mathf.Sqrt(-2f * Mathf.Log(u1)) * Mathf.Sin(2f * Mathf.PI * u2);
        return mean + stdDev * randStdNormal;
    }

    /// <summary>
    /// 11 observations, every one of which the physical rover can measure.
    /// Behavior Parameters -> Vector Observation Space Size MUST be 11.
    /// </summary>
    public override void CollectObservations(VectorSensor sensor)
    {
        // 0. Yaw rate, normalised (1 obs) -- MPU6050 gz.
        float yaw = Mathf.Clamp(rb.angularVelocity.y / maxAngularSpeed, -1f, 1f);
        if (simulateSensorNoise)
            yaw = Mathf.Clamp(yaw + gyroBias + SampleGaussian(0f, gyroNoiseStdDev), -1f, 1f);
        sensor.AddObservation(yaw);

        // 1-2. Ultrasonic (2 obs)
        sensor.AddObservation(ultrasonicFrontReading);
        sensor.AddObservation(ultrasonicRearReading);

        // 3-8. IR (6 obs)
        sensor.AddObservation(irFLReading);
        sensor.AddObservation(irFRReading);
        sensor.AddObservation(irRLReading);
        sensor.AddObservation(irRRReading);
        sensor.AddObservation(irLReading);
        sensor.AddObservation(irRReading);

        // 9-10. Motor latency state (2 obs). Now the ONLY speed cue the agent
        // has, since linear velocity is gone. Already in [-1, 1].
        sensor.AddObservation(smoothedLeft);
        sensor.AddObservation(smoothedRight);

        // Total = 11
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        episodeStepCount++;

        // Action space is (throttle, steer), unchanged from the target-seeking
        // agent so the rover-side controller mapping stays identical.
        float throttle = actions.ContinuousActions[0];
        float steer = actions.ContinuousActions[1];
        float targetLeft = Mathf.Clamp(throttle + steer, -1f, 1f);
        float targetRight = Mathf.Clamp(throttle - steer, -1f, 1f);

        smoothedLeft = Mathf.MoveTowards(smoothedLeft, targetLeft, motorResponseRate * Time.fixedDeltaTime);
        smoothedRight = Mathf.MoveTowards(smoothedRight, targetRight, motorResponseRate * Time.fixedDeltaTime);

        if (leftWheelPowerPoint != null && rightWheelPowerPoint != null)
        {
            rb.AddForceAtPosition(transform.forward * smoothedLeft * motorForce, leftWheelPowerPoint.position, ForceMode.Acceleration);
            rb.AddForceAtPosition(transform.forward * smoothedRight * motorForce, rightWheelPowerPoint.position, ForceMode.Acceleration);
        }
        else
        {
            Debug.LogWarning("Wheel anchors are unassigned! Physics cannot execute correctly.");
        }

        float forwardMovement = (smoothedLeft + smoothedRight) / 2f;

        // --- Reward: MEASURED forward speed ---
        // This is privileged simulator state, and that is fine: rewards never
        // run on the rover. Using measured velocity rather than commanded
        // throttle is deliberate - a car wedged against a wall with the motors
        // straining reads near zero here and earns nothing, whereas a
        // commanded-throttle reward would pay it full price for wall-humping.
        float localForwardVel = transform.InverseTransformDirection(rb.linearVelocity).z;
        float normalisedForward = Mathf.Clamp(localForwardVel / maxLinearSpeed, -1f, 1f);
        AddReward(normalisedForward * forwardSpeedReward);

        // --- Penalty: idling (default 0; see tooltip before raising) ---
        if (idlePenalty > 0f && Mathf.Abs(normalisedForward) < idleSpeedThreshold)
            AddReward(-idlePenalty);

        // --- Penalty: obstacle proximity, graded by worst-case ultrasonic ---
        float worstUltrasonic = Mathf.Min(ultrasonicFrontReading, ultrasonicRearReading);
        if (worstUltrasonic < 0.5f)
            AddReward(-(0.5f - worstUltrasonic) * obstacleProximityPenalty);

        // --- Penalty: IR near-field trip, scaled by how many fire out of 6 ---
        int irHits = 0;
        if (irFLReading > 0f) irHits++;
        if (irFRReading > 0f) irHits++;
        if (irRLReading > 0f) irHits++;
        if (irRRReading > 0f) irHits++;
        if (irLReading > 0f) irHits++;
        if (irRReading > 0f) irHits++;
        if (irHits > 0)
            AddReward(-irPenalty * irHits);

        // --- Penalty: jerky motor commands ---
        float jitter = Mathf.Abs(targetLeft - prevLeftAction) + Mathf.Abs(targetRight - prevRightAction);
        AddReward(-jitter * actionJitterPenalty);
        prevLeftAction = targetLeft;
        prevRightAction = targetRight;

        // --- Penalty: reversing ---
        if (forwardMovement < 0f)
            AddReward(-reversingPenalty);

        // NOTE: there is deliberately NO existential penalty here. In the
        // target-seeking agent it pushed the car to finish quickly, and reaching
        // the target stopped the bleeding. With no success state the only way to
        // stop it would be to crash, so it would have been a standing incentive
        // to end the episode. Forward speed pays for progress instead.

        // --- Timeout: bootstrap, and treat it as the SUCCESSFUL outcome ---
        if (maxEpisodeSteps > 0 && episodeStepCount >= maxEpisodeSteps)
        {
            EndEpisodeWithOutcome("survived", 0f, true);
            return;
        }
    }

    /// <summary>
    /// Single exit point for every episode end. Wonder mode has two outcomes:
    /// "collision" (true terminal, -1) and "survived" (a time-limit interruption
    /// that bootstraps and gets no terminal reward). Outcome/Survived is the
    /// metric to watch in TensorBoard - it should climb toward 1.0.
    /// </summary>
    private void EndEpisodeWithOutcome(string outcome, float finalReward, bool interrupted)
    {
        if (!interrupted)
            SetReward(finalReward);

        if (statsRecorder != null)
        {
            statsRecorder.Add("Outcome/Collision", outcome == "collision" ? 1f : 0f);
            statsRecorder.Add("Outcome/Survived",  outcome == "survived"  ? 1f : 0f);
        }

        if (interrupted)
            EpisodeInterrupted();
        else
            EndEpisode();
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;

        float leftMotor = 0f;
        float rightMotor = 0f;

        if (Gamepad.current != null)
        {
            float leftStick = Gamepad.current.leftStick.y.ReadValue();
            float rightStick = Gamepad.current.rightStick.y.ReadValue();

            if (Mathf.Abs(leftStick) > 0.1f) leftMotor = leftStick;
            if (Mathf.Abs(rightStick) > 0.1f) rightMotor = rightStick;
        }

        if (Keyboard.current != null)
        {
            bool reverse = Keyboard.current.leftShiftKey.isPressed || Keyboard.current.rightShiftKey.isPressed;

            float leftKey = 0f;
            if (Keyboard.current.aKey.isPressed || Keyboard.current.leftArrowKey.isPressed)
                leftKey = reverse ? -1f : 1f;

            float rightKey = 0f;
            if (Keyboard.current.dKey.isPressed || Keyboard.current.rightArrowKey.isPressed)
                rightKey = reverse ? -1f : 1f;

            if (Mathf.Abs(leftMotor) <= 0.1f && leftKey != 0f) leftMotor = leftKey;
            if (Mathf.Abs(rightMotor) <= 0.1f && rightKey != 0f) rightMotor = rightKey;
        }

        float throttle = (leftMotor + rightMotor) / 2f;
        float steer = (leftMotor - rightMotor) / 2f;

        continuousActionsOut[0] = Mathf.Clamp(throttle, -1f, 1f);
        continuousActionsOut[1] = Mathf.Clamp(steer, -1f, 1f);
    }

    private void OnDrawGizmos()
    {
        if (leftWheelPowerPoint != null && rightWheelPowerPoint != null && Application.isPlaying)
        {
            float visualScale = 0.1f;
            Gizmos.color = Color.cyan;
            Vector3 leftStart = leftWheelPowerPoint.position;
            Vector3 leftForceVector = transform.forward * smoothedLeft * motorForce * visualScale;
            Gizmos.DrawRay(leftStart, leftForceVector);
            DrawGizmoArrowHead(leftStart, leftForceVector);

            Gizmos.color = Color.magenta;
            Vector3 rightStart = rightWheelPowerPoint.position;
            Vector3 rightForceVector = transform.forward * smoothedRight * motorForce * visualScale;
            Gizmos.DrawRay(rightStart, rightForceVector);
            DrawGizmoArrowHead(rightStart, rightForceVector);
        }

        DrawUltrasonicGizmo(ultrasonicFrontOrigin, ultrasonicFrontReading);
        DrawUltrasonicGizmo(ultrasonicRearOrigin, ultrasonicRearReading);

        DrawIRGizmo(irFrontLeft, irFLReading);
        DrawIRGizmo(irFrontRight, irFRReading);
        DrawIRGizmo(irRearLeft, irRLReading);
        DrawIRGizmo(irRearRight, irRRReading);
        DrawIRGizmo(irLeft, irLReading);
        DrawIRGizmo(irRight, irRReading);
    }

    private void DrawUltrasonicGizmo(Transform origin, float currentReading)
    {
        if (origin == null) return;

        Gizmos.color = Color.Lerp(Color.red, Color.green, currentReading);
        int numRays = 15;
        float totalAngle = 30f;

        for (int i = 0; i < numRays; i++)
        {
            float angle = -totalAngle / 2f + (totalAngle / (numRays - 1)) * i;
            Vector3 direction = Quaternion.Euler(0f, angle, 0f) * origin.forward;
            Gizmos.DrawRay(origin.position, direction * ultrasonicRange);
        }
    }

    private void DrawIRGizmo(Transform origin, float currentReading)
    {
        if (origin == null) return;

        Gizmos.color = (currentReading > 0.5f) ? Color.red : Color.yellow;
        int numRays = 6;
        float totalAngle = 6f;

        for (int i = 0; i < numRays; i++)
        {
            float angle = -totalAngle / 2f + (totalAngle / (numRays - 1)) * i;
            Vector3 direction = Quaternion.Euler(0f, angle, 0f) * origin.forward;
            Gizmos.DrawRay(origin.position, direction * irRange);
        }
    }

    private void DrawGizmoArrowHead(Vector3 pos, Vector3 direction)
    {
        if (direction.magnitude < 0.05f) return;

        Vector3 lookDir = direction.normalized;
        Vector3 right = Quaternion.LookRotation(lookDir) * Quaternion.Euler(0, 180 + 30, 0) * Vector3.forward;
        Vector3 left = Quaternion.LookRotation(lookDir) * Quaternion.Euler(0, 180 - 30, 0) * Vector3.forward;

        Vector3 arrowEnd = pos + direction;
        Gizmos.DrawRay(arrowEnd, right * 0.15f);
        Gizmos.DrawRay(arrowEnd, left * 0.15f);
    }

    /// <summary>
    /// Collision with obstacle or wall is the only true terminal state in wonder
    /// mode. All target-collider handling from the parent agent is gone.
    /// </summary>
    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("obstacle") || collision.gameObject.CompareTag("Wall"))
        {
            EndEpisodeWithOutcome("collision", -1.0f, false);
        }
    }

    private float GetObjectRadius(Transform t)
    {
        Collider col = t.GetComponent<Collider>();
        if (col != null)
            return Mathf.Max(col.bounds.extents.x, col.bounds.extents.z);
        return 0.5f;
    }

    private Vector3 GetRandomPointInSpawnArea()
    {
        if (spawnArea == null) return transform.position;
        Bounds bounds = spawnArea.bounds;
        float randomX = Random.Range(bounds.min.x, bounds.max.x);
        float randomZ = Random.Range(bounds.min.z, bounds.max.z);
        return new Vector3(randomX, transform.position.y, randomZ);
    }

    /// <summary>
    /// Places activeCount furniture pieces with clearance from each other.
    /// Unlike the target-seeking version there is no target to clear around, and
    /// no car clearance check here either - the car is placed afterwards by
    /// PlaceCar(), which does its own clearance test against what landed.
    /// </summary>
    private void RandomizeEnvironment(int activeCount)
    {
        if (furnitureList == null) return;

        int total = furnitureList.Count;
        activeCount = Mathf.Clamp(activeCount, 0, total);

        List<int> order = new List<int>(total);
        for (int i = 0; i < total; i++) order.Add(i);
        for (int i = total - 1; i > 0; i--)
        {
            int j = Random.Range(0, i + 1);
            int tmp = order[i];
            order[i] = order[j];
            order[j] = tmp;
        }

        List<Transform> placed = new List<Transform>(activeCount);

        for (int k = 0; k < total; k++)
        {
            Transform item = furnitureList[order[k]];
            if (item == null) continue;

            if (k >= activeCount)
            {
                item.gameObject.SetActive(false);
                continue;
            }

            item.gameObject.SetActive(true);
            float itemRadius = GetObjectRadius(item);

            Vector3 potentialPosition = Vector3.zero;
            bool validPosition = false;
            int attempts = 0;

            while (!validPosition && attempts < 100)
            {
                attempts++;
                potentialPosition = GetRandomPointInSpawnArea();

                bool tooCloseToOthers = false;
                foreach (var otherItem in placed)
                {
                    float combinedBuffer = itemRadius + GetObjectRadius(otherItem);
                    if (Vector3.Distance(potentialPosition, otherItem.position) < combinedBuffer)
                    {
                        tooCloseToOthers = true;
                        break;
                    }
                }

                if (!tooCloseToOthers)
                    validPosition = true;
            }

            if (validPosition)
            {
                item.position = potentialPosition;
                item.rotation = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);
                placed.Add(item);
            }
            else
            {
                item.gameObject.SetActive(false);
            }
        }
    }
}
