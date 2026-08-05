using UnityEngine;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using UnityEngine.InputSystem; // Required for Modern Input System

/// <summary>
/// A Sim-to-Real RL Agent controlling a differential-drive robot (rover/car).
/// Features physical latency modeling, domain randomization, sensor noise injection, 
/// and normalized observations critical for identical execution on physical deployment hardware (e.g., Nvidia Jetson).
/// </summary>
[RequireComponent(typeof(Rigidbody))]
public class DifferentialCarAgent : Agent
{
    [Header("Procedural Furniture Placement")]
    [Tooltip("List of all furniture GameObjects to randomize in the scene.")]
    public List<Transform> furnitureList; 
    
    [Tooltip("Clearance zone radius around the target cube.")]
    public float targetBuffer = 2.0f;     
    
    [Tooltip("Clearance zone radius around the car's spawn point.")]
    public float carBuffer = 2.0f;

    [Header("Environment")]
    [Tooltip("The objective transform the car is trying to reach.")]
    public Transform targetLocation;
    
    [Tooltip("BoxCollider on the floor defining where the target can randomly spawn.")]
    public Collider spawnArea;
    
    [Tooltip("Distance (m) at which the car is considered to have arrived. Keep this larger than the real car's footprint - a physical rover shouldn't need to ram the target.")]
    public float successRadius = 0.25f;

    [Header("Episode Limits")]
    [Tooltip("Max decision steps before the episode times out and BOOTSTRAPS (EpisodeInterrupted, not EndEpisode) so a stuck/wedged car in clutter can't run forever and poison the buffer. Set to 0 to disable and rely on the Agent's inspector Max Step instead. If you use this, LEAVE the inspector Max Step at 0 so there's a single source of truth.")]
    public int maxEpisodeSteps = 2000;

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

    public float maxTargetDistance = 20f;
    
    [Tooltip("Maximum expected forward/backward velocity used to scale linear speed observations.")]
    public float maxLinearSpeed = 3f;
    
    [Tooltip("Maximum expected yaw rate used to scale angular speed observations.")]
    public float maxAngularSpeed = 6f;

    [Header("Sim-to-Real: Sensor Noise")]
    [Tooltip("Toggle to enable/disable reality-gap sensor noise injection during training.")]
    public bool simulateSensorNoise = true;
    
    [Tooltip("Std dev of Gaussian noise added to the normalised ultrasonic reading.")]
    [Range(0f, 0.15f)] public float ultrasonicNoiseStdDev = 0.03f;
    
    [Tooltip("Chance an IR reading randomly flips each physics step, mimicking real false pos/neg.")]
    [Range(0f, 0.3f)] public float irNoiseFlipChance = 0.04f;

    [Header("Sim-to-Real: Physics Randomization")]
    [Tooltip("Toggle Domain Randomization across training episodes to build policy robust to mass/friction variances.")]
    public bool randomizePhysicsPerEpisode = true;
    
    [Tooltip("Min/Max scaling factors applied to the agent's Rigidbody mass.")]
    public Vector2 massMultiplierRange = new Vector2(0.85f, 1.15f);
    
    [Tooltip("Min/Max scaling factors applied to motor force.")]
    public Vector2 motorForceMultiplierRange = new Vector2(0.85f, 1.15f);

    [Header("Reward Shaping")]
    [Tooltip("Multiplier for rewards granted by progressing closer to the target.")]
    public float speedRewardMultiplier = 1.0f;
    
    [Tooltip("Multiplier for penalizing the car as it gets closer to obstacles via sonar.")]
    public float obstacleProximityPenalty = 0.01f;
    
    [Tooltip("Flat penalty applied per active short-range IR sensor trigger.")]
    public float irPenalty = 0.02f;
    
    [Tooltip("Penalty multiplier for rapid, high-frequency oscillations in motor commands.")]
    public float actionJitterPenalty = 0.001f;
    
    [Tooltip("Small penalty applied every physics step to encourage the agent to find the fastest path.")]
    public float existentialPenalty = 0.001f;

    [Tooltip("Multiplier for rewards granted by turning to reduce heading error toward the target.")]
    public float headingRewardMultiplier = 1.0f;

    [Tooltip("Penalty applied per step while the mixed drive is negative (reversing). Exposed so you can shrink or zero it for the obstacle phase, where backing out of a furniture pocket is a legitimate maneuver. Was a hard-coded 0.005.")]
    public float reversingPenalty = 0.005f;

    // Component and Baseline State Caching
    private Rigidbody rb;
    private Vector3 startingPosition;
    private Quaternion startingRotation;
    private float baseMotorForce;
    private float baseMass;

    // Curriculum: lets the training YAML drive how hard the starting heading is,
    // and (new) how many furniture obstacles are active this episode.
    private Unity.MLAgents.EnvironmentParameters envParams;

    // Cached so per-episode outcome logging doesn't re-fetch it every time.
    private StatsRecorder statsRecorder;

    // Tracking state variables for rewards and motor smoothing
    private float previousDistanceToTarget;
    private float prevLeftAction, prevRightAction;
    private float smoothedLeft, smoothedRight;
    private float previousAngleError;

    // Time-limit bookkeeping for the bootstrap-on-timeout path.
    private int episodeStepCount;

    // Cached once per physics step so CollectObservations and OnActionReceived
    // never raycast the same sensor twice in the same step.
    // Cached readings
    private float ultrasonicFrontReading, ultrasonicRearReading;
    private float irFLReading, irFRReading, irRLReading, irRRReading, irLReading, irRReading;

    /// <summary>
    /// Called once when the agent is instantiated. Caches structural transforms and initial rigid body settings.
    /// </summary>
    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        startingPosition = transform.position;
        startingRotation = transform.rotation;
        baseMotorForce = motorForce;
        baseMass = rb.mass;

        // Grab the Academy's environment parameters once so OnEpisodeBegin can
        // read the current curriculum lesson (heading range + obstacle count).
        envParams = Academy.Instance.EnvironmentParameters;

        // Cache the stats recorder so we can log WHY each episode ended.
        statsRecorder = Academy.Instance.StatsRecorder;
    }

    /// <summary>
    /// Sets up environmental and physical conditions at the beginning of each training run.
    /// </summary>
    public override void OnEpisodeBegin()
    {
        // Reset physical dynamics
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        transform.position = startingPosition;

        // Reset motor control state to prevent action carry-over between episodes
        smoothedLeft = smoothedRight = 0f;
        prevLeftAction = prevRightAction = 0f;

        // Reset the time-limit counter for the timeout/bootstrap path.
        episodeStepCount = 0;

        // Execute Domain Randomization if enabled
        if (randomizePhysicsPerEpisode)
            RandomizePhysics();

        // How many furniture obstacles should be live this episode?
        // Curriculum-driven: start at 0 (confirms the warm-started empty policy
        // didn't regress), then ramp toward furnitureList.Count. Defaulting to the
        // full list reproduces the "all furniture always on" behavior when no
        // obstacle_count lesson is configured in the YAML.
        int obstacleCount = (furnitureList != null) ? furnitureList.Count : 0;
        if (envParams != null)
            obstacleCount = Mathf.RoundToInt(envParams.GetWithDefault("obstacle_count", obstacleCount));

        // Environment reset sequence (handles furniture and clearance buffers).
        // NOTE: this now runs BEFORE we set the heading, so the target already has
        // its position and we can orient the car relative to it (curriculum below).
        RandomizeEnvironment(obstacleCount);

        // Curriculum-driven starting heading.
        // "heading_range_deg" controls how far the spawn heading may deviate from
        // facing the target: start small (target basically ahead -> easy) and widen
        // toward 360 as the agent improves. Defaulting to 360 reproduces the old
        // fully-random spawn when no curriculum lesson is configured in the YAML.
        float headingRange = 360f;
        if (envParams != null)
            headingRange = envParams.GetWithDefault("heading_range_deg", 360f);
        headingRange = Mathf.Clamp(headingRange, 0f, 360f);

        Vector3 toTarget = targetLocation.position - transform.position;
        toTarget.y = 0f;
        float baseHeading = (toTarget.sqrMagnitude > 1e-6f)
            ? Quaternion.LookRotation(toTarget.normalized, Vector3.up).eulerAngles.y
            : Random.Range(0f, 360f);
        float headingNoise = Random.Range(-headingRange * 0.5f, headingRange * 0.5f);
        transform.rotation = Quaternion.Euler(0f, baseHeading + headingNoise, 0f);

        UpdateSensorReadings();

        previousDistanceToTarget = Vector3.Distance(transform.position, targetLocation.position);

        // Baseline heading error, tracked the same way distance is, so the
        // reward can be based on improvement rather than a flat snapshot.
        previousAngleError = Vector3.Angle(transform.forward,
            (targetLocation.position - transform.position).normalized) * Mathf.Deg2Rad;
    }

    /// <summary>
    /// Randomizes physical properties within specified bounds to bridge the Sim-to-Real gap.
    /// </summary>
    private void RandomizePhysics()
    {
        rb.mass = baseMass * Random.Range(massMultiplierRange.x, massMultiplierRange.y);
        motorForce = baseMotorForce * Random.Range(motorForceMultiplierRange.x, motorForceMultiplierRange.y);
    }

    /// <summary>
    /// Randomly positions the target within the boundaries of the designated spawn area.
    /// </summary>
    private void MoveTargetToRandomPosition()
    {
        if (spawnArea == null)
        {
            Debug.LogWarning("Spawn Area not assigned - target will not randomise.");
            return;
        }
        Bounds bounds = spawnArea.bounds;
        float randomX = Random.Range(bounds.min.x, bounds.max.x);
        float randomZ = Random.Range(bounds.min.z, bounds.max.z);
        targetLocation.position = new Vector3(randomX, targetLocation.position.y, randomZ);
    }

    // Runs once per physics step - single source of truth for all five sensors.
    private void FixedUpdate()
    {
        UpdateSensorReadings();
    }

    /// <summary>
    /// Centralized update loop for hardware sensor emulation raycasts.
    /// </summary>
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
    /// Emulates an HC-SR04 ultrasonic sensor using a 15-ray fan cluster to mimic a physical cone wave.
    /// Returns a normalized value from 0 (obstacle touching sensor) to 1 (clear path).
    /// </summary>
    private float ReadUltrasonic(Transform origin)
    {
        if (origin == null) return 1f;

        float minDistance = ultrasonicRange;
        int numRays = 15;
        float totalAngle = 30f; // -15 to +15 degrees

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
    /// Emulates a digital IR proximity sensor using a narrow 6-ray line bundle.
    /// Returns 1f if an obstacle is tripped by any ray within range, otherwise 0f.
    /// </summary>
    private float ReadIR(Transform origin)
    {
        if (origin == null) return 0f;

        float reading = 0f;
        int numRays = 6;
        float totalAngle = 6f; // -3 to +3 degrees

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

    // Box-Muller transform - Unity's Random has no built-in Gaussian sampler.
    private float SampleGaussian(float mean, float stdDev)
    {
        float u1 = 1f - Random.value;
        float u2 = 1f - Random.value;
        float randStdNormal = Mathf.Sqrt(-2f * Mathf.Log(u1)) * Mathf.Sin(2f * Mathf.PI * u2);
        return mean + stdDev * randStdNormal;
    }

    /// <summary>
    /// Packages all internal and environmental state data to pass directly into the Neural Network.
    /// Vector Observation Space Size in Behaviour Parameters must be set to 18.
    /// </summary>
    public override void CollectObservations(VectorSensor sensor)
    {
        // 1. Target direction, local space (3 obs)
        Vector3 localTargetPos = transform.InverseTransformPoint(targetLocation.position);
        sensor.AddObservation(localTargetPos.normalized);

        // 2. Target distance, normalised 0-1 (1 obs)
        float distance = Vector3.Distance(transform.position, targetLocation.position);
        sensor.AddObservation(Mathf.Clamp01(distance / maxTargetDistance));

        // 3. Linear velocity, local space, normalised (3 obs)
        Vector3 localVel = transform.InverseTransformDirection(rb.linearVelocity) / maxLinearSpeed;
        sensor.AddObservation(localVel);

        // 4. Angular velocity (yaw), normalised (1 obs)
        sensor.AddObservation(Mathf.Clamp(rb.angularVelocity.y / maxAngularSpeed, -1f, 1f));

        // 5. Ultrasonic sensors (2 obs)
        sensor.AddObservation(ultrasonicFrontReading);
        sensor.AddObservation(ultrasonicRearReading);

        // 6. IR sensors (6 obs)
        sensor.AddObservation(irFLReading);
        sensor.AddObservation(irFRReading);
        sensor.AddObservation(irRLReading);
        sensor.AddObservation(irRRReading);
        sensor.AddObservation(irLReading);
        sensor.AddObservation(irRReading);

        // 7. Motor latency state (2 obs).
        // The rate-limited smoothedLeft/Right are hidden state the policy commits to
        // but couldn't previously see, making the env partially observable. Feeding
        // them back in restores the Markov property so the network can anticipate lag.
        // Already in [-1, 1] (they track clamped action targets), so no scaling needed.
        sensor.AddObservation(smoothedLeft);
        sensor.AddObservation(smoothedRight);

        // Total = 18
    }

    /// <summary>
    /// Transforms the policy outputs into physical forces while calculating execution rewards and penalties.
    /// </summary>
    public override void OnActionReceived(ActionBuffers actions)
    {
        episodeStepCount++;

        // Action space is (throttle, steer) instead of raw (left, right).
        // Mixing to wheels here means an in-place turn is a single push on the steer
        // axis rather than a rare anti-correlated pair of two independent Gaussians,
        // so exploration discovers turning almost immediately. This is also exactly
        // what a real differential-drive controller does, so it stays sim-to-real safe.
        float throttle = actions.ContinuousActions[0];
        float steer = actions.ContinuousActions[1];
        float targetLeft = Mathf.Clamp(throttle + steer, -1f, 1f);
        float targetRight = Mathf.Clamp(throttle - steer, -1f, 1f);

        // Rate-limit toward the commanded value instead of applying it instantly
        smoothedLeft = Mathf.MoveTowards(smoothedLeft, targetLeft, motorResponseRate * Time.fixedDeltaTime);
        smoothedRight = Mathf.MoveTowards(smoothedRight, targetRight, motorResponseRate * Time.fixedDeltaTime);

        // Apply physical forces exactly where tires cross the ground
        if (leftWheelPowerPoint != null && rightWheelPowerPoint != null)
        {
            rb.AddForceAtPosition(transform.forward * smoothedLeft * motorForce, leftWheelPowerPoint.position, ForceMode.Acceleration);
            rb.AddForceAtPosition(transform.forward * smoothedRight * motorForce, rightWheelPowerPoint.position, ForceMode.Acceleration);
        }
        else
        {
            Debug.LogWarning("Wheel anchors are unassigned! Physics cannot execute correctly.");
        }

        // Calculate simplified mixed values to determine general direction for rewards
        float forwardMovement = (smoothedLeft + smoothedRight) / 2f;

        // --- Reward: closing distance to target ---
        float currentDistance = Vector3.Distance(transform.position, targetLocation.position);
        float distanceDelta = previousDistanceToTarget - currentDistance;
        AddReward(distanceDelta * speedRewardMultiplier);
        previousDistanceToTarget = currentDistance;

        // --- Reward: reducing heading error toward target ---
        Vector3 dirToTarget = (targetLocation.position - transform.position).normalized;
        float currentAngleError = Vector3.Angle(transform.forward, dirToTarget) * Mathf.Deg2Rad;
        float angleDelta = previousAngleError - currentAngleError;
        AddReward(angleDelta * headingRewardMultiplier);
        previousAngleError = currentAngleError;

        // --- Penalty: obstacle proximity, graded by the worst-case ultrasonic reading ---
        float worstUltrasonic = Mathf.Min(ultrasonicFrontReading, ultrasonicRearReading);
        if (worstUltrasonic < 0.5f)
            AddReward(-(0.5f - worstUltrasonic) * obstacleProximityPenalty);

        // --- Penalty: IR near-field trip, scaled by how many sensors fire out of 6 ---
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

        // --- Penalty: reversing (now inspector-tunable; shrink/zero for the obstacle phase) ---
        if (forwardMovement < 0f)
            AddReward(-reversingPenalty);

        // --- Existential penalty ---
        AddReward(-existentialPenalty);

        // --- Success: distance-based arrival ---
        if (currentDistance < successRadius)
        {
            EndEpisodeWithOutcome("success", 1.0f, false);
            return;
        }

        // --- Timeout: bootstrap instead of treating a time limit as a true terminal ---
        // A stuck/wedged car in clutter would otherwise never end its episode. We use
        // EpisodeInterrupted() (not EndEpisode) so PPO bootstraps the value from the
        // final state rather than assuming the return truly ended at 0.
        if (maxEpisodeSteps > 0 && episodeStepCount >= maxEpisodeSteps)
        {
            EndEpisodeWithOutcome("timeout", 0f, true);
            return;
        }
    }

    /// <summary>
    /// Single exit point for every episode end. Logs WHY it ended (success / collision /
    /// timeout) as 0-1 stats that read as rates in TensorBoard, then ends or interrupts.
    /// Terminal outcomes (success/collision) set a final reward and EndEpisode();
    /// a timeout is an interruption that bootstraps and gets no terminal reward.
    /// </summary>
    private void EndEpisodeWithOutcome(string outcome, float finalReward, bool interrupted)
    {
        if (!interrupted)
            SetReward(finalReward);

        if (statsRecorder != null)
        {
            statsRecorder.Add("Outcome/Success",   outcome == "success"   ? 1f : 0f);
            statsRecorder.Add("Outcome/Collision", outcome == "collision" ? 1f : 0f);
            statsRecorder.Add("Outcome/Timeout",   outcome == "timeout"   ? 1f : 0f);
        }

        if (interrupted)
            EpisodeInterrupted();
        else
            EndEpisode();
    }

    /// <summary>
    /// Provides manual fallback keyboard and gamepad configurations for testing configurations in the Editor using the Modern Input System.
    /// </summary>
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;

        float leftMotor = 0f;
        float rightMotor = 0f;

        // --- Controller: Independent Left/Right analog stick tank control ---
        if (Gamepad.current != null)
        {
            float leftStick = Gamepad.current.leftStick.y.ReadValue();
            float rightStick = Gamepad.current.rightStick.y.ReadValue();

            if (Mathf.Abs(leftStick) > 0.1f) leftMotor = leftStick;
            if (Mathf.Abs(rightStick) > 0.1f) rightMotor = rightStick;
        }

        // --- Keyboard fallback: A/Left Arrow = left tread, D/Right Arrow = right tread ---
        if (Keyboard.current != null)
        {
            bool reverse = Keyboard.current.leftShiftKey.isPressed || Keyboard.current.rightShiftKey.isPressed;
            
            float leftKey = 0f;
            if (Keyboard.current.aKey.isPressed || Keyboard.current.leftArrowKey.isPressed)
                leftKey = reverse ? -1f : 1f;

            float rightKey = 0f;
            if (Keyboard.current.dKey.isPressed || Keyboard.current.rightArrowKey.isPressed)
                rightKey = reverse ? -1f : 1f;

            // Use keyboard inputs if gamepad sticks are idle
            if (Mathf.Abs(leftMotor) <= 0.1f && leftKey != 0f) leftMotor = leftKey;
            if (Mathf.Abs(rightMotor) <= 0.1f && rightKey != 0f) rightMotor = rightKey;
        }

        // The controls above still feel like tank sticks, but the policy now expects
        // (throttle, steer). Convert so manual driving and any recorded demos speak the
        // same language as OnActionReceived: throttle = average, steer = half-difference.
        // (A pure spin left+1/right-1 -> throttle 0, steer 1, exactly what we want.)
        float throttle = (leftMotor + rightMotor) / 2f;
        float steer = (leftMotor - rightMotor) / 2f;

        continuousActionsOut[0] = Mathf.Clamp(throttle, -1f, 1f);
        continuousActionsOut[1] = Mathf.Clamp(steer, -1f, 1f);
    }

    /// <summary>
    /// Draws persistent vector lines directly inside Scene/Game views for motors and all 8 sensors.
    /// </summary>
    private void OnDrawGizmos()
    {
        // 1. Motor Vector Arrows
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

        // 2. Draw Ultrasonic Fans
        DrawUltrasonicGizmo(ultrasonicFrontOrigin, ultrasonicFrontReading);
        DrawUltrasonicGizmo(ultrasonicRearOrigin, ultrasonicRearReading);

        // 3. Draw IR Sensor Fans
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

    /// <summary>
    /// Helper method generating custom terminal arrows to track positive/negative tire vectors.
    /// </summary>
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
    /// Immediate failure handling when entering physical contact with rigid obstacle/wall layers,
    /// and immediate success handling when physically contacting the target.
    /// </summary>
    private void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("obstacle") || collision.gameObject.CompareTag("Wall"))
        {
            EndEpisodeWithOutcome("collision", -1.0f, false);
            return;
        }

        if (IsTargetCollider(collision.transform))
        {
            HandleTargetReached();
        }
    }

    /// <summary>
    /// Handles the case where the target's collider is configured as a trigger volume
    /// rather than a solid collider (common for "goal" objects so the car can pass through it).
    /// </summary>
    private void OnTriggerEnter(Collider other)
    {
        if (IsTargetCollider(other.transform))
        {
            HandleTargetReached();
        }
    }

    /// <summary>
    /// Identifies whether a given transform corresponds to the current target,
    /// either by direct reference or by a "Target" tag on the object/its parent.
    /// </summary>
    private bool IsTargetCollider(Transform t)
    {
        if (targetLocation == null) return false;
        return t == targetLocation || t.IsChildOf(targetLocation) || t.CompareTag("Target");
    }

    /// <summary>
    /// Shared success path for reaching the target, whether detected via distance
    /// threshold (OnActionReceived) or physical contact (OnCollisionEnter/OnTriggerEnter).
    /// </summary>
    private void HandleTargetReached()
    {
        EndEpisodeWithOutcome("success", 1.0f, false);
    }

    /// <summary>
    /// Measures the horizontal footprint radius of an object using its collider bounds.
    /// </summary>
    private float GetObjectRadius(Transform t)
    {
        Collider col = t.GetComponent<Collider>();
        if (col != null)
        {
            // Returns the largest horizontal half-size (extents) of the bounding box
            return Mathf.Max(col.bounds.extents.x, col.bounds.extents.z);
        }
        return 0.5f; // Fallback radius if no collider is attached
    }

    /// <summary>
    /// Generates a random coordinate vector inside the boundaries of the designated spawn area.
    /// </summary>
    private Vector3 GetRandomPointInSpawnArea()
    {
        if (spawnArea == null) return transform.position;
        Bounds bounds = spawnArea.bounds;
        float randomX = Random.Range(bounds.min.x, bounds.max.x);
        float randomZ = Random.Range(bounds.min.z, bounds.max.z);
        return new Vector3(randomX, transform.position.y, randomZ);
    }

    /// <summary>
    /// Randomizes target and furniture locations while enforcing safe clearance spaces using
    /// distance validation. Only <paramref name="activeCount"/> furniture pieces are placed and
    /// left active this episode (curriculum-driven); the rest are disabled so their colliders
    /// neither trip sensors nor cause phantom collisions.
    /// </summary>
    private void RandomizeEnvironment(int activeCount)
    {
        // Always place target first
        MoveTargetToRandomPosition();

        if (furnitureList == null) return;

        int total = furnitureList.Count;
        activeCount = Mathf.Clamp(activeCount, 0, total);

        // Shuffle the index order (Fisher-Yates) so which pieces are active varies
        // per episode rather than always using the first N in the list.
        List<int> order = new List<int>(total);
        for (int i = 0; i < total; i++) order.Add(i);
        for (int i = total - 1; i > 0; i--)
        {
            int j = Random.Range(0, i + 1);
            int tmp = order[i];
            order[i] = order[j];
            order[j] = tmp;
        }

        // Track pieces actually placed this episode so clearance checks only ever run
        // against live, current-position obstacles (not stale/inactive ones).
        List<Transform> placed = new List<Transform>(activeCount);

        for (int k = 0; k < total; k++)
        {
            Transform item = furnitureList[order[k]];
            if (item == null) continue;

            // Anything beyond the active budget is switched off for this episode.
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

            // Re-roll loop: drops a point and checks clearance criteria up to 100 times
            while (!validPosition && attempts < 100)
            {
                attempts++;
                potentialPosition = GetRandomPointInSpawnArea();

                // 1. Check distance to target cube (accounting for item size)
                if (Vector3.Distance(potentialPosition, targetLocation.position) < (targetBuffer + itemRadius))
                    continue;

                // 2. Check distance to car spawn point (accounting for item size)
                if (Vector3.Distance(potentialPosition, startingPosition) < (carBuffer + itemRadius))
                    continue;

                // 3. Check distance to the pieces already placed this episode
                bool tooCloseToOthers = false;
                foreach (var otherItem in placed)
                {
                    float otherRadius = GetObjectRadius(otherItem);
                    float combinedBuffer = itemRadius + otherRadius;

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
                // Couldn't find a clear spot in 100 tries. Disable rather than leave the
                // piece at a stale/overlapping placement from a previous episode.
                item.gameObject.SetActive(false);
            }
        }
    }
}