using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Linq;
using System.Drawing.Text;

namespace DroneAgent
{
    public class MyAgent : Agent
    {

#region Variables & Initialization
        // Declare variables and components for the drone and environment
        [SerializeField] private Transform GateTransform; // Reference to the Gate transform
        [SerializeField] private Transform GateGoal; // Reference to the target gate for the drone
        [SerializeField] private Transform Agent; // Reference to the drone itself
        [SerializeField] private Transform propeller1, propeller2, propeller3, propeller4;
        [SerializeField] private float propRotSpeed = 300f; // Propeller rotation speed

        // Engine and control properties
        [Header("Engine Properties")]
        [SerializeField] private float maxpower = 10f; // Maximum engine power

        [Header("Control Properties")]
        [SerializeField] private float minMaxPitch = 20f; // Maximum pitch angle
        [SerializeField] private float minMaxRoll = 20f; // Maximum roll angle
        [SerializeField] private float YawPower = 5f; // Yaw control power
        [SerializeField] private float weighTnbls = 1f; // Drone weight in lbs

        [Header("Speed Settings")]
        [SerializeField] private float horizontalSpeedFactor = 2f; // Multiplier for horizontal speed
        [SerializeField] private float lerpSpeed = 2f; // Interpolation speed for smooth control

        const float lbsTokg = 0.78f; // Conversion factor for lbs to kg
        private Vector3 lastPosition; // Store the last position of the drone

        // List of engines in the drone
        private List<IEngine> engines = new List<IEngine>(); 

        // Control variables for pitch, roll, and yaw
        private float finalPitch;
        private float finalRoll;
        private float yaw;
        private float finalYaw;

        private Rigidbody rb; // Rigidbody component of the drone
        protected float startDrag = 1f; // Initial drag
        protected float startAngularDrag; // Initial angular drag
#endregion

#region Environment Reset
        [SerializeField] private Material targetMaterial; // Material to change color
        [SerializeField] private List<Color> colors; // List of colors for random selection

        private float previousDistanceToTarget = 0f;
        private float currentDistanceToTarget = 0f;
        private float previousYawToTarget = 0f;
        private float currentYawToTarget = 0f;

        public override void OnEpisodeBegin()
        {
            // Reset the quadcopter's position and rotation at the beginning of each episode
            Agent.position = new Vector3(11.5f, 1f, -22f);
            Agent.rotation = Quaternion.Euler(0f, 0f, 0f); // Reset rotation to (0, 0, 0)
            yaw = 0f ;

            // Set Gate's fixed position with random offsets
            Vector3 fixedGatePosition = new Vector3(12f, 0.35f, -20f);
            float offsetX = Random.Range(-0.5f, 0.5f);
            float offsetY = Random.Range(-0.5f, 0.5f);
            float offsetZ = Random.Range(-0.5f, 0.5f);
            GateTransform.position = fixedGatePosition + new Vector3(offsetX, offsetY, offsetZ);
            Agent.position  = Agent.position  + new Vector3(offsetX, offsetY, offsetZ);

            // Set random rotation for the Gate
            float randomRotationY = Random.Range(-50f, 50f);
            GateTransform.eulerAngles = new Vector3(0f, 180f + randomRotationY, 0f); 
        
            // Reset reward
            SetReward(0f);

            // Reset all the target-related variables
            previousDistanceToTarget = 0f;
            currentDistanceToTarget = 0f;
            previousYawToTarget = 0f;
            currentYawToTarget = 0f;
        }
#endregion

#region Engine Handling
        void Awake()
        {
            // Initialize Rigidbody and engine list
            rb = GetComponent<Rigidbody>();
            engines = GetComponentsInChildren<IEngine>().ToList();

            // Set the mass of the drone based on its weight
            if (rb)
            {
                rb.mass = weighTnbls * lbsTokg;
                startDrag = rb.linearDamping; // Store initial drag
                startAngularDrag = rb.angularDamping; // Store initial angular drag
            }
        }

        public void UpdateEngine()
        {
            // Calculate vertical thrust compensation based on tilt angle
            Vector3 upVec = transform.up;
            upVec.x = 0f;
            upVec.z = 0f;

            float tiltAngle = Vector3.Angle(Vector3.up, transform.up);
            float compensation = Mathf.Cos(tiltAngle * Mathf.Deg2Rad); // Calculate compensation

            // Calculate and apply engine force
            Vector3 engineForce = transform.up * (rb.mass * Physics.gravity.magnitude) / compensation / 4f;
            rb.AddForce(engineForce, ForceMode.Force); // Apply vertical force

            // Rotate propellers
            HandlePropellers();
        }

        void HandlePropellers()
        {
            // Rotate each propeller if it's assigned
            if (propeller1)
            {
                propeller1.Rotate(Vector3.forward, propRotSpeed);
                propeller2.Rotate(Vector3.forward, propRotSpeed);
                propeller3.Rotate(Vector3.forward, propRotSpeed);
                propeller4.Rotate(Vector3.forward, propRotSpeed);
            }
        }

        protected virtual void HandleEngines()
        {
            // Update each engine
            foreach (IEngine engine in engines)
            {
                UpdateEngine();
            }
        }
#endregion

#region Action Processing
        public override void OnActionReceived(ActionBuffers actions)
        {
            // Process engine control actions (pitch, roll, yaw)
            HandleEngines();

            float roll = -actions.ContinuousActions[0] * minMaxRoll; // Roll input
            float pitch = actions.ContinuousActions[1] * minMaxPitch; // Pitch input
            yaw += actions.ContinuousActions[2] * YawPower; // Yaw input

            // Smoothly interpolate pitch and roll
            finalPitch = Mathf.Lerp(finalPitch, pitch, Time.deltaTime * lerpSpeed);
            finalRoll = Mathf.Lerp(finalRoll, roll, Time.deltaTime * lerpSpeed);
            finalYaw = Mathf.Lerp(finalYaw, yaw, Time.deltaTime * lerpSpeed);
        
            // Apply throttle input for vertical force
            float throttle = actions.ContinuousActions[3];
            Vector3 engineForce = transform.up * (throttle * maxpower) / 4f;
            // Apply forces to the Rigidbody
            rb.AddForce(engineForce, ForceMode.Force);

            // Calculate the forward and right directions based on the current yaw
            float yawInRadians = finalYaw * Mathf.Deg2Rad; // Convert yaw to radians
            float forwardMovementX = Mathf.Sin(yawInRadians) * finalPitch * horizontalSpeedFactor; // Forward direction
            float forwardMovementZ = Mathf.Cos(yawInRadians) * finalPitch * horizontalSpeedFactor; // Forward direction
            float rightMovementX = -Mathf.Cos(yawInRadians) * finalRoll * horizontalSpeedFactor; // Right direction
            float rightMovementZ = Mathf.Sin(yawInRadians) * finalRoll * horizontalSpeedFactor; // Right direction

            // Combine forward and sideways movement
            Vector3 horizontalMovement = new Vector3(forwardMovementX + rightMovementX, 0, forwardMovementZ + rightMovementZ);

            // Apply forces to the Rigidbody using the local horizontal movement
            rb.AddForce(horizontalMovement, ForceMode.Force);

            // Apply rotation to the drone
            Quaternion rot = Quaternion.Euler(finalPitch, finalYaw, finalRoll);
            rb.MoveRotation(rot);
        }

        public override void Heuristic(in ActionBuffers actionsOut)
        {
            // Manual control inputs for heuristic mode
            ActionSegment<float> continuousActions = actionsOut.ContinuousActions;
            continuousActions[0] = Input.GetAxisRaw("Horizontal"); // Roll control
            continuousActions[1] = Input.GetAxisRaw("Vertical"); // Pitch control
            continuousActions[2] = Input.GetKey(KeyCode.Q) ? -1f : (Input.GetKey(KeyCode.E) ? 1f : 0f); // Yaw control
            continuousActions[3] = Input.GetKey(KeyCode.Space) ? 1f : (Input.GetKey(KeyCode.LeftShift) ? -1f : 0f); // Throttle control
        }
#endregion

#region Reward Function
        // FixedUpdate is called every physics frame. This is where reward updates happen.
        private void FixedUpdate()
        {
            // Debug.Log($"Cumulative Reward: {GetCumulativeReward()}");
            
            // Reward based on how well the drone maintains a good vertical position relative to the target.
            RewardUpdatePosition();
            
            // Reward based on how well the drone aligns its angles (yaw and pitch) towards the target.
            RewardUpdateAngles();

            // Check how close the drone is to walls or Boundarys and penalize accordingly.
            // CheckProximityToWalls();

            // Check for action sign changes
            // RewardActionSignChange();
        }

        // The allowed distance for proximity checks. If the drone is within this distance from Boundarys,
        // it will receive a negative reward. Separate values are used for vertical and horizontal checks.
        private float allowedDistanceVertical = 0.5f; // Allowed distance for proximity checks in the vertical direction (up/down)
        private float allowedDistanceHorizontal = 2f; // Allowed distance for proximity checks in the horizontal directions (forward/backward, left/right)

        // This method casts rays in six directions (forward, backward, up, down, right, left) to check for Boundarys.
        // If the drone gets too close to an Boundary, it receives a negative reward to encourage it to maintain safe distances.
        private void CheckProximityToWalls()
        {
            // Define the six directions for raycasting to check for Boundarys around the drone.
            Vector3[] directions = { transform.forward, -transform.forward, transform.up, -transform.up, transform.right, -transform.right };

            // Iterate through each direction to cast a ray and check for Boundarys.
            foreach (var direction in directions)
            {
                RaycastHit hit; // Holds information about what the ray hit
                float allowedDistanceForDirection;

                // If the direction is up or down, use the allowed vertical distance.
                if (direction == transform.up || direction == -transform.up)
                {
                    allowedDistanceForDirection = allowedDistanceVertical;
                }
                else
                {
                    // For other directions, use the allowed horizontal distance.
                    allowedDistanceForDirection = allowedDistanceHorizontal;
                }

                // Cast a ray from the agent's position in the current direction, with a maximum distance limit.
                if (Physics.Raycast(Agent.position, direction, out hit, allowedDistanceForDirection))
                {
                    // Check if the object hit by the ray has an Boundary component (is an Boundary).
                    if (hit.collider.GetComponent<Boundary>() != null)
                    {
                        // If the drone is closer to the Boundary than the allowed distance, give a negative reward.
                        float distanceToWall = hit.distance;
                        if (distanceToWall < allowedDistanceForDirection)
                        {
                            SetReward(-1f); // Penalty for being too close to the wall or Boundary.
                            // Debug.Log("Penalty for getting closer to wall");
                        }
                    }
                }
            }
        }

        // This function rewards the drone based on its vertical distance to the target (goal).
        // The closer the drone is to the goal vertically, the higher the reward. If it moves away, the reward decreases.
        // Declare this variable at the class level to retain its value across calls.
        private void RewardUpdatePosition()
        {
            // Get the position of the target (goal) to calculate the distance to it.
            Vector3 targetPosition = GateGoal.position;
            float currentDistanceToTarget = Vector3.Distance(Agent.position, targetPosition);

            // Calculate the reward based on the absolute distance to the target. Closer distance gives higher reward.

            // Threshold for significant change in position
            float PositionChangeThreshold = 0.01f; 
            float PositionChange = Mathf.Abs(currentDistanceToTarget - previousDistanceToTarget);
            float reward = Mathf.Exp(Mathf.Abs(PositionChange)); // Decay based on distance

            // If the change is greater than the threshold, add the reward.
            if (PositionChange > PositionChangeThreshold)
            {
                if (currentDistanceToTarget > previousDistanceToTarget)
                {
                    AddReward(-reward); // Reward for significant changes
                    // Debug.Log($"Position Changed! Reward Added: {reward}");
                }
                else
                {
                    AddReward(reward); // Reward for significant changes
                    // Debug.Log($"Position Changed! Reward Added: {reward}");
                }
            }
            // Update the previous distance to target for the next comparison.
            previousDistanceToTarget = currentDistanceToTarget;

            // Optional: You can log the current distance and reward for debugging.
            // Debug.Log("Current Reward for Distance: " + reward + ", Current Distance: " + currentDistanceToTarget);
        }

        // This method rewards the drone based on how well it aligns its angles (yaw and pitch) with the target (goal).
        // The more aligned the drone is with the target, the higher the reward.
        // Declare these variables at the class level to retain their values across calls.
        private float previousPitchToTarget = 0f;

        private void RewardUpdateAngles()
        {
            // Calculate the direction vector from the drone to the target.
            Vector3 toTarget = (GateGoal.position - Agent.position).normalized;

            // Calculate the yaw (horizontal angle) between the drone's forward direction and the direction to the target.
            float currentYawToTarget = Vector3.SignedAngle(new Vector3(Agent.forward.x, 0, Agent.forward.z), new Vector3(toTarget.x, 0, toTarget.z), Vector3.up);

            // Calculate the pitch (vertical angle) between the drone's forward direction and the direction to the target.
            float currentPitchToTarget = Vector3.SignedAngle(new Vector3(0, Agent.forward.y, Agent.forward.z), new Vector3(0, toTarget.y, toTarget.z), Vector3.right);

            float currentYaw = Mathf.Abs(currentYawToTarget);

            // Calculate change in yaw and pitch
            float yawChange = Mathf.Abs(currentYaw - previousYawToTarget);
            float pitchChange = Mathf.Abs(currentPitchToTarget - previousPitchToTarget);

            // Set thresholds for significant changes
            float angleChangeThreshold = 0.1f; 

            // If yaw change is greater than the threshold, apply reward or penalty
            if (yawChange > angleChangeThreshold)
            {
                if (Mathf.Abs(currentYaw) > Mathf.Abs(previousYawToTarget))
                {
                    AddReward(-yawChange); // Penalty for getting worse in yaw alignment
                }
                else
                {
                    AddReward(yawChange); // Reward for improving yaw alignment
                }
            }

            // // If pitch change is greater than the threshold, apply reward or penalty
            // if (pitchChange > angleChangeThreshold)
            // {
            //     if (Mathf.Abs(currentPitchToTarget) > Mathf.Abs(previousPitchToTarget))
            //     {
            //         AddReward(-pitchReward); // Penalty for getting worse in pitch alignment
            //     }
            //     else
            //     {
            //         AddReward(pitchReward); // Reward for improving pitch alignment
            //     }
            // }

            // Update previous yaw and pitch to target for the next comparison.
            previousYawToTarget = currentYaw;
            previousPitchToTarget = currentPitchToTarget;

            // Optionally, log the changes and rewards for debugging.
            // Debug.Log($"Yaw Changed: {yawChange}, Yaw Reward: {yawReward}");
            // Debug.Log($"Pitch Changed: {pitchChange}, Pitch Reward: {pitchReward}");
        }

        // Variables to store the previous values of actions
        private float previousFinalRoll = 0f;
        private float previousFinalPitch = 0f;
        private float previousFinalYaw = 0f;
        private float previousThrottle = 0f;

        private void RewardActionSignChange()
        {
            // Check for sign change and apply penalty for Final Roll
            if (Mathf.Sign(finalRoll) != Mathf.Sign(previousFinalRoll))
            {
                float penalty = Mathf.Abs(finalRoll - previousFinalRoll);
                AddReward(-penalty/5);
            }

            // Check for sign change and apply penalty for Final Pitch
            if (Mathf.Sign(finalPitch) != Mathf.Sign(previousFinalPitch))
            {
                float penalty = Mathf.Abs(finalPitch - previousFinalPitch);
                AddReward(-penalty/5);
            }

            // Check for sign change and apply penalty for Final Yaw
            if (Mathf.Sign(finalYaw) != Mathf.Sign(previousFinalYaw))
            {
                float penalty = Mathf.Abs(finalYaw - previousFinalYaw);
                AddReward(-penalty/5);
            }

            // // Check for sign change and apply penalty for Throttle
            // if (Mathf.Sign(throttle) != Mathf.Sign(previousThrottle))
            // {
            //     float penalty = Mathf.Abs(throttle - previousThrottle);
            //     AddReward(-penalty);
            // }

            // Update previous values to current values for the next frame
            previousFinalRoll = finalRoll;
            previousFinalPitch = finalPitch;
            previousFinalYaw = finalYaw;
            // previousThrottle = throttle;
        }

        // This method handles collision detection to apply rewards or penalties when the drone collides with different objects.
        private void OnCollisionEnter(Collision collision)
        {
            // If the drone successfully passes through a Gate, reward it and end the episode.
            if (collision.gameObject.TryGetComponent<GatePassing>(out GatePassing Gatepassing))
            {
                // SetReward(50f); // Positive reward for passing through the Gate.
                AddReward(50f);

                // floorMeshRenderer.material = GateMaterial; // Optional: Change floor color to indicate success.
                Debug.Log("GatePassing"); 
                EndEpisode(); // End the episode as the task is complete.
            }

            // If the drone hits an Boundary, penalize it and end the episode.
            if (collision.gameObject.TryGetComponent<Boundary>(out Boundary Boundary))
            {
                // SetReward(-1f); // Negative reward for hitting an Boundary.
                AddReward(-50f);
                // floorMeshRenderer.material = loseMaterial; // Optional: Change floor color to indicate failure.
                Debug.Log("Boundary Collision");
                EndEpisode(); // End the episode as the task failed.
            }

            // If the drone collides with a Gate itself, penalize it and end the episode.
            if (collision.gameObject.TryGetComponent<Gate>(out Gate Gate))
            {
                // SetReward(-10f); // Negative reward for colliding with the Gate.
                AddReward(-25f);

                // floorMeshRenderer.material = loseMaterial; // Optional: Change floor color to indicate failure.
                Debug.Log("Gate Collision"); 
                EndEpisode(); // End the episode as the task failed.
            }
            // If the drone collides with a Gate itself, penalize it and end the episode.
            if (collision.gameObject.TryGetComponent<Gatebase>(out Gatebase Gatebase))
            {
                // SetReward(-10f); // Negative reward for colliding with the Gate.
                AddReward(-50f);

                // floorMeshRenderer.material = loseMaterial; // Optional: Change floor color to indicate failure.
                Debug.Log("Gatebase Collision"); 
                // EndEpisode(); // End the episode as the task failed.
            }
        }
#endregion
    }
}


