using UnityEngine;
using UnityEngine.Splines;
using Unity.Mathematics;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using System.Collections.Generic;

public class Cart : Agent
{
	// Constants
	const int OBSERVATION_POINT_COUNT = 8;
	const float OBSERVATION_POINT_DISTANCE = 0.33f;
	
	// Editor References
	[SerializeField]
	private SplineExtrude _splineExtrude;
	
	// Editor Tweakable Fields
	[SerializeField]
	float _collisionReward = -1f;
	[SerializeField]
	float _checkpointReward = 1f;
	[SerializeField]
	float _rewardLossPerSecond = -0.1f;
	[SerializeField]
	private float _checkpointSeparation;
	[SerializeField]
	private float _accelSmoothingSpeed;
	[SerializeField]
	private float _turnSmoothingSpeed;
	[SerializeField]
	private float _accelSpeed;
	[SerializeField]
	private float _turnSpeed;
	
	// Private Fields
	private List<float3> _checkpoints;
	private float _checkpointRadius;
	private int _currentCheckpoint;
	private float _splineLength;
	private float3[] _observationPoints = new float3[OBSERVATION_POINT_COUNT];
	private float _accelInput;
	private float _accelSmooth;
	private float _turnInput;
	private float _turnSmooth;
	private bool _trackIsInitialized = false;
	
	
	#region ML Agents
	
	public override void CollectObservations(VectorSensor sensor)
	{
		for (int i = 0; i < OBSERVATION_POINT_COUNT; i++) sensor.AddObservation(_observationPoints[i]);
	}
	
	public override void OnActionReceived(ActionBuffers actions)
	{
		int accel = actions.DiscreteActions[0] - 1;
		int turn = actions.DiscreteActions[1] - 1;
		SetInput(accel, turn);
	}

	public override void Heuristic(in ActionBuffers actionsOut)
	{
		ActionSegment<int> discreteActions = actionsOut.DiscreteActions;
		discreteActions[0] = Mathf.RoundToInt(Input.GetAxisRaw("Vertical")) + 1;
		discreteActions[1] = Mathf.RoundToInt(-Input.GetAxisRaw("Horizontal")) + 1;
	}

	public override void OnEpisodeBegin()
	{
		Start();
	}
	
	#endregion


	#region Unity Callbacks

	private void Start()
	{
		InitializeCheckpoints();
	}

	private void Update()
	{
		AddReward(_rewardLossPerSecond * Time.deltaTime);
	
		// Capture human input.
		// _accelInput += Input.GetAxisRaw("Vertical");
		// _turnInput += -Input.GetAxisRaw("Horizontal");
		
		_accelInput = Mathf.Clamp(_accelInput, -1f, 1f);
		_turnInput = Mathf.Clamp(_turnInput, -1f, 1f) * Mathf.Sqrt(Mathf.Abs(_accelSmooth)) * Mathf.Sign(_accelSmooth);
		
		// Perform smoothing on input to make motion less jarring (may eliminate if AI doesn't do well with learning to handle the additional complexity).
		_turnSmooth = Mathf.MoveTowards(_turnSmooth, _turnInput, _turnSmoothingSpeed * Time.deltaTime);
		_accelSmooth = Mathf.MoveTowards(_accelSmooth, _accelInput, _accelSmoothingSpeed * Time.deltaTime);
		
		// Apply acceleration.
		transform.eulerAngles += _turnSmooth * _turnSpeed * Time.deltaTime * Vector3.forward;
		float3 cartPos = transform.position;
		float3 forward = transform.up;
		float3 deltaPos = _accelSmooth * _accelSpeed * Time.deltaTime * forward;
		cartPos += deltaPos;
		
		// Here we handle 'collisions' with the edge of the track.
		// Cart position in local space of spline:
		Vector3 localCartPos = _splineExtrude.transform.InverseTransformPoint(cartPos);
		
		// Nearest point along spline to cart pos, in spline's local space:
		SplineUtility.GetNearestPoint(_splineExtrude.Spline, localCartPos, out float3 nearest, out float t, 4, 2);
		
		// Nearest point along spline to cart pos, in world space:
		nearest = _splineExtrude.transform.TransformPoint(nearest);
		float3 toSpline = nearest - cartPos;
		float distance = math.length(toSpline);
		float3 direction = math.normalize(toSpline + math.float3(0.0001f));
		float splineRadius = _splineExtrude.Radius * 0.5f;
		float depenetration = Mathf.Max(0f, distance - splineRadius);
		
		// Punish agent if colliding with track edge:
		if (!Mathf.Approximately(depenetration, 0f)) AddReward(_collisionReward * Time.deltaTime);
		
		float3 finalPos = cartPos + depenetration * direction;
		
		// Clamp transform onto track:
		transform.position = finalPos;
		
		// Update current checkpoint.
		while (math.length(_checkpoints[_currentCheckpoint] - (float3)transform.position) < _checkpointRadius)
		{
			AddReward(_checkpointReward);
			if (GetCumulativeReward() > _checkpoints.Count * 2 * _checkpointReward) EndEpisode();
			_currentCheckpoint = (_currentCheckpoint + 1 >= _checkpoints.Count) ? 0 : _currentCheckpoint + 1;
		}
		
		// Observe upcoming track, so as to learn from its layout.
		ObservationPoints();
		
		// Zero inputs.
		// _accelInput = 0f;
		// _turnInput = 0f;
	}

	private void OnDrawGizmos()
	{
		if (_checkpoints == null) return;
		for (int i = 0; i < _checkpoints.Count; i++)
		{
			Gizmos.color = i == _currentCheckpoint ? Color.green : Color.white;
			Gizmos.DrawWireSphere(_checkpoints[i], _checkpointRadius);
		}
		if (!_trackIsInitialized) return;
		for (int i = 0; i < OBSERVATION_POINT_COUNT; i++)
		{
			Gizmos.color = Color.yellow;
			Gizmos.DrawWireSphere(transform.TransformPoint(_observationPoints[i]), OBSERVATION_POINT_DISTANCE / 2);
		}
	}

	#endregion
	
	
	#region Misc. Methods
	
	/// <summary>
	/// Accumulate acceleration and turn input.
	/// </summary>
	/// <param name="acceleration"></param>
	/// <param name="turn"></param>
	public void SetInput(float acceleration, float turn)
	{
		_accelInput = acceleration;
		_turnInput = turn;
	}
	
	/// <summary>
	/// Construct checkpoint list, determine spline length and checkpoint radius, and initialize current checkpoint.
	/// </summary>
	public void InitializeCheckpoints()
	{
		// Initialize checkpoint system.
		float splineRadius = _splineExtrude.Radius;
		SplineContainer container = _splineExtrude.Container;
		_splineLength = container.CalculateLength();
		
		// The number of checkpoints that fit in the spline's length, given their separation:
		int checkpointCount = Mathf.FloorToInt(_splineLength / _checkpointSeparation);
		_checkpoints = new(checkpointCount);
		_currentCheckpoint = 0;
		float currentMinDistance = Mathf.Infinity;
		for (int i = 0; i < checkpointCount; i++)
		{
			float time = 1f / checkpointCount * (i + 1);
			_checkpoints.Add(container.EvaluatePosition(time));
			float distance = math.length(_checkpoints[i] - (float3)transform.position);
			if (distance >= currentMinDistance) continue;
			_currentCheckpoint = i;
			currentMinDistance = distance;
		}
		
		// Ensure that checkpoints will actually be collided with, by making their radius > spline's radius.
		_checkpointRadius = splineRadius * 0.67f;
		
		_trackIsInitialized = true;
	}
	
	/// <summary>
	/// Update list of points near cart, along the center of track spline, as observational data for ML-Agents.
	/// </summary>
	private void ObservationPoints()
	{
		// Cart position in local space of spline:
		Vector3 localCartPos = _splineExtrude.transform.InverseTransformPoint(transform.position);
		
		// Nearest point along spline to cart pos, in spline's local space:
		SplineUtility.GetNearestPoint(_splineExtrude.Spline, localCartPos, out float3 nearest, out float t, 4, 2);
		
		float timeIncrement = OBSERVATION_POINT_DISTANCE / _splineLength;
		float offsetBehind = timeIncrement * (OBSERVATION_POINT_COUNT / 2);
		for (int i = 0; i < OBSERVATION_POINT_COUNT; i++)
		{
			// The 'time' of each observation point along the spline:
			float time = (timeIncrement * (i + 1) + t - offsetBehind + 1f) % 1f;
			// The localspace position of each observation point:
			_observationPoints[i] = transform.InverseTransformPoint(_splineExtrude.transform.TransformPoint(SplineUtility.EvaluatePosition(_splineExtrude.Spline, time)));
		}
	}
	
	#endregion
}
