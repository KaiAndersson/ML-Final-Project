using UnityEngine;
using UnityEngine.Splines;
using Unity.Mathematics;

public class Cart : MonoBehaviour
{
	private float _humanMoveInput;
	private float _accelSmooth;
	private float _humanTurnInput;
	private float _turnSmooth;
	[SerializeField]
	private float _accelSmoothingSpeed;
	[SerializeField]
	private float _turnSmoothingSpeed;
	[SerializeField]
	private float _speed;
	[SerializeField]
	private float _turnSpeed;
	[SerializeField]
	private SplineExtrude _splineExtrude;

	private void Start()
	{
		
	}

	/// <summary>
	/// Physics tick updates cart transform.
	/// </summary>
	private void Update()
	{
		// Capture human input.
		_humanMoveInput = Mathf.Clamp(Input.GetAxisRaw("Vertical"), -1f, 1f);
		_humanTurnInput = Mathf.Clamp(-Input.GetAxisRaw("Horizontal"), -1f, 1f) * Mathf.Sqrt(Mathf.Abs(_accelSmooth)) * Mathf.Sign(_accelSmooth);
		// ^^^ (Must scale by forward motion to prevent turning in place.)
		
		// Perform smoothing on input to make motion less jarring (may eliminate if AI doesn't do well with learning to handle the additional complexity).
		_turnSmooth = Mathf.MoveTowards(_turnSmooth, _humanTurnInput, _turnSmoothingSpeed * Time.deltaTime);
		_accelSmooth = Mathf.MoveTowards(_accelSmooth, _humanMoveInput, _accelSmoothingSpeed * Time.deltaTime);
		
		// Apply acceleration.
		transform.eulerAngles += _turnSmooth * _turnSpeed * Vector3.forward * Time.deltaTime;
		float3 cartPos = transform.position;
		float3 forward = transform.up;
		cartPos += _accelSmooth * _speed * forward * Time.deltaTime;
		
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
		float splineRadius = _splineExtrude.Radius;
		float depenetration = Mathf.Max(0f, distance - splineRadius);
		float3 finalPos = cartPos + depenetration * direction;
		// Clamp transform onto track:
		transform.position = finalPos;
	}
}
