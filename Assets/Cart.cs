using UnityEngine;
using UnityEngine.Splines;
using Unity.Mathematics;

public class Cart : MonoBehaviour
{
	private float _humanAccel;
	private float _accelSmooth;
	private float _humanTurn;
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
	/// Per physics frame tick.
	/// </summary>
	private void Update()
	{
		// Capture human input.
		_humanAccel = Mathf.Clamp(Input.GetAxisRaw("Vertical"), -1f, 1f);
		_humanTurn = Mathf.Clamp(-Input.GetAxisRaw("Horizontal"), -1f, 1f) * Mathf.Sqrt(Mathf.Abs(_accelSmooth)) * Mathf.Sign(_accelSmooth);
		
		// Smooth input.
		_turnSmooth = Mathf.MoveTowards(_turnSmooth, _humanTurn, _turnSmoothingSpeed * Time.deltaTime);
		_accelSmooth = Mathf.MoveTowards(_accelSmooth, _humanAccel, _accelSmoothingSpeed * Time.deltaTime);
		
		// Apply acceleration.
		transform.eulerAngles += _turnSmooth * _turnSpeed * Vector3.forward;
		float3 cartPos = transform.position;
		float3 forward = transform.up;
		cartPos += _accelSmooth * _speed * forward * Time.deltaTime;
		
		// Track edge collisions.
		Vector3 localCartPos = _splineExtrude.transform.InverseTransformPoint(cartPos);
		SplineUtility.GetNearestPoint(_splineExtrude.Spline, localCartPos, out float3 nearest, out float t, 4, 2);
		nearest = _splineExtrude.transform.TransformPoint(nearest);
		print("Nearest point: " + nearest);
		float3 toSpline = nearest - cartPos;
		print("To Spline: " + toSpline);
		float distance = math.length(toSpline);
		print("Distance to nearest point: " + distance);
		float3 direction = math.normalize(toSpline + math.float3(0.0001f));
		float splineRadius = _splineExtrude.Radius;
		float depenetration = Mathf.Max(0f, distance - splineRadius);
		float3 finalPos = cartPos + depenetration * direction;
		
		transform.position = finalPos;
	}
}
