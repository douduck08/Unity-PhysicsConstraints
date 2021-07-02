using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

public class PointConstraint : MonoBehaviour {
    [SerializeField] Transform targetPoint;
    [SerializeField] Transform deformedObject;
    [SerializeField] float3 offset = new float3 (0.5f, 0.5f, 0.5f);
    [SerializeField] float3 extraForce = new float3 (0, -9.8f, 0);

    [SerializeField, Range (0f, 1f)] float beta = 1f;
    [SerializeField, Range (0f, 1f)] float damping = 0.99f;

    float mass;
    float massInv;
    float3x3 inertia;
    float3x3 inertiaInv;
    float3 linarVelocity;
    float3 angularVelocity;

    void Start () {
        mass = 1f;
        massInv = 1f / mass;

        var size = 1f;
        inertia = (mass * size * size / 6f) * float3x3.identity; // Inertia Tensor for a Cube
        inertiaInv = math.inverse (inertia);
    }

    void FixedUpdate () {
        UpdateDynamics (Time.fixedDeltaTime);
    }

    void UpdateDynamics (float dt) {
        float3 targetPos = targetPoint.position;
        float3 position = deformedObject.position;
        float3 r = deformedObject.rotation * offset;

        linarVelocity += extraForce * dt;
        float3 positionConstraint = (position + r) - targetPos;
        float3 velocityConstraint = linarVelocity + math.cross (angularVelocity, r);

        float3x3 localToWorld = new float3x3
        (
            deformedObject.localToWorldMatrix.m00, deformedObject.localToWorldMatrix.m01, deformedObject.localToWorldMatrix.m02,
            deformedObject.localToWorldMatrix.m10, deformedObject.localToWorldMatrix.m11, deformedObject.localToWorldMatrix.m12,
            deformedObject.localToWorldMatrix.m20, deformedObject.localToWorldMatrix.m21, deformedObject.localToWorldMatrix.m22
        );
        float3x3 inertiaInvWs = math.mul (localToWorld, math.mul (inertiaInv, math.transpose (localToWorld)));

        float3x3 sr = MathUtils.SkewSymmetric (-r); // for cross dot
        float3x3 sr_t = math.transpose (sr);
        float3x3 k = (massInv * float3x3.identity) + math.mul (sr, math.mul (inertiaInvWs, sr_t));
        float3x3 effectiveMass = math.inverse (k);
        float3 b = (beta / dt) * positionConstraint; // bias velocity vector
        float3 lambda = math.mul (effectiveMass, -(velocityConstraint + b)); // = -1 * effectiveMass * (JV + b)

        linarVelocity += massInv * lambda;
        angularVelocity += math.mul (inertiaInvWs, math.mul (sr_t, lambda));
        linarVelocity *= damping;
        angularVelocity *= damping;

        // integration
        deformedObject.position += (Vector3)linarVelocity * dt;
        deformedObject.rotation = Quaternion.Euler (angularVelocity * dt * Mathf.Rad2Deg) * deformedObject.rotation;
    }
}
