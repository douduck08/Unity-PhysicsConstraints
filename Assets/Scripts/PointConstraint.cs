using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

public class PointConstraint : MonoBehaviour {

    [System.Serializable]
    public struct ConstraintConfig {
        public Transform targetPoint;
        public Transform deformedObject;
        public float3 offset;
    }

    public struct ConstraintData {
        public float mass;
        public float massInv;
        public float3x3 inertia;
        public float3x3 inertiaInv;
        public float3 linarVelocity;
        public float3 angularVelocity;
    }

    [SerializeField] ConstraintConfig constraint;
    ConstraintData constraintData;

    [Header ("Global Config")]
    [SerializeField] float3 extraForce = new float3 (0, -9.8f, 0);
    [SerializeField, Range (0f, 1f)] float beta = 1f;
    [SerializeField, Range (0f, 1f)] float damping = 0.999f;

    void Start () {
        var mass = 1f;
        var size = 1f;
        constraintData.mass = mass;
        constraintData.massInv = 1f / mass;
        constraintData.inertia = (mass * size * size / 6f) * float3x3.identity; // Inertia Tensor for a Cube
        constraintData.inertiaInv = math.inverse (constraintData.inertia);
    }

    void FixedUpdate () {
        var dt = Time.fixedDeltaTime;
        UpdateConstraint (dt);

        var deformedObject = constraint.deformedObject;
        deformedObject.position += (Vector3)constraintData.linarVelocity * dt;
        deformedObject.rotation = Quaternion.Euler (constraintData.angularVelocity * dt * Mathf.Rad2Deg) * deformedObject.rotation;
    }

    void UpdateConstraint (float dt) {
        var deformedObject = constraint.deformedObject;
        float3 targetPos = constraint.targetPoint.position;
        float3 position = deformedObject.position;
        float3 r = deformedObject.rotation * constraint.offset;

        // apply extra forces and caculate constraints
        float3 linarVelocity = constraintData.linarVelocity + extraForce * dt;
        float3 angularVelocity = constraintData.angularVelocity;
        float3 positionConstraint = (position + r) - targetPos;
        float3 velocityConstraint = linarVelocity + math.cross (angularVelocity, r);

        // transform inertia tensor
        float3x3 localToWorld = new float3x3
        (
            deformedObject.localToWorldMatrix.m00, deformedObject.localToWorldMatrix.m01, deformedObject.localToWorldMatrix.m02,
            deformedObject.localToWorldMatrix.m10, deformedObject.localToWorldMatrix.m11, deformedObject.localToWorldMatrix.m12,
            deformedObject.localToWorldMatrix.m20, deformedObject.localToWorldMatrix.m21, deformedObject.localToWorldMatrix.m22
        );
        float3x3 inertiaInvWS = math.mul (localToWorld, math.mul (constraintData.inertiaInv, math.transpose (localToWorld)));

        // solve constraints
        float3x3 s = MathUtils.SkewSymmetric (-r); // matrix for cross dot
        float3x3 st = math.transpose (s);
        float3x3 k = (constraintData.massInv * float3x3.identity) + math.mul (s, math.mul (inertiaInvWS, st));
        float3 bias = (beta / dt) * positionConstraint; // bias velocity vector
        float3 lambda = math.mul (math.inverse (k), -(velocityConstraint + bias)); // = effectiveMass * -(JV + b)

        // add constraint forces
        linarVelocity += constraintData.massInv * lambda;
        angularVelocity += math.mul (inertiaInvWS, math.mul (st, lambda));
        constraintData.linarVelocity = linarVelocity * damping;
        constraintData.angularVelocity = angularVelocity * damping;
    }
}
