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

    [SerializeField] ConstraintConfig[] constraints;

    [Header ("Global Config")]
    [SerializeField] float3 extraForce = new float3 (0, -9.8f, 0);
    [SerializeField, Range (0f, 1f)] float beta = 1f;
    [SerializeField, Range (0f, 1f)] float damping = 0.99f;

    ConstraintData[] constraintDatas;

    void Start () {
        constraintDatas = new ConstraintData[constraints.Length];

        var mass = 1f;
        var size = 1f;
        for (int i = 0; i < constraints.Length; i++) {
            constraintDatas[i].mass = mass;
            constraintDatas[i].massInv = 1f / mass;
            constraintDatas[i].inertia = (mass * size * size / 6f) * float3x3.identity; // Inertia Tensor for a Cube
            constraintDatas[i].inertiaInv = math.inverse (constraintDatas[i].inertia);
        }
    }

    void FixedUpdate () {
        UpdatePerConstraint (Time.fixedDeltaTime);
    }

    void UpdatePerConstraint (float dt) {
        for (int i = 0; i < constraints.Length; i++) {
            var deformedObject = constraints[i].deformedObject;
            float3 targetPos = constraints[i].targetPoint.position;
            float3 position = deformedObject.position;
            float3 r = deformedObject.rotation * constraints[i].offset;

            float3 linarVelocity = constraintDatas[i].linarVelocity + extraForce * dt;
            float3 angularVelocity = constraintDatas[i].angularVelocity;
            float3 positionConstraint = (position + r) - targetPos;
            float3 velocityConstraint = linarVelocity + math.cross (angularVelocity, r);

            float3x3 localToWorld = new float3x3
            (
                deformedObject.localToWorldMatrix.m00, deformedObject.localToWorldMatrix.m01, deformedObject.localToWorldMatrix.m02,
                deformedObject.localToWorldMatrix.m10, deformedObject.localToWorldMatrix.m11, deformedObject.localToWorldMatrix.m12,
                deformedObject.localToWorldMatrix.m20, deformedObject.localToWorldMatrix.m21, deformedObject.localToWorldMatrix.m22
            );
            float3x3 inertiaInvWs = math.mul (localToWorld, math.mul (constraintDatas[i].inertiaInv, math.transpose (localToWorld)));

            float3x3 sr = MathUtils.SkewSymmetric (-r); // for cross dot
            float3x3 sr_t = math.transpose (sr);
            float3x3 k = (constraintDatas[i].massInv * float3x3.identity) + math.mul (sr, math.mul (inertiaInvWs, sr_t));
            float3x3 effectiveMass = math.inverse (k);
            float3 b = (beta / dt) * positionConstraint; // bias velocity vector
            float3 lambda = math.mul (effectiveMass, -(velocityConstraint + b)); // = -1 * effectiveMass * (JV + b)

            linarVelocity += constraintDatas[i].massInv * lambda;
            angularVelocity += math.mul (inertiaInvWs, math.mul (sr_t, lambda));
            linarVelocity *= damping;
            angularVelocity *= damping;

            // integration and update
            deformedObject.position += (Vector3)linarVelocity * dt;
            deformedObject.rotation = Quaternion.Euler (angularVelocity * dt * Mathf.Rad2Deg) * deformedObject.rotation;
            constraintDatas[i].linarVelocity = linarVelocity;
            constraintDatas[i].angularVelocity = angularVelocity;
        }
    }
}
