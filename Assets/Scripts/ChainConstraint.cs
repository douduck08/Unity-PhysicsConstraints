using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

public class ChainConstraint : MonoBehaviour {

    [System.Serializable]
    public struct ConstraintConfig {
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

    [SerializeField] Transform targetPoint;
    [SerializeField] ConstraintConfig[] constraints;
    ConstraintData[] constraintDatas;

    [Header ("Global Config")]
    [SerializeField] float3 extraForce = new float3 (0, -9.8f, 0);
    [SerializeField, Range (0f, 1f)] float beta = 1f;
    [SerializeField, Range (0f, 1f)] float damping = 0.999f;
    [SerializeField, Range (1, 10)] int iteration = 10;

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
        var dt = Time.fixedDeltaTime;
        UpdateDynamics (dt, Mathf.Pow (damping, dt));

        for (int i = 0; i < constraints.Length; i++) {
            var deformedObject = constraints[i].deformedObject;
            deformedObject.position += (Vector3)constraintDatas[i].linarVelocity * dt;
            deformedObject.rotation = Quaternion.Euler (constraintDatas[i].angularVelocity * dt * Mathf.Rad2Deg) * deformedObject.rotation;
        }
    }

    void UpdateDynamics (float dt, float damping) {
        for (int i = 0; i < constraints.Length; i++) {
            // apply extra forces
            constraintDatas[i].linarVelocity = constraintDatas[i].linarVelocity + constraintDatas[i].massInv * extraForce * dt;
            constraintDatas[i].linarVelocity *= damping;
            constraintDatas[i].angularVelocity *= damping;
        }

        for (int it = 0; it < iteration; it++) {
            for (int i = 0; i < constraints.Length; i++) {
                var deformedObject = constraints[i].deformedObject;
                float3 targetPos = (i == 0) ? targetPoint.position : constraints[i - 1].deformedObject.position;
                float3 position = deformedObject.position;
                float3 r = deformedObject.rotation * constraints[i].offset;

                // caculate constraints
                float3 linarVelocity = constraintDatas[i].linarVelocity;
                float3 angularVelocity = constraintDatas[i].angularVelocity;
                float3 positionConstraint = (position + r) - targetPos;
                float3 velocityConstraint = linarVelocity + math.cross (angularVelocity, r);
                if (i > 0) {
                    velocityConstraint -= constraintDatas[i - 1].linarVelocity;
                }

                // transform inertia tensor
                float3x3 localToWorld = new float3x3
                (
                    deformedObject.localToWorldMatrix.m00, deformedObject.localToWorldMatrix.m01, deformedObject.localToWorldMatrix.m02,
                    deformedObject.localToWorldMatrix.m10, deformedObject.localToWorldMatrix.m11, deformedObject.localToWorldMatrix.m12,
                    deformedObject.localToWorldMatrix.m20, deformedObject.localToWorldMatrix.m21, deformedObject.localToWorldMatrix.m22
                );
                float3x3 inertiaInvWs = math.mul (localToWorld, math.mul (constraintDatas[i].inertiaInv, math.transpose (localToWorld)));

                // solve constraints
                float3x3 s = MathUtils.SkewSymmetric (-r); // matrix for cross dot
                float3x3 st = math.transpose (s);
                float3x3 k = (constraintDatas[i].massInv * float3x3.identity) + math.mul (s, math.mul (inertiaInvWs, st));
                if (i > 0) {
                    k = (constraintDatas[i - 1].massInv * float3x3.identity) + k;
                }
                float3 bias = (beta / dt) * positionConstraint; // bias velocity vector
                float3 lambda = math.mul (math.inverse (k), -(velocityConstraint + bias)); // = effectiveMass * -(JV + b)

                // add constraint forces
                linarVelocity += constraintDatas[i].massInv * lambda;
                angularVelocity += math.mul (inertiaInvWs, math.mul (st, lambda));
                constraintDatas[i].linarVelocity = linarVelocity;
                constraintDatas[i].angularVelocity = angularVelocity;

                if (i > 0) {
                    float3 v1 = constraintDatas[i - 1].linarVelocity;
                    v1 -= constraintDatas[i - 1].massInv * lambda;
                    constraintDatas[i - 1].linarVelocity = v1 * damping;
                }
            }
        }
    }
}
