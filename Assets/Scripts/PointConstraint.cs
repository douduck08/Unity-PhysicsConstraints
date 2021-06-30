using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

public class PointConstraint : MonoBehaviour {
    [SerializeField] Transform targetPoint;
    [SerializeField] Transform deformObject;
    [SerializeField, Range (0f, 1f)] float beta = 1f;
    [SerializeField, Range (0f, 1f)] float damping = 0.99f;

    float3 offset = new float3 (0.5f, 0.5f, 0.5f);
    float3 gravity = new float3 (0, -9.8f, 0);

    float mass;
    float massInv;
    float3x3 inertia;
    float3x3 inertiaInv;
    float3 linarVelocity;
    float3 angularVelocity;

    void Start () {
        mass = 1f;
        massInv = 1f / mass;

        inertia = float3x3.identity; // tmp
        inertiaInv = math.inverse (inertia);
    }

    void Update () {
        UpdateDynamics ();
    }

    void UpdateDynamics () {
        float3 targetPos = targetPoint.position;
        float3 position = deformObject.position;
        float3 r = deformObject.rotation * offset;
        float dt = Time.deltaTime;

        linarVelocity += gravity * dt;

        float3x3 local2World = new float3x3
        (
            deformObject.localToWorldMatrix.m00, deformObject.localToWorldMatrix.m01, deformObject.localToWorldMatrix.m02,
            deformObject.localToWorldMatrix.m10, deformObject.localToWorldMatrix.m11, deformObject.localToWorldMatrix.m12,
            deformObject.localToWorldMatrix.m20, deformObject.localToWorldMatrix.m21, deformObject.localToWorldMatrix.m22
        );
        float3x3 inertiaInvWs = math.mul (local2World, math.mul (inertiaInv, math.transpose (local2World)));

        float3 cPos = (position + r) - targetPos;
        float3 cVel = linarVelocity + math.cross (angularVelocity, r);

        float3x3 s = new float3x3
        (
            0.0f, r.z, -r.y,
            -r.z, 0.0f, r.x,
            r.y, -r.x, 0.0f
        );
        float3x3 k = (massInv * float3x3.identity) + math.mul (s, math.mul (inertiaInvWs, math.transpose (s)));
        float3x3 effectiveMass = math.inverse (k);
        float3 lambda = math.mul (effectiveMass, (-(cVel + (beta / dt) * cPos)));

        linarVelocity += massInv * lambda;
        angularVelocity += math.mul (math.mul (inertiaInvWs, math.transpose (s)), lambda);
        linarVelocity *= damping;
        angularVelocity *= damping;

        // integration
        deformObject.position += (Vector3)linarVelocity * dt;
        deformObject.rotation = Quaternion.Euler (angularVelocity * dt * Mathf.Rad2Deg) * deformObject.rotation;
    }
}
