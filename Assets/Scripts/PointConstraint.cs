using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PointConstraint : MonoBehaviour
{
    [SerializeField] Transform targetPoint;
    [SerializeField] Transform deformObject;
    [SerializeField, Range(0f, 1f)] float beta = 1f;
    [SerializeField, Range(0f, 1f)] float damping = 0.99f;

    Vector3 offset = new Vector3(0.5f, 0.5f, 0.5f);
    Vector3 gravity = new Vector3(0, -9.8f, 0);
    float mass;
    float massInv;
    Matrix3x3 inertia;
    Matrix3x3 inertiaInv;

    Vector3 linarVelocity;
    Vector3 angularVelocity;

    void Start()
    {
        mass = 1f;
        massInv = 1f / mass;
        inertia = Matrix3x3.Identity; // tmp
        inertiaInv = inertia.Inverted;
    }

    void Update()
    {
        var targetPos = targetPoint.position;
        var position = deformObject.position;
        var r = deformObject.rotation * offset;
        var dt = Time.deltaTime;

        linarVelocity += gravity * dt;

        Matrix3x3 world2Local = Matrix3x3.FromRows
        (
            deformObject.TransformVector(new Vector3(1.0f, 0.0f, 0.0f)),
            deformObject.TransformVector(new Vector3(0.0f, 1.0f, 0.0f)),
            deformObject.TransformVector(new Vector3(0.0f, 0.0f, 1.0f))
        );
        Matrix3x3 inertiaInvWs = world2Local.Transposed * inertiaInv * world2Local;

        var cPos = (position + r) - targetPos;
        var cVel = linarVelocity + Vector3.Cross(angularVelocity, r);

        var s = Matrix3x3.Skew(-r);
        var k = (massInv * Matrix3x3.Identity) + s * inertiaInvWs * s.Transposed;
        Matrix3x3 effectiveMass = k.Inverted;
        Vector3 lambda = effectiveMass * (-(cVel + (beta / dt) * cPos));

        linarVelocity += massInv * lambda;
        angularVelocity += (inertiaInvWs * s.Transposed) * lambda;
        linarVelocity *= damping;
        angularVelocity *= damping;

        // integration
        deformObject.position += linarVelocity * dt;
        deformObject.rotation = Quaternion.Euler(angularVelocity * dt * Mathf.Rad2Deg) * deformObject.rotation;
    }
}
