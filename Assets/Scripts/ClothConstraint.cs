using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

public class ClothConstraint : MonoBehaviour {

    [SerializeField] List<int> pinnedVertices;
    [SerializeField] List<SphereCollider> colliders;

    [SerializeField, Range (0f, 1f)] float stretchStiffness = 1f;
    [SerializeField, Range (0f, 1f)] float bendStiffness = 1f;
    [SerializeField] float particleRadius = 0.01f;

    [Header ("Global Config")]
    [SerializeField] Vector3 extraForce = new Vector3 (0, -9.8f, 0);
    [SerializeField, Range (0f, 1f)] float beta = 1f;
    [SerializeField, Range (0f, 1f)] float damping = 0.999f;
    [SerializeField, Range (1, 10)] int iteration = 10;

    MeshFilter meshFilter;
    Mesh deformedMesh;
    Vector3[] vertices;

    struct ParticleData {
        public int vertexId;
        public float weight;
        public float3 position;
        public float3 lastPosition;
        public float3 velocity;
    }

    struct ColliderData {
        public Transform trandform;
        public float3 position;
        public float radius;
    }

    struct DistanceConstraint {
        public int particleId1;
        public int particleId2;
        public float distance;
    }

    struct BendConstraint {
        public int particleId1;
        public int particleId2;
        public int particleIdV;
        public float restLength;
    }

    ParticleData[] particleDatas;
    ColliderData[] colliderDatas;
    DistanceConstraint[] stretchConstraints;
    BendConstraint[] bendConstraints;

    void Start () {
        CheckResources ();
        InitParticles ();
        InitColliders ();
        InitConstraints ();
    }

    void FixedUpdate () {
        var dt = Time.fixedDeltaTime;
        UpdateDynamics (dt, Mathf.Pow (damping, dt));
    }

    void LateUpdate () {
        for (int i = 0; i < particleDatas.Length; i++) {
            vertices[particleDatas[i].vertexId] = particleDatas[i].position;
        }
        deformedMesh.SetVertices (vertices);
        deformedMesh.RecalculateNormals ();
        deformedMesh.RecalculateBounds ();
    }

    void OnDrawGizmos () {
        CheckResources ();

        Gizmos.matrix = transform.localToWorldMatrix;
        Gizmos.color = new Color (1f, .2f, .2f);
        if (pinnedVertices != null && pinnedVertices.Count > 0) {
            foreach (var index in pinnedVertices) {
                if (index < vertices.Length) {
                    var position = vertices[index];
                    Gizmos.DrawSphere (position, 0.15f);
                }
            }
        }
    }

    void CheckResources () {
        if (meshFilter == null) {
            meshFilter = GetComponent<MeshFilter> ();
            if (Application.isPlaying) {
                deformedMesh = meshFilter.mesh;
            } else {
                deformedMesh = meshFilter.sharedMesh;
            }
            vertices = deformedMesh.vertices;
        }
    }

    void InitParticles () {
        particleDatas = new ParticleData[vertices.Length];
        for (int i = 0; i < particleDatas.Length; i++) {
            particleDatas[i].vertexId = i;
            particleDatas[i].weight = pinnedVertices.Contains (i) ? 0f : 1f;
            particleDatas[i].position = particleDatas[i].lastPosition = vertices[i];
            particleDatas[i].velocity = float3.zero;
        }
    }

    void InitColliders () {
        colliderDatas = new ColliderData[colliders.Count];
        for (int i = 0; i < colliderDatas.Length; i++) {
            colliderDatas[i].trandform = colliders[i].transform;
            var scale = colliderDatas[i].trandform.lossyScale;
            colliderDatas[i].radius = colliders[i].radius * Mathf.Max (scale.x, scale.y, scale.z);
        }
    }

    void InitConstraints () {
        stretchConstraints = new DistanceConstraint[10 * 11 * 2];
        var constraintId = 0;
        for (int i = 0; i < 10; i++) {
            for (int j = 0; j < 11; j++) {
                var particleId1 = i * 11 + j;
                var particleId2 = particleId1 + 11;
                stretchConstraints[constraintId].particleId1 = particleId1;
                stretchConstraints[constraintId].particleId2 = particleId2;
                var posDiff = particleDatas[particleId1].position - particleDatas[particleId2].position;
                stretchConstraints[constraintId].distance = math.length (posDiff);
                constraintId += 1;
            }
        }

        for (int i = 0; i < 10; i++) {
            for (int j = 0; j < 11; j++) {
                var particleId1 = i + j * 11;
                var particleId2 = particleId1 + 1;
                stretchConstraints[constraintId].particleId1 = particleId1;
                stretchConstraints[constraintId].particleId2 = particleId2;
                var posDiff = particleDatas[particleId1].position - particleDatas[particleId2].position;
                stretchConstraints[constraintId].distance = math.length (posDiff);
                constraintId += 1;
            }
        }

        bendConstraints = new BendConstraint[9 * 11 * 2];
        constraintId = 0;
        for (int i = 0; i < 9; i++) {
            for (int j = 0; j < 11; j++) {
                var particleId1 = i * 11 + j;
                var particleId2 = particleId1 + 22;
                var particleIdV = particleId1 + 11;
                bendConstraints[constraintId].particleId1 = particleId1;
                bendConstraints[constraintId].particleId2 = particleId2;
                bendConstraints[constraintId].particleIdV = particleIdV;
                var c = (1f / 3f) * (particleDatas[particleId1].position + particleDatas[particleId2].position + particleDatas[particleIdV].position);
                var h = particleDatas[particleIdV].position - c;
                bendConstraints[constraintId].restLength = math.length (h);
                constraintId += 1;
            }
        }

        for (int i = 0; i < 9; i++) {
            for (int j = 0; j < 11; j++) {
                var particleId1 = i + j * 11;
                var particleId2 = particleId1 + 2;
                var particleIdV = particleId1 + 1;
                bendConstraints[constraintId].particleId1 = particleId1;
                bendConstraints[constraintId].particleId2 = particleId2;
                bendConstraints[constraintId].particleIdV = particleIdV;
                var c = (1f / 3f) * (particleDatas[particleId1].position + particleDatas[particleId2].position + particleDatas[particleIdV].position);
                var h = particleDatas[particleIdV].position - c;
                bendConstraints[constraintId].restLength = math.length (h);
                constraintId += 1;
            }
        }

    }

    void UpdateDynamics (float dt, float damping) {
        // cache last position
        // apply extra forces
        // apply velocity to position
        float3 localExtraForce = transform.InverseTransformDirection (extraForce);
        for (int i = 0; i < particleDatas.Length; i++) {
            particleDatas[i].velocity += particleDatas[i].weight * localExtraForce * dt;
            particleDatas[i].velocity *= damping;
            particleDatas[i].position += particleDatas[i].velocity * dt;
        }

        // update colliders
        var worldToLocal = transform;
        for (int i = 0; i < colliderDatas.Length; i++) {
            colliderDatas[i].position = worldToLocal.InverseTransformPoint (colliderDatas[i].trandform.position);
        }

        // constraints
        float ks = 1f - Mathf.Pow (1f - stretchStiffness, 1f / iteration);
        float kb = 1f - Mathf.Pow (1f - bendStiffness, 1f / iteration);
        for (int it = 0; it < iteration; it++) {
            // bend constraints
            for (int i = 0; i < bendConstraints.Length; i++) {
                var particleId1 = bendConstraints[i].particleId1;
                var particleId2 = bendConstraints[i].particleId2;
                var particleIdV = bendConstraints[i].particleIdV;
                var weight1 = particleDatas[particleId1].weight;
                var weight2 = particleDatas[particleId2].weight;
                var weightV = particleDatas[particleIdV].weight;

                float3 c = (1f / 3f) * (particleDatas[particleId1].position + particleDatas[particleId2].position + particleDatas[particleIdV].position);
                float3 h = particleDatas[particleIdV].position - c;
                float lambda = 1f - (bendConstraints[i].restLength + 0.05f) / (math.length (h) + 0.0001f);

                float w = weight1 + weight2 + weightV * 2f;
                particleDatas[particleId1].position += (2f * weight1 / w) * lambda * kb * h;
                particleDatas[particleId2].position += (2f * weight2 / w) * lambda * kb * h;
                particleDatas[particleIdV].position += (-4f * weightV / w) * lambda * kb * h;
            }

            // distance constraints
            for (int i = 0; i < stretchConstraints.Length; i++) {
                var particleId1 = stretchConstraints[i].particleId1;
                var particleId2 = stretchConstraints[i].particleId2;
                var weight1 = particleDatas[particleId1].weight;
                var weight2 = particleDatas[particleId2].weight;

                float3 diff = particleDatas[particleId1].position - particleDatas[particleId2].position;
                float3 n = math.normalize (diff);
                float lambda = math.length (diff) - stretchConstraints[i].distance;

                particleDatas[particleId1].position += -weight1 / (weight1 + weight2) * lambda * ks * n;
                particleDatas[particleId2].position += weight2 / (weight1 + weight2) * lambda * ks * n;
            }

            // collider constraint
            for (int i = 0; i < colliderDatas.Length; i++) {
                var position = colliderDatas[i].position;
                var distance = colliderDatas[i].radius + particleRadius;
                for (int particleId = 0; particleId < particleDatas.Length; particleId++) {
                    float3 diff = particleDatas[particleId].position - position;
                    float lambda = math.length (diff) - distance;
                    if (lambda < 0) {
                        float3 n = math.normalize (diff);
                        particleDatas[particleId].position -= lambda * n;
                    }

                }
            }
        }

        // update velocity
        for (int i = 0; i < particleDatas.Length; i++) {
            particleDatas[i].velocity = (particleDatas[i].position - particleDatas[i].lastPosition) / dt;
            particleDatas[i].lastPosition = particleDatas[i].position;
        }
    }
}
