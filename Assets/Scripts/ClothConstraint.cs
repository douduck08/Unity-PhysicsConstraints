using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

public class ClothConstraint : MonoBehaviour {

    [SerializeField] List<int> pinnedVertices;

    [Header ("Global Config")]
    [SerializeField] float3 extraForce = new float3 (0, -9.8f, 0);
    [SerializeField, Range (0f, 1f)] float beta = 1f;
    [SerializeField, Range (0f, 1f)] float damping = 0.999f;
    [SerializeField, Range (1, 10)] int iteration = 10;

    MeshFilter meshFilter;
    Mesh deformedMesh;
    Vector3[] vertices;

    struct ParticleData {
        public int vertexId;
        public float weight;
        public float3 lastPosition;
        public float3 position;
        public float3 velocity;
    }

    struct DistanceConstraint {
        public int particleId1;
        public int particleId2;
        public float distance;
    }

    ParticleData[] particleDatas;
    DistanceConstraint[] distanceConstraints;

    void Start () {
        CheckResources ();
        InitParticles ();
        InitConstraints ();
    }

    void FixedUpdate () {
        UpdateDynamics (Time.fixedDeltaTime);
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
            particleDatas[i].position = vertices[i];
            particleDatas[i].velocity = float3.zero;
        }
    }

    void InitConstraints () {
        distanceConstraints = new DistanceConstraint[220];
        var constraintId = 0;
        for (int i = 0; i < 10; i++) {
            for (int j = 0; j <= 10; j++) {
                var particleId1 = i * 11 + j;
                var particleId2 = particleId1 + 11;
                distanceConstraints[constraintId].particleId1 = particleId1;
                distanceConstraints[constraintId].particleId2 = particleId2;
                var posDiff = particleDatas[particleId1].position - particleDatas[particleId2].position;
                distanceConstraints[constraintId].distance = math.length (posDiff);
                constraintId += 1;
            }
        }

        for (int i = 0; i < 10; i++) {
            for (int j = 0; j <= 10; j++) {
                var particleId1 = i + j * 11;
                var particleId2 = particleId1 + 1;
                distanceConstraints[constraintId].particleId1 = particleId1;
                distanceConstraints[constraintId].particleId2 = particleId2;
                var posDiff = particleDatas[particleId1].position - particleDatas[particleId2].position;
                distanceConstraints[constraintId].distance = math.length (posDiff);
                constraintId += 1;
            }
        }
    }

    void UpdateDynamics (float dt) {
        // cache last position
        // apply extra forces
        // apply velocity to position
        for (int i = 0; i < particleDatas.Length; i++) {
            particleDatas[i].lastPosition = particleDatas[i].position;
            particleDatas[i].velocity += particleDatas[i].weight * extraForce * dt;
            particleDatas[i].position += particleDatas[i].velocity * dt;
        }

        // constraints
        for (int it = 0; it < iteration; it++) {
            for (int i = 0; i < distanceConstraints.Length; i++) {
                var particleId1 = distanceConstraints[i].particleId1;
                var particleId2 = distanceConstraints[i].particleId2;
                var weight1 = particleDatas[particleId1].weight;
                var weight2 = particleDatas[particleId2].weight;

                float3 diff = particleDatas[particleId1].position - particleDatas[particleId2].position;
                float3 n = math.normalize (diff);
                float lambda = math.length (diff) - distanceConstraints[i].distance;

                particleDatas[particleId1].position += -weight1 / (weight1 + weight2) * lambda * n;
                particleDatas[particleId2].position += weight2 / (weight1 + weight2) * lambda * n;
            }
        }

        // update velocity
        for (int i = 0; i < particleDatas.Length; i++) {
            particleDatas[i].velocity = (particleDatas[i].position - particleDatas[i].lastPosition) / dt;
        }
    }
}
