using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

public static class MathUtils {
    public static float3x3 SkewSymmetric (float3 v) {
        return new float3x3
        (
            0.0f, -v.z, v.y,
            v.z, 0.0f, -v.x,
            -v.y, v.x, 0.0f
        );
    }
}
