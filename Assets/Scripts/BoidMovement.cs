using System.Collections;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Analytics;
using UnityEngine.Jobs;

public class BoidMovements : MonoBehaviour
{
    [SerializeField] private ListBoidVariable boids;
    private float radius = 2f;
    private float forwardSpeed = 5f;
    private float visionAngle = 270f;
    private float turnSpeed = 10f;
    private TransformAccessArray transformAccessArray;
    private NativeArray<BoidData> boidData;
    private struct BoidData
    {
        public float3 position;
        public float3 velocity;
    }
    private void Start()
    {
        var boidCount = boids.boidTransform.Count;
        transformAccessArray = new TransformAccessArray(boidCount);
        boidData = new NativeArray<BoidData>(boidCount, Allocator.Persistent);
        for (int i = 0; i < boidCount; i++)
        {
            transformAccessArray.Add(boids.boidTransform[i].transform);
            boidData[i] = new BoidData
            {
                position = boids.boidTransform[i].transform.position,
                velocity = boids.boidTransform[i].transform.forward
            };
        }
    }
    private void Update()
    {
        var boidMovementsJob = new BoidMovementsJob
        {
            boidData = boidData,
            turnSpeed = turnSpeed,
            forwardSpeed = forwardSpeed,
            radius = radius,
            visionAngle = visionAngle,
            deltaTime = Time.deltaTime,
        };
        JobHandle boidMovementsJobHandle = boidMovementsJob.Schedule(transformAccessArray);
        boidMovementsJobHandle.Complete();
    }
    private void OnDestroy()
    {
        transformAccessArray.Dispose();
        boidData.Dispose();
    }
    [BurstCompile]
    private struct BoidMovementsJob : IJobParallelForTransform
    {
        [NativeDisableContainerSafetyRestriction]
        public NativeArray<BoidData> boidData;
        public float turnSpeed;
        public float forwardSpeed;
        public float radius;
        public float visionAngle;
        public float deltaTime;
        public void Execute(int index, TransformAccess transform)
        {
            Vector3 velocity = boidData[index].velocity;
            velocity = Vector2.Lerp(velocity, CalculateVelocity(transform), turnSpeed / 2 * deltaTime);
            transform.position += velocity * deltaTime;
            if (velocity != Vector3.zero)
            {
                transform.rotation = Quaternion.Slerp(transform.localRotation,
                    Quaternion.LookRotation(velocity), turnSpeed * deltaTime);
            }

            boidData[index] = new BoidData
            {
                position = transform.position,
                velocity = velocity,
            };
        }
        private Vector2 CalculateVelocity(TransformAccess transform)
        {
            var separation = Vector2.zero;
            var aligment = Vector2.zero;
            var cohesion = Vector2.zero;
            Vector2 currentForward = transform.localToWorldMatrix.MultiplyVector(Vector3.forward);
            var boidsInRange = BoidsInRange(transform.position, currentForward);
            var boidCount = boidsInRange.Length;
            for (var i = 0; i < boidCount; i++)
            {
                separation -= Separation(transform.position, boidsInRange[i].position.xy);
                aligment += (Vector2)boidsInRange[i].velocity.xy;
                cohesion += (Vector2)boidsInRange[i].position.xy;
            }
            separation = separation.normalized;
            aligment = Aligment(aligment, currentForward, boidCount);
            cohesion = Cohesion(cohesion, transform.position, boidCount);
            Vector3 velocity = (currentForward
                + separation
                + 0.2f * aligment
                + cohesion
                ).normalized * forwardSpeed;
            return velocity;
        }
        private NativeArray<BoidData> BoidsInRange(float3 position, float2 forward)
        {
            NativeList<BoidData> boidsInRange = new NativeList<BoidData>(Allocator.Temp);
            for (int i = 0; i < boidData.Length; i++)
            {
                if (math.distance(position, boidData[i].position) < radius
                    && InVisionCone(position.xy, forward, boidData[i].position.xy))
                {
                    boidsInRange.Add(boidData[i]);
                }
            }
            NativeArray<BoidData> boids = new NativeArray<BoidData>(boidsInRange.AsArray(), Allocator.Temp);
            boidsInRange.Dispose();
            return boids;
        }
        private bool InVisionCone(Vector2 position, Vector2 forward, Vector2 boidPosition)
        {
            Vector2 directionToPosition = boidPosition - position;
            float dotProduct = Vector2.Dot(forward.normalized, directionToPosition);
            float cosHalfVisionAngle = Mathf.Cos(visionAngle * 0.5f * Mathf.Deg2Rad);
            return dotProduct >= cosHalfVisionAngle;
        }
        private Vector2 Separation(Vector2 currentPosition, Vector2 boidPosition)
        {
            float ratio = Mathf.Clamp01((boidPosition - currentPosition)
                .magnitude / radius);
            return (1 - ratio) * (boidPosition - currentPosition);
        }
        private Vector2 Aligment(Vector2 direction, Vector2 forward, int boidCount)
        {
            if (boidCount != 0) direction /= boidCount;
            else direction = forward;

            return direction.normalized;
        }

        private Vector2 Cohesion(Vector2 center, Vector2 position, int boidCount)
        {
            if (boidCount != 0) center /= boidCount;
            else center = position;

            return (center - position).normalized;
        }
    }
}
