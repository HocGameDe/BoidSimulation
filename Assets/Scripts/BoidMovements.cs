﻿using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Jobs;

public class BoidMovements : MonoBehaviour
{
    [SerializeField] private ListBoidVariable boids;
    [SerializeField] private QuadBounds quadBounds;
    [SerializeField] private NativeQuadTree<float2> quadTree;
    private TransformAccessArray transformAccessArray;
    private NativeArray<float2> velocities;

    private float radius = 2f;
    private float forwardSpeed = 5f;
    private float visionAngle = 270f;
    private float turnSpeed = 10f;

    private void Start()
    {
        quadTree = new NativeQuadTree<float2>(quadBounds, Allocator.Persistent, maxDepth: 8, maxLeafElements: 256);
        var boidCount = boids.boidTransform.Count;
        transformAccessArray = new TransformAccessArray(boidCount, JobsUtility.JobWorkerCount);
        velocities = new NativeArray<float2>(boidCount, Allocator.Persistent);
        for (int i = 0; i < boidCount; i++)
        {
            transformAccessArray.Add(boids.boidTransform[i].transform);
            velocities[i] = (Vector2)boids.boidTransform[i].forward;
        }
    }
    private void Update()
    {
        UpdateQuadTree();
        ApplyRule();
    }

    private void ApplyRule()
    {
        var boidMovementsJob = new BoidMovementsJob
        {
            quadTree = quadTree,
            velocities = velocities,
            quadBounds = quadBounds,
            turnSpeed = turnSpeed,
            forwardSpeed = forwardSpeed,
            radius = radius,
            visionAngle = visionAngle,
            deltaTime = Time.deltaTime,
        };
        JobHandle boidMovementsJobHandle = boidMovementsJob.Schedule(transformAccessArray);
        boidMovementsJobHandle.Complete();
    }

    private void UpdateQuadTree()
    {
        var forward = new NativeArray<QuadElement<float2>>(boids.boidTransform.Count, Allocator.TempJob);
        var updateQuadElementJob = new UpdateQuadElementJob { forward = forward };
        var updateQuadElementJobHandle = updateQuadElementJob.Schedule(transformAccessArray);
        updateQuadElementJobHandle.Complete();
        quadTree.ClearAndBulkInsert(forward);
        forward.Dispose();
    }

    private void OnDrawGizmosSelected() => quadTree.DrawGizmos(quadBounds);
    private void OnDestroy()
    {
        transformAccessArray.Dispose();
        quadTree.Dispose();
        velocities.Dispose();
    }

    [BurstCompile]
    private struct UpdateQuadElementJob : IJobParallelForTransform
    {
        public NativeArray<QuadElement<float2>> forward;
        public void Execute(int index, TransformAccess transform)
        {
            float3 position = transform.position;
            float3 forward = transform.localToWorldMatrix.MultiplyVector(Vector3.forward);
            this.forward[index] = new QuadElement<float2>
            {
                position = position.xy,
                element = forward.xy,
            };
        }
    }

    [BurstCompile]
    private struct BoidMovementsJob : IJobParallelForTransform
    {
        public NativeQuadTree<float2> quadTree;
        public NativeArray<float2> velocities;
        public QuadBounds quadBounds;
        public float turnSpeed;
        public float forwardSpeed;
        public float radius;
        public float visionAngle;
        public float deltaTime;
        public void Execute(int index, TransformAccess transform)
        {
            Vector3 velocity = (Vector2)velocities[index];
            velocity = Vector2.Lerp(velocity, CalculateVelocity(transform), turnSpeed / 2 * deltaTime);
            transform.position += velocity * deltaTime;
            if (velocity != Vector3.zero)
            {
                transform.rotation = Quaternion.Slerp(transform.localRotation,
                    Quaternion.LookRotation(velocity), turnSpeed * deltaTime);
            }

            velocities[index] = (Vector2)velocity;
        }
        private Vector2 CalculateVelocity(TransformAccess transform)
        {
            float3 currentPosition = transform.position;
            Vector2 currentForward = transform.localToWorldMatrix.MultiplyVector(Vector3.forward);
            var separation = Vector2.zero;
            var aligment = Vector2.zero;
            var cohesion = Vector2.zero;
            var boidsInRange = BoidsInRange(currentPosition);
            var boidCount = boidsInRange.Length;
            for (var i = 0; i < boidCount; i++)
            {
                //if (!InVisionCone(currentPosition.xy, boidsInRange[i].position, currentForward))
                //{
                //    boidsInRange.RemoveAtSwapBack(i);
                //    boidCount--;
                //    i--;
                //    continue;
                //}
                separation -= Separation(currentPosition.xy, boidsInRange[i].position.xy);
                aligment += (Vector2)boidsInRange[i].element.xy;
                cohesion += (Vector2)boidsInRange[i].position.xy;
            }
            separation = separation.normalized;
            aligment = Aligment(aligment, currentForward, boidCount);
            cohesion = Cohesion(cohesion, currentPosition.xy, boidCount);
            Vector3 velocity = (currentForward
                + separation
                + 0.2f * aligment
                + cohesion
                ).normalized * forwardSpeed;

            transform = Boundary(transform, currentPosition);
            boidsInRange.Dispose();
            return velocity;
        }
        private NativeList<QuadElement<float2>> BoidsInRange(float3 position)
        {
            var results = new NativeList<QuadElement<float2>>(Allocator.Temp);
            QuadBounds queryBounds = new QuadBounds(position.xy, new float2(radius, radius));
            quadTree.RangeQuery(queryBounds, results);
            return results;
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
        private readonly TransformAccess Boundary(TransformAccess transform, float3 currentPosition)
        {
            float2 limitBounds = quadBounds.extents * 0.95f;
            if (currentPosition.x > quadBounds.center.x + limitBounds.x ||
                currentPosition.x < quadBounds.center.x - limitBounds.x)
            {
                currentPosition.x = currentPosition.x > 0 ?
                    quadBounds.center.x - limitBounds.x : quadBounds.center.x + limitBounds.x;
                transform.position = currentPosition;
            }
            if (currentPosition.y > quadBounds.center.y + limitBounds.y ||
                currentPosition.y < quadBounds.center.y - limitBounds.y)
            {
                currentPosition.y = currentPosition.y > 0 ?
                    quadBounds.center.y - limitBounds.y : quadBounds.center.y + limitBounds.y;
                transform.position = currentPosition;
            }

            return transform;
        }
    }
}
