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
    [SerializeField] private QuadBounds quadBounds;
    [SerializeField] private NativeQuadTree<float2> quadTree;
    private float radius = 2f;
    private float forwardSpeed = 5f;
    private float visionAngle = 270f;
    private float turnSpeed = 10f;
    private TransformAccessArray transformAccessArray;
    private NativeArray<float2> velocities;
    // private NativeArray<BoidData> boidData;
    //private struct BoidData
    //{
    //    public float3 position;
    //    public float3 velocity;
    //}
    private void Start()
    {
        quadTree = new NativeQuadTree<float2>(quadBounds, Allocator.Persistent, maxDepth: 8, maxLeafElements: 128);
        var boidCount = boids.boidTransform.Count;
        transformAccessArray = new TransformAccessArray(boidCount);
        velocities = new NativeArray<float2>(boidCount, Allocator.Persistent);
        // boidData = new NativeArray<BoidData>(boidCount, Allocator.Persistent);
        for (int i = 0; i < boidCount; i++)
        {
            transformAccessArray.Add(boids.boidTransform[i].transform);
            velocities[i] = (Vector2)boids.boidTransform[i].forward;
            //boidData[i] = new BoidData
            //{
            //    position = boids.boidTransform[i].transform.position,
            //    velocity = boids.boidTransform[i].transform.forward
            //};
        }
    }
    private void Update()
    {
        var forward = new NativeArray<QuadElement<float2>>(boids.boidTransform.Count, Allocator.TempJob);
        var updateQuadElementJob = new UpdateQuadElementJob
        {
            forward = forward
        };
        JobHandle updateQuadElementJobHandle = updateQuadElementJob.Schedule(transformAccessArray);
        updateQuadElementJobHandle.Complete();
        quadTree.ClearAndBulkInsert(forward);
        forward.Dispose();

        var boidMovementsJob = new BoidMovementsJob
        {
            //boidData = boidData,
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
    private void OnDestroy()
    {
        transformAccessArray.Dispose();
        quadTree.Dispose();
        //boidData.Dispose();
        velocities.Dispose();
    }
    private void OnDrawGizmosSelected() => quadTree.DrawGizmos(quadBounds);

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
                element = forward.xy
            };
        }
    }
    [BurstCompile]
    private struct BoidMovementsJob : IJobParallelForTransform
    {
        //[NativeDisableContainerSafetyRestriction]
        //public NativeArray<BoidData> boidData;
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
        //private NativeArray<BoidData> BoidsInRange(float3 position, float2 forward)
        //{
        //    NativeList<BoidData> boidsInRange = new NativeList<BoidData>(Allocator.Temp);
        //    for (int i = 0; i < boidData.Length; i++)
        //    {
        //        if (math.distance(position, boidData[i].position) < radius
        //            && InVisionCone(position.xy, forward, boidData[i].position.xy))
        //        {
        //            boidsInRange.Add(boidData[i]);
        //        }
        //    }
        //    NativeArray<BoidData> boids = new NativeArray<BoidData>(boidsInRange.AsArray(), Allocator.Temp);
        //    boidsInRange.Dispose();
        //    return boids;
        //}
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
            float2 litmitBounds = quadBounds.extents * 0.95f;
            if (currentPosition.x > quadBounds.center.x + litmitBounds.x ||
                currentPosition.x < quadBounds.center.x - litmitBounds.x)
            {
                currentPosition.x = currentPosition.x > 0 ?
                quadBounds.center.x - litmitBounds.x : quadBounds.center.x + litmitBounds.x;
                transform.position = currentPosition;
            }
            if (currentPosition.y > quadBounds.center.y + litmitBounds.y ||
                currentPosition.y < quadBounds.center.y - litmitBounds.y)
            {
                currentPosition.y = currentPosition.y > 0 ?
                quadBounds.center.y - litmitBounds.y : quadBounds.center.y + litmitBounds.y;
                transform.position = currentPosition;
            }

            return transform;
        }
    }
}
