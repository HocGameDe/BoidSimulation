using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;
using UnityEngine;

public struct QuadElement<T> where T : unmanaged
{
    public float2 position;
    public T element;
}
public struct QuadNode
{
    public int firstChildIndex;
    public ushort count;
    public bool isLeaf;
}
public unsafe partial struct NativeQuadTree<T> : IDisposable where T : unmanaged
{
    [NativeDisableUnsafePtrRestriction]
    UnsafeList<QuadElement<T>>* elements;

    [NativeDisableUnsafePtrRestriction]
    UnsafeList<int>* lookup;

    [NativeDisableUnsafePtrRestriction]
    UnsafeList<QuadNode>* nodes;

    QuadBounds bounds;
    byte maxDepth;
    ushort maxLeafElements;
    int elementsCount;

    public NativeQuadTree(QuadBounds bounds, Allocator allocator = Allocator.Temp,
        byte maxDepth = 6, ushort maxLeafElements = 16, int initialElementsCapacity = 256)
    {
        this.bounds = bounds;
        this.maxDepth = maxDepth;
        this.maxLeafElements = maxLeafElements;
        elementsCount = 0;

        if (maxDepth > 8 || maxDepth <= 0)
            throw new ArgumentOutOfRangeException(nameof(maxDepth), "Max depth must be less than 9 and higher than 0");

        var totalSize = LookupTables.DepthSizeLookup[maxDepth + 1];

        lookup = UnsafeList<int>.Create(
            totalSize,
            allocator,
            NativeArrayOptions.ClearMemory);

        nodes = UnsafeList<QuadNode>.Create(
            totalSize,
            allocator,
            NativeArrayOptions.ClearMemory);

        elements = UnsafeList<QuadElement<T>>.Create(
            initialElementsCapacity,
            allocator);
    }
    public void ClearAndBulkInsert(NativeArray<QuadElement<T>> incomingElements)
    {
        Clear();

        if (elements->Capacity < incomingElements.Length)
        {
            elements->Resize(math.max(incomingElements.Length, elements->Capacity * 2));
        }

        var mortonCodes = new NativeArray<int>(incomingElements.Length, Allocator.Temp);
        var depthExtentsScaling = LookupTables.DepthLookup[maxDepth] / bounds.extents;
        for (int i = 0; i < incomingElements.Length; i++)
        {
            var positionElement = incomingElements[i].position;
            positionElement -= bounds.center;
            positionElement.y = -positionElement.y;
            var position = (positionElement + bounds.extents) * .5f;
            position *= depthExtentsScaling;
            mortonCodes[i] = LookupTables.MortonLookup[(int)position.x] | (LookupTables.MortonLookup[(int)position.y] << 1);

            int atIndex = 0;
            for (int depth = maxDepth; depth >= 0; depth--)
            {
                (*(int*)((IntPtr)lookup->Ptr + atIndex * sizeof(int)))++;
                atIndex = IncrementIndex(mortonCodes[i], depth);
            }
        }
        RecursivePrepareLeaves(1, 1);

        for (int i = 0; i < incomingElements.Length; i++)
        {
            int atIndex = 0;
            for (int depth = maxDepth; depth >= 0; depth--)
            {
                var node = UnsafeUtility.ReadArrayElement<QuadNode>(nodes->Ptr, atIndex);
                if (node.isLeaf)
                {
                    UnsafeUtility.WriteArrayElement(elements->Ptr, node.firstChildIndex + node.count, incomingElements[i]);
                    node.count++;
                    UnsafeUtility.WriteArrayElement(nodes->Ptr, atIndex, node);
                    break;
                }
                atIndex = IncrementIndex(mortonCodes[i], depth);
            }
        }
    }

    private void RecursivePrepareLeaves(int previousOffset, int depth)
    {
        for (int i = 0; i < 4; i++)
        {
            var atIndex = previousOffset + i * LookupTables.DepthSizeLookup[maxDepth - depth + 1];
            var elementCount = UnsafeUtility.ReadArrayElement<int>(lookup->Ptr, atIndex);
            if (elementCount > maxLeafElements && depth < maxDepth)
            {
                RecursivePrepareLeaves(atIndex + 1, depth + 1);
            }
            else if (elementCount != 0)
            {
                var node = new QuadNode { firstChildIndex = elementsCount, count = 0, isLeaf = true };
                UnsafeUtility.WriteArrayElement(nodes->Ptr, atIndex, node);
                elementsCount += elementCount;
            }
        }
    }
    private int IncrementIndex(int mortonCode, int depth)
    {
        int shiftedMortonCode = (mortonCode >> (depth - 1) * 2) & 0b11;
        return (LookupTables.DepthSizeLookup[depth] * shiftedMortonCode) + 1;
    }
    public void RangeQuery(QuadBounds bounds, NativeList<QuadElement<T>> results)
    {
        new QuadTreeRangeQuery(this, bounds, results);
    }
    public void Clear()
    {
        UnsafeUtility.MemClear(lookup->Ptr, lookup->Capacity * UnsafeUtility.SizeOf<int>());
        UnsafeUtility.MemClear(nodes->Ptr, nodes->Capacity * UnsafeUtility.SizeOf<QuadNode>());
        UnsafeUtility.MemClear(elements->Ptr, elements->Capacity * UnsafeUtility.SizeOf<QuadElement<T>>());
        elementsCount = 0;
    }
    public void Dispose()
    {
        UnsafeList<QuadElement<T>>.Destroy(elements);
        UnsafeList<int>.Destroy(lookup);
        UnsafeList<QuadNode>.Destroy(nodes);
        elements = null;
        lookup = null;
        nodes = null;
    }
}
