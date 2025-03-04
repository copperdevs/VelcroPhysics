﻿/*
 * Velcro Physics:
 * Copyright (c) 2017 Ian Qvist
 *
 * Original source Box2D:
 * Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Numerics;
using VelcroPhysics.Collision.RayCast;
using VelcroPhysics.Shared;
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Collision.Broadphase
{
    /// <summary>
    /// A dynamic tree arranges data in a binary tree to accelerate queries such as volume queries and ray casts.
    /// Leafs are proxies with an AABB. In the tree we expand the proxy AABB by Settings.b2_fatAABBFactor so that the proxy
    /// AABB is bigger than the client object. This allows the client object to move by small amounts without triggering a tree
    /// update. Nodes are pooled and relocatable, so we use node indices rather than pointers.
    /// </summary>
    public class DynamicTree<T>
    {
        public const int NullNode = -1;
        private int _freeList;
        private int _nodeCapacity;
        private int _nodeCount;
        private TreeNode<T>[] _nodes;
        private Stack<int> _queryStack = new(256);
        private Stack<int> _raycastStack = new(256);
        private int _root;

        /// <summary>Constructing the tree initializes the node pool.</summary>
        public DynamicTree()
        {
            _root = NullNode;

            _nodeCapacity = 16;
            _nodeCount = 0;
            _nodes = new TreeNode<T>[_nodeCapacity];

            // Build a linked list for the free list.
            for (var i = 0; i < _nodeCapacity - 1; ++i)
            {
                _nodes[i] = new TreeNode<T>
                {
                    ParentOrNext = i + 1,
                    Height = 1
                };
            }

            _nodes[_nodeCapacity - 1] = new TreeNode<T>
            {
                ParentOrNext = NullNode,
                Height = 1
            };
            _freeList = 0;
        }

        /// <summary>Compute the height of the binary tree in O(N) time. Should not be called often.</summary>
        public int Height
        {
            get
            {
                if (_root == NullNode)
                    return 0;

                return _nodes[_root].Height;
            }
        }

        /// <summary>Get the ratio of the sum of the node areas to the root area.</summary>
        public float AreaRatio
        {
            get
            {
                if (_root == NullNode)
                    return 0.0f;

                var root = _nodes[_root];
                var rootArea = root.AABB.Perimeter;

                var totalArea = 0.0f;
                for (var i = 0; i < _nodeCapacity; ++i)
                {
                    var node = _nodes[i];
                    if (node.Height < 0)
                    {
                        // Free node in pool
                        continue;
                    }

                    totalArea += node.AABB.Perimeter;
                }

                return totalArea / rootArea;
            }
        }

        /// <summary>
        /// Get the maximum balance of an node in the tree. The balance is the difference in height of the two children of
        /// a node.
        /// </summary>
        public int MaxBalance
        {
            get
            {
                var maxBalance = 0;
                for (var i = 0; i < _nodeCapacity; ++i)
                {
                    var node = _nodes[i];
                    if (node.Height <= 1)
                        continue;

                    Debug.Assert(!node.IsLeaf());

                    var child1 = node.Child1;
                    var child2 = node.Child2;
                    var balance = Math.Abs(_nodes[child2].Height - _nodes[child1].Height);
                    maxBalance = Math.Max(maxBalance, balance);
                }

                return maxBalance;
            }
        }

        /// <summary>
        /// Create a proxy in the tree as a leaf node. We return the index of the node instead of a pointer so that we can
        /// grow the node pool.
        /// </summary>
        /// <param name="aabb">The AABB.</param>
        /// <param name="userData">The user data.</param>
        /// <returns>Index of the created proxy</returns>
        public int CreateProxy(ref AABB aabb, T userData)
        {
            var proxyId = AllocateNode();

            // Fatten the AABB.
            var r = new Vector2(Settings.AABBExtension, Settings.AABBExtension);
            _nodes[proxyId].AABB.LowerBound = aabb.LowerBound - r;
            _nodes[proxyId].AABB.UpperBound = aabb.UpperBound + r;
            _nodes[proxyId].UserData = userData;
            _nodes[proxyId].Height = 0;
            _nodes[proxyId].Moved = true;

            InsertLeaf(proxyId);

            return proxyId;
        }

        /// <summary>Destroy a proxy. This asserts if the id is invalid.</summary>
        /// <param name="proxyId">The proxy id.</param>
        public void DestroyProxy(int proxyId)
        {
            Debug.Assert(0 <= proxyId && proxyId < _nodeCapacity);
            Debug.Assert(_nodes[proxyId].IsLeaf());

            RemoveLeaf(proxyId);
            FreeNode(proxyId);
        }

        /// <summary>
        /// Move a proxy with a swepted AABB. If the proxy has moved outside of its fattened AABB, then the proxy is
        /// removed from the tree and re-inserted. Otherwise the function returns immediately.
        /// </summary>
        /// <param name="proxyId">The proxy id.</param>
        /// <param name="aabb">The AABB.</param>
        /// <param name="displacement">The displacement.</param>
        /// <returns>true if the proxy was re-inserted.</returns>
        public bool MoveProxy(int proxyId, ref AABB aabb, Vector2 displacement)
        {
            Debug.Assert(0 <= proxyId && proxyId < _nodeCapacity);

            Debug.Assert(_nodes[proxyId].IsLeaf());

            // Extend AABB
            var fatAABB = new AABB();
            var r = new Vector2(Settings.AABBExtension, Settings.AABBExtension);
            fatAABB.LowerBound = aabb.LowerBound - r;
            fatAABB.UpperBound = aabb.UpperBound + r;

            // Predict AABB movement
            var d = Settings.AABBMultiplier * displacement;

            if (d.X < 0.0f)
                fatAABB.LowerBound.X += d.X;
            else
                fatAABB.UpperBound.X += d.X;

            if (d.Y < 0.0f)
                fatAABB.LowerBound.Y += d.Y;
            else
                fatAABB.UpperBound.Y += d.Y;

            var treeAABB = _nodes[proxyId].AABB;
            if (treeAABB.Contains(ref aabb))
            {
                // The tree AABB still contains the object, but it might be too large.
                // Perhaps the object was moving fast but has since gone to sleep.
                // The huge AABB is larger than the new fat AABB.
                var hugeAABB = new AABB
                {
                    LowerBound = fatAABB.LowerBound - 4.0f * r,
                    UpperBound = fatAABB.UpperBound + 4.0f * r
                };

                if (hugeAABB.Contains(ref treeAABB))
                {
                    // The tree AABB contains the object AABB and the tree AABB is
                    // not too large. No tree update needed.
                    return false;
                }

                // Otherwise the tree AABB is huge and needs to be shrunk
            }

            RemoveLeaf(proxyId);

            _nodes[proxyId].AABB = fatAABB;

            InsertLeaf(proxyId);

            _nodes[proxyId].Moved = true;

            return true;
        }

        public bool WasMoved(int proxyId)
        {
            Debug.Assert(0 <= proxyId && proxyId < _nodeCapacity);
            return _nodes[proxyId].Moved;
        }

        public void ClearMoved(int proxyId)
        {
            Debug.Assert(0 <= proxyId && proxyId < _nodeCapacity);
            _nodes[proxyId].Moved = false;
        }

        /// <summary>Get proxy user data.</summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="proxyId">The proxy id.</param>
        /// <returns>the proxy user data or 0 if the id is invalid.</returns>
        public T GetUserData(int proxyId)
        {
            Debug.Assert(0 <= proxyId && proxyId < _nodeCapacity);
            return _nodes[proxyId].UserData;
        }

        /// <summary>Get the fat AABB for a proxy.</summary>
        /// <param name="proxyId">The proxy id.</param>
        /// <param name="fatAABB">The fat AABB.</param>
        public void GetFatAABB(int proxyId, out AABB fatAABB)
        {
            Debug.Assert(0 <= proxyId && proxyId < _nodeCapacity);
            fatAABB = _nodes[proxyId].AABB;
        }

        /// <summary>
        /// Query an AABB for overlapping proxies. The callback class is called for each proxy that overlaps the supplied
        /// AABB.
        /// </summary>
        /// <param name="callback">The callback.</param>
        /// <param name="aabb">The AABB.</param>
        public void Query(Func<int, bool> callback, ref AABB aabb)
        {
            _queryStack.Clear();
            _queryStack.Push(_root);

            while (_queryStack.Count > 0)
            {
                var nodeId = _queryStack.Pop();
                if (nodeId == NullNode)
                    continue;

                var node = _nodes[nodeId];

                if (AABB.TestOverlap(ref node.AABB, ref aabb))
                {
                    if (node.IsLeaf())
                    {
                        var proceed = callback(nodeId);
                        if (!proceed)
                            return;
                    }
                    else
                    {
                        _queryStack.Push(node.Child1);
                        _queryStack.Push(node.Child2);
                    }
                }
            }
        }

        /// <summary>
        /// Ray-cast against the proxies in the tree. This relies on the callback to perform a exact ray-cast in the case
        /// were the proxy contains a Shape. The callback also performs the any collision filtering. This has performance roughly
        /// equal to k * log(n), where k is the number of collisions and n is the number of proxies in the tree.
        /// </summary>
        /// <param name="callback">A callback class that is called for each proxy that is hit by the ray.</param>
        /// <param name="input">The ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).</param>
        public void RayCast(Func<RayCastInput, int, float> callback, ref RayCastInput input)
        {
            var p1 = input.Point1;
            var p2 = input.Point2;
            var r = Vector2.Normalize(p2 - p1);
            Debug.Assert(r.LengthSquared() > 0.0f);

            // v is perpendicular to the segment.
            var absV = MathUtils.Abs(new Vector2(-r.Y, r.X)); //Velcro: Inlined the 'v' variable

            // Separating axis for segment (Gino, p80).
            // |dot(v, p1 - c)| > dot(|v|, h)

            var maxFraction = input.MaxFraction;

            // Build a bounding box for the segment.
            var segmentAABB = new AABB();
            {
                var t = p1 + maxFraction * (p2 - p1);
                segmentAABB.LowerBound = Vector2.Min(p1, t);
                segmentAABB.UpperBound = Vector2.Max(p1, t);
            }

            _raycastStack.Clear();
            _raycastStack.Push(_root);

            while (_raycastStack.Count > 0)
            {
                var nodeId = _raycastStack.Pop();
                if (nodeId == NullNode)
                    continue;

                var node = _nodes[nodeId];

                if (!AABB.TestOverlap(ref node.AABB, ref segmentAABB))
                    continue;

                // Separating axis for segment (Gino, p80).
                // |dot(v, p1 - c)| > dot(|v|, h)
                var c = node.AABB.Center;
                var h = node.AABB.Extents;
                var separation = Math.Abs(Vector2.Dot(new Vector2(-r.Y, r.X), p1 - c)) - Vector2.Dot(absV, h);
                if (separation > 0.0f)
                    continue;

                if (node.IsLeaf())
                {
                    RayCastInput subInput;
                    subInput.Point1 = input.Point1;
                    subInput.Point2 = input.Point2;
                    subInput.MaxFraction = maxFraction;

                    var value = callback(subInput, nodeId);

                    if (value == 0.0f)
                    {
                        // the client has terminated the raycast.
                        return;
                    }

                    if (value > 0.0f)
                    {
                        // Update segment bounding box.
                        maxFraction = value;
                        var t = p1 + maxFraction * (p2 - p1);
                        segmentAABB.LowerBound = Vector2.Min(p1, t);
                        segmentAABB.UpperBound = Vector2.Max(p1, t);
                    }
                }
                else
                {
                    _raycastStack.Push(node.Child1);
                    _raycastStack.Push(node.Child2);
                }
            }
        }

        private int AllocateNode()
        {
            // Expand the node pool as needed.
            if (_freeList == NullNode)
            {
                Debug.Assert(_nodeCount == _nodeCapacity);

                // The free list is empty. Rebuild a bigger pool.
                var oldNodes = _nodes;
                _nodeCapacity *= 2;
                _nodes = new TreeNode<T>[_nodeCapacity];
                Array.Copy(oldNodes, _nodes, _nodeCount);

                // Build a linked list for the free list. The parent
                // pointer becomes the "next" pointer.
                for (var i = _nodeCount; i < _nodeCapacity - 1; ++i)
                {
                    _nodes[i] = new TreeNode<T>
                    {
                        ParentOrNext = i + 1,
                        Height = -1
                    };
                }

                _nodes[_nodeCapacity - 1] = new TreeNode<T>
                {
                    ParentOrNext = NullNode,
                    Height = -1
                };
                _freeList = _nodeCount;
            }

            // Peel a node off the free list.
            var nodeId = _freeList;
            _freeList = _nodes[nodeId].ParentOrNext;
            _nodes[nodeId].ParentOrNext = NullNode;
            _nodes[nodeId].Child1 = NullNode;
            _nodes[nodeId].Child2 = NullNode;
            _nodes[nodeId].Height = 0;
            _nodes[nodeId].UserData = default;
            _nodes[nodeId].Moved = false;
            ++_nodeCount;
            return nodeId;
        }

        private void FreeNode(int nodeId)
        {
            Debug.Assert(0 <= nodeId && nodeId < _nodeCapacity);
            Debug.Assert(0 < _nodeCount);
            _nodes[nodeId].ParentOrNext = _freeList;
            _nodes[nodeId].Height = -1;
            _freeList = nodeId;
            --_nodeCount;
        }

        private void InsertLeaf(int leaf)
        {
            if (_root == NullNode)
            {
                _root = leaf;
                _nodes[_root].ParentOrNext = NullNode;
                return;
            }

            // Find the best sibling for this node
            var leafAABB = _nodes[leaf].AABB;
            var index = _root;
            while (!_nodes[index].IsLeaf())
            {
                var child1 = _nodes[index].Child1;
                var child2 = _nodes[index].Child2;

                var area = _nodes[index].AABB.Perimeter;

                var combinedAABB = new AABB();
                combinedAABB.Combine(ref _nodes[index].AABB, ref leafAABB);
                var combinedArea = combinedAABB.Perimeter;

                // Cost of creating a new parent for this node and the new leaf
                var cost = 2.0f * combinedArea;

                // Minimum cost of pushing the leaf further down the tree
                var inheritanceCost = 2.0f * (combinedArea - area);

                // Cost of descending into child1
                float cost1;
                if (_nodes[child1].IsLeaf())
                {
                    var aabb = new AABB();
                    aabb.Combine(ref leafAABB, ref _nodes[child1].AABB);
                    cost1 = aabb.Perimeter + inheritanceCost;
                }
                else
                {
                    var aabb = new AABB();
                    aabb.Combine(ref leafAABB, ref _nodes[child1].AABB);
                    var oldArea = _nodes[child1].AABB.Perimeter;
                    var newArea = aabb.Perimeter;
                    cost1 = newArea - oldArea + inheritanceCost;
                }

                // Cost of descending into child2
                float cost2;
                if (_nodes[child2].IsLeaf())
                {
                    var aabb = new AABB();
                    aabb.Combine(ref leafAABB, ref _nodes[child2].AABB);
                    cost2 = aabb.Perimeter + inheritanceCost;
                }
                else
                {
                    var aabb = new AABB();
                    aabb.Combine(ref leafAABB, ref _nodes[child2].AABB);
                    var oldArea = _nodes[child2].AABB.Perimeter;
                    var newArea = aabb.Perimeter;
                    cost2 = newArea - oldArea + inheritanceCost;
                }

                // Descend according to the minimum cost.
                if (cost < cost1 && cost1 < cost2)
                    break;

                // Descend
                if (cost1 < cost2)
                    index = child1;
                else
                    index = child2;
            }

            var sibling = index;

            // Create a new parent.
            var oldParent = _nodes[sibling].ParentOrNext;
            var newParent = AllocateNode();
            _nodes[newParent].ParentOrNext = oldParent;
            _nodes[newParent].UserData = default;
            _nodes[newParent].AABB.Combine(ref leafAABB, ref _nodes[sibling].AABB);
            _nodes[newParent].Height = _nodes[sibling].Height + 1;

            if (oldParent != NullNode)
            {
                // The sibling was not the root.
                if (_nodes[oldParent].Child1 == sibling)
                    _nodes[oldParent].Child1 = newParent;
                else
                    _nodes[oldParent].Child2 = newParent;

                _nodes[newParent].Child1 = sibling;
                _nodes[newParent].Child2 = leaf;
                _nodes[sibling].ParentOrNext = newParent;
                _nodes[leaf].ParentOrNext = newParent;
            }
            else
            {
                // The sibling was the root.
                _nodes[newParent].Child1 = sibling;
                _nodes[newParent].Child2 = leaf;
                _nodes[sibling].ParentOrNext = newParent;
                _nodes[leaf].ParentOrNext = newParent;
                _root = newParent;
            }

            // Walk back up the tree fixing heights and AABBs
            index = _nodes[leaf].ParentOrNext;
            while (index != NullNode)
            {
                index = Balance(index);

                var child1 = _nodes[index].Child1;
                var child2 = _nodes[index].Child2;

                Debug.Assert(child1 != NullNode);
                Debug.Assert(child2 != NullNode);

                _nodes[index].Height = 1 + Math.Max(_nodes[child1].Height, _nodes[child2].Height);
                _nodes[index].AABB.Combine(ref _nodes[child1].AABB, ref _nodes[child2].AABB);

                index = _nodes[index].ParentOrNext;
            }

            //Validate();
        }

        private void RemoveLeaf(int leaf)
        {
            if (leaf == _root)
            {
                _root = NullNode;
                return;
            }

            var parent = _nodes[leaf].ParentOrNext;
            var grandParent = _nodes[parent].ParentOrNext;
            int sibling;
            if (_nodes[parent].Child1 == leaf)
                sibling = _nodes[parent].Child2;
            else
                sibling = _nodes[parent].Child1;

            if (grandParent != NullNode)
            {
                // Destroy parent and connect sibling to grandParent.
                if (_nodes[grandParent].Child1 == parent)
                    _nodes[grandParent].Child1 = sibling;
                else
                    _nodes[grandParent].Child2 = sibling;
                _nodes[sibling].ParentOrNext = grandParent;
                FreeNode(parent);

                // Adjust ancestor bounds.
                var index = grandParent;
                while (index != NullNode)
                {
                    index = Balance(index);

                    var child1 = _nodes[index].Child1;
                    var child2 = _nodes[index].Child2;

                    _nodes[index].AABB.Combine(ref _nodes[child1].AABB, ref _nodes[child2].AABB);
                    _nodes[index].Height = 1 + Math.Max(_nodes[child1].Height, _nodes[child2].Height);

                    index = _nodes[index].ParentOrNext;
                }
            }
            else
            {
                _root = sibling;
                _nodes[sibling].ParentOrNext = NullNode;
                FreeNode(parent);
            }

            //Validate();
        }

        /// <summary>Perform a left or right rotation if node A is imbalanced.</summary>
        /// <param name="iA"></param>
        /// <returns>the new root index.</returns>
        private int Balance(int iA)
        {
            Debug.Assert(iA != NullNode);

            var A = _nodes[iA];
            if (A.IsLeaf() || A.Height < 2)
                return iA;

            var iB = A.Child1;
            var iC = A.Child2;
            Debug.Assert(0 <= iB && iB < _nodeCapacity);
            Debug.Assert(0 <= iC && iC < _nodeCapacity);

            var B = _nodes[iB];
            var C = _nodes[iC];

            var balance = C.Height - B.Height;

            // Rotate C up
            if (balance > 1)
            {
                var iF = C.Child1;
                var iG = C.Child2;
                var F = _nodes[iF];
                var G = _nodes[iG];
                Debug.Assert(0 <= iF && iF < _nodeCapacity);
                Debug.Assert(0 <= iG && iG < _nodeCapacity);

                // Swap A and C
                C.Child1 = iA;
                C.ParentOrNext = A.ParentOrNext;
                A.ParentOrNext = iC;

                // A's old parent should point to C
                if (C.ParentOrNext != NullNode)
                {
                    if (_nodes[C.ParentOrNext].Child1 == iA)
                        _nodes[C.ParentOrNext].Child1 = iC;
                    else
                    {
                        Debug.Assert(_nodes[C.ParentOrNext].Child2 == iA);
                        _nodes[C.ParentOrNext].Child2 = iC;
                    }
                }
                else
                    _root = iC;

                // Rotate
                if (F.Height > G.Height)
                {
                    C.Child2 = iF;
                    A.Child2 = iG;
                    G.ParentOrNext = iA;
                    A.AABB.Combine(ref B.AABB, ref G.AABB);
                    C.AABB.Combine(ref A.AABB, ref F.AABB);

                    A.Height = 1 + Math.Max(B.Height, G.Height);
                    C.Height = 1 + Math.Max(A.Height, F.Height);
                }
                else
                {
                    C.Child2 = iG;
                    A.Child2 = iF;
                    F.ParentOrNext = iA;
                    A.AABB.Combine(ref B.AABB, ref F.AABB);
                    C.AABB.Combine(ref A.AABB, ref G.AABB);

                    A.Height = 1 + Math.Max(B.Height, F.Height);
                    C.Height = 1 + Math.Max(A.Height, G.Height);
                }

                return iC;
            }

            // Rotate B up
            if (balance < -1)
            {
                var iD = B.Child1;
                var iE = B.Child2;
                var D = _nodes[iD];
                var E = _nodes[iE];
                Debug.Assert(0 <= iD && iD < _nodeCapacity);
                Debug.Assert(0 <= iE && iE < _nodeCapacity);

                // Swap A and B
                B.Child1 = iA;
                B.ParentOrNext = A.ParentOrNext;
                A.ParentOrNext = iB;

                // A's old parent should point to B
                if (B.ParentOrNext != NullNode)
                {
                    if (_nodes[B.ParentOrNext].Child1 == iA)
                        _nodes[B.ParentOrNext].Child1 = iB;
                    else
                    {
                        Debug.Assert(_nodes[B.ParentOrNext].Child2 == iA);
                        _nodes[B.ParentOrNext].Child2 = iB;
                    }
                }
                else
                    _root = iB;

                // Rotate
                if (D.Height > E.Height)
                {
                    B.Child2 = iD;
                    A.Child1 = iE;
                    E.ParentOrNext = iA;
                    A.AABB.Combine(ref C.AABB, ref E.AABB);
                    B.AABB.Combine(ref A.AABB, ref D.AABB);

                    A.Height = 1 + Math.Max(C.Height, E.Height);
                    B.Height = 1 + Math.Max(A.Height, D.Height);
                }
                else
                {
                    B.Child2 = iE;
                    A.Child1 = iD;
                    D.ParentOrNext = iA;
                    A.AABB.Combine(ref C.AABB, ref D.AABB);
                    B.AABB.Combine(ref A.AABB, ref E.AABB);

                    A.Height = 1 + Math.Max(C.Height, D.Height);
                    B.Height = 1 + Math.Max(A.Height, E.Height);
                }

                return iB;
            }

            return iA;
        }

        /// <summary>Compute the height of a sub-tree.</summary>
        /// <param name="nodeId">The node id to use as parent.</param>
        /// <returns>The height of the tree.</returns>
        public int ComputeHeight(int nodeId)
        {
            Debug.Assert(0 <= nodeId && nodeId < _nodeCapacity);
            var node = _nodes[nodeId];

            if (node.IsLeaf())
                return 0;

            var height1 = ComputeHeight(node.Child1);
            var height2 = ComputeHeight(node.Child2);
            return 1 + Math.Max(height1, height2);
        }

        /// <summary>Compute the height of the entire tree.</summary>
        /// <returns>The height of the tree.</returns>
        public int ComputeHeight()
        {
            var height = ComputeHeight(_root);
            return height;
        }

        public void ValidateStructure(int index)
        {
            if (index == NullNode)
                return;

            if (index == _root)
                Debug.Assert(_nodes[index].ParentOrNext == NullNode);

            var node = _nodes[index];

            var child1 = node.Child1;
            var child2 = node.Child2;

            if (node.IsLeaf())
            {
                Debug.Assert(child1 == NullNode);
                Debug.Assert(child2 == NullNode);
                Debug.Assert(node.Height == 0);
                return;
            }

            Debug.Assert(0 <= child1 && child1 < _nodeCapacity);
            Debug.Assert(0 <= child2 && child2 < _nodeCapacity);

            Debug.Assert(_nodes[child1].ParentOrNext == index);
            Debug.Assert(_nodes[child2].ParentOrNext == index);

            ValidateStructure(child1);
            ValidateStructure(child2);
        }

        public void ValidateMetrics(int index)
        {
            if (index == NullNode)
                return;

            var node = _nodes[index];

            var child1 = node.Child1;
            var child2 = node.Child2;

            if (node.IsLeaf())
            {
                Debug.Assert(child1 == NullNode);
                Debug.Assert(child2 == NullNode);
                Debug.Assert(node.Height == 0);
                return;
            }

            Debug.Assert(0 <= child1 && child1 < _nodeCapacity);
            Debug.Assert(0 <= child2 && child2 < _nodeCapacity);

            var height1 = _nodes[child1].Height;
            var height2 = _nodes[child2].Height;
            var height = 1 + Math.Max(height1, height2);
            Debug.Assert(node.Height == height);

            var AABB = new AABB();
            AABB.Combine(ref _nodes[child1].AABB, ref _nodes[child2].AABB);

            Debug.Assert(AABB.LowerBound == node.AABB.LowerBound);
            Debug.Assert(AABB.UpperBound == node.AABB.UpperBound);

            ValidateMetrics(child1);
            ValidateMetrics(child2);
        }

        /// <summary>Validate this tree. For testing.</summary>
        public void Validate()
        {
            ValidateStructure(_root);
            ValidateMetrics(_root);

            var freeCount = 0;
            var freeIndex = _freeList;
            while (freeIndex != NullNode)
            {
                Debug.Assert(0 <= freeIndex && freeIndex < _nodeCapacity);
                freeIndex = _nodes[freeIndex].ParentOrNext;
                ++freeCount;
            }

            Debug.Assert(Height == ComputeHeight());

            Debug.Assert(_nodeCount + freeCount == _nodeCapacity);
        }

        /// <summary>Build an optimal tree. Very expensive. For testing.</summary>
        public void RebuildBottomUp()
        {
            var nodes = new int[_nodeCount];
            var count = 0;

            // Build array of leaves. Free the rest.
            for (var i = 0; i < _nodeCapacity; ++i)
            {
                if (_nodes[i].Height < 0)
                {
                    // free node in pool
                    continue;
                }

                if (_nodes[i].IsLeaf())
                {
                    _nodes[i].ParentOrNext = NullNode;
                    nodes[count] = i;
                    ++count;
                }
                else
                    FreeNode(i);
            }

            while (count > 1)
            {
                var minCost = MathConstants.MaxFloat;
                int iMin = -1, jMin = -1;
                for (var i = 0; i < count; ++i)
                {
                    var AABBi = _nodes[nodes[i]].AABB;

                    for (var j = i + 1; j < count; ++j)
                    {
                        var AABBj = _nodes[nodes[j]].AABB;
                        var b = new AABB();
                        b.Combine(ref AABBi, ref AABBj);
                        var cost = b.Perimeter;
                        if (cost < minCost)
                        {
                            iMin = i;
                            jMin = j;
                            minCost = cost;
                        }
                    }
                }

                var index1 = nodes[iMin];
                var index2 = nodes[jMin];
                var child1 = _nodes[index1];
                var child2 = _nodes[index2];

                var parentIndex = AllocateNode();
                var parent = _nodes[parentIndex];
                parent.Child1 = index1;
                parent.Child2 = index2;
                parent.Height = 1 + Math.Max(child1.Height, child2.Height);
                parent.AABB.Combine(ref child1.AABB, ref child2.AABB);
                parent.ParentOrNext = NullNode;

                child1.ParentOrNext = parentIndex;
                child2.ParentOrNext = parentIndex;

                nodes[jMin] = nodes[count - 1];
                nodes[iMin] = parentIndex;
                --count;
            }

            _root = nodes[0];

            Validate();
        }

        /// <summary>Shift the origin of the nodes</summary>
        /// <param name="newOrigin">The displacement to use.</param>
        public void ShiftOrigin(ref Vector2 newOrigin)
        {
            // Build array of leaves. Free the rest.
            for (var i = 0; i < _nodeCapacity; ++i)
            {
                _nodes[i].AABB.LowerBound -= newOrigin;
                _nodes[i].AABB.UpperBound -= newOrigin;
            }
        }
    }
}