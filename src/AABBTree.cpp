#include "AABBTree.hpp"

#include <iostream>

sas::AABB sas::ComputeFatAABB(const Body &body, float margin) noexcept
{
    // TODO: Add support for square
    return {
        body.transform.position.x - body.shape.radius - margin,
        body.transform.position.y - body.shape.radius - margin,
        body.transform.position.x + body.shape.radius + margin,
        body.transform.position.y + body.shape.radius + margin};
}

sas::AABB sas::ComputeTightAABB(const Body &body) noexcept
{
    return ComputeFatAABB(body, 0.0f);
}

bool sas::AABBOverlap(const AABB &a, const AABB &b) noexcept
{
    return (a.minX <= b.maxX && a.maxX >= b.minX) &&
           (a.minY <= b.maxY && a.maxY >= b.minY);
}

sas::AABB sas::AABBUnion(const AABB &a, const AABB &b) noexcept
{
    return {
        std::min(a.minX, b.minX), std::min(a.minY, b.minY),
        std::max(a.maxX, b.maxX), std::max(a.maxY, b.maxY)};
}

float sas::GetAreaAABB(const AABB &a) noexcept
{
    float width = a.maxX - a.minX;
    float height = a.maxY - a.minY;

    return width * height;
}

void sas::AABBTree::insert(uint32_t bodyID, const AABB &aabb) noexcept
{
    Node *leaf = new Node();
    leaf->objectID = bodyID;
    leaf->aabb = aabb;

    leafMap[bodyID] = leaf;

    if (!root)
    {
        root = leaf;
        return;
    }

    Node *sibling = root;
    while (!sibling->isLeaf())
    {
        float area0 = GetAreaAABB(AABBUnion(sibling->children[0]->aabb, leaf->aabb));
        float area1 = GetAreaAABB(AABBUnion(sibling->children[1]->aabb, leaf->aabb));

        if (area0 < area1)
            sibling = sibling->children[0];
        else
            sibling = sibling->children[1];
    }

    Node *oldParent = sibling->parent;
    Node *newParent = new Node();
    newParent->parent = oldParent;

    if (oldParent)
    {
        if (oldParent->children[0] == sibling)
            oldParent->children[0] = newParent;
        else
            oldParent->children[1] = newParent;
    }
    else
    {
        root = newParent;
    }

    newParent->children[0] = sibling;
    newParent->children[1] = leaf;
    sibling->parent = newParent;
    leaf->parent = newParent;

    Node *walk = newParent;
    while (walk != nullptr)
    {
        walk->aabb = AABBUnion(walk->children[0]->aabb, walk->children[1]->aabb);
        walk = walk->parent;
    }
}

void sas::AABBTree::Query(Node *node, const AABB &targetAABB, std::vector<uint32_t> &results) const noexcept
{
    if (!node || !AABBOverlap(node->aabb, targetAABB))
    {
        return;
    }

    if (node->isLeaf())
    {
        results.push_back(node->objectID);
    }
    else
    {
        Query(node->children[0], targetAABB, results);
        Query(node->children[1], targetAABB, results);
    }
}

void sas::AABBTree::Query(const AABB &targetAABB, std::vector<uint32_t> &results) const noexcept
{
    Query(root, targetAABB, results);
}

void sas::AABBTree::remove(Node *leaf) noexcept
{
    if (leaf == root)
    {
        root = nullptr;
        return;
    }

    Node *parent = leaf->parent;
    Node *grandParent = parent->parent;

    Node *sibling = (parent->children[0] == leaf) ? parent->children[1] : parent->children[0];

    if (grandParent)
    {
        if (grandParent->children[0] == parent)
            grandParent->children[0] = sibling;
        else
            grandParent->children[1] = sibling;

        sibling->parent = grandParent;
        delete parent;

        Node *walk = grandParent;
        while (walk)
        {
            walk->aabb = AABBUnion(walk->children[0]->aabb, walk->children[1]->aabb);
            walk = walk->parent;
        }
    }
    else
    {
        root = sibling;
        sibling->parent = nullptr;
        delete parent;
    }
}

void sas::AABBTree::internal_remove_node(Node *leaf) noexcept
{
    if (leaf == root)
    {
        root = nullptr;
        return;
    }

    Node *parent = leaf->parent;
    Node *grandParent = parent->parent;
    Node *sibling = (parent->children[0] == leaf) ? parent->children[1] : parent->children[0];

    if (grandParent)
    {
        if (grandParent->children[0] == parent) grandParent->children[0] = sibling;
        else grandParent->children[1] = sibling;

        sibling->parent = grandParent;
        
        // Update AABBs up the tree
        Node *walk = grandParent;
        while (walk)
        {
            walk->aabb = AABBUnion(walk->children[0]->aabb, walk->children[1]->aabb);
            walk = walk->parent;
        }
    }
    else
    {
        root = sibling;
        sibling->parent = nullptr;
    }

    // IMPORTANT: The leaf is now detached. 
    // We delete the PARENT (the internal node) because it's no longer needed.
    delete parent; 
}

void sas::AABBTree::remove(uint32_t id) noexcept
{
    auto it = leafMap.find(id);
    if(it != leafMap.end())
    {
        Node* leaf = it->second;
        internal_remove_node(leaf); 
        
        leafMap.erase(it);

        delete leaf; 
    }
}

void sas::AABBTree::UpdateObject(const Body &body, float margin) noexcept
{
    if (leafMap.find(body.bodyID) == leafMap.end())
        return;
    float cx = body.transform.position.x;
    float cy = body.transform.position.y;
    float rad = body.shape.radius;

    AABB actual = {cx - rad, cy - rad, cx + rad, cy + rad};

    Node *curNode = leafMap[body.bodyID];
    if (actual.minX < curNode->aabb.minX || actual.maxX > curNode->aabb.maxX ||
        actual.minY < curNode->aabb.minY || actual.maxY > curNode->aabb.maxY)
    {

        remove(curNode);
        delete curNode;
        insert(body.bodyID, ComputeFatAABB(body, margin));
    }
}

void sas::AABBTree::Clear(Node *node) noexcept
{
    if (!node)
        return;

    Clear(node->children[0]);
    Clear(node->children[1]);

    delete node;
}

void sas::Node::Draw(const DrawCallback &cb) const
{
    cb(aabb, isLeaf());

    if (children[0])
        children[0]->Draw(cb);
    if (children[1])
        children[1]->Draw(cb);
}

void sas::AABBTree::Draw(const DrawCallback &cb) const
{
    root->Draw(cb);
}

void sas::AABBTree::Clear() noexcept
{
    Clear(root);
    root = nullptr;
    leafMap.clear();
}
