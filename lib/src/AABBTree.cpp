#include "AABBTree.hpp"

sas::AABB2D sas::computeFatAABB2D(const Body2D &body, float margin) noexcept
{
    float x = body.transform.position.x;
    float y = body.transform.position.y;
    float angle = body.transform.rotation;

    float minX, minY, maxX, maxY;

    if (body.shape.type == ShapeType2D::Circle)
    {
        // Rotation for circle is trivial
        minX = x - body.shape.radius * body.transform.scale.x;
        minY = y - body.shape.radius * body.transform.scale.y;
        maxX = x + body.shape.radius * body.transform.scale.x;
        maxY = y + body.shape.radius * body.transform.scale.y;
    }
    else
    {
        float hx = body.shape.halfSize.x * body.transform.scale.x;
        float hy = body.shape.halfSize.y * body.transform.scale.y;

        float c = std::abs(std::cos(angle));
        float s = std::abs(std::sin(angle));

        float newHx = hx * c + hy * s;
        float newHy = hx * s + hy * c;

        minX = x - newHx;
        minY = y - newHy;
        maxX = x + newHx;
        maxY = y + newHy;
    }

    return {minX - margin, minY - margin, maxX + margin, maxY + margin};
}

sas::AABB2D sas::computeTightAABB2D(const Body2D &body) noexcept
{
    return computeFatAABB2D(body, 0.0f);
}

bool sas::AABBOverlap2D(const AABB2D &a, const AABB2D &b) noexcept
{
    return (a.minX <= b.maxX && a.maxX >= b.minX) &&
           (a.minY <= b.maxY && a.maxY >= b.minY);
}

sas::AABB2D sas::AABBUnion2D(const AABB2D &a, const AABB2D &b) noexcept
{
    return {
        std::min(a.minX, b.minX), std::min(a.minY, b.minY),
        std::max(a.maxX, b.maxX), std::max(a.maxY, b.maxY)};
}

float sas::getAreaAABB2D(const AABB2D &a) noexcept
{
    float width = a.maxX - a.minX;
    float height = a.maxY - a.minY;

    return width * height;
}

void sas::AABBTree2D::insert(uint32_t bodyID, const AABB2D &aabb) noexcept
{

    Node2D *leaf = new Node2D();
    leaf->objectID = bodyID;
    leaf->aabb = aabb;

    leafMap[bodyID] = leaf;

    if (!root)
    {
        root = leaf;
        return;
    }

    Node2D *sibling = root;
    while (!sibling->isLeaf())
    {
        float area0 = getAreaAABB2D(AABBUnion2D(sibling->children[0]->aabb, leaf->aabb));
        float area1 = getAreaAABB2D(AABBUnion2D(sibling->children[1]->aabb, leaf->aabb));

        if (area0 < area1)
            sibling = sibling->children[0];
        else
            sibling = sibling->children[1];
    }

    Node2D *oldParent = sibling->parent;
    Node2D *newParent = new Node2D();
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

    Node2D *walk = newParent;
    while (walk != nullptr)
    {
        walk->aabb = AABBUnion2D(walk->children[0]->aabb, walk->children[1]->aabb);
        walk = walk->parent;
    }
}

void sas::AABBTree2D::query(Node2D *node, const AABB2D &targetAABB, std::vector<uint32_t> &results) const noexcept
{
    if (!node || !AABBOverlap2D(node->aabb, targetAABB))
    {
        return;
    }

    if (node->isLeaf())
    {
        results.push_back(node->objectID);
    }
    else
    {
        query(node->children[0], targetAABB, results);
        query(node->children[1], targetAABB, results);
    }
}

void sas::AABBTree2D::query(const AABB2D &targetAABB, std::vector<uint32_t> &results) const noexcept
{
    query(root, targetAABB, results);
}

void sas::AABBTree2D::remove(Node2D *leaf) noexcept
{
    if (leaf == root)
    {
        root = nullptr;
        return;
    }

    Node2D *parent = leaf->parent;
    Node2D *grandParent = parent->parent;

    Node2D *sibling = (parent->children[0] == leaf) ? parent->children[1] : parent->children[0];

    if (grandParent)
    {
        if (grandParent->children[0] == parent)
            grandParent->children[0] = sibling;
        else
            grandParent->children[1] = sibling;

        sibling->parent = grandParent;
        delete parent;

        Node2D *walk = grandParent;
        while (walk)
        {
            walk->aabb = AABBUnion2D(walk->children[0]->aabb, walk->children[1]->aabb);
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

void sas::AABBTree2D::remove(uint32_t id) noexcept
{
    Node2D *leaf = leafMap[id];
    remove(leaf);

    leafMap.erase(id);

    delete leaf;
}

void sas::AABBTree2D::updateObject(const Body2D &body, float margin) noexcept
{

    AABB2D actual = computeTightAABB2D(body);

    Node2D *curNode = leafMap[body.bodyID];
    if (actual.minX < curNode->aabb.minX || actual.maxX > curNode->aabb.maxX ||
        actual.minY < curNode->aabb.minY || actual.maxY > curNode->aabb.maxY)
    {

        remove(body.bodyID);
        insert(body.bodyID, computeFatAABB2D(body, margin));
    }
}

void sas::AABBTree2D::clear(Node2D *node) noexcept
{
    if (!node)
        return;

    clear(node->children[0]);
    clear(node->children[1]);

    delete node;
}

void sas::Node2D::Draw(const DrawCallback2D &cb) const
{
    cb(aabb, isLeaf());

    if (children[0])
        children[0]->Draw(cb);
    if (children[1])
        children[1]->Draw(cb);
}

void sas::AABBTree2D::draw(const DrawCallback2D &cb) const
{
    if (root != nullptr)
    {
        root->Draw(cb);
    }
}

void sas::AABBTree2D::clear() noexcept
{
    clear(root);
    root = nullptr;
    leafMap.clear();
}

sas::AABB3D sas::computeFatAABB3D(const Body3D &body, float margin) noexcept
{
    const float x = body.transform.position.x;
    const float y = body.transform.position.y;
    const float z = body.transform.position.z;
    const auto& rot = body.transform.rotMatrix;

    // Ignore rotation entirely
    if (body.shape.type == ShapeType3D::Sphere)
    {
        const float rx = body.shape.radius * body.transform.scale.x + margin;
        const float ry = body.shape.radius * body.transform.scale.y + margin;
        const float rz = body.shape.radius * body.transform.scale.z + margin;

        return {x - rx, y - ry, z - rz, x + rx, y + ry, z + rz};
    }

    float extX, extY, extZ;

    if (body.shape.type == ShapeType3D::Orthotope)
    {
        const float hx = body.shape.halfSize.x * body.transform.scale.x;
        const float hy = body.shape.halfSize.y * body.transform.scale.y;
        const float hz = body.shape.halfSize.z * body.transform.scale.z;

        extX = std::abs(rot(0, 0)) * hx + std::abs(rot(0, 1)) * hy + std::abs(rot(0,2)) * hz + margin;
        extY = std::abs(rot(1, 0)) * hx + std::abs(rot(1, 1)) * hy + std::abs(rot(1,2)) * hz + margin;
        extZ = std::abs(rot(2, 0)) * hx + std::abs(rot(2, 1)) * hy + std::abs(rot(2,2)) * hz + margin;
    }
    else 
    {
        const float r = body.shape.radius * std::max(body.transform.scale.x, body.transform.scale.y);
        const float hz = body.shape.halfSize.z * body.transform.scale.z;

        const float dirX = rot(0, 2);
        const float dirY = rot(1, 2);
        const float dirZ = rot(2, 2);

        extX = std::abs(dirX) * hz + r * std::sqrt(std::max(0.0f, 1.0f - dirX * dirX)) + margin;
        extY = std::abs(dirY) * hz + r * std::sqrt(std::max(0.0f, 1.0f - dirY * dirY)) + margin;
        extZ = std::abs(dirZ) * hz + r * std::sqrt(std::max(0.0f, 1.0f - dirZ * dirZ)) + margin;
    }

    return {x - extX, y - extY, z - extZ, x + extX, y + extY, z + extZ};
}

sas::AABB3D sas::computeTightAABB3D(const Body3D &body) noexcept
{
    return computeFatAABB3D(body, 0.0f);
}

bool sas::AABBOverlap3D(const AABB3D &a, const AABB3D &b) noexcept
{
    return (a.minX <= b.maxX && a.maxX >= b.minX) &&
           (a.minY <= b.maxY && a.maxY >= b.minY) &&
           (a.minZ <= b.maxZ && a.maxZ >= b.minZ);
}

sas::AABB3D sas::AABBUnion3D(const AABB3D &a, const AABB3D &b) noexcept
{
    return {
        std::min(a.minX, b.minX), std::min(a.minY, b.minY), std::min(a.minZ, b.minZ),
        std::max(a.maxX, b.maxX), std::max(a.maxY, b.maxY), std::max(a.maxZ, b.maxZ)};
}

float sas::getAreaAABB3D(const AABB3D &a) noexcept
{
    float w = a.maxX - a.minX;
    float h = a.maxY - a.minY;
    float l = a.maxZ - a.minZ;
    return 2.0f * (w * h + h * l + l * w);
}

void sas::AABBTree3D::insert(uint32_t bodyID, const AABB3D &aabb) noexcept
{

    Node3D *leaf = new Node3D();
    leaf->objectID = bodyID;
    leaf->aabb = aabb;

    leafMap[bodyID] = leaf;

    if (!root)
    {
        root = leaf;
        return;
    }

    Node3D *sibling = root;
    while (!sibling->isLeaf())
    {
        float area0 = getAreaAABB3D(AABBUnion3D(sibling->children[0]->aabb, leaf->aabb));
        float area1 = getAreaAABB3D(AABBUnion3D(sibling->children[1]->aabb, leaf->aabb));

        if (area0 < area1)
            sibling = sibling->children[0];
        else
            sibling = sibling->children[1];
    }

    Node3D *oldParent = sibling->parent;
    Node3D *newParent = new Node3D();
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

    Node3D *walk = newParent;
    while (walk != nullptr)
    {
        walk->aabb = AABBUnion3D(walk->children[0]->aabb, walk->children[1]->aabb);
        walk = walk->parent;
    }
}

void sas::AABBTree3D::query(Node3D *node, const AABB3D &targetAABB, std::vector<uint32_t> &results) const noexcept
{
    if (!node || !AABBOverlap3D(node->aabb, targetAABB))
    {
        return;
    }

    if (node->isLeaf())
    {
        results.push_back(node->objectID);
    }
    else
    {
        query(node->children[0], targetAABB, results);
        query(node->children[1], targetAABB, results);
    }
}

void sas::AABBTree3D::query(const AABB3D &targetAABB, std::vector<uint32_t> &results) const noexcept
{
    query(root, targetAABB, results);
}

void sas::AABBTree3D::remove(Node3D *leaf) noexcept
{
    if (leaf == root)
    {
        root = nullptr;
        return;
    }

    Node3D *parent = leaf->parent;
    Node3D *grandParent = parent->parent;

    Node3D *sibling = (parent->children[0] == leaf) ? parent->children[1] : parent->children[0];

    if (grandParent)
    {
        if (grandParent->children[0] == parent)
            grandParent->children[0] = sibling;
        else
            grandParent->children[1] = sibling;

        sibling->parent = grandParent;
        delete parent;

        Node3D *walk = grandParent;
        while (walk)
        {
            walk->aabb = AABBUnion3D(walk->children[0]->aabb, walk->children[1]->aabb);
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

void sas::AABBTree3D::remove(uint32_t id) noexcept
{
    Node3D *leaf = leafMap[id];
    remove(leaf);

    leafMap.erase(id);

    delete leaf;
}

void sas::AABBTree3D::updateObject(const Body3D &body, float margin) noexcept
{

    AABB3D actual = computeTightAABB3D(body);

    Node3D *curNode = leafMap[body.bodyID];
    if (actual.minX < curNode->aabb.minX || actual.maxX > curNode->aabb.maxX ||
        actual.minY < curNode->aabb.minY || actual.maxY > curNode->aabb.maxY)
    {

        remove(body.bodyID);
        insert(body.bodyID, computeFatAABB3D(body, margin));
    }
}

void sas::AABBTree3D::clear(Node3D *node) noexcept
{
    if (!node)
        return;

    clear(node->children[0]);
    clear(node->children[1]);

    delete node;
}

void sas::Node3D::Draw(const DrawCallback3D &cb) const
{
    cb(aabb, isLeaf());

    if (children[0])
        children[0]->Draw(cb);
    if (children[1])
        children[1]->Draw(cb);
}

void sas::AABBTree3D::draw(const DrawCallback3D &cb) const
{
    if (root != nullptr)
    {
        root->Draw(cb);
    }
}

void sas::AABBTree3D::clear() noexcept
{
    clear(root);
    root = nullptr;
    leafMap.clear();
}
