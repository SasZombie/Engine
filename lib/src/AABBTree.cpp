#include "AABBTree.hpp"

// sas::AABB2D sas::computeFatAABB2D(const Body2D &body, float margin) noexcept
// {
// }

template<typename T>
void sas::AABBTree<T>::insert(uint32_t bodyID, const AABB &aabb) noexcept
{

    NodeType *leaf = new NodeType();
    leaf->objectID = bodyID;
    leaf->aabb = aabb;

    leafMap[bodyID] = leaf;

    if (!root)
    {
        root = leaf;
        return;
    }

    NodeType *sibling = root;
    while (!sibling->isLeaf())
    {
        float area0 = sibling->children[0]->aabb.aabbUnion(leaf->aabb).getArea();
        float area1 = sibling->children[1]->aabb.aabbUnion(leaf->aabb).getArea();

        if (area0 < area1)
            sibling = sibling->children[0];
        else
            sibling = sibling->children[1];
    }

    NodeType *oldParent = sibling->parent;
    NodeType *newParent = new NodeType();
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

    NodeType *walk = newParent;
    while (walk != nullptr)
    {
        walk->aabb = walk->children[0]->aabb.aabbUnion(walk->children[1]->aabb);
        walk = walk->parent;
    }
}

template<typename T>
void sas::AABBTree<T>::remove(NodeType *leaf) noexcept
{
    if (leaf == root)
    {
        root = nullptr;
        return;
    }

    NodeType *parent = leaf->parent;
    NodeType *grandParent = parent->parent;
    NodeType *sibling = (parent->children[0] == leaf) ? parent->children[1] : parent->children[0];

    if (grandParent)
    {
        if (grandParent->children[0] == parent)
            grandParent->children[0] = sibling;
        else
            grandParent->children[1] = sibling;

        sibling->parent = grandParent;
        delete parent;

        NodeType *walk = grandParent;
        while (walk)
        {
            walk->aabb = walk->children[0]->aabb.aabbUnion(walk->children[1]->aabb);
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
template<typename T>
void sas::AABBTree<T>::remove(uint32_t id) noexcept
{
    NodeType *leaf = leafMap[id];
    remove(leaf);

    leafMap.erase(id);

    delete leaf;
}

template<typename T>
void sas::AABBTree<T>::updateObject(const Body &body, float margin) noexcept
{

    AABB actual = T::computeTight(body);

    NodeType *curNode = leafMap[body.bodyID];
    if (actual.minX < curNode->aabb.minX || actual.maxX > curNode->aabb.maxX ||
        actual.minY < curNode->aabb.minY || actual.maxY > curNode->aabb.maxY)
    {

        remove(body.bodyID);
        insert(body.bodyID, T::computeFat(body, margin));
    }
}
template<typename T>
void sas::AABBTree<T>::clear(NodeType *node) noexcept
{
    if (!node)
        return;

    clear(node->children[0]);
    clear(node->children[1]);

    delete node;
}

template<typename T>
void sas::AABBTree<T>::clear() noexcept
{
    clear(root);
    root = nullptr;
    leafMap.clear();
}

// sas::AABB3D sas::computeFatAABB3D(const Body3D &body, float margin) noexcept
// {
    
// }

template class sas::AABBTree<sas::TreeTraits<sas::AABB2D>>;
template class sas::AABBTree<sas::TreeTraits<sas::AABB3D>>;