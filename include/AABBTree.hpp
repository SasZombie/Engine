#pragma once

#include <unordered_map>
#include <vector>

#include "Shape.hpp"
namespace sas
{
    struct AABB
    {
        float minX, minY;
        float maxX, maxY;
    };

    AABB ComputeFatAABB(const Body &body) noexcept;

    bool AABBOverlap(const AABB &a, const AABB &b) noexcept;

    AABB AABBUnion(const AABB &a, const AABB &b) noexcept;

    float GetAreaAABB(const AABB &a) noexcept;

    struct Node
    {

        AABB aabb;

        Node *parent = nullptr;
        Node *children[2] = {nullptr, nullptr};

        int objectID = -1;

        bool isLeaf() const noexcept
        {
            return children[0] == nullptr;
        }
    };

    class AABBTree
    {
    private:
        Node *root = nullptr;
        std::unordered_map<uint32_t, Node *> leafMap;
        void Clear(Node *node) noexcept;

    public:
        void insert(const Body &c) noexcept;

        void Query(Node *node, const AABB &targetAABB, std::vector<uint32_t> &results) const noexcept;

        void remove(Node *leaf) noexcept;

        void UpdateObject(const Body &body) noexcept;


        ~AABBTree() noexcept
        {
            Clear(root);
        }
    };

} // namespace sas
