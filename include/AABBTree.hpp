#pragma once

#include <unordered_map>
#include <vector>
#include <functional>

#include "Body.hpp"

namespace sas
{
    struct AABB
    {
        float minX, minY;
        float maxX, maxY;
    };

    AABB ComputeFatAABB(const Body &body, float margin = 10.f) noexcept;

    bool AABBOverlap(const AABB &a, const AABB &b) noexcept;

    AABB AABBUnion(const AABB &a, const AABB &b) noexcept;
    AABB ComputeTightAABB(const Body &body) noexcept;

    float GetAreaAABB(const AABB &a) noexcept;

    using DrawCallback = std::function<void(const AABB&, bool isLeaf)>;
    
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

        void Draw(const DrawCallback& cb) const;
    };

    class AABBTree
    {
    private:
        Node *root = nullptr;
        std::unordered_map<uint32_t, Node *> leafMap;
        void Clear(Node *node) noexcept;
        void remove(Node *leaf) noexcept;
        
    public:
        void insert(uint32_t bodyID, const AABB& aabb) noexcept;

        void Query(const AABB &targetAABB, std::vector<uint32_t> &results) const noexcept;
        void Query(Node *node, const AABB &targetAABB, std::vector<uint32_t> &results) const noexcept;

        void remove(uint32_t id) noexcept;

        void UpdateObject(const Body &body, float margin = 0.f) noexcept;

        void Draw(const DrawCallback& cb) const;

        void Clear() noexcept;
        
        ~AABBTree() noexcept
        {
            Clear(root);
        }
    };

} // namespace sas
