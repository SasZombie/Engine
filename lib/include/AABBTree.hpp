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

    AABB computeFatAABB(const Body &body, float margin = 10.f) noexcept;

    bool AABBOverlap(const AABB &a, const AABB &b) noexcept;

    AABB AABBUnion(const AABB &a, const AABB &b) noexcept;
    AABB computeTightAABB(const Body &body) noexcept;

    float getAreaAABB(const AABB &a) noexcept;

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
        void clear(Node *node) noexcept;
        void remove(Node *leaf) noexcept;
        
    public:
        void insert(uint32_t bodyID, const AABB& aabb) noexcept;

        void query(const AABB &targetAABB, std::vector<uint32_t> &results) const noexcept;
        void query(Node *node, const AABB &targetAABB, std::vector<uint32_t> &results) const noexcept;

        void remove(uint32_t id) noexcept;

        void updateObject(const Body &body, float margin = 0.f) noexcept;

        void draw(const DrawCallback& cb) const;

        void clear() noexcept;
        
        ~AABBTree() noexcept
        {
            clear(root);
        }
    };

} // namespace sas
