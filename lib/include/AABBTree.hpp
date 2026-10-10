#pragma once

#include <unordered_map>
#include <vector>
#include <functional>

#include "Body2D.hpp"

namespace sas
{
    struct AABB2D
    {
        float minX, minY;
        float maxX, maxY;
    };

    AABB2D computeFatAABB(const Body2D &body, float margin = 10.f) noexcept;

    bool AABBOverlap(const AABB2D &a, const AABB2D &b) noexcept;

    AABB2D AABBUnion(const AABB2D &a, const AABB2D &b) noexcept;
    AABB2D computeTightAABB(const Body2D &body) noexcept;

    float getAreaAABB(const AABB2D &a) noexcept;

    using DrawCallback = std::function<void(const AABB2D&, bool isLeaf)>;
    
    struct Node
    {
        AABB2D aabb;

        Node *parent = nullptr;
        Node *children[2] = {nullptr, nullptr};

        int objectID = -1;

        bool isLeaf() const noexcept
        {
            return children[0] == nullptr;
        }

        void Draw(const DrawCallback& cb) const;
    };

    class AABBTree2D
    {
    private:
        Node *root = nullptr;
        std::unordered_map<uint32_t, Node *> leafMap;
        void clear(Node *node) noexcept;
        void remove(Node *leaf) noexcept;
        
    public:
        void insert(uint32_t bodyID, const AABB2D& aabb) noexcept;

        void query(const AABB2D &targetAABB, std::vector<uint32_t> &results) const noexcept;
        void query(Node *node, const AABB2D &targetAABB, std::vector<uint32_t> &results) const noexcept;

        void remove(uint32_t id) noexcept;

        void updateObject(const Body2D &body, float margin = 0.f) noexcept;

        void draw(const DrawCallback& cb) const;

        void clear() noexcept;
        
        ~AABBTree2D() noexcept
        {
            clear(root);
        }
    };

} // namespace sas
