#pragma once

#include <unordered_map>
#include <vector>
#include <functional>

#include "Body.hpp"

namespace sas
{
    struct AABB2D
    {
        float minX, minY;
        float maxX, maxY;
    };

    [[nodiscard]] AABB2D computeFatAABB2D(const Body2D &body, float margin = 10.f) noexcept;

    [[nodiscard]] bool AABBOverlap2D(const AABB2D &a, const AABB2D &b) noexcept;

    [[nodiscard]] AABB2D AABBUnion2D(const AABB2D &a, const AABB2D &b) noexcept;
    [[nodiscard]] AABB2D computeTightAABB2D(const Body2D &body) noexcept;

    [[nodiscard]] float getAreaAABB2D(const AABB2D &a) noexcept;

    using DrawCallback2D = std::function<void(const AABB2D&, bool isLeaf)>;
    
    struct Node2D
    {
        AABB2D aabb;

        Node2D *parent = nullptr;
        Node2D *children[2] = {nullptr, nullptr};

        int objectID = -1;

        bool isLeaf() const noexcept
        {
            return children[0] == nullptr;
        }

        void Draw(const DrawCallback2D& cb) const;
    };

    class AABBTree2D
    {
    private:
        Node2D *root = nullptr;
        std::unordered_map<uint32_t, Node2D *> leafMap;
        void clear(Node2D *node) noexcept;
        void remove(Node2D *leaf) noexcept;
        
    public:
        void insert(uint32_t bodyID, const AABB2D& aabb) noexcept;

        void query(const AABB2D &targetAABB, std::vector<uint32_t> &results) const noexcept;
        void query(Node2D *node, const AABB2D &targetAABB, std::vector<uint32_t> &results) const noexcept;

        void remove(uint32_t id) noexcept;

        void updateObject(const Body2D &body, float margin = 0.f) noexcept;

        void draw(const DrawCallback2D& cb) const;

        void clear() noexcept;
        
        ~AABBTree2D() noexcept
        {
            clear(root);
        }
    };

    struct AABB3D
    {
        float minX, minY, minZ;
        float maxX, maxY, maxZ;
    };

    [[nodiscard]] AABB3D computeFatAABB3D(const Body3D &body, float margin) noexcept;

    [[nodiscard]] bool AABBOverlap3D(const AABB3D &a, const AABB3D &b) noexcept;

    [[nodiscard]] AABB3D AABBUnion3D(const AABB3D &a, const AABB3D &b) noexcept;
    [[nodiscard]] AABB3D computeTightAABB3D(const Body3D &body) noexcept;

    [[nodiscard]] float getAreaAABB3D(const AABB3D &a) noexcept;

    using DrawCallback3D = std::function<void(const AABB3D&, bool isLeaf)>;
    
    struct Node3D
    {
        AABB3D aabb;

        Node3D *parent = nullptr;
        Node3D *children[2] = {nullptr, nullptr};

        int objectID = -1;

        bool isLeaf() const noexcept
        {
            return children[0] == nullptr;
        }

        void Draw(const DrawCallback3D& cb) const;
    };

    class AABBTree3D
    {
    private:
        Node3D *root = nullptr;
        std::unordered_map<uint32_t, Node3D *> leafMap;
        void clear(Node3D *node) noexcept;
        void remove(Node3D *leaf) noexcept;
        
    public:
        void insert(uint32_t bodyID, const AABB3D& aabb) noexcept;

        void query(const AABB3D &targetAABB, std::vector<uint32_t> &results) const noexcept;
        void query(Node3D *node, const AABB3D &targetAABB, std::vector<uint32_t> &results) const noexcept;

        void remove(uint32_t id) noexcept;

        void updateObject(const Body3D &body, float margin = 0.f) noexcept;

        void draw(const DrawCallback3D& cb) const;

        void clear() noexcept;
        
        ~AABBTree3D() noexcept
        {
            clear(root);
        }
    };


} // namespace sas
