#pragma once

#include <unordered_map>
#include <vector>
#include <functional>

#include "BodyCore.hpp"

#if HAS_CUDA
#include "CudaObjects.hpp"

#endif

namespace sas
{
    struct AABB2D
    {
        float minX, minY;
        float maxX, maxY;

        [[nodiscard]] inline bool overlaps(const AABB2D &b) const noexcept
        {
            return (minX <= b.maxX && maxX >= b.minX) &&
                   (minY <= b.maxY && maxY >= b.minY);
        }

        [[nodiscard]] inline AABB2D aabbUnion(const AABB2D &b) const noexcept
        {
            return {
                std::min(minX, b.minX), std::min(minY, b.minY),
                std::max(maxX, b.maxX), std::max(maxY, b.maxY)};
        }

        [[nodiscard]] inline float getArea() const noexcept
        {
            float width = maxX - minX;
            float height = maxY - minY;

            return width * height;
        }
    };

    struct AABB3D
    {
        float minX, minY, minZ;
        float maxX, maxY, maxZ;

        [[nodiscard]] constexpr inline bool overlaps(const AABB3D &b) const noexcept
        {
            return (minX <= b.maxX && maxX >= b.minX) &&
                   (minY <= b.maxY && maxY >= b.minY) &&
                   (minZ <= b.maxZ && maxZ >= b.minZ);
        }

        [[nodiscard]] constexpr inline AABB3D aabbUnion(const AABB3D &b) const noexcept
        {
            return {
                std::min(minX, b.minX), std::min(minY, b.minY), std::min(minZ, b.minZ),
                std::max(maxX, b.maxX), std::max(maxY, b.maxY), std::max(maxZ, b.maxZ)};
        }

        [[nodiscard]] constexpr inline float getArea() const noexcept
        {
            float w = maxX - minX;
            float h = maxY - minY;
            float l = maxZ - minZ;
            return 2.0f * (w * h + h * l + l * w);
        }
    };

    [[nodiscard]] constexpr AABB2D computeFatAABB2D(const Body2D &body, float margin) noexcept
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
    [[nodiscard]] constexpr AABB3D computeFatAABB3D(const Body3D &body, float margin) noexcept
    {
        const float x = body.transform.position.x;
        const float y = body.transform.position.y;
        const float z = body.transform.position.z;
        const auto &rot = body.transform.rotMatrix;

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

            extX = std::abs(rot(0, 0)) * hx + std::abs(rot(0, 1)) * hy + std::abs(rot(0, 2)) * hz + margin;
            extY = std::abs(rot(1, 0)) * hx + std::abs(rot(1, 1)) * hy + std::abs(rot(1, 2)) * hz + margin;
            extZ = std::abs(rot(2, 0)) * hx + std::abs(rot(2, 1)) * hy + std::abs(rot(2, 2)) * hz + margin;
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

    [[nodiscard]] constexpr inline AABB2D computeTightAABB2D(const Body2D &body) noexcept
    {
        return computeFatAABB2D(body, 0.f);
    }

    [[nodiscard]] constexpr inline AABB3D computeTightAABB3D(const Body3D &body) noexcept
    {
        return computeFatAABB3D(body, 0.f);
    }

    template <typename T>
    struct TreeTraits;

    template <>
    struct TreeTraits<AABB2D>
    {
        using AABBType = AABB2D;
        using BodyType = Body2D;

        static constexpr AABB2D computeFat(const Body2D &body, float margin) noexcept
        {
            return computeFatAABB2D(body, margin);
        }
        static constexpr AABB2D computeTight(const Body2D &body) noexcept
        {
            return computeTightAABB2D(body);
        }
    };

    template <>
    struct TreeTraits<AABB3D>
    {
        using AABBType = AABB3D;
        using BodyType = Body3D;

        static constexpr AABB3D computeFat(const Body3D &body, float margin) noexcept
        {
            return computeFatAABB3D(body, margin);
        }
        static constexpr AABB3D computeTight(const Body3D &body) noexcept
        {
            return computeTightAABB3D(body);
        }
    };

    template <typename AABB>
    struct Node
    {
        AABB aabb;

        Node *parent = nullptr;
        Node *children[2] = {nullptr, nullptr};

        int objectID = -1;

        [[nodiscard]] bool isLeaf() const noexcept
        {
            return children[0] == nullptr;
        }

        template <typename Callback>
        void Draw(const Callback &cb) const
        {
            cb(aabb, isLeaf());
            if (!isLeaf())
            {
                if (children[0])
                    children[0]->Draw(cb);
                if (children[1])
                    children[1]->Draw(cb);
            }
        }
    };

    template <typename AABB>
    struct alignas(16) FlatNode
    {
        AABB aabb;
        int32_t objectID;
        int32_t missOffset;
    };

    using FlatNode2D = FlatNode<sas::AABB2D>;
    using FlatNode3D = FlatNode<sas::AABB3D>;

    template <typename Traits>
    class AABBTree
    {
    public:
        using AABB = typename Traits::AABBType;
        using Body = typename Traits::BodyType;
        using NodeType = Node<AABB>;

        using FlatNodeType = FlatNode<AABB>;

        using DrawCallback = std::function<void(const AABB &, bool isLeaf)>;

    private:
        NodeType *root = nullptr;
        std::unordered_map<uint32_t, NodeType *> leafMap;

        void clear(NodeType *node) noexcept;
        void remove(NodeType *leaf) noexcept;

    public:
        AABBTree() = default;

        ~AABBTree() noexcept
        {
            clear(root);
        }

        std::vector<FlatNodeType> flatten() const
        {
            std::vector<FlatNodeType> flatNodes;
            if (!root)
                return flatNodes;

            flattenRec(root, flatNodes);
            return flatNodes;
        }

        int flattenRec(const NodeType *node, std::vector<FlatNodeType> &flatNodes) const
        {
            if (!node)
                return -1;

            int myIndex = static_cast<int>(flatNodes.size());

            flatNodes.push_back(FlatNodeType{
                node->aabb,
                node->objectID,
                -1});

            if (node->isLeaf())
            {
                flatNodes[myIndex].missOffset = myIndex + 1;
            }
            else
            {
                flattenRec(node->children[0], flatNodes);
                flattenRec(node->children[1], flatNodes);

                flatNodes[myIndex].missOffset = static_cast<int>(flatNodes.size());
            }

            return myIndex;
        }

        AABBTree(const AABBTree &) = delete;
        AABBTree &operator=(const AABBTree &) = delete;
        AABBTree(AABBTree &&) noexcept = default;
        AABBTree &operator=(AABBTree &&) noexcept = default;

        void insert(uint32_t bodyID, const AABB &aabb) noexcept;

        void query(const AABB &targetAABB, std::vector<uint32_t> &results) const noexcept
        {
            query(root, targetAABB, results);
        }

        void query(NodeType *node, const AABB &targetAABB, std::vector<uint32_t> &results) const noexcept
        {
            if (!node || !node->aabb.overlaps(targetAABB))
                return;

            if (node->isLeaf())
            {
                results.push_back(static_cast<uint32_t>(node->objectID));
            }
            else
            {
                query(node->children[0], targetAABB, results);
                query(node->children[1], targetAABB, results);
            }
        }

        void remove(uint32_t id) noexcept;

        void updateObject(const Body &body, float margin = 0.f) noexcept;

        template <typename Callback>
        void draw(const Callback &cb) const
        {
            if (root)
            {
                root->Draw(cb);
            }
        }

        void clear() noexcept;
    };

    using AABBTree2D = AABBTree<TreeTraits<AABB2D>>;
    using AABBTree3D = AABBTree<TreeTraits<AABB3D>>;

    using Node2D = Node<AABB2D>;
    using Node3D = Node<AABB3D>;

} // namespace sas
