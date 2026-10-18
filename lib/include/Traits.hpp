#pragma once
#include "Logger.hpp"
#include "Contact.hpp"
#include "AABBTree.hpp"
#include "Body.hpp"

namespace sas
{

    struct PhysicsSettings;
    struct Traits2D
    {
        static constexpr size_t Dimension = 2;
        using Body = Body2D;
        using Shape = Shape2D;
        using Vector = math::Vec2;
        using Contact = Contact2D;
        using AABBTree = AABBTree2D;
        using Transform = Transform2D;
        using Kinematics = Kinematics2D;
        using DrawCallback = AABBTree::DrawCallback;

        // ======= Creation =======
        static void initializeBodyPhysics(Body &body) noexcept;
        [[nodiscard]] static AABB2D computeTightAABB(Body &body) noexcept
        {
            return computeTightAABB2D(body);
        }

        // ======= Physics =======
        static void applyForces(Body &obj, const PhysicsSettings &settings) noexcept;
        static void integrateVelocity(Body &obj, float dt) noexcept;
        static void integratePosition(Body &obj, float dt) noexcept;
        static void resolveCollision(Body &obj, Body &other, Contact &contact) noexcept;
        static void correctPosition(Body &obj, Body &other, Contact &contact) noexcept;
        static void reset(Body &obj) noexcept
        {
            obj.kinematics.acceleration = math::Vec2{0};
        }

        // ======= Collision =======
        static bool testCircleCircle(const Body &obj, const Body &other, Contact &outContact) noexcept;
        static bool testCircleBox(const Body &circle, const Body &box, Contact &outContact) noexcept;
        static bool testBoxBox(const Body &obj, const Body &other, Contact &outContact) noexcept;

        using CollisionHandler = bool (*)(const Body &, const Body &, Contact &) noexcept;

        static inline const CollisionHandler DispatchTable[static_cast<size_t>(ShapeType2DCount)][static_cast<size_t>(ShapeType2DCount)] = {
            {&testCircleCircle, &testCircleBox},
            {[](const Body &obj, const Body &other, Contact &c) noexcept
             {
                 bool hit = testCircleBox(other, obj, c);
                 if (hit)
                 {
                     c.normal = -c.normal;
                 }
                 return hit;
             },
             &testBoxBox}};

        static void checkCollisionDispatcher(const Body &a, const Body &b, std::vector<Contact> &contacts) noexcept
        {
            size_t typeA = static_cast<size_t>(a.shape.type);
            size_t typeB = static_cast<size_t>(b.shape.type);

            Contact contact;
            contact.bodyA = a.bodyID;
            contact.bodyB = b.bodyID;

            if (DispatchTable[typeA][typeB](a, b, contact))
            {
                contacts.push_back(contact);
            }
        }
    };

    /*

    Circle + Any --> direct
    Box + Box --> SAT

    Restul --> GJK + EPA (Orice ar fiiii astea)

    */

    struct Traits3D
    {
        static constexpr size_t Dimension = 3;
        using Body = Body3D;
        using Shape = Shape3D;
        using Vector = math::Vec3;
        using Contact = Contact3D;
        using AABBTree = AABBTree3D;
        using Transform = Transform3D;
        using Kinematics = Kinematics3D;
        using DrawCallback = AABBTree::DrawCallback;


        // ======= Creation =======
        static void initializeBodyPhysics(Body &body) noexcept;
        [[nodiscard]] static AABB3D computeTightAABB(Body &body) noexcept
        {
            return computeTightAABB3D(body);
        }

        // ======= Physics =======
        static void applyForces(Body &obj, const PhysicsSettings &settings) noexcept;
        static void integrateVelocity(Body &obj, float dt) noexcept;
        static void integratePosition(Body &obj, float dt) noexcept;
        static void resolveCollision(Body &obj, Body &other, Contact &contact) noexcept;
        static void correctPosition(Body &obj, Body &other, Contact &contact) noexcept;
        static void reset(Body &obj) noexcept
        {
            obj.kinematics.acceleration = math::Vec3{0};
        }

        // ======= Collision =======
        [[nodiscard]] static bool SphereSphere(const Body &obj, const Body &other, Contact &outContact) noexcept;
        [[nodiscard]] static bool SphereOrthotope(const Body &circle, const Body &box, Contact &outContact) noexcept;
        [[nodiscard]] static bool SphereCilinder(const Body &obj, const Body &other, Contact &outContact) noexcept;

        [[nodiscard]] static inline bool OrthopeSphere(const Body &obj, const Body &other, Contact &outContact) noexcept
        {
            if (SphereOrthotope(other, obj, outContact))
            {
                outContact.normal = -outContact.normal;
                return true;
            }

            return false;
        }
        [[nodiscard]] static bool OrthopeOrthope(const Body &obj, const Body &other, Contact &outContact) noexcept;
        [[nodiscard]] static inline bool CilinderSphere(const Body &obj, const Body &other, Contact &outContact) noexcept
        {
            if (SphereCilinder(other, obj, outContact))
            {
                outContact.normal = -outContact.normal;
                return true;
            }

            return false;
        }
        [[nodiscard]] static bool GJKEPA(const Body &obj, const Body &other, Contact &outContact) noexcept;

        using CollisionHandler = bool (*)(const Body &, const Body &, Contact &) noexcept;

        static inline const CollisionHandler DispatchTable[static_cast<size_t>(ShapeType3DCount)][static_cast<size_t>(ShapeType3DCount)] = {
            {&SphereSphere, &SphereOrthotope, &SphereCilinder},
            {&OrthopeSphere, &OrthopeOrthope, &GJKEPA},
            {&CilinderSphere, &GJKEPA, &GJKEPA}};

        static void checkCollisionDispatcher(const Body &a, const Body &b, std::vector<Contact> &contacts) noexcept
        {
            size_t typeA = static_cast<size_t>(a.shape.type);
            size_t typeB = static_cast<size_t>(b.shape.type);

            Contact contact;
            contact.bodyA = a.bodyID;
            contact.bodyB = b.bodyID;

            // BaseLogger::log("Checking collision started");

            if (DispatchTable[typeA][typeB](a, b, contact))
            {
                contacts.push_back(contact);
            }
        }
    };

    template <typename T>
    concept DimensionalTraits = std::same_as<T, Traits2D> || std::same_as<T, Traits3D>;

} // namespace sas
