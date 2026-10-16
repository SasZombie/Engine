#pragma once

#include <meta>
#include "BodyCore.hpp"

namespace sas
{

    constexpr std::size_t ShapeType2DCount = std::meta::enumerators_of(^^ShapeType2D).size();
    constexpr std::size_t ShapeType3DCount = std::meta::enumerators_of(^^ShapeType3D).size();

    template <typename E>
        requires std::is_enum_v<E>
    constexpr std::string_view enumToString(E val) noexcept
    {
        static constexpr auto enumerators = std::define_static_array(std::meta::enumerators_of(^^E));

        template for (constexpr auto e : enumerators)
        {
            if (val == [:e:])
            {
                return std::meta::identifier_of(e);
            }
        }

        return "Unknown code";
    }
    
} // namespace sas