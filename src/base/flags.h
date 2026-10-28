// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_FLAGS_H
#define GARNET_FLAGS_H

#include <concepts>
#include <type_traits>


template<typename T> requires(std::is_enum_v<T>)
class Flags
{
public:
    using U = std::underlying_type_t<T>;
    static_assert(std::is_unsigned_v<U>, "enum underlying type must be unsigned");

    constexpr Flags() = default;
    constexpr explicit Flags(U raw) : raw_(raw) {}
    constexpr explicit Flags(T initial_value) : raw_(static_cast<U>(initial_value)) {}

    [[nodiscard]] constexpr U raw_value() const { return raw_; }

    constexpr void reset() { raw_ = 0; }

    constexpr void set_raw(U value) { raw_ |= value; }
    constexpr void clear_raw(U value) { raw_ &= ~value; }

    constexpr void set(T value) { raw_ |= static_cast<U>(value); }
    constexpr void set(Flags value) { raw_ |= value.raw_value(); }

    template<std::same_as<T>...Args>
    constexpr void set(Args...args) { raw_ |= (0 | ... | static_cast<U>(args)); }

    constexpr void clear(T value) { raw_ &= ~static_cast<U>(value); }
    constexpr void clear(Flags value) { raw_ &= ~value.raw_value(); }

    template<std::same_as<T>...Args>
    constexpr void clear(Args...args) { raw_ &= ~(0 | ... | static_cast<U>(args)); }

    [[nodiscard]] constexpr  bool has(T value) const {
        return (static_cast<U>(value) & raw_) == static_cast<U>(value);
    }

    template<std::same_as<T>...Args>
    [[nodiscard]] constexpr bool has_any(Args...args) const {
        return (false || ... || has(args));
    }

    template<std::same_as<T>...Args>
    [[nodiscard]] constexpr bool has_all(Args...args) const {
        return (true && ... && has(args));
    }

private:
    U raw_{0};
};

#endif //GARNET_FLAGS_H
