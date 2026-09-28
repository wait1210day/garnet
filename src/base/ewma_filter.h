// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_EWMA_FILTER_H
#define GARNET_EWMA_FILTER_H

template <typename T>
class EwmaFilter
{
    static_assert(std::is_floating_point_v<T>, "T must be a floating-point type");

public:
    constexpr explicit EwmaFilter(T alpha, T initial_value = T{0}) noexcept
        : alpha_(alpha), state_(initial_value)
    {
    }

    constexpr void reset(T value = T{0})
    {
        state_ = value;
    }

    constexpr T update(T data)
    {
        state_ += alpha_ * (data - state_);
        return state_;
    }

    [[nodiscard]] constexpr T value() const { return state_; }
    [[nodiscard]] constexpr T alpha() const { return alpha_; }

private:
    T alpha_;
    T state_;
};

using EwmaFilterF32 = EwmaFilter<float>;

#endif // GARNET_EWMA_FILTER_H
