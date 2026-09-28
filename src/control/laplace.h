// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_LAPLACE_H
#define GARNET_LAPLACE_H

// Pure compile-time compensator calculator and fixed-point quantizer.
// An equivalent script is `//tools/2p2z.py`, with more functions like analog
// design and bode plot.

#include <cstdint>
#include <bit>
#include <numbers>
#include <type_traits>
#include <concepts>
#include <limits>

namespace laplace
{

// Trigger a compile error by calling non-const function.
// Note that exceptions are disabled (-fno-exception).
void report_error(const char *);

constexpr double const_abs(double x)
{
    if (x < 0)
        return -x;
    return x;
}

constexpr double const_list_max(std::convertible_to<double> auto first,
                                std::convertible_to<double> auto ...rest)
{
    auto max_value = static_cast<double>(first);
    ((max_value = static_cast<double>(rest) > max_value
        ? static_cast<double>(rest) : max_value), ...);
    return max_value;
}

// A tricky function to implement `ceil(log2(x))`
constexpr int ceil_log2(double x)
{
    static_assert(std::numeric_limits<double>::is_iec559, "requires IEEE 754 double");

    if (x <= 0.0)
        report_error("x must be positive for log2");

    auto bits = std::bit_cast<uint64_t>(x);
    uint64_t exp_bits = (bits >> 52) & 0x7FFULL;
    uint64_t frac_bits = bits & 0x000F'FFFF'FFFF'FFFFULL;

    // exclude +Infinity and NaN
    if (exp_bits == 0x7FFULL)
        report_error("x must be finite and not NaN");

    // normal numbers
    if (exp_bits > 0)
    {
        int exp = static_cast<int>(exp_bits) - 1023;
        // frac_bits=0, x can be represented as `2^exp`.
        // otherwise `x = (2^exp, 2^(exp+1))`，ceiling to `exp + 1`
        return (frac_bits == 0) ? exp : (exp + 1);
    }

    // subnormal numbers, `exp_bits == 0` and `frac_bits > 0`
    // we have `x = 2^(-1074) * frac_bits` in this case,
    // so `ceil(log2(n)) == std::bit_width(n - 1)` if n is a positive integer
    return -1074 + static_cast<int>(std::bit_width(frac_bits - 1));
}

template<typename ResultT>
constexpr ResultT const_signed_round(double val)
{
    return static_cast<ResultT>(val >= 0.0 ? val + 0.5 : val - 0.5);
}

template<typename T> requires(std::is_floating_point_v<T>)
struct DiscreteComp
{
    T b0;
    T b1;
    T b2;
    T a1;
    T a2;

    template<typename R> requires(std::is_floating_point_v<R>)
    constexpr DiscreteComp<R> cast_to() const
    {
        return DiscreteComp<R>{
            .b0 = static_cast<R>(b0),
            .b1 = static_cast<R>(b1),
            .b2 = static_cast<R>(b2),
            .a1 = static_cast<R>(a1),
            .a2 = static_cast<R>(a2)
        };
    }
};

template<std::signed_integral FixedT>
struct FixedDiscreteComp
{
    constexpr static uint32_t kAccumulatorBits = sizeof(FixedT) * 8;

    // The common implementation of IIR filter contains multiply-and-accumulate operations:
    //   acc += b0 * err
    //   acc += b1 * e_n1
    //   ...
    // To prevent `acc` from overflow, all the coefficients must be half width.
    constexpr static uint32_t kBestFitCoeffWidthBits = kAccumulatorBits >> 1;

    uint32_t q_bits;
    FixedT b0;
    FixedT b1;
    FixedT b2;
    FixedT a1;
    FixedT a2;

    constexpr static uint32_t best_fit_q_bits(const DiscreteComp<double>& comp)
    {
        const auto max_abs_coeff = const_list_max(
            const_abs(comp.a1), const_abs(comp.a2), const_abs(comp.b0),
            const_abs(comp.b1), const_abs(comp.b2));

        // +1 for the sign bit
        const uint32_t integer_bits = ceil_log2(max_abs_coeff + 1) + 1;
        if (integer_bits > kBestFitCoeffWidthBits)
            report_error("coefficients cannot be quantized with the accumulator of this bits");

        return kBestFitCoeffWidthBits - integer_bits;
    }

    constexpr static FixedDiscreteComp best_quantize(const DiscreteComp<double>& comp)
    {
        uint32_t q_bits = best_fit_q_bits(comp);
        FixedT scalar = static_cast<FixedT>(1) << q_bits;
        FixedT max_value = (static_cast<FixedT>(1) << (kBestFitCoeffWidthBits - 1)) - 1;
        FixedT min_value = -(static_cast<FixedT>(1) << (kBestFitCoeffWidthBits - 1));

        FixedDiscreteComp<FixedT> result{};

#define SCALE(field)                                                                         \
        result.field = static_cast<FixedT>(const_signed_round<FixedT>(comp.field * scalar)); \
        if (result.field < min_value || result.field > max_value)                            \
            report_error("coefficient " #field " overflow when scaling");

        SCALE(b0)
        SCALE(b1)
        SCALE(b2)
        SCALE(a1)
        SCALE(a2)
#undef SCALE

        result.q_bits = q_bits;

        return result;
    }
};

struct ContinuousParams
{
    double wp;
    double wz;
    double k_int;

    constexpr static ContinuousParams from_freqs(double fz_khz, double fp_khz, double k_int)
    {
        constexpr double kPI = std::numbers::pi;
        return {
            .wp = fp_khz * 1000 * kPI * 2,
            .wz = fz_khz * 1000 * kPI * 2,
            .k_int = k_int
        };
    }

    [[nodiscard]] constexpr DiscreteComp<double> solve_2p2z(double fs_hz) const
    {
        // Tustin transform constant: s = c * (1 - z^-1) / (1 + z^-1)
        const double c = 2.0 * fs_hz;

        // Calculate the normalization factor for the denominator
        // A0 = c * (1 + c/wp)
        const double den = c * (1.0 + c / wp);

        // Calculate numerator coefficients (b0, b1, b2)
        const double b0 = (k_int * (1.0 + c / wz)) / den;
        const double b1 = (2.0 * k_int) / den;
        const double b2 = (k_int * (1.0 - c / wz)) / den;

        // Calculate denominator coefficients (a1, a2)
        // Note: The target transfer function format is 1 - a1*z^(-1) - a2*z^(-2)
        const double a1 = (2.0 * c) / (c + wp);
        const double a2 = (wp - c) / (c + wp);

        return {
            .b0 = b0,
            .b1 = b1,
            .b2 = b2,
            .a1 = a1,
            .a2 = a2
        };
    }
};

} // namespace laplace

#endif //GARNET_LAPLACE_H
