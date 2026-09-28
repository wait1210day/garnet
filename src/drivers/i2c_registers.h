// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_I2C_REGISTERS_H
#define GARNET_I2C_REGISTERS_H

#include <cstdint>
#include <cstddef>
#include <type_traits>
#include <utility>

namespace i2c
{

// Register R/W protection attribute
struct RW {};
struct RO {};
template<typename RegT>
concept ReadableReg = std::is_same_v<typename RegT::AccessTagT, RO> ||
                      std::is_same_v<typename RegT::AccessTagT, RW>;
template<typename RegT>
concept WritableReg = std::is_same_v<typename RegT::AccessTagT, RW>;

// i2c register address, 8bit or 16bit
template<typename AddressT>
concept RegAddress = std::is_same_v<AddressT, uint8_t> ||
                     std::is_same_v<AddressT, uint16_t>;

// Wire-format byte order (big-endian or little-endian)
struct BE {};
struct LE {};
template<typename T>
concept WireFormatEndian = std::is_same_v<T, BE> || std::is_same_v<T, LE>;

/**
 * Full description of a device register.
 *
 * @tparam ADDR             Address of the register, 8bit or 16bit.
 * @tparam ADDR_BYTES       Size of register address, must be 1 or 2.
 * @tparam RawT_            A type to hold the raw representation of register value.
 *                          E.g. `uint8_t` for 8bit registers, or `uint16_t` for 16bit registers, etc.
 * @tparam WIDTH_BYTES      Register data width in bytes.
 * @tparam AccessTagT_      Access tag, must be either `i2cdev::RW` or `i2cdev::RO`.
 * @tparam DataEndianT_     Register data endian, must be `i2cdev::BE` or `i2cdev::LE`.
 * @tparam AddrEndianT_     Like `DataEndianT_`, but for register address.
 */
template<uint16_t ADDR, uint8_t ADDR_BYTES, typename RawT_, uint8_t WIDTH_BYTES, typename AccessTagT_,
         WireFormatEndian DataEndianT_, WireFormatEndian AddrEndianT_ = DataEndianT_>
struct Register
{
    // This structure just stores all the descriptions of a register.
    static constexpr uint16_t kAddress = ADDR;
    static constexpr uint8_t kAddressBytes = ADDR_BYTES;
    static constexpr uint8_t kWidthBytes = WIDTH_BYTES;
    using RawT = RawT_;
    using AccessTagT = AccessTagT_;
    using DataEndianT = DataEndianT_;
    using AddrEndianT = AddrEndianT_;


    // Address range and width check
    static_assert(kAddressBytes == 1 || kAddressBytes == 2, "ADDR_BYTES must be 1 or 2");
    static_assert(kAddressBytes != 1 || kAddress <= 0xff, "8-bit register address overflows");

    // Register data type check
    static_assert(kWidthBytes > 0, "register width must be non-zero");
    static_assert(sizeof(RawT) >= kWidthBytes, "register reg type too small");
    static_assert(std::is_unsigned_v<RawT_>, "register Raw type must be unsigned");
};


namespace detail
{
    template<typename FieldT_>
    struct FieldValue
    {
        using FieldT = FieldT_;
        typename FieldT::ValueT value;
    };
} // namespace detail

template<typename RegT_, uint8_t POS_BITS, uint8_t WIDTH_BITS, typename ValueT_ = typename RegT_::RawT>
struct Field
{
    static constexpr uint8_t kPosBits = POS_BITS;
    static constexpr uint8_t kWidthBits = WIDTH_BITS;
    using RegT = RegT_;
    using ValueT = ValueT_;
    using RawT = typename RegT::RawT;
    static constexpr uint8_t kRawBits = sizeof(RawT) * 8;

    static_assert(WIDTH_BITS > 0, "field width must be non-zero");
    static_assert(POS_BITS + WIDTH_BITS <= kRawBits, "field exceeds register raw width");

    // Creates a mask like: 0x0ff0
    //                         ^^--- width=8bits, pos=4bits
    static constexpr RawT kMask =
        (WIDTH_BITS == kRawBits)
            ? static_cast<RawT>(~static_cast<RawT>(0))
            : static_cast<RawT>(((static_cast<RawT>(1) << WIDTH_BITS) - 1) << POS_BITS);

    static constexpr ValueT decode_from_raw(RawT raw)
    {
        return static_cast<ValueT>((raw & kMask) >> POS_BITS);
    }

    static constexpr RawT encode_to_raw(ValueT value)
    {
        return (static_cast<RawT>(value) << POS_BITS) & kMask;
    }

    static constexpr RawT insert_to_raw(RawT origin, ValueT value)
    {
        return (origin & ~kMask) | encode_to_raw(value);
    }

    static constexpr detail::FieldValue<Field> assign(ValueT value)
    {
        return detail::FieldValue<Field>{value};
    }
};

template<typename RegT>
class Mutator
{
public:
    using RawT = typename RegT::RawT;

    constexpr Mutator() : raw_(0) {}
    constexpr explicit Mutator(RawT raw) : raw_(raw) {}

    // Set the specified field, and return a new `Mutator` instance.
    template<typename FieldT>
    [[nodiscard]] constexpr Mutator with(typename FieldT::ValueT value) const
    {
        static_assert(std::is_same_v<RegT, typename FieldT::RegT>, "register/field mismatch");
        return Mutator{FieldT::insert_to_raw(raw_, value)};
    }

    // Set the specified field, change this `Mutator` instance itself.
    template<typename FieldT>
    constexpr Mutator& set(typename FieldT::ValueT value)
    {
        static_assert(std::is_same_v<RegT, typename FieldT::RegT>, "register/field mismatch");
        raw_ = FieldT::insert_to_raw(raw_, value);
        return *this;
    }

    // Get the specified field
    template<typename FieldT>
    constexpr auto get() const -> typename FieldT::ValueT
    {
        static_assert(std::is_same_v<RegT, typename FieldT::RegT>, "register/field mismatch");
        return FieldT::decode_from_raw(raw_);
    }

    constexpr RawT raw() const
    {
        return raw_;
    }

private:
    RawT raw_;
};

namespace detail
{
    template<typename RegT>
    [[nodiscard]] Err store_to_buf(typename RegT::RawT raw, uint8_t *dst, size_t dst_free_bytes)
    {
        if (dst == nullptr || dst_free_bytes < RegT::kWidthBytes)
            return Err::kInvalidArg;

        if constexpr (std::is_same_v<typename RegT::DataEndianT, BE>)
        {
            for (size_t i = 0; i < RegT::kWidthBytes; i++)
            {
                const size_t shift = 8u * (RegT::kWidthBytes - i - 1u);
                dst[i] = static_cast<uint8_t>((raw >> shift) & 0xff);
            }
        }
        else
        {
            for (size_t i = 0; i < RegT::kWidthBytes; i++)
            {
                const size_t shift = 8u * i;
                dst[i] = static_cast<uint8_t>((raw >> shift) & 0xff);
            }
        }

        return Err::kOk;
    }

    template<typename RegT>
    [[nodiscard]] Err load_from_buf(const uint8_t *src, size_t src_bytes, typename RegT::RawT& raw)
    {
        using RawT = typename RegT::RawT;

        if (src == nullptr || src_bytes < RegT::kWidthBytes)
            return Err::kInvalidArg;

        RawT result = 0;
        if constexpr (std::is_same_v<typename RegT::DataEndianT, BE>)
        {
            for (size_t i = 0; i < RegT::kWidthBytes; i++)
            {
                const size_t shift = 8u * (RegT::kWidthBytes - i - 1u);
                result |= static_cast<RawT>(src[i]) << shift;
            }
        }
        else
        {
            for (size_t i = 0; i < RegT::kWidthBytes; i++)
            {
                const size_t shift = 8u * i;
                result |= static_cast<RawT>(src[i]) << shift;
            }
        }
        raw = result;

        return Err::kOk;
    }

    // This function does NOT check the remaining space of `dst`.
    template<typename RegT>
    void fill_reg_addr_unchecked(uint8_t *dst)
    {
        static constexpr uint16_t kAddr = RegT::kAddress;
        if constexpr (RegT::kAddressBytes == 2)
        {
            if constexpr (std::is_same_v<typename RegT::AddrEndianT, BE>)
            {
                dst[0] = kAddr >> 8;
                dst[1] = kAddr & 0xff;
            }
            else
            {
                dst[0] = kAddr & 0xff;
                dst[1] = kAddr >> 8;
            }
        }
        else
        {
            dst[0] = kAddr & 0xff;
        }
    }
} // namespace detail

} // namespace i2c

#endif //GARNET_I2C_REGISTERS_H
