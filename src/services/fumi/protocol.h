// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_PROTOCOL_H
#define GARNET_PROTOCOL_H

#include <cstddef>
#include <cstdint>
#include <utility>
#include <type_traits>
#include <cstring>

namespace fumi::proto
{

// Protocol layer maximum size, not the transport layer (wire format).
// I.e. without CRC and COBS encoding.
constexpr std::size_t kMaxFrameBytes = 512;

constexpr uint8_t kMagic = 0xd0;
constexpr uint8_t kVersion = 0x01;

// Basic frame format, before COBS encoding:
//   [Magic:u8] [Version:u8] [ChannelId:u8] [PayloadLen:u16] [...payload] [CRC16:u16]
//   ^-------------------------------------------------------------------
//       kMaxFrameBytes
//
// - little endian byte order
// - COBS encoding, using 0x00 as frame delimiter
constexpr std::size_t kSizeOfHeader = sizeof(uint8_t) * 3 + sizeof(uint16_t);

enum class Channel : uint8_t
{
    // device => host, sent by device unsolicitedly
    kLog = 0x01,

    // host => device
    kCmd = 0x02,

    // device => host
    kResponse = 0x03,
};

// Tagged<...> format:
//   [data_type_tag:u8] [data:...]
//
// Annotations convention:
//   - `Tagged<primitive>`: tagged format which has primitive type tag.
//   - `Tagged<any>`: a tagged format which has primitive or non-primitive type tag.
//   - or a concrete annotation like `Tagged<i8>`
//
// Bare `i8`, `u8`, `istr`, `ibytes` ... means the data is directly encoded without
// a type tag.
enum class DataTypeTag : uint8_t
{
    // Primitive types (fixed length):

    // i8, i16, i32, i64
    kI8 = 0x00,
    kI16 = 0x01,
    kI32 = 0x02,
    kI64 = 0x03,
    // u8, u16, u32, u64
    kU8 = 0x04,
    kU16 = 0x05,
    kU32 = 0x06,
    kU64 = 0x07,
    // bool
    kBool = 0x08,
    // float, double
    kFloat = 0x09,
    kDouble = 0x0a,
    // sstr, a pointer to an address of flash, and the host can locate and extract
    // the actual string using this address from the firmware ELF file.
    kStaticString = 0x0b,
    // numeric raw pointer, will not be dereferenced or resolved further
    kRawPointer = 0x0c,

    // Non-primitive types:

    // istr, immediate string is embedded directly into the packet's byte stream,
    // in UTF-8 encoding. Data format:
    //   [byte_length:u16] [...bytes]
    kImmString = 0x20,

    // ibytes, like istr, but represents raw bytes
    kImmBytes = 0x21
};

template<typename T>
constexpr DataTypeTag get_prim_type_tag()
{
    if constexpr (std::is_integral_v<T>)
    {
        if constexpr (std::is_same_v<T, bool>)
            return DataTypeTag::kBool;

        if constexpr (std::is_signed_v<T>)
        {
            if constexpr (sizeof(T) == 1)
                return DataTypeTag::kI8;
            if constexpr (sizeof(T) == 2)
                return DataTypeTag::kI16;
            if constexpr (sizeof(T) == 4)
                return DataTypeTag::kI32;
            if constexpr (sizeof(T) == 8)
                return DataTypeTag::kI64;
        }
        else
        {
            if constexpr (sizeof(T) == 1)
                return DataTypeTag::kU8;
            if constexpr (sizeof(T) == 2)
                return DataTypeTag::kU16;
            if constexpr (sizeof(T) == 4)
                return DataTypeTag::kU32;
            if constexpr (sizeof(T) == 8)
                return DataTypeTag::kU64;
        }
    }
    else if constexpr (std::is_same_v<T, float>)
        return DataTypeTag::kFloat;
    else if constexpr (std::is_same_v<T, double>)
        return DataTypeTag::kDouble;
    else if constexpr (std::is_same_v<T, const char*>)
        return DataTypeTag::kStaticString;
    else if constexpr (std::is_same_v<T, void*>)
        return DataTypeTag::kRawPointer;
    else
        static_assert(false, "not a supported primitive type");

    std::unreachable();
}

constexpr bool is_primitive_type_tag(DataTypeTag tag)
{
    return tag == DataTypeTag::kI8
        || tag == DataTypeTag::kI16
        || tag == DataTypeTag::kI32
        || tag == DataTypeTag::kI64
        || tag == DataTypeTag::kU8
        || tag == DataTypeTag::kU16
        || tag == DataTypeTag::kU32
        || tag == DataTypeTag::kU64
        || tag == DataTypeTag::kBool
        || tag == DataTypeTag::kFloat
        || tag == DataTypeTag::kDouble
        || tag == DataTypeTag::kStaticString
        || tag == DataTypeTag::kRawPointer;
}

template<typename T>
constexpr bool is_primitive_type()
{
    return is_primitive_type_tag(get_prim_type_tag<T>());
}

template<typename...Args>
constexpr std::size_t prim_tagged_data_tight_packed_size()
{
    static_assert((is_primitive_type<Args>() && ... && true),
                  "arguments contain at least one non-primitive type");

    // equivalent to `sizeof<Args1>(args1) + 1 + ...`
    //                ^-----------------------
    //                 size of data + size of type tag
    return (0 + ... + sizeof(Args)) + sizeof...(Args);
}

template<typename T>
[[gnu::always_inline]]
inline void prim_tagged_data_store_unsafe(uint8_t *dst, std::size_t& offset, T value)
{
    static_assert(is_primitive_type<T>(), "T must be a primitive type");

    dst[offset] = static_cast<uint8_t>(get_prim_type_tag<T>());
    offset += 1;
    std::memcpy(dst + offset, &value, sizeof(T));
    offset += sizeof(T);
}

// Log channel payload format:
//   [level:u8] [timestamp:u64] [tag_str:sstr] [format_str:sstr] [nb_params:u8]
//   [param0:Tagged<primitive>]...
//
enum class LogLevel : uint8_t
{
#define FUMI_LEVEL_DEBUG    0x00
#define FUMI_LEVEL_INFO     0x01
#define FUMI_LEVEL_WARN     0x02
#define FUMI_LEVEL_ERROR    0x03

    kDebug = FUMI_LEVEL_DEBUG,
    kInfo = FUMI_LEVEL_INFO,
    kWarn = FUMI_LEVEL_WARN,
    kError = FUMI_LEVEL_ERROR
};

[[gnu::always_inline]] inline std::size_t log_payload_size(std::size_t argsbuf_len)
{
    return sizeof(uint8_t)
            + sizeof(uint64_t)
            + sizeof(uint32_t)
            + sizeof(uint32_t)
            + sizeof(uint8_t)
            + argsbuf_len;
}


// Cmd channel payload format:
//   [opcode:u8] [transaction_id:u16] [nb_operands:u8] [operand0:Tagged<any>]...
enum class CmdOpcode : uint8_t
{
    // Protocol defined commands:

    // Query for the basic info of device and fumi protocol.
    // Operands: 0
    // Response payload:
    //   [device_name:sstr] [device_desc:sstr] [build_id_sha1:ibytes]
    kHostHello = 0x01,
};

// Response channel payload format:
//   [transaction_id:u16] [resp_seq:u16] [flags:u8] [...response_payload]
enum class ResponseFlag : uint8_t
{
    kDone = (1 << 0),

    // If err flags are set, a `sstr` in response_payload describes
    // the detailed information of the error.
    kErrIllFormedPacket = (1 << 1),
    kErrInvalidOpcode = (1 << 2),
    kErrInvalidOperands = (1 << 3),
    kErrExecution = (1 << 4)
};

struct Consume {};
struct Peek {};
template<typename T>
concept ConsumeOrPeek = std::is_same_v<T, Consume> || std::is_same_v<T, Peek>;

// For little-endian only
class PacketReader
{
public:
    static_assert(std::endian::native == std::endian::little,
        "requires little-endian architecture");

    explicit PacketReader(std::span<const uint8_t> buf) : buf_(buf) {}

    bool has_error() const { return has_error_; }

    [[nodiscard]] std::size_t remaining_bytes() const {
        if (pos_ >= buf_.size())
            return 0;
        return buf_.size() - pos_;
    }

#define CHECK_SIZE_ERROR(need_bytes, ret_error)         \
    if (buf_.size() - pos_ < need_bytes || has_error_)  \
    {                                                   \
        has_error_ = true;                              \
        return ret_error;                               \
    }

    template<typename T, ConsumeOrPeek Method = Consume>
        requires((std::is_integral_v<T> || std::is_floating_point_v<T>)
                 && !std::is_same_v<T, bool>)
    T read_numeric()
    {
        CHECK_SIZE_ERROR(sizeof(T), 0)
        T result;
        std::memcpy(&result, buf_.data() + pos_, sizeof(T));
        if constexpr (std::is_same_v<Method, Consume>)
            pos_ += sizeof(T);
        return result;
    }

    template<typename T, ConsumeOrPeek Method = Consume>
        requires(std::is_enum_v<T>)
    T read_enum()
    {
        using U = typename std::underlying_type_t<T>;
        CHECK_SIZE_ERROR(sizeof(T), static_cast<T>(0))
        return static_cast<T>(read_numeric<U, Method>());
    }

    // ...

#undef CHECK_SIZE_ERROR
private:
    std::span<const uint8_t> buf_;
    std::size_t pos_{0};
    bool has_error_{false};
};

class PacketWriter
{
public:
    static_assert(std::endian::native == std::endian::little,
        "requires little-endian architecture");

    explicit PacketWriter(std::span<uint8_t> buf) : buf_(buf) {}

    [[nodiscard]] bool has_error() const { return has_error_; }
    [[nodiscard]] std::size_t current_pos() const { return pos_; }

#define CHECK_SIZE_ERROR(need_bytes, ret_error)         \
    if (buf_.size() - pos_ < need_bytes || has_error_)  \
    {                                                   \
        has_error_ = true;                              \
        return ret_error;                               \
    }

    bool skip_bytes(std::size_t bytes)
    {
        CHECK_SIZE_ERROR(bytes, false)
        pos_ += bytes;
        return true;
    }

    bool seek(std::size_t pos)
    {
        if (pos >= buf_.size())
        {
            has_error_ = true;
            return false;
        }
        pos_ = pos;
        return true;
    }

    template<typename T>
        requires((std::is_integral_v<T> || std::is_floating_point_v<T>)
                 && !std::is_same_v<T, bool>)
    bool write_numeric(T value)
    {
        CHECK_SIZE_ERROR(sizeof(T), false)
        std::memcpy(buf_.data() + pos_, &value, sizeof(T));
        pos_ += sizeof(T);
        return true;
    }

    template<typename T>
        requires((std::is_integral_v<T> || std::is_floating_point_v<T>)
                 && !std::is_same_v<T, bool>)
    bool write_numeric_at(T value, std::size_t pos)
    {
        if (buf_.size() - pos < sizeof(T) || has_error_)
        {
            has_error_ = true;
            return false;
        }
        std::memcpy(buf_.data() + pos, &value, sizeof(T));
        return true;
    }

    template<typename T> requires(std::is_enum_v<T>)
    bool write_enum(T value)
    {
        using U = std::underlying_type_t<T>;
        return write_numeric<U>(static_cast<U>(value));
    }

    bool write_sstr(const char *ptr)
    {
        static_assert(sizeof(const char*) == sizeof(uint32_t));
        CHECK_SIZE_ERROR(sizeof(uint32_t), false)
        return write_numeric(reinterpret_cast<uintptr_t>(ptr));
    }

    bool write_bytes(std::span<const uint8_t> bytes)
    {
        CHECK_SIZE_ERROR(bytes.size(), false)
        std::memcpy(buf_.data() + pos_, bytes.data(), bytes.size());
        pos_ += bytes.size();
        return true;
    }

    bool write_ibytes(std::span<const uint8_t> bytes)
    {
        CHECK_SIZE_ERROR(bytes.size() + sizeof(uint16_t), false)
        uint16_t size = bytes.size();
        std::memcpy(buf_.data() + pos_, &size, sizeof(uint16_t));
        pos_ += sizeof(uint16_t);
        std::memcpy(buf_.data() + pos_, bytes.data(), size);
        pos_ += size;
        return true;
    }

    bool write_istr(std::string_view sv)
    {
        CHECK_SIZE_ERROR(sv.size() + sizeof(uint16_t), false)
        uint16_t size = sv.size();
        std::memcpy(buf_.data() + pos_, &size, sizeof(uint16_t));
        pos_ += sizeof(uint16_t);
        std::memcpy(buf_.data() + pos_, sv.data(), size);
        pos_ += size;
        return true;
    }

    std::span<uint8_t> result() const
    {
        if (has_error_)
            return {};
        return buf_.first(pos_);
    }

#undef CHECK_SIZE_ERROR
private:
    std::span<uint8_t> buf_;
    std::size_t pos_{0};
    bool has_error_{false};
};

} // namespace fumi::proto

#endif //GARNET_PROTOCOL_H
