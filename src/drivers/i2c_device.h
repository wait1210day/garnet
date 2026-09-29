//
// Created by sora on 2026/9/28.
//

#ifndef GARNET_I2C_DEVICE_H
#define GARNET_I2C_DEVICE_H

#include "drivers/i2c_bus.h"
#include "drivers/i2c_registers.h"

namespace i2c
{

template<WritableReg RegT>
[[nodiscard]] Err write(Bus::Scoped& bus, uint8_t dev_addr, typename RegT::RawT raw,
                        XferMethod method, TickType_t xfer_timeout)
{
    constexpr size_t N = RegT::kAddressBytes + RegT::kWidthBytes;
    uint8_t buf[N];

    detail::fill_reg_addr_unchecked<RegT>(buf);

    Err err = detail::store_to_buf<RegT>(raw, &buf[RegT::kAddressBytes], RegT::kWidthBytes);
    if (err != Err::kOk)
        return Err::kInvalidArg;

    return bus.transmit(dev_addr, buf, method, xfer_timeout);
}

template<WritableReg RegT>
[[nodiscard]] std::expected<void, Err>
write_expected(Bus::Scoped& bus, uint8_t dev_addr, typename RegT::RawT raw,
               XferMethod method, TickType_t xfer_timeout)
{
    Err err = write<RegT>(bus, dev_addr, raw, method, xfer_timeout);
    if (err != Err::kOk)
        return std::unexpected(err);
    return {};
}

template<ReadableReg RegT>
[[nodiscard]] Err read(Bus::Scoped& bus, uint8_t dev_addr, typename RegT::RawT& raw,
                       XferMethod method, TickType_t xfer_timeout)
{
    uint8_t addrbuf[RegT::kAddressBytes];
    detail::fill_reg_addr_unchecked<RegT>(addrbuf);

    uint8_t recvbuf[RegT::kWidthBytes];

    Err err = bus.transmit_receive(dev_addr, addrbuf, recvbuf, method, xfer_timeout);
    if (err != Err::kOk)
        return err;

    return detail::load_from_buf<RegT>(recvbuf, sizeof(recvbuf), raw);
}

template<ReadableReg RegT>
[[nodiscard]] std::expected<Mutator<RegT>, Err>
read_expected(Bus::Scoped& bus, uint8_t dev_addr, XferMethod method, TickType_t xfer_timeout)
{
    typename RegT::RawT raw;
    Err err = read<RegT>(bus, dev_addr, raw, method, xfer_timeout);
    if (err != Err::kOk)
        return std::unexpected(err);
    return Mutator<RegT>{raw};
}

template<WritableReg RegT, typename Fn>
[[nodiscard]] Err update(Bus::Scoped& bus, uint8_t dev_addr, Fn&& fn, XferMethod method,
                         TickType_t xfer_timeout)
{
    static_assert(std::is_same_v<std::invoke_result_t<Fn, Mutator<RegT>>, Mutator<RegT>>,
                  "the callable object must return a Mutator<RegT>");

    // Scoped keeps the entire read-modify-write sequence under one lock.

    typename RegT::RawT raw;
    Err err = read<RegT>(bus, dev_addr, raw, method, xfer_timeout);
    if (err != Err::kOk)
        return err;

    Mutator<RegT> mut{raw};
    Mutator<RegT> result = fn(mut);

    return write<RegT>(bus, dev_addr, result.raw(), method, xfer_timeout);
}

template<WritableReg RegT, typename Fn>
[[nodiscard]] std::expected<void, Err>
update_expected(Bus::Scoped& bus, uint8_t dev_addr, Fn&& fn, XferMethod method,
                TickType_t xfer_timeout)
{
    Err err = update<RegT, Fn>(bus, dev_addr, std::forward<Fn>(fn), method, xfer_timeout);
    if (err != Err::kOk)
        return std::unexpected(err);
    return {};
}

// A wrapper class for transfer parameters that does not hold the ownership
// of any resources.
// Copyable and movable. Borrows Scoped: do not outlive or move its owner
// while using this device wrapper. Several devices may share one Scoped.
class Dev
{
public:
    Dev(Bus::Scoped& bus, uint8_t dev_addr, XferMethod method, TickType_t timeout)
        : bus_(bus), dev_addr_(dev_addr), method_(method), timeout_(timeout) {}

    void set_xfer_method(XferMethod method)
    {
        method_ = method;
    }

    void set_timeout(TickType_t timeout)
    {
        timeout_ = timeout;
    }

    [[nodiscard]] Err transmit(std::span<const uint8_t> txbuf)
    {
        return bus_.transmit(dev_addr_, txbuf, method_, timeout_);
    }

    [[nodiscard]] Err receive(std::span<uint8_t> rxbuf)
    {
        return bus_.receive(dev_addr_, rxbuf, method_, timeout_);
    }

    [[nodiscard]] Err transmit_receive(std::span<const uint8_t> txbuf, std::span<uint8_t> rxbuf)
    {
        return bus_.transmit_receive(dev_addr_, txbuf, rxbuf, method_, timeout_);
    }

    template<WritableReg RegT>
    [[nodiscard]] std::expected<void, Err> write(const typename RegT::RawT& raw)
    {
        return i2c::write_expected<RegT>(bus_, dev_addr_, raw, method_, timeout_);
    }

    template<WritableReg RegT>
    [[nodiscard]] std::expected<void, Err> write(const Mutator<RegT>& mut)
    {
        return i2c::write_expected<RegT>(bus_, dev_addr_, mut.raw(), method_, timeout_);
    }

    template<typename FirstFieldValue, typename ... RestFieldValues>
        requires((std::is_same_v<typename FirstFieldValue::FieldT::RegT,
                                 typename RestFieldValues::FieldT::RegT>
                  && ...))
    [[nodiscard]] std::expected<void, Err> write(FirstFieldValue first, RestFieldValues...rest)
    {
        using RegT = typename FirstFieldValue::FieldT::RegT;
        Mutator<RegT> mut{};
        mut.template set<typename FirstFieldValue::FieldT>(first.value);
        (mut.template set<typename RestFieldValues::FieldT>(rest.value), ...);

        return i2c::write_expected<RegT>(bus_, dev_addr_, mut.raw(), method_, timeout_);
    }

    template<ReadableReg RegT>
    [[nodiscard]] Err read(typename RegT::RawT& out_raw)
    {
        return i2c::read<RegT>(bus_, dev_addr_, out_raw, method_, timeout_);
    }

    template<ReadableReg RegT>
    [[nodiscard]] std::expected<Mutator<RegT>, Err> read()
    {
        return i2c::read_expected<RegT>(bus_, dev_addr_, method_, timeout_);
    }

    template<typename FirstFieldValue, typename ... RestFieldValues>
        requires((std::is_same_v<typename FirstFieldValue::FieldT::RegT,
                                 typename RestFieldValues::FieldT::RegT>
                  && ...))
    [[nodiscard]] std::expected<void, Err> update(FirstFieldValue first, RestFieldValues...rest)
    {
        using RegT = typename FirstFieldValue::FieldT::RegT;
        typename RegT::RawT raw;
        Err err = i2c::read<RegT>(bus_, dev_addr_, raw, method_, timeout_);
        if (err != Err::kOk)
            return std::unexpected(err);

        Mutator<RegT> mut{raw};
        mut.template set<typename FirstFieldValue::FieldT>(first.value);
        (mut.template set<typename RestFieldValues::FieldT>(rest.value), ...);

        return i2c::write_expected<RegT>(bus_, dev_addr_, mut.raw(), method_, timeout_);
    }

private:
    Bus::Scoped& bus_;
    uint8_t dev_addr_;
    XferMethod method_;
    TickType_t timeout_;
};

} // namespace i2c

#endif //GARNET_I2C_DEVICE_H
