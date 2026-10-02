// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_I2C_DEVICE_H
#define GARNET_I2C_DEVICE_H

#include <expected>
#include <type_traits>
#include <utility>

#include "drivers/i2c_bus.h"
#include "drivers/i2c_registers.h"

namespace i2c
{

// Borrows a locked bus and stores the transfer parameters for one device.
// Do not outlive or move the owning Scoped while using this wrapper.
class Dev
{
public:
    Dev(Bus::Scoped& bus, uint8_t dev_addr, XferMethod method, TickType_t timeout)
        : bus_(bus), dev_addr_(dev_addr), method_(method), timeout_(timeout) {}

    template<WritableReg RegT>
    [[nodiscard]] std::expected<void, Err> write(typename RegT::RawT raw)
    {
        uint8_t buf[RegT::kAddressBytes + RegT::kWidthBytes];
        detail::fill_reg_addr_unchecked<RegT>(buf);

        Err err = detail::store_to_buf<RegT>(raw, buf + RegT::kAddressBytes,
                                              RegT::kWidthBytes);
        if (err != Err::kOk)
            return std::unexpected(err);

        err = bus_.transmit(dev_addr_, buf, method_, timeout_);
        if (err != Err::kOk)
            return std::unexpected(err);
        return {};
    }

    template<typename FirstFieldValue, typename ... RestFieldValues>
        requires((std::is_same_v<typename FirstFieldValue::FieldT::RegT,
                                 typename RestFieldValues::FieldT::RegT>
                  && ...))
    [[nodiscard]] std::expected<void, Err> write(FirstFieldValue first, RestFieldValues... rest)
    {
        using RegT = typename FirstFieldValue::FieldT::RegT;
        Mutator<RegT> mut{};
        mut.template set<typename FirstFieldValue::FieldT>(first.value);
        (mut.template set<typename RestFieldValues::FieldT>(rest.value), ...);
        return write<RegT>(mut.raw());
    }

    template<ReadableReg RegT>
    [[nodiscard]] std::expected<Mutator<RegT>, Err> read()
    {
        uint8_t addrbuf[RegT::kAddressBytes];
        detail::fill_reg_addr_unchecked<RegT>(addrbuf);
        uint8_t recvbuf[RegT::kWidthBytes];

        Err err = bus_.transmit_receive(dev_addr_, addrbuf, recvbuf, method_, timeout_);
        if (err != Err::kOk)
            return std::unexpected(err);

        typename RegT::RawT raw;
        err = detail::load_from_buf<RegT>(recvbuf, sizeof(recvbuf), raw);
        if (err != Err::kOk)
            return std::unexpected(err);
        return Mutator<RegT>{raw};
    }

    template<WritableReg RegT, typename Fn>
    [[nodiscard]] std::expected<void, Err> update(Fn&& fn)
    {
        static_assert(std::is_same_v<std::invoke_result_t<Fn, Mutator<RegT>>, Mutator<RegT>>,
                      "the callable object must return a Mutator<RegT>");

        // Scoped holds the bus lock across both transfers.
        auto current = read<RegT>();
        if (!current)
            return std::unexpected(current.error());
        return write<RegT>(std::forward<Fn>(fn)(*current).raw());
    }

    template<typename FirstFieldValue, typename ... RestFieldValues>
        requires((std::is_same_v<typename FirstFieldValue::FieldT::RegT,
                                 typename RestFieldValues::FieldT::RegT>
                  && ...))
    [[nodiscard]] std::expected<void, Err> update(FirstFieldValue first, RestFieldValues... rest)
    {
        using RegT = typename FirstFieldValue::FieldT::RegT;
        return update<RegT>([=](Mutator<RegT> mut) {
            mut.template set<typename FirstFieldValue::FieldT>(first.value);
            (mut.template set<typename RestFieldValues::FieldT>(rest.value), ...);
            return mut;
        });
    }

private:
    Bus::Scoped& bus_;
    uint8_t dev_addr_;
    XferMethod method_;
    TickType_t timeout_;
};

} // namespace i2c

#endif //GARNET_I2C_DEVICE_H
