// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef GARNET_HASH_H
#define GARNET_HASH_H

#include <cstddef>
#include <cstdint>

#include "services/fumi/protocol.h"

namespace fumi
{

// 32bit FNV-1a hash
template<std::size_t N>
consteval uint32_t string_literal_hash(const char (&text)[N])
{
    uint32_t hash = 2166136261u;
    // not including '\0' at the end
    for (std::size_t i = 0; i + 1 < N; ++i)
        hash = (hash ^ static_cast<unsigned char>(text[i])) * 16777619u;
    return hash;
}

// Packed struct helps the post-processing script parsing the hash
// dict embedded in the ELF file.
template<std::size_t N>
struct [[gnu::packed]] HashDictRecord
{
    uint32_t hash;
    char text[N];
};

template<std::size_t N>
consteval HashDictRecord<N> hash_make_record(uint32_t hash, const char (&text)[N])
{
    HashDictRecord<N> record{hash, {}};
    for (std::size_t i = 0; i < N; i++)
        record.text[i] = text[i];
    return record;
}

using LiteralStrId = proto::LiteralStrId;

#define FUMI_LITERAL_STR(s)                                             \
    ([] {                                                               \
        constexpr uint32_t hash = fumi::string_literal_hash(s);         \
        [[gnu::used, gnu::section(".fumi.dict"), gnu::aligned(1)]]      \
        static constexpr auto record = fumi::hash_make_record(hash, s); \
        return fumi::LiteralStrId{hash};                                \
    }())

} // namespace fumi

#endif //GARNET_HASH_H
