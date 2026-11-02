// SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
// SPDX-License-Identifier: GPL-3.0-or-later

#include "base/err.h"
#include "base/utils.h"

const char *get_error_str(ErrStatus err)
{
    switch (err)
    {
#define CASE(e, s) case e: return s;
        GARNET_ERR_STATUS_STR_LIST(CASE)
        default:
            return "unknown error";
#undef CASE
    }
}

// NOLINTNEXTLINE
extern "C" const uint8_t __build_id_start[];

// "GNU\0", 4 bytes
constexpr std::size_t kElfNoteNameSize = 4;
constexpr std::size_t kElfNoteTypeNtGnuBuildId = 3;

struct ElfNoteHeader
{
    uint32_t name_size;
    uint32_t desc_size;
    uint32_t type;
};

bool check_build_id_sha1()
{
    auto *hdr = reinterpret_cast<const ElfNoteHeader*>(__build_id_start);

    // strlen("GNU\0") == 4, and sha1 has 20 bytes
    return hdr->name_size == kElfNoteNameSize
        && hdr->desc_size == 20
        && hdr->type == kElfNoteTypeNtGnuBuildId;
}

std::span<const uint8_t, 20> get_build_id_sha1()
{
    const uint8_t *hash_ptr = __build_id_start + sizeof(ElfNoteHeader) + kElfNoteNameSize;
    return std::span<const uint8_t, 20>(hash_ptr, 20);
}
