#!/usr/bin/env python3

# SPDX-FileCopyrightText: 2026 masshiroio <masshirosora.io@gmail.com>
# SPDX-License-Identifier: GPL-3.0-or-later


import sys
import json
import struct
from dataclasses import dataclass

from elftools.elf.elffile import ELFFile
from elftools.elf.sections import NoteSection, Section


@dataclass
class StrHashEntry:
    content: str
    offset: int
    idx: int

@dataclass
class ElfMetadata:
    build_id: str
    sstr_hash_dict: dict[int, StrHashEntry]


def extract_build_id_from_note(section: NoteSection) -> bytes | None:
    for note in section.iter_notes():
        if note['n_name'] == 'GNU' and note['n_type'] == 'NT_GNU_BUILD_ID':
            build_id = bytes(note['n_descdata'])
            if len(build_id) != 20:
                raise ValueError('ELF GNU build-id must be 20-byte SHA-1')
            return build_id
    return None


def parse_fumi_dict(metadata: ElfMetadata, data: bytes, little_endian: bool):
    endian = '<' if little_endian else '>'

    offset = 0
    idx = 0
    total_len = len(data)
    fmt = f'{endian}I'
    u32_size = struct.calcsize(fmt)

    nb_elim_entries = 0

    while offset < total_len:
        if offset + u32_size > total_len:
            raise ValueError(f'fumidict: cannot read a u32 at offset {offset}')

        curr_entry_offset = offset
        (num,) = struct.unpack_from(fmt, data, offset)
        offset += u32_size

        null_pos = data.find(b'\x00', offset)
        if null_pos == -1:
            raise ValueError(f'fumidict: invalid c-string at offset {offset}')

        try:
            str_val = data[offset:null_pos].decode('utf-8')
        except UnicodeDecodeError as exc:
            raise ValueError(f'fumidict: entry [{idx}] at offset {offset} '
                             + f'failed to decode data as utf8 string: {exc}')
        offset = null_pos + 1

        previous = metadata.sstr_hash_dict.get(num)
        if previous is not None:
            if previous.content != str_val:
                print(f'fumidict: hash collision {num}!')
                print(f'    A: [{previous.idx}] at offset {previous.offset}: {previous.content}')
                print(f'    B: [{idx}] at offset {curr_entry_offset}: {str_val}')
                raise ValueError('fumidict: hash collision')
            else:
                print(f'fumidict: [{idx}] eliminated redundant entry at offset {curr_entry_offset}')
                nb_elim_entries += 1
        else:
            metadata.sstr_hash_dict[num] = StrHashEntry(str_val, curr_entry_offset, idx)

        idx = idx + 1

    print(f'fumidict: collected {len(metadata.sstr_hash_dict)} entries, {nb_elim_entries} eliminated')


def main():
    if len(sys.argv) != 3:
        print(f'usage: {sys.argv[0]} <firmware elf file> <output json file>')
        exit(1)

    elf_path = sys.argv[1]
    json_path = sys.argv[2]

    fumidict_raw_data: bytes | None = None
    build_id: bytes | None = None
    is_little_endian: bool = False

    with open(elf_path, 'rb') as stream:
        elf = ELFFile(stream)
        is_little_endian = elf.little_endian

        for section in elf.iter_sections():
            if section.name == '.fumi.dict':
                fumidict_raw_data = section.data()
            elif isinstance(section, NoteSection):
                build_id = extract_build_id_from_note(section)

    if fumidict_raw_data is None or build_id is None:
        raise ValueError('required section .fumi.dict and/or build id not found')

    metadata = ElfMetadata(build_id.hex(), {})
    parse_fumi_dict(metadata, fumidict_raw_data, is_little_endian)

    json_data = {
        'build_id': metadata.build_id,
        'strhash': {}
    }
    for hash, entry in metadata.sstr_hash_dict.items():
        json_data['strhash'][hash] = entry.content

    with open(json_path, 'w') as fp:
        json.dump(json_data, fp=fp, indent=2)


if __name__ == '__main__':
    main()
