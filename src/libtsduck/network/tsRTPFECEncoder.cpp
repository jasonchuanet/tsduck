//----------------------------------------------------------------------------
//
// TSDuck - The MPEG Transport Stream Toolkit
// Copyright (c) 2026, Jason Chua
// BSD-2-Clause license, see LICENSE.txt file or https://tsduck.io/license
//
//----------------------------------------------------------------------------
// Original implementation of ST 2022-1:2007 sections 7, 8 and 9 / Annexes B and C.
// No external FEC implementation or standards document is redistributed.
// Protocol references: https://pub.smpte.org/pub/st2022-1/ and /st2022-2/.

#include "tsRTPFECEncoder.h"
#include "tsIPProtocols.h"
#include "tsMemory.h"
#include "tsTS.h"

namespace ts {
    namespace {
        constexpr uint8_t RTP_FLAGS = 0x80; // Version 2, no padding, extension or CSRC.
        constexpr size_t SEQUENCE_OFFSET = 2; // Network byte order; never overlay a native structure.
        constexpr size_t TIMESTAMP_OFFSET = 4; // Media clock, XOR-protected independently of sequence.
        constexpr size_t SSRC_OFFSET = 8; // Media remains fixed; the FEC SSRC is zero.
        constexpr size_t BASE_OFFSET = 12; // SNBase starts the extended FEC header.
        constexpr size_t LENGTH_OFFSET = 14; // XOR media length, excluding its fixed RTP header.
        constexpr size_t PT_OFFSET = 16; // E and PT recovery share a byte.
        constexpr size_t RECOVERY_TIME_OFFSET = 20; // XOR original 32-bit media RTP timestamps.
        constexpr size_t DIRECTION_OFFSET = 24; // N, type and index stay zero; D identifies rows.
        constexpr size_t STRIDE_OFFSET = 25; // L for columns, one for rows.
        constexpr size_t COUNT_OFFSET = 26; // D for columns, L for rows; constant for the session.
        constexpr size_t PAYLOAD_OFFSET = RTP_HEADER_SIZE + RTPFECEncoder::FEC_HEADER_SIZE;
        constexpr uint8_t EXTENSION_BIT = 0x80; // E=1 selects the extended XOR header.
        constexpr uint8_t ROW_BIT = 0x40; // Second FEC stream, transmitted to media port +4.
    }
}

// Anchor exported vtables in the library instead of emitting them in every caller.
ts::RTPFECEncoder::~RTPFECEncoder() = default;
ts::RTPFECEncoder::Datagram::~Datagram() = default;

//----------------------------------------------------------------------------
// Configuration bounds are validated before allocating or discarding state.
//----------------------------------------------------------------------------

bool ts::RTPFECEncoder::reset(size_t columns, size_t rows, bool two_dimensional, bool block_aligned)
{
    if (columns == 0 || columns > MAX_COLUMNS || rows < MIN_ROWS || rows > MAX_ROWS ||
        columns * rows > MAX_MATRIX || (two_dimensional && columns < MIN_2D_COLUMNS)) {
        return false;
    }
    clear(); // Never mix parity from two sessions, even when their geometry matches.
    _columns_count = columns;
    _rows_count = rows;
    _two_dimensional = two_dimensional;
    _block_aligned = block_aligned;
    _columns.resize(columns); // Only L equations, not an entire matrix of media packets.
    return true;
}

void ts::RTPFECEncoder::clear()
{
    _columns_count = _rows_count = 0; // Adding media is rejected until reset establishes geometry.
    _two_dimensional = _initialized = false;
    _block_aligned = true;
    _expected_sequence = _column_sequence = _row_sequence = 0; // Independent wire counters.
    _ssrc = 0;
    _position = _next_column = 0;
    _row.clear(); // A partial row has fewer than the configured NA members and cannot be emitted.
    _columns.clear(); // Release incomplete current columns.
    _pending.clear(); // No future media exists to honor the required column emission spacing.
}

//----------------------------------------------------------------------------
// Check the full profile before advancing sequence, equations or scheduling.
//----------------------------------------------------------------------------

bool ts::RTPFECEncoder::_validMedia(const _Media& media) const
{
    if (_columns_count == 0 || media[0] != RTP_FLAGS || media[1] != RTP_PT_MP2T ||
        (_initialized && (GetUInt16(media.data() + SEQUENCE_OFFSET) != _expected_sequence || GetUInt32(media.data() + SSRC_OFFSET) != _ssrc))) {
        return false;
    }
    const size_t length = media.size() - RTP_HEADER_SIZE;
    for (const size_t& packet_size : {PKT_SIZE, PKT_RS_SIZE}) {
        if (length % packet_size == 0 && length <= MAX_TS_PACKETS * packet_size) {
            bool valid = true;
            for (size_t offset = RTP_HEADER_SIZE; valid && offset < media.size(); offset += packet_size) {
                valid = media[offset] == SYNC_BYTE; // Also validate every RS204 boundary.
            }
            if (valid) {
                return true;
            }
        }
    }
    return false;
}

//----------------------------------------------------------------------------
// Zero-extension preserves variable media lengths, including a final short burst.
// The non-recovery header fields remain fixed and are never included in XOR.
//----------------------------------------------------------------------------

void ts::RTPFECEncoder::_accumulate(ByteBlock& parity, const _Media& media, bool row) const
{
    const size_t length = media.size() - RTP_HEADER_SIZE;
    if (parity.empty()) {
        parity.resize(PAYLOAD_OFFSET, 0); // Mask, N, type, index, SNBase extension, FEC timestamp and SSRC remain zero.
        parity[0] = RTP_FLAGS;
        parity[1] = FEC_PAYLOAD_TYPE;
        PutUInt16(parity.data() + BASE_OFFSET, GetUInt16(media.data() + SEQUENCE_OFFSET)); // First protected media sequence.
        parity[PT_OFFSET] = EXTENSION_BIT;
        parity[DIRECTION_OFFSET] = row ? ROW_BIT : 0;
        parity[STRIDE_OFFSET] = static_cast<uint8_t>(row ? 1 : _columns_count);
        parity[COUNT_OFFSET] = static_cast<uint8_t>(row ? _columns_count : _rows_count);
    }
    parity.resize(std::max(parity.size(), PAYLOAD_OFFSET + length), 0); // Keep the longest associated payload.
    PutUInt16(parity.data() + LENGTH_OFFSET, GetUInt16(parity.data() + LENGTH_OFFSET) ^ static_cast<uint16_t>(length));
    parity[PT_OFFSET] ^= media[1]; // E stays set because the media marker bit must be zero.
    PutUInt32(parity.data() + RECOVERY_TIME_OFFSET, GetUInt32(parity.data() + RECOVERY_TIME_OFFSET) ^ GetUInt32(media.data() + TIMESTAMP_OFFSET));
    for (size_t index = 0; index < length; ++index) {
        parity[PAYLOAD_OFFSET + index] ^= media[RTP_HEADER_SIZE + index]; // Missing tail bytes contribute zero.
    }
}

void ts::RTPFECEncoder::_emit(ByteBlock& parity, bool row, Datagrams& output)
{
    assert(parity.size() > PAYLOAD_OFFSET); // Only fully accumulated, nonempty equations can reach emission.
    uint16_t& sequence = row ? _row_sequence : _column_sequence;
    PutUInt16(parity.data() + SEQUENCE_OFFSET, sequence); // Independent stream numbering, not media numbering.
    sequence = static_cast<uint16_t>(sequence + 1); // Deliberate modulo-65536 wire wrap.
    auto& packet = output.emplace_back();
    packet._data = std::move(parity); // Each output owns its storage until the caller sends or destroys it.
    parity.clear(); // A moved-from vector is valid but not required to be empty by its interface contract.
    packet._row = row;
}

//----------------------------------------------------------------------------
// Stagger column c by c modulo D rows. The modulo bounds startup to one matrix
// even when L > D, and supports non-coprime L/D without leaving columns unused.
// Like Annex B's 4x5 example, SNBase values are 0,5,10,15,20,... in that case.
//----------------------------------------------------------------------------

void ts::RTPFECEncoder::_staggeredColumn(const _Media& media, Datagrams& output)
{
    const size_t column = _position % _columns_count;
    const size_t phase = (column % _rows_count) * _columns_count + column;
    auto& parity = _columns[column]; // One equation spans a block boundary instead of awaiting a block interleaver.
    if (_position == phase && !parity.empty()) {
        _emit(parity, false, output); // The next group's first member is L intervals after the previous group's last.
    }
    if (!parity.empty() || _position >= phase) {
        _accumulate(parity, media, false); // Skip only the incomplete startup prefix; NA is never shortened.
    }
}

//----------------------------------------------------------------------------
// Aligned Annex C schedule: column c of the previous matrix follows media c*D
// of the next one. Its delay is L+c*(D-1), always within [L,L*D].
// Rows follow their last media immediately, within the allowed [0,L] delay.
//----------------------------------------------------------------------------

bool ts::RTPFECEncoder::addMedia(const void* address, size_t size, Datagrams& output)
{
    output.clear(); // Failed calls never leave earlier output masquerading as newly generated parity.
    if (address == nullptr || size <= RTP_HEADER_SIZE || size > RTP_HEADER_SIZE + MAX_TS_PACKETS * PKT_RS_SIZE) {
        return false;
    }
    const _Media media(static_cast<const uint8_t*>(address), size);
    if (!_validMedia(media)) {
        return false; // Validation leaves the current equations, sequence and scheduling unchanged.
    }
    _ssrc = GetUInt32(media.data() + SSRC_OFFSET);
    _initialized = true;
    _expected_sequence = static_cast<uint16_t>(GetUInt16(media.data() + SEQUENCE_OFFSET) + 1);
    if (!_pending.empty() && _position == _next_column * _rows_count) {
        _emit(_pending[_next_column], false, output); // Spread complete columns over the next matrix instead of bursting them.
        if (++_next_column == _columns_count) {
            _pending.clear(); // The previous matrix is fully emitted before this matrix completes.
        }
    }
    if (_block_aligned) {
        _accumulate(_columns[_position % _columns_count], media, false); // Aligned columns share a matrix boundary.
    }
    else {
        _staggeredColumn(media, output); // Staggered columns emit independently at their next group's start.
    }
    if (_two_dimensional) {
        _accumulate(_row, media, true);
        if (_position % _columns_count == _columns_count - 1) {
            _emit(_row, true, output); // No buffering delay for a completed row.
        }
    }
    if (++_position == _columns_count * _rows_count) {
        if (_block_aligned) {
            assert(_pending.empty()); // All older columns were emitted at least D-1 media before this boundary.
            _pending.swap(_columns); // Move complete equations without copying their payloads.
            _columns.resize(_columns_count); // Reuse empty equation slots for the new matrix.
        }
        _position = _next_column = 0;
    }
    return true;
}
