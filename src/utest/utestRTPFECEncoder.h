//----------------------------------------------------------------------------
//
// TSDuck - The MPEG Transport Stream Toolkit
// Copyright (c) 2026, Jason Chua
// BSD-2-Clause license, see LICENSE.txt file or https://tsduck.io/license
//
//----------------------------------------------------------------------------
// Independent wire checks for the encoder, without calling its XOR helpers.

#pragma once
#include "tsRTPFECEncoder.h"
#include "tsRTPFECDecoder.h"
#include "tsMemory.h"
#include "tsTS.h"
#include "tsunit.h"

namespace ts {
    class RTPFECEncoderTest: public tsunit::Test
    {
    public:
        using SuperClass = tsunit::Test; // Shared test lifecycle, independent of encoder implementation.

        TSUNIT_DECLARE_TEST(MandatoryMatrices); // Every required L/D pair, both stream modes and sequence wrap.
        TSUNIT_DECLARE_TEST(Payloads); // TS188, RS204 and variable-length zero-extension.
        TSUNIT_DECLARE_TEST(InvalidConfiguration); // Reject geometry before discarding a valid session.
        TSUNIT_DECLARE_TEST(InvalidMedia); // Rejected packets must not advance sequence or group state.
        TSUNIT_DECLARE_TEST(RestartAndStop); // Partial groups are discarded; a new session never inherits parity.
        TSUNIT_DECLARE_TEST(Recovery); // Exercise burst/iterative reconstruction through the input decoder.
        TSUNIT_DECLARE_TEST(LongRun); // Independent parity sequence counters wrap without leaking media storage.
        TSUNIT_DECLARE_TEST(NonBlockArrangement); // Exact Annex B 4x5 example, independent of encoder scheduling.
    private:
        using _Encoder = RTPFECEncoder;
        using _Decoder = RTPFECDecoder;
        using _Media = std::vector<ByteBlock>; // Original bytes retained independently of the encoder.
        static constexpr uint16_t _BASE = 65520; // Every mandatory matrix crosses the media sequence boundary.
        static constexpr uint32_t _SSRC = 0x10203040; // Media source; parity source must remain zero.

        static ByteBlock _Packet(uint16_t sequence, size_t packet_size = PKT_SIZE, size_t count = 1);
        static void _Check(const _Encoder::Datagram& parity, const _Media& media, size_t columns, size_t rows, size_t emitted_at, bool block_aligned = true);
        static void _Exercise(size_t columns, size_t rows, bool two_dimensional, size_t packet_size = PKT_SIZE, size_t count = 1, bool block_aligned = true);
    };

    inline ByteBlock RTPFECEncoderTest::_Packet(uint16_t sequence, size_t packet_size, size_t count)
    {
        TSUNIT_ASSERT((packet_size == PKT_SIZE || packet_size == PKT_RS_SIZE) && count > 0 && count <= 7);
        ByteBlock data(12 + packet_size * count); // Independent media generator, including all RTP wire fields.
        data[0] = 0x80; // Reject extensions or padding rather than assuming their layout.
        data[1] = 33; // MPEG-TS payload type; marker participates in profile validation.
        PutUInt16(data.data() + 2, sequence); // The supplied value is the actual 16-bit wire sequence.
        PutUInt32(data.data() + 4, 0xFFFF0000 + static_cast<uint32_t>(sequence) * 900); // Deliberate timestamp wrap.
        PutUInt32(data.data() + 8, _SSRC); // Parity must not accidentally inherit this nonzero identifier.
        for (size_t index = 12; index < data.size(); ++index) {
            data[index] = static_cast<uint8_t>(index * 13 + sequence * 7); // Distinct bytes expose wrong member selection.
        }
        for (size_t index = 12; index < data.size(); index += packet_size) {
            data[index] = 0x47; // Both 188 and 204-byte boundaries must be valid TS sync.
        }
        return data;
    }

    inline void RTPFECEncoderTest::_Check(const _Encoder::Datagram& parity, const _Media& media, size_t columns, size_t rows, size_t emitted_at, bool block_aligned)
    {
        const auto& data = parity.data();
        TSUNIT_ASSERT(data.size() > 28); // Header access is checked before interpreting any wire field.
        const size_t base = static_cast<uint16_t>(GetUInt16(data.data() + 12) - _BASE);
        const size_t stride = parity.isRow() ? 1 : columns;
        const size_t count = parity.isRow() ? columns : rows;
        TSUNIT_ASSERT(base < media.size() && count <= 1 + (media.size() - 1 - base) / stride);
        const size_t last = base + (count - 1) * stride;
        TSUNIT_ASSERT(last <= emitted_at); // No parity may precede any of its protected media.
        TSUNIT_ASSERT(parity.isRow() ? emitted_at - last <= columns : emitted_at - last >= columns && emitted_at - last <= columns * rows);
        if (!parity.isRow()) {
            // SNBase exposes alignment directly; equal counts or successful XOR alone cannot prove it.
            TSUNIT_EQUAL(block_aligned ? base % columns : (base % columns % rows) * columns + base % columns, base % (columns * rows));
            TSUNIT_ASSERT(block_aligned || emitted_at - last == columns); // Staggered columns use exactly the minimum allowed delay.
        }
        TSUNIT_EQUAL(0x80, data[0]); // Fixed RTP profile, independently verified.
        TSUNIT_EQUAL(96, data[1]);
        TSUNIT_EQUAL(0, GetUInt32(data.data() + 8)); // Required FEC SSRC.
        TSUNIT_EQUAL(0, GetUInt24(data.data() + 17)); // Mask is unused in extended XOR mode.
        TSUNIT_EQUAL(parity.isRow() ? 0x40 : 0, data[24]); // No N, type or index bits.
        TSUNIT_EQUAL(stride, data[25]); // Member spacing is explicit and cannot be inferred from timing.
        TSUNIT_EQUAL(count, data[26]); // NA must stay fixed, including the final partial matrix.
        TSUNIT_EQUAL(0, data[27]); // No extended sequence bits for 16-bit RTP.
        size_t longest = 0; // Storage length differs from the XOR-recovered length.
        uint16_t length = 0; // Even equal-size groups legitimately have zero XOR length.
        uint8_t pt = 0; // E is added only after independently reducing the media PT.
        uint32_t timestamp = 0; // Use original media times, never the FEC RTP timestamp.
        for (size_t index = base; index <= last; index += stride) {
            longest = std::max(longest, media[index].size() - 12);
            length ^= static_cast<uint16_t>(media[index].size() - 12);
            pt ^= media[index][1];
            timestamp ^= GetUInt32(media[index].data() + 4);
        }
        TSUNIT_EQUAL(28 + longest, data.size()); // Parity has the longest member's payload, never an XOR length allocation.
        TSUNIT_EQUAL(length, GetUInt16(data.data() + 14));
        TSUNIT_EQUAL(0x80 | pt, data[16]);
        TSUNIT_EQUAL(timestamp, GetUInt32(data.data() + 20));
        for (size_t offset = 0; offset < longest; ++offset) {
            uint8_t expected = 0;
            for (size_t index = base; index <= last; index += stride) {
                if (12 + offset < media[index].size()) {
                    expected ^= media[index][12 + offset]; // Independently XOR each output byte, zero-extending short members.
                }
            }
            TSUNIT_EQUAL(expected, data[28 + offset]);
        }
    }

    inline void RTPFECEncoderTest::_Exercise(size_t columns, size_t rows, bool two_dimensional, size_t packet_size, size_t count, bool block_aligned)
    {
        _Encoder encoder;
        TSUNIT_ASSERT(encoder.reset(columns, rows, two_dimensional, block_aligned));
        _Media media;
        _Encoder::Datagrams output;
        size_t row_count = 0; // Separate oracles detect accidentally shared sequence counters.
        size_t column_count = 0; // Shaping changes packet order but not each stream's numbering.
        for (size_t index = 0; index < 2 * columns * rows; ++index) {
            media.push_back(_Packet(static_cast<uint16_t>(_BASE + index), packet_size, count));
            TSUNIT_ASSERT(encoder.addMedia(media.back().data(), media.back().size(), output));
            TSUNIT_ASSERT(output.size() <= 2); // Traffic shaping bounds both instantaneous output and pending storage.
            for (const auto& parity : output) {
                _Check(parity, media, columns, rows, index, block_aligned);
                auto& sequence = parity.isRow() ? row_count : column_count;
                TSUNIT_EQUAL(static_cast<uint16_t>(sequence), GetUInt16(parity.data().data() + 2));
                ++sequence; // Independent stream counters must never inherit media sequence.
            }
        }
        TSUNIT_EQUAL(columns, column_count); // The final complete matrix still needs later media for column shaping.
        TSUNIT_EQUAL(two_dimensional ? 2 * rows : 0, row_count);
    }
}
