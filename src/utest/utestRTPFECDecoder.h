//----------------------------------------------------------------------------
//
// TSDuck - The MPEG Transport Stream Toolkit
// Copyright (c) 2026, Jason Chua
// BSD-2-Clause license, see LICENSE.txt file or https://tsduck.io/license
//
//----------------------------------------------------------------------------
//
// TSUnit tests for SMPTE ST 2022-1 recovery. The test sender is independent of
// the decoder and constructs XOR equations directly from the wire definition.
//
//----------------------------------------------------------------------------

#pragma once
#include "tsRTPFECDecoder.h"
#include "tsunit.h"
#include "tsMemory.h"
#include "tsTS.h"

namespace ts {
    class RTPFECDecoderTest: public tsunit::Test
    {
        TSUNIT_DECLARE_TEST(Column); // Single missing member in an interleaved column.
        TSUNIT_DECLARE_TEST(Iterative); // Recovery must propagate between row and column equations.
        TSUNIT_DECLARE_TEST(MandatoryMatrices); // Cover every required L/D combination and full bursts.
        TSUNIT_DECLARE_TEST(StaggeredColumns); // Never infer block alignment from adjacent parity packets.
        TSUNIT_DECLARE_TEST(EarlyParity); // Recover leading losses when parity arrives first.
        TSUNIT_DECLARE_TEST(WrapAndReorder); // Ordering and deduplication across the 16-bit boundary.
        TSUNIT_DECLARE_TEST(LengthsAndRS204); // Payload size, zero-extension and both TS packet formats.
        TSUNIT_DECLARE_TEST(Unrecoverable); // A rectangle cannot be solved by row/column XOR.
        TSUNIT_DECLARE_TEST(LateParity); // Deadlines finalize gaps without repeated output.
        TSUNIT_DECLARE_TEST(InvalidPackets); // Malformed headers and recovered payloads are discarded.
        TSUNIT_DECLARE_TEST(Bounds); // A non-consuming caller cannot grow the decoder indefinitely.
        TSUNIT_DECLARE_TEST(Restart); // No parity from an old SSRC survives session restart.
        TSUNIT_DECLARE_TEST(ParityBounds); // Bound payloads and reverse references under overlapping parity.
        TSUNIT_DECLARE_TEST(History); // Played media must still help later column parity.
        TSUNIT_DECLARE_TEST(LongRun); // Recover while crossing multiple sequence-number cycles.
        TSUNIT_DECLARE_TEST(DuplicateEquations); // Count a common singleton recovery only once.
        TSUNIT_DECLARE_TEST(RejectedParity); // Rejected deep validation must not change startup playout.
        TSUNIT_DECLARE_TEST(Pressure); // Shorten playout latency without discarding original media.

    private:
        using Decoder = ts::RTPFECDecoder;
        using Datagram = Decoder::Datagram;
        using Media = std::vector<Datagram>; // Expected packets are kept independently of decoder storage.
        static constexpr uint32_t _ssrc = 0x12345678; // Parity RTP SSRC is zero; media use another value.

        // Deterministic clock: these tests never sleep or depend on wall time.
        static Decoder::TimePoint _now()
        {
            return Decoder::TimePoint(cn::seconds(1));
        }
        static Datagram _makePacket(uint16_t sequence, size_t ts_size = 188, size_t count = 1);
        static Media _makePackets(size_t count, uint16_t base = 1000, size_t ts_size = 188, size_t ts_count = 1);
        static Datagram _makeParity(const Media& media, size_t base, size_t offset, size_t count, bool row = false);
        static void _send(Decoder& decoder, const Media& media, const std::set<size_t>& lost = {});
        static void _checkOutput(Decoder& decoder, const Media& media, const std::set<size_t>& lost = {});
    };

    // Header-only fixture helpers keep the test cases short and independently readable.
    inline ts::RTPFECDecoder::Datagram RTPFECDecoderTest::_makePacket(uint16_t sequence, size_t ts_size, size_t count)
    {
        Datagram packet; // Construct a complete wire packet rather than a decoder object.
        packet.setArrival(_now()); // Each test decides which later arrivals or deadlines differ.
        packet.setReceiveTimestamp(cn::microseconds(123456), ts::UDPSocket::TimeStampType::SOFTWARE); // Original kernel timestamps must survive buffering.
        packet.data().resize(12 + ts_size * count); // Support all mandatory 188/204-byte operating points.
        packet.data()[0] = 0x80; // Fixed RTP header: no padding, extension or CSRC.
        packet.data()[1] = 33; // The MPEG-TS RTP payload type, with marker zero.
        ts::PutUInt16(packet.data().data() + 2, sequence); // The supplied sequence already includes wire wrap.
        // Exercise 32-bit RTP timestamp wrapping as well as payload recovery.
        ts::PutUInt32(packet.data().data() + 4, 0xFFFF0000 + static_cast<uint32_t>(sequence) * 900);
        ts::PutUInt32(packet.data().data() + 8, _ssrc); // Use a nonzero media source identifier.
        for (size_t index = 12; index < packet.data().size(); ++index) {
            packet.data()[index] = static_cast<uint8_t>(sequence * 7 + index * 13); // Distinct payloads expose incorrect XOR or packet ordering.
        }
        for (size_t index = 12; index < packet.data().size(); index += ts_size) {
            packet.data()[index] = 0x47; // TS sync at every boundary, including RS204 trailers.
            packet.data()[index + 1] = 0x1F; // Use null packets so no PSI/PES parsing is required.
            packet.data()[index + 2] = 0xFF; // Null PID 0x1FFF.
            packet.data()[index + 3] = 0x10 | (sequence & 0x0F); // Distinct continuity values also participate in parity.
        }
        return packet;
    }

    inline RTPFECDecoderTest::Media RTPFECDecoderTest::_makePackets(size_t count, uint16_t base, size_t ts_size, size_t ts_count)
    {
        Media packets;
        for (size_t index = 0; index < count; ++index) {
            packets.push_back(_makePacket(static_cast<uint16_t>(base + index), ts_size, ts_count)); // Casting deliberately wraps the RTP sequence.
        }
        return packets;
    }

    inline ts::RTPFECDecoder::Datagram RTPFECDecoderTest::_makeParity(const Media& media, size_t base, size_t offset, size_t count, bool row)
    {
        // ST 2022-1 section 8.4: SNBase + j * Offset, 0 <= j < NA.
        // Compute a separate wire packet without calling any decoder methods.
        TSUNIT_ASSERT(count > 0 && offset > 0 && base < media.size());
        TSUNIT_ASSERT(count <= 1 + (media.size() - 1 - base) / offset);
        size_t length = 0; // Parity payload has the longest associated media length.
        for (size_t index = 0; index < count; ++index) {
            length = std::max(length, media[base + index * offset].data().size() - 12);
        }
        Datagram fec;
        fec.setArrival(_now() + cn::milliseconds(10)); // Parity normally follows media, but tests may override it.
        fec.data().resize(28 + length, 0); // Zero padding, mask, type, index and SSRC fields.
        fec.data()[0] = 0x80; // Parity also has the fixed 12-byte RTP header.
        fec.data()[1] = 96; // ST 2022-1 uses the first dynamic RTP payload type.
        ts::PutUInt16(fec.data().data() + 12, ts::GetUInt16(media[base].data().data() + 2)); // SNBase identifies media, not FEC RTP sequence.
        fec.data()[16] = 0x80; // E=1; recovery PT bits are XORed below.
        fec.data()[24] = row ? 0x40 : 0; // D distinguishes rows from columns.
        fec.data()[25] = static_cast<uint8_t>(offset); // Select periodic media members independently of block alignment.
        fec.data()[26] = static_cast<uint8_t>(count); // NA is a count, not the last member index.
        uint16_t length_xor = 0; // Length recovery is the XOR of payload lengths.
        uint32_t timestamp_xor = 0; // Timestamp recovery is independent of parity RTP timestamp.
        for (size_t index = 0; index < count; ++index) {
            const auto& packet = media[base + index * offset].data(); // Select the exact members named by the FEC header.
            length_xor ^= static_cast<uint16_t>(packet.size() - 12); // Exclude the fixed RTP header from length recovery.
            timestamp_xor ^= ts::GetUInt32(packet.data() + 4); // Include wrap in the unsigned 32-bit timestamp.
            fec.data()[16] ^= packet[1] & 0x7F; // Keep E while computing the independent PT recovery bits.
            for (size_t pos = 12; pos < packet.size(); ++pos) {
                fec.data()[28 + pos - 12] ^= packet[pos]; // Shorter members leave zero-extension bytes untouched.
            }
        }
        ts::PutUInt16(fec.data().data() + 14, length_xor); // Serialize the final recovery length in network byte order.
        ts::PutUInt32(fec.data().data() + 20, timestamp_xor); // Serialize the recovery timestamp in network byte order.
        return fec;
    }

    inline void RTPFECDecoderTest::_send(Decoder& decoder, const Media& media, const std::set<size_t>& lost)
    {
        for (size_t index = 0; index < media.size(); ++index) {
            if (!lost.contains(index)) { // Drop originals before they can enter the decoder.
                auto copy = media[index]; // Keep the independent expected wire packet intact.
                TSUNIT_ASSERT(decoder.addMedia(copy));
            }
        }
    }

    inline void RTPFECDecoderTest::_checkOutput(Decoder& decoder, const Media& media, const std::set<size_t>& lost)
    {
        Datagram output;
        // Nothing leaves before its deadline unless the finite window needs space.
        if (!decoder.isPlayoutBufferFull()) {
            TSUNIT_ASSERT(!decoder.getDatagram(output, _now()));
        }
        for (size_t index = 0; index < media.size(); ++index) {
            if (!lost.contains(index)) { // Compare only media which are expected to survive the selected losses.
                TSUNIT_ASSERT(decoder.getDatagram(output, _now() + cn::seconds(2)));
                TSUNIT_ASSERT(output.data() == media[index].data()); // Compare the whole RTP header and TS payload, not only sync.
                if (output.isRecovered()) {
                    TSUNIT_EQUAL(-1, output.receiveTimestamp().count()); // A recovered datagram has no actual kernel reception.
                    TSUNIT_ASSERT(output.receiveTimestampType() == ts::UDPSocket::TimeStampType::NONE); // Do not mislabel parity receive time as media receive time.
                }
                else {
                    TSUNIT_EQUAL(media[index].receiveTimestamp().count(), output.receiveTimestamp().count()); // Check metadata on unrecovered originals as well.
                    TSUNIT_ASSERT(output.receiveTimestampType() == media[index].receiveTimestampType()); // Original timestamp provenance is preserved.
                }
            }
        }
        TSUNIT_ASSERT(!decoder.getDatagram(output, _now() + cn::seconds(2)));
    }
}
