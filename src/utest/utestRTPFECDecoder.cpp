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

#include "tsRTPFECDecoder.h"
#include "tsMemory.h"
#include "tsTS.h"
#include "tsunit.h"


//----------------------------------------------------------------------------
// Fixture and independent media/parity generator.
//----------------------------------------------------------------------------

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
    TSUNIT_DECLARE_TEST(Pressure); // Shorten playout latency without discarding original media.

    using Decoder = ts::RTPFECDecoder;
    using Datagram = Decoder::Datagram;
    using Media = std::vector<Datagram>; // Expected packets are kept independently of decoder storage.
    static constexpr uint32_t SSRC = 0x12345678; // Parity RTP SSRC is zero; media use another value.

    // Deterministic clock: these tests never sleep or depend on wall time.
    static Decoder::TimePoint Now() { return Decoder::TimePoint(cn::seconds(1)); }
    static Datagram Packet(uint16_t sequence, size_t ts_size = 188, size_t count = 1);
    static Media Packets(size_t count, uint16_t base = 1000, size_t ts_size = 188, size_t ts_count = 1);
    static Datagram Parity(const Media& media, size_t base, size_t offset, size_t count, bool row = false);
    static void Send(Decoder& decoder, const Media& media, const std::set<size_t>& lost = {});
    static void CheckOutput(Decoder& decoder, const Media& media, const std::set<size_t>& lost = {});
};

TSUNIT_REGISTER(RTPFECDecoderTest);

ts::RTPFECDecoder::Datagram RTPFECDecoderTest::Packet(uint16_t sequence, size_t ts_size, size_t count)
{
    Datagram packet; // Construct a complete wire packet rather than a decoder object.
    packet.arrival = Now(); // Each test decides which later arrivals or deadlines differ.
    packet.timestamp = cn::microseconds(123456); // Original kernel timestamps must survive buffering.
    packet.timestamp_type = ts::UDPSocket::TimeStampType::SOFTWARE; // Recovery must not invent this metadata.
    packet.data.resize(12 + ts_size * count); // Support all mandatory 188/204-byte operating points.
    packet.data[0] = 0x80; // Fixed RTP header: no padding, extension or CSRC.
    packet.data[1] = 33; // The MPEG-TS RTP payload type, with marker zero.
    ts::PutUInt16(packet.data.data() + 2, sequence); // The supplied sequence already includes wire wrap.
    // Exercise 32-bit RTP timestamp wrapping as well as payload recovery.
    ts::PutUInt32(packet.data.data() + 4, 0xFFFF0000 + uint32_t(sequence) * 900);
    ts::PutUInt32(packet.data.data() + 8, SSRC); // Use a nonzero media source identifier.
    for (size_t index = 12; index < packet.data.size(); ++index) {
        packet.data[index] = uint8_t(sequence * 7 + index * 13); // Distinct payloads expose incorrect XOR or packet ordering.
    }
    for (size_t index = 12; index < packet.data.size(); index += ts_size) {
        packet.data[index] = 0x47; // TS sync at every boundary, including RS204 trailers.
        packet.data[index + 1] = 0x1F; // Use null packets so no PSI/PES parsing is required.
        packet.data[index + 2] = 0xFF; // Null PID 0x1FFF.
        packet.data[index + 3] = 0x10 | (sequence & 0x0F); // Distinct continuity values also participate in parity.
    }
    return packet;
}

RTPFECDecoderTest::Media RTPFECDecoderTest::Packets(size_t count, uint16_t base, size_t ts_size, size_t ts_count)
{
    Media packets;
    for (size_t index = 0; index < count; ++index) {
        packets.push_back(Packet(uint16_t(base + index), ts_size, ts_count)); // Casting deliberately wraps the RTP sequence.
    }
    return packets;
}

ts::RTPFECDecoder::Datagram RTPFECDecoderTest::Parity(const Media& media, size_t base, size_t offset, size_t count, bool row)
{
    // ST 2022-1 section 8.4: SNBase + j * Offset, 0 <= j < NA.
    // Compute a separate wire packet without calling any decoder methods.
    size_t length = 0; // Parity payload has the longest associated media length.
    for (size_t index = 0; index < count; ++index) {
        length = std::max(length, media[base + index * offset].data.size() - 12);
    }
    Datagram fec;
    fec.arrival = Now() + cn::milliseconds(10); // Parity normally follows media, but tests may override it.
    fec.data.resize(28 + length, 0); // Zero padding, mask, type, index and SSRC fields.
    fec.data[0] = 0x80; // Parity also has the fixed 12-byte RTP header.
    fec.data[1] = 96; // ST 2022-1 uses the first dynamic RTP payload type.
    ts::PutUInt16(fec.data.data() + 12, ts::GetUInt16(media[base].data.data() + 2)); // SNBase identifies media, not FEC RTP sequence.
    fec.data[16] = 0x80; // E=1; recovery PT bits are XORed below.
    fec.data[24] = row ? 0x40 : 0; // D distinguishes rows from columns.
    fec.data[25] = uint8_t(offset); // Select periodic media members independently of block alignment.
    fec.data[26] = uint8_t(count); // NA is a count, not the last member index.
    uint16_t length_xor = 0; // Length recovery is the XOR of payload lengths.
    uint32_t timestamp_xor = 0; // Timestamp recovery is independent of parity RTP timestamp.
    for (size_t index = 0; index < count; ++index) {
        const auto& packet = media[base + index * offset].data; // Select the exact members named by the FEC header.
        length_xor ^= uint16_t(packet.size() - 12); // Exclude the fixed RTP header from length recovery.
        timestamp_xor ^= ts::GetUInt32(packet.data() + 4); // Include wrap in the unsigned 32-bit timestamp.
        fec.data[16] ^= packet[1] & 0x7F; // Keep E while computing the independent PT recovery bits.
        for (size_t pos = 12; pos < packet.size(); ++pos) {
            fec.data[28 + pos - 12] ^= packet[pos]; // Shorter members leave zero-extension bytes untouched.
        }
    }
    ts::PutUInt16(fec.data.data() + 14, length_xor); // Serialize the final recovery length in network byte order.
    ts::PutUInt32(fec.data.data() + 20, timestamp_xor); // Serialize the recovery timestamp in network byte order.
    return fec;
}

void RTPFECDecoderTest::Send(Decoder& decoder, const Media& media, const std::set<size_t>& lost)
{
    for (size_t index = 0; index < media.size(); ++index) {
        if (!lost.contains(index)) { // Drop originals before they can enter the decoder.
            auto copy = media[index]; // Keep the independent expected wire packet intact.
            TSUNIT_ASSERT(decoder.addMedia(copy));
        }
    }
}

void RTPFECDecoderTest::CheckOutput(Decoder& decoder, const Media& media, const std::set<size_t>& lost)
{
    Datagram output;
    // Nothing leaves before its deadline unless the finite window needs space.
    if (!decoder.isPlayoutBufferFull()) {
        TSUNIT_ASSERT(!decoder.getDatagram(output, Now()));
    }
    for (size_t index = 0; index < media.size(); ++index) {
        if (!lost.contains(index)) { // Drop originals before they can enter the decoder.
            TSUNIT_ASSERT(decoder.getDatagram(output, Now() + cn::seconds(2)));
            TSUNIT_ASSERT(output.data == media[index].data); // Compare the whole RTP header and TS payload, not only sync.
            if (output.recovered) {
                TSUNIT_EQUAL(-1, output.timestamp.count()); // A recovered datagram has no actual kernel reception.
                TSUNIT_ASSERT(output.timestamp_type == ts::UDPSocket::TimeStampType::NONE); // Do not mislabel parity receive time as media receive time.
            }
            else {
                TSUNIT_EQUAL(media[index].timestamp.count(), output.timestamp.count()); // Check metadata on unrecovered originals as well.
                TSUNIT_ASSERT(output.timestamp_type == media[index].timestamp_type); // Original timestamp provenance is preserved.
            }
        }
    }
    TSUNIT_ASSERT(!decoder.getDatagram(output, Now() + cn::seconds(2)));
}


//----------------------------------------------------------------------------
// Recovery, including burst losses and row -> column -> row propagation.
//----------------------------------------------------------------------------

TSUNIT_DEFINE_TEST(Column)
{
    Decoder decoder;
    const Media media(Packets(16)); // A 4x4 source block with uniquely identifiable packets.
    Send(decoder, media, {5}); // Remove an interior column member, preserving both neighbors.
    TSUNIT_ASSERT(decoder.addFEC(Parity(media, 1, 4, 4), false));
    TSUNIT_EQUAL(1, decoder.recoveredPackets()); // A singleton recovery must be counted exactly once.
    CheckOutput(decoder, media);
    TSUNIT_EQUAL(0, decoder.lostPackets()); // Recovery or buffer pressure must not silently lose media.
}

TSUNIT_DEFINE_TEST(Iterative)
{
    Decoder decoder;
    const Media media(Packets(16)); // A 4x4 source block with uniquely identifiable packets.
    Send(decoder, media, {1, 5, 6}); // These losses require alternating row and column recovery.
    // These two equations cannot be solved until row zero recovers packet 1.
    TSUNIT_ASSERT(decoder.addFEC(Parity(media, 1, 4, 4), false));
    TSUNIT_ASSERT(decoder.addFEC(Parity(media, 4, 1, 4, true), true));
    TSUNIT_EQUAL(0, decoder.recoveredPackets()); // No underdetermined or invalid equation may produce media.
    TSUNIT_ASSERT(decoder.addFEC(Parity(media, 0, 1, 4, true), true));
    TSUNIT_EQUAL(3, decoder.recoveredPackets()); // All three intended iterative losses were reconstructed.
    // Repeated parity and late originals cannot create repeated output.
    TSUNIT_ASSERT(decoder.addFEC(Parity(media, 0, 1, 4, true), true));
    Send(decoder, media);
    TSUNIT_EQUAL(3, decoder.recoveredPackets()); // All three intended iterative losses were reconstructed.
    CheckOutput(decoder, media);
}

TSUNIT_DEFINE_TEST(MandatoryMatrices)
{
    // Every mandatory ST 2022-2 operating point, with a full L-packet burst loss.
    for (size_t columns = 1; columns <= 20; ++columns) {
        for (size_t rows = 4; rows <= 20 && rows * columns <= 100; ++rows) {
            Decoder decoder;
            const Media media(Packets(columns * rows, 65500));
            std::set<size_t> lost;
            for (size_t column = 0; column < columns; ++column) {
                lost.insert(2 * columns + column); // A whole interior row is one loss per column.
            }
            Send(decoder, media, lost);
            for (size_t column = 0; column < columns; ++column) {
                TSUNIT_ASSERT(decoder.addFEC(Parity(media, column, columns, rows), false));
            }
            TSUNIT_EQUAL(columns, decoder.recoveredPackets());
            CheckOutput(decoder, media);
        }
    }
}

TSUNIT_DEFINE_TEST(StaggeredColumns)
{
    Decoder decoder;
    const Media media(Packets(24)); // Enough media for staggered columns extending into later rows.
    Send(decoder, media, {4, 9, 14, 19}); // Lose one member of each independently staggered column.
    // The Annex B style columns begin in different rows, without block alignment.
    for (size_t base = 0; base < 4; ++base) {
        TSUNIT_ASSERT(decoder.addFEC(Parity(media, base * 5, 4, 3), false));
    }
    CheckOutput(decoder, media);
    TSUNIT_EQUAL(4, decoder.recoveredPackets());
}

TSUNIT_DEFINE_TEST(EarlyParity)
{
    Decoder decoder;
    const Media media(Packets(4));
    // Parity arrives first, and a missing leading packet must also be recovered.
    TSUNIT_ASSERT(decoder.addFEC(Parity(media, 0, 1, 4, true), true));
    Send(decoder, media, {0});
    CheckOutput(decoder, media);
    TSUNIT_EQUAL(1, decoder.recoveredPackets()); // A singleton recovery must be counted exactly once.
}


//----------------------------------------------------------------------------
// Sequence and payload handling, expiration and uncorrectable loss patterns.
//----------------------------------------------------------------------------

TSUNIT_DEFINE_TEST(WrapAndReorder)
{
    Decoder decoder;
    const Media media(Packets(8, 65532)); // Four packets before wrap and four after it.
    // Reorder across wrap and repeat originals before and after recovery.
    for (const size_t index : {2, 0, 1, 6, 4, 7, 3, 6}) {
        auto copy = media[index]; // Keep the independent expected wire packet intact.
        TSUNIT_ASSERT(decoder.addMedia(copy));
    }
    TSUNIT_ASSERT(decoder.addFEC(Parity(media, 4, 1, 4, true), true));
    CheckOutput(decoder, media);
    auto late = media[5]; // A recovered original can arrive after all output has drained.
    TSUNIT_ASSERT(decoder.addMedia(late));
    Datagram output;
    TSUNIT_ASSERT(!decoder.getDatagram(output, Now() + cn::seconds(3)));
}

TSUNIT_DEFINE_TEST(LengthsAndRS204)
{
    // All mandatory TS counts with both 188-byte and RS204 payloads.
    for (const size_t ts_size : {188, 204}) {
        for (const size_t count : {1, 4, 7}) {
            Decoder decoder;
            const Media media(Packets(16, 1000, ts_size, count));
            Send(decoder, media, {8}); // Recover the same position at every mandatory TS payload size.
            TSUNIT_ASSERT(decoder.addFEC(Parity(media, 0, 4, 4), false));
            CheckOutput(decoder, media);
        }
    }
    // Different lengths also exercise length recovery and zero-extension.
    Decoder decoder;
    const Media media {Packet(1000, 188, 1), Packet(1001, 188, 7), Packet(1002, 188, 4)};
    Send(decoder, media, {0});
    TSUNIT_ASSERT(decoder.addFEC(Parity(media, 0, 1, 3, true), true));
    CheckOutput(decoder, media);
}

TSUNIT_DEFINE_TEST(Unrecoverable)
{
    Decoder decoder;
    const Media media(Packets(16)); // A 4x4 source block with uniquely identifiable packets.
    const std::set<size_t> lost {5, 6, 9, 10}; // Two losses in each affected row and column.
    Send(decoder, media, lost);
    // A rectangle has two losses in each affected row and column.
    for (size_t index = 0; index < 4; ++index) {
        TSUNIT_ASSERT(decoder.addFEC(Parity(media, index, 4, 4), false));
        TSUNIT_ASSERT(decoder.addFEC(Parity(media, index * 4, 1, 4, true), true));
    }
    TSUNIT_EQUAL(0, decoder.recoveredPackets()); // No underdetermined or invalid equation may produce media.
    CheckOutput(decoder, media, lost);
    TSUNIT_EQUAL(4, decoder.lostPackets()); // All rectangle gaps must eventually expire.
    TSUNIT_EQUAL(0, decoder.bufferedFEC()); // Completed, expired or reset equations release their storage.
}

TSUNIT_DEFINE_TEST(LateParity)
{
    Decoder decoder;
    const Media media(Packets(4));
    Send(decoder, media, {1}); // A single gap whose expiry can be checked precisely.
    TSUNIT_EQUAL(1000, decoder.timeToNextDatagram(Now()).count());
    TSUNIT_EQUAL(1, decoder.timeToNextDatagram(Now() + cn::microseconds(999001)).count());
    CheckOutput(decoder, media, {1});
    TSUNIT_EQUAL(1, decoder.lostPackets()); // Late parity must not undo already reported media loss.
    // Past its deadline, missing media cannot be reinserted into playout.
    TSUNIT_ASSERT(!decoder.addFEC(Parity(media, 0, 1, 4, true), true));
    TSUNIT_EQUAL(0, decoder.recoveredPackets()); // No underdetermined or invalid equation may produce media.
    TSUNIT_ASSERT(decoder.timeToNextDatagram(Now()) == cn::milliseconds::max());
}


//----------------------------------------------------------------------------
// Reject malformed data, bound memory and isolate a restarted RTP session.
//----------------------------------------------------------------------------

TSUNIT_DEFINE_TEST(InvalidPackets)
{
    Decoder decoder;
    const Media media(Packets(4));
    Send(decoder, media, {1}); // A single gap whose expiry can be checked precisely.
    const Datagram good(Parity(media, 0, 1, 4, true));
    for (size_t size = 0; size <= 28; ++size) {
        auto bad = good;
        bad.data.resize(size); // Truncate at every boundary of the RTP and FEC headers.
        TSUNIT_ASSERT(!decoder.addFEC(bad, true));
    }
    // Version, RTP padding/extension/CC, PT, SSRC, E, mask, N, D, type,
    // index, row offset, NA and SNBase extension must all be checked.
    for (const auto& field : std::vector<std::pair<size_t, uint8_t>> {
             {0, 0x40}, {0, 0xA0}, {0, 0x90}, {0, 0x81}, {1, 97}, {8, 1},
             {16, 0}, {17, 1}, {24, 0xC0}, {24, 0}, {24, 0x48}, {24, 0x41},
             {25, 0}, {25, 2}, {26, 0}, {27, 1}}) {
        auto bad = good;
        bad.data[field.first] = field.second; // Change one field while keeping the rest of the packet valid.
        TSUNIT_ASSERT(!decoder.addFEC(bad, true));
    }
    // Valid headers with invalid recovery length, PT or TS sync cannot poison media.
    for (const auto& field : std::vector<std::pair<size_t, uint8_t>> {{14, 0xFF}, {16, 0xFF}, {28, 0xFF}}) {
        auto bad = good;
        bad.data[field.first] = field.second; // Change one field while keeping the rest of the packet valid.
        decoder.addFEC(bad, true);
        TSUNIT_EQUAL(0, decoder.recoveredPackets()); // No underdetermined or invalid equation may produce media.
    }
    for (const auto& field : std::vector<std::pair<size_t, uint8_t>> {{0, 0}, {1, 34}, {12, 0}}) {
        auto bad = media[1];
        bad.data[field.first] = field.second; // Change one field while keeping the rest of the packet valid.
        TSUNIT_ASSERT(!decoder.addMedia(bad));
    }
    TSUNIT_ASSERT(decoder.addFEC(good, true));
    CheckOutput(decoder, media);
}

TSUNIT_DEFINE_TEST(Bounds)
{
    Decoder decoder;
    decoder.reset(cn::seconds(60), 256); // Large latency forces the sequence-window limit to take effect.
    // A non-consuming receiver cannot retain unlimited media.
    for (uint16_t sequence = 0; sequence < 2000; ++sequence) {
        auto packet = Packet(sequence);
        TSUNIT_ASSERT(decoder.addMedia(packet));
        TSUNIT_ASSERT(decoder.bufferedMedia() <= 256);
    }
    Datagram output;
    TSUNIT_ASSERT(decoder.getDatagram(output, Now()));
    TSUNIT_EQUAL(1744, ts::GetUInt16(output.data.data() + 2)); // Only the newest 256 media remain after a stalled consumer.
    TSUNIT_EQUAL(1744, decoder.lostPackets()); // Overwritten original media are included in gap accounting.
    // A far-future, stale or overlong FEC equation is rejected without buffering.
    const Media future(Packets(300, 4000)); // This parity is outside the receiver sequence window.
    TSUNIT_ASSERT(!decoder.addFEC(Parity(future, 0, 1, 4, true), true));
    TSUNIT_EQUAL(0, decoder.bufferedFEC()); // Completed, expired or reset equations release their storage.
    decoder.reset();
    TSUNIT_EQUAL(0, decoder.bufferedMedia());
    TSUNIT_EQUAL(0, decoder.lostPackets()); // Recovery or buffer pressure must not silently lose media.
}

TSUNIT_DEFINE_TEST(Restart)
{
    Decoder decoder;
    const Media media(Packets(4));
    Send(decoder, media, {1, 2}); // Both equations initially have two unknown members.
    TSUNIT_ASSERT(decoder.addFEC(Parity(media, 0, 1, 4, true), true));
    TSUNIT_EQUAL(1, decoder.bufferedFEC());
    auto fresh = Packet(50); // The restarted session can use a completely different sequence.
    ts::PutUInt32(fresh.data.data() + 8, SSRC + 1); // A new SSRC must not inherit old equations.
    const auto expected = fresh;
    TSUNIT_ASSERT(decoder.addMedia(fresh));
    TSUNIT_EQUAL(0, decoder.bufferedFEC()); // Completed, expired or reset equations release their storage.
    CheckOutput(decoder, {expected});
}

TSUNIT_DEFINE_TEST(ParityBounds)
{
    Decoder decoder;
    decoder.reset(cn::seconds(1), 256); // Use the minimum buffer to make flood bounds observable.
    const Media media(Packets(256));
    auto first = media.front();
    TSUNIT_ASSERT(decoder.addMedia(first));
    size_t accepted = 0; // Count accepted overlap separately from media recovery.
    // Overlapping incomplete equations also bound their reverse index, not
    // just parity payload storage. Normal FEC has at most two references per media.
    for (size_t base = 0; base < 252; ++base) {
        accepted += decoder.addFEC(Parity(media, base, 1, 4), false) ? 1 : 0;
        TSUNIT_ASSERT(decoder.bufferedFEC() <= 128);
    }
    TSUNIT_ASSERT(accepted > 0 && accepted < 252);
    // Completing the equations must release every reverse reference safely.
    Send(decoder, media);
    TSUNIT_EQUAL(0, decoder.bufferedFEC()); // Completed, expired or reset equations release their storage.
    CheckOutput(decoder, media);

    // Before sequence anchoring, even a flood of duplicated parity is bounded.
    decoder.reset(cn::seconds(1), 256); // Use the minimum buffer to make flood bounds observable.
    const Media small(Packets(4));
    const auto parity = Parity(small, 0, 1, 4, true);
    for (size_t index = 0; index < 1000; ++index) {
        TSUNIT_ASSERT(decoder.addFEC(parity, true) == (index < 256));
        TSUNIT_ASSERT(decoder.bufferedFEC() <= 256);
    }
    Send(decoder, small, {1});
    CheckOutput(decoder, small);
    TSUNIT_EQUAL(1, decoder.recoveredPackets()); // A singleton recovery must be counted exactly once.
}

TSUNIT_DEFINE_TEST(History)
{
    Decoder decoder;
    decoder.reset(cn::milliseconds(100));
    Media media(Packets(4));
    media[2].arrival += cn::milliseconds(100); // Later media establish a later deadline for the missing gap.
    media[3].arrival += cn::milliseconds(100); // The first played packet must remain available as history.
    Send(decoder, media, {1}); // A single gap whose expiry can be checked precisely.
    Datagram output;
    // Packet zero leaves playout before column parity arrives, but must remain
    // in history so that packet one can still be recovered before the gap expires.
    TSUNIT_ASSERT(decoder.getDatagram(output, Now() + cn::milliseconds(100)));
    TSUNIT_ASSERT(output.data == media[0].data); // The first original has already left playout.
    TSUNIT_ASSERT(!decoder.getDatagram(output, Now() + cn::milliseconds(150)));
    TSUNIT_ASSERT(decoder.addFEC(Parity(media, 0, 1, 4), false));
    TSUNIT_ASSERT(decoder.getDatagram(output, Now() + cn::milliseconds(150)));
    TSUNIT_ASSERT(output.data == media[1].data); // The next output must be the recovered missing member.
    TSUNIT_EQUAL(0, decoder.lostPackets()); // Recovery or buffer pressure must not silently lose media.
}

TSUNIT_DEFINE_TEST(LongRun)
{
    Decoder decoder;
    decoder.reset(cn::milliseconds(10), 256); // Continuously drain a small window over multiple RTP cycles.
    size_t emitted = 0;
    const size_t groups = 17500; // Seventy thousand media starting near the wrap boundary.
    auto drain = [&](Decoder::TimePoint now) {
        Datagram output;
        while (decoder.getDatagram(output, now)) {
            // More than a full sequence cycle must preserve exact packet order.
            TSUNIT_ASSERT(output.data == Packet(uint16_t(65000 + emitted)).data);
            ++emitted;
        }
    };
    for (size_t group = 0; group < groups; ++group) {
        Media media(Packets(4, uint16_t(65000 + 4 * group)));
        for (size_t index = 0; index < 4; ++index) {
            media[index].arrival += cn::milliseconds(4 * group + index);
        }
        const bool lost = group % 8 == 0; // A regular sparse loss pattern with a known recovery count.
        Send(decoder, media, lost ? std::set<size_t> {1} : std::set<size_t> {});
        auto parity = Parity(media, 0, 1, 4, true);
        parity.arrival = Now() + cn::milliseconds(4 * group + 4);
        TSUNIT_ASSERT(decoder.addFEC(parity, true));
        drain(parity.arrival); // Advance monotonic time while continuing to receive media.
        TSUNIT_ASSERT(decoder.bufferedMedia() <= 256);
    }
    drain(Now() + cn::seconds(1000)); // Drain the finite tail without sleeping.
    TSUNIT_EQUAL(4 * groups, emitted); // Every original sequence must appear exactly once.
    TSUNIT_EQUAL((groups + 7) / 8, decoder.recoveredPackets()); // One recovery for each selected group.
    TSUNIT_EQUAL(0, decoder.lostPackets()); // Recovery or buffer pressure must not silently lose media.
}

TSUNIT_DEFINE_TEST(DuplicateEquations)
{
    Decoder decoder;
    const Media media(Packets(4));
    Send(decoder, media, {1, 2}); // Both equations initially have two unknown members.
    // Two overlapping equations become solvable on the same media insertion.
    // They must count and insert their common recovered packet only once.
    TSUNIT_ASSERT(decoder.addFEC(Parity(media, 0, 1, 4), false));
    TSUNIT_ASSERT(decoder.addFEC(Parity(media, 0, 1, 4, true), true));
    auto original = media[1]; // One known member makes both pending equations solvable.
    TSUNIT_ASSERT(decoder.addMedia(original));
    TSUNIT_EQUAL(1, decoder.recoveredPackets()); // A singleton recovery must be counted exactly once.
    TSUNIT_EQUAL(0, decoder.bufferedFEC()); // Completed, expired or reset equations release their storage.
    CheckOutput(decoder, media);
}

TSUNIT_DEFINE_TEST(Pressure)
{
    Decoder decoder;
    decoder.reset(cn::seconds(60), 256); // Large latency forces the sequence-window limit to take effect.
    const Media media(Packets(1024)); // Four complete sequence windows with no network loss.
    size_t emitted = 0;
    Datagram output;
    for (const auto& packet : media) {
        auto copy = packet;
        TSUNIT_ASSERT(decoder.addMedia(copy));
        // Drain at the window boundary before new input can overwrite history.
        // A too-small buffer shortens latency without losing original media.
        while (decoder.getDatagram(output, Now())) {
            TSUNIT_ASSERT(output.data == media[emitted++].data);
        }
        TSUNIT_ASSERT(!decoder.isPlayoutBufferFull()); // Playout must release enough space before the next insertion.
        TSUNIT_ASSERT(decoder.bufferedMedia() <= 256);
    }
    while (decoder.getDatagram(output, Now() + cn::seconds(61))) {
        TSUNIT_ASSERT(output.data == media[emitted++].data);
    }
    TSUNIT_EQUAL(media.size(), emitted); // Pressure may shorten latency, but must preserve all originals.
    TSUNIT_EQUAL(0, decoder.lostPackets()); // Recovery or buffer pressure must not silently lose media.
}
