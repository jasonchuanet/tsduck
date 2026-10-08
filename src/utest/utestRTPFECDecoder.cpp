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

#include "utestRTPFECDecoder.h"
#include "tsMemory.h"
#include "tsTS.h"


//----------------------------------------------------------------------------
// Fixture and independent media/parity generator.
//----------------------------------------------------------------------------

namespace ts {

    TSUNIT_REGISTER(RTPFECDecoderTest);

    //----------------------------------------------------------------------------
    // Recovery, including burst losses and row -> column -> row propagation.
    //----------------------------------------------------------------------------

    TSUNIT_DEFINE_TEST(Column)
    {
        Decoder decoder;
        const Media media(_makePackets(16)); // A 4x4 source block with uniquely identifiable packets.
        _send(decoder, media, {5}); // Remove an interior column member, preserving both neighbors.
        TSUNIT_ASSERT(decoder.addFEC(_makeParity(media, 1, 4, 4), false));
        TSUNIT_EQUAL(1, decoder.recoveredPackets()); // A singleton recovery must be counted exactly once.
        _checkOutput(decoder, media);
        TSUNIT_EQUAL(0, decoder.lostPackets()); // Recovery or buffer pressure must not silently lose media.
    }

    TSUNIT_DEFINE_TEST(Iterative)
    {
        Decoder decoder;
        const Media media(_makePackets(16)); // A 4x4 source block with uniquely identifiable packets.
        _send(decoder, media, {1, 5, 6}); // These losses require alternating row and column recovery.
        // These two equations cannot be solved until row zero recovers packet 1.
        TSUNIT_ASSERT(decoder.addFEC(_makeParity(media, 1, 4, 4), false));
        TSUNIT_ASSERT(decoder.addFEC(_makeParity(media, 4, 1, 4, true), true));
        TSUNIT_EQUAL(0, decoder.recoveredPackets()); // No underdetermined or invalid equation may produce media.
        TSUNIT_ASSERT(decoder.addFEC(_makeParity(media, 0, 1, 4, true), true));
        TSUNIT_EQUAL(3, decoder.recoveredPackets()); // All three intended iterative losses were reconstructed.
        // Repeated parity and late originals cannot create repeated output.
        TSUNIT_ASSERT(decoder.addFEC(_makeParity(media, 0, 1, 4, true), true));
        _send(decoder, media);
        TSUNIT_EQUAL(3, decoder.recoveredPackets()); // All three intended iterative losses were reconstructed.
        _checkOutput(decoder, media);
    }

    TSUNIT_DEFINE_TEST(MandatoryMatrices)
    {
        // Every mandatory ST 2022-2 operating point, with a full L-packet burst loss.
        for (size_t columns = 1; columns <= 20; ++columns) {
            for (size_t rows = 4; rows <= 20 && rows * columns <= 100; ++rows) {
                Decoder decoder;
                const Media media(_makePackets(columns * rows, 65500));
                std::set<size_t> lost;
                for (size_t column = 0; column < columns; ++column) {
                    lost.insert(2 * columns + column); // A whole interior row is one loss per column.
                }
                _send(decoder, media, lost);
                for (size_t column = 0; column < columns; ++column) {
                    TSUNIT_ASSERT(decoder.addFEC(_makeParity(media, column, columns, rows), false));
                }
                TSUNIT_EQUAL(columns, decoder.recoveredPackets());
                _checkOutput(decoder, media);
            }
        }
    }

    TSUNIT_DEFINE_TEST(StaggeredColumns)
    {
        Decoder decoder;
        const Media media(_makePackets(24)); // Enough media for staggered columns extending into later rows.
        _send(decoder, media, {4, 9, 14, 19}); // Lose one member of each independently staggered column.
        // The Annex B style columns begin in different rows, without block alignment.
        for (size_t base = 0; base < 4; ++base) {
            TSUNIT_ASSERT(decoder.addFEC(_makeParity(media, base * 5, 4, 3), false));
        }
        _checkOutput(decoder, media);
        TSUNIT_EQUAL(4, decoder.recoveredPackets());
    }

    TSUNIT_DEFINE_TEST(EarlyParity)
    {
        Decoder decoder;
        const Media media(_makePackets(4));
        // Parity arrives first, and a missing leading packet must also be recovered.
        TSUNIT_ASSERT(decoder.addFEC(_makeParity(media, 0, 1, 4, true), true));
        _send(decoder, media, {0});
        _checkOutput(decoder, media);
        TSUNIT_EQUAL(1, decoder.recoveredPackets()); // A singleton recovery must be counted exactly once.
    }


    //----------------------------------------------------------------------------
    // Sequence and payload handling, expiration and uncorrectable loss patterns.
    //----------------------------------------------------------------------------

    TSUNIT_DEFINE_TEST(WrapAndReorder)
    {
        Decoder decoder;
        const Media media(_makePackets(8, 65532)); // Four packets before wrap and four after it.
        // Reorder across wrap and repeat originals before and after recovery.
        for (const size_t index : {2, 0, 1, 6, 4, 7, 3, 6}) {
            auto copy = media[index]; // Keep the independent expected wire packet intact.
            TSUNIT_ASSERT(decoder.addMedia(copy));
        }
        TSUNIT_ASSERT(decoder.addFEC(_makeParity(media, 4, 1, 4, true), true));
        _checkOutput(decoder, media);
        auto late = media[5]; // A recovered original can arrive after all output has drained.
        TSUNIT_ASSERT(decoder.addMedia(late));
        Datagram output;
        TSUNIT_ASSERT(!decoder.getDatagram(output, _now() + cn::seconds(3)));
    }

    TSUNIT_DEFINE_TEST(LengthsAndRS204)
    {
        // All mandatory TS counts with both 188-byte and RS204 payloads.
        for (const size_t& ts_size : std::initializer_list<size_t> {188, 204}) {
            for (const size_t& count : std::initializer_list<size_t> {1, 4, 7}) {
                Decoder decoder;
                const Media media(_makePackets(16, 1000, ts_size, count));
                _send(decoder, media, {8}); // Recover the same position at every mandatory TS payload size.
                TSUNIT_ASSERT(decoder.addFEC(_makeParity(media, 0, 4, 4), false));
                _checkOutput(decoder, media);
            }
        }
        // Different lengths also exercise length recovery and zero-extension.
        Decoder decoder;
        const Media media {_makePacket(1000, 188, 1), _makePacket(1001, 188, 7), _makePacket(1002, 188, 4)};
        _send(decoder, media, {0});
        TSUNIT_ASSERT(decoder.addFEC(_makeParity(media, 0, 1, 3, true), true));
        _checkOutput(decoder, media);
    }

    TSUNIT_DEFINE_TEST(Unrecoverable)
    {
        Decoder decoder;
        const Media media(_makePackets(16)); // A 4x4 source block with uniquely identifiable packets.
        const std::set<size_t> lost {5, 6, 9, 10}; // Two losses in each affected row and column.
        _send(decoder, media, lost);
        // A rectangle has two losses in each affected row and column.
        for (size_t index = 0; index < 4; ++index) {
            TSUNIT_ASSERT(decoder.addFEC(_makeParity(media, index, 4, 4), false));
            TSUNIT_ASSERT(decoder.addFEC(_makeParity(media, index * 4, 1, 4, true), true));
        }
        TSUNIT_EQUAL(0, decoder.recoveredPackets()); // No underdetermined or invalid equation may produce media.
        _checkOutput(decoder, media, lost);
        TSUNIT_EQUAL(4, decoder.lostPackets()); // All rectangle gaps must eventually expire.
        TSUNIT_EQUAL(0, decoder.bufferedFEC()); // Completed, expired or reset equations release their storage.
    }

    TSUNIT_DEFINE_TEST(LateParity)
    {
        // Expired gaps cannot be undone or emitted a second time.
        Decoder decoder;
        const Media media(_makePackets(4));
        _send(decoder, media, {1}); // A single gap whose expiry can be checked precisely.
        TSUNIT_EQUAL(1000, decoder.timeToNextDatagram(_now()).count());
        TSUNIT_EQUAL(1, decoder.timeToNextDatagram(_now() + cn::microseconds(999001)).count());
        _checkOutput(decoder, media, {1});
        TSUNIT_EQUAL(1, decoder.lostPackets()); // Late parity must not undo already reported media loss.
        // Past its deadline, missing media cannot be reinserted into playout.
        TSUNIT_ASSERT(!decoder.addFEC(_makeParity(media, 0, 1, 4, true), true));
        TSUNIT_EQUAL(0, decoder.recoveredPackets()); // No underdetermined or invalid equation may produce media.
        TSUNIT_ASSERT(decoder.timeToNextDatagram(_now()) == cn::milliseconds::max());
    }


    //----------------------------------------------------------------------------
    // Reject malformed data, bound memory and isolate a restarted RTP session.
    //----------------------------------------------------------------------------

    TSUNIT_DEFINE_TEST(InvalidPackets)
    {
        // Each independent header restriction is tested before accepting valid parity.
        Decoder decoder;
        const Media media(_makePackets(4));
        _send(decoder, media, {1}); // A single gap whose expiry can be checked precisely.
        const Datagram good(_makeParity(media, 0, 1, 4, true));
        for (size_t size = 0; size <= 28; ++size) {
            auto bad = good;
            bad.data().resize(size); // Truncate at every boundary of the RTP and FEC headers.
            TSUNIT_ASSERT(!decoder.addFEC(bad, true));
        }
        // Version, RTP padding/extension/CC, PT, SSRC, E, mask, N, D, type,
        // index, row offset, NA and SNBase extension must all be checked.
        for (const auto& field : std::vector<std::pair<size_t, uint8_t>> {
                 {0, 0x40}, {0, 0xA0}, {0, 0x90}, {0, 0x81}, {1, 97}, {8, 1},
                 {16, 0}, {17, 1}, {24, 0xC0}, {24, 0}, {24, 0x48}, {24, 0x41},
                 {25, 0}, {25, 2}, {26, 0}, {27, 1}}) {
            auto bad = good;
            bad.data()[field.first] = field.second; // Change one field while keeping the rest of the packet valid.
            TSUNIT_ASSERT(!decoder.addFEC(bad, true));
        }
        // Valid headers with invalid recovery length, PT or TS sync cannot poison media.
        for (const auto& field : std::vector<std::pair<size_t, uint8_t>> {{14, 0xFF}, {16, 0xFF}, {28, 0xFF}}) {
            auto bad = good;
            bad.data()[field.first] = field.second; // Change one field while keeping the rest of the packet valid.
            TSUNIT_ASSERT(decoder.addFEC(bad, true));
            TSUNIT_EQUAL(0, decoder.recoveredPackets()); // No underdetermined or invalid equation may produce media.
        }
        for (const auto& field : std::vector<std::pair<size_t, uint8_t>> {{0, 0}, {1, 34}, {12, 0}}) {
            auto bad = media[1];
            bad.data()[field.first] = field.second; // Change one field while keeping the rest of the packet valid.
            TSUNIT_ASSERT(!decoder.addMedia(bad));
        }
        TSUNIT_ASSERT(decoder.addFEC(good, true));
        _checkOutput(decoder, media);
    }

    TSUNIT_DEFINE_TEST(Bounds)
    {
        // Stalled consumers must have a finite, observable history bound.
        Decoder decoder;
        decoder.reset(cn::seconds(60), 256); // Large latency forces the sequence-window limit to take effect.
        // A non-consuming receiver cannot retain unlimited media.
        for (uint16_t sequence = 0; sequence < 2000; ++sequence) {
            auto packet = _makePacket(sequence);
            TSUNIT_ASSERT(decoder.addMedia(packet));
            TSUNIT_ASSERT(decoder.bufferedMedia() <= 256);
        }
        Datagram output;
        TSUNIT_ASSERT(decoder.getDatagram(output, _now()));
        TSUNIT_EQUAL(1744, ts::GetUInt16(output.data().data() + 2)); // Only the newest 256 media remain after a stalled consumer.
        TSUNIT_EQUAL(1744, decoder.lostPackets()); // Overwritten original media are included in gap accounting.
        // A far-future, stale or overlong FEC equation is rejected without buffering.
        const Media future(_makePackets(300, 4000)); // This parity is outside the receiver sequence window.
        TSUNIT_ASSERT(!decoder.addFEC(_makeParity(future, 0, 1, 4, true), true));
        TSUNIT_EQUAL(0, decoder.bufferedFEC()); // Completed, expired or reset equations release their storage.
        decoder.reset();
        TSUNIT_EQUAL(0, decoder.bufferedMedia());
        TSUNIT_EQUAL(0, decoder.lostPackets()); // Recovery or buffer pressure must not silently lose media.
    }

    TSUNIT_DEFINE_TEST(Restart)
    {
        // Parity from an old SSRC must not reconstruct media in the new session.
        Decoder decoder;
        const Media media(_makePackets(4));
        _send(decoder, media, {1, 2}); // Both equations initially have two unknown members.
        TSUNIT_ASSERT(decoder.addFEC(_makeParity(media, 0, 1, 4, true), true));
        TSUNIT_EQUAL(1, decoder.bufferedFEC());
        auto fresh = _makePacket(50); // The restarted session can use a completely different sequence.
        ts::PutUInt32(fresh.data().data() + 8, _ssrc + 1); // A new SSRC must not inherit old equations.
        const auto expected = fresh;
        TSUNIT_ASSERT(decoder.addMedia(fresh));
        TSUNIT_EQUAL(0, decoder.bufferedFEC()); // Completed, expired or reset equations release their storage.
        _checkOutput(decoder, {expected});
    }

    TSUNIT_DEFINE_TEST(ParityBounds)
    {
        // Payload bounds alone do not bound overlapping reverse references.
        Decoder decoder;
        decoder.reset(cn::seconds(1), 256); // Use the minimum buffer to make flood bounds observable.
        const Media media(_makePackets(256));
        auto first = media.front();
        TSUNIT_ASSERT(decoder.addMedia(first));
        size_t accepted = 0; // Count accepted overlap separately from media recovery.
        // Overlapping incomplete equations also bound their reverse index, not
        // just parity payload storage. Normal FEC has at most two references per media.
        for (size_t base = 0; base < 252; ++base) {
            accepted += decoder.addFEC(_makeParity(media, base, 1, 4), false) ? 1 : 0;
            TSUNIT_ASSERT(decoder.bufferedFEC() <= 128);
        }
        TSUNIT_ASSERT(accepted > 0 && accepted < 252);
        // Completing the equations must release every reverse reference safely.
        _send(decoder, media);
        TSUNIT_EQUAL(0, decoder.bufferedFEC()); // Completed, expired or reset equations release their storage.
        _checkOutput(decoder, media);

        // Before sequence anchoring, even a flood of duplicated parity is bounded.
        decoder.reset(cn::seconds(1), 256); // Use the minimum buffer to make flood bounds observable.
        const Media small(_makePackets(4));
        const auto parity = _makeParity(small, 0, 1, 4, true);
        for (size_t index = 0; index < 1000; ++index) {
            TSUNIT_ASSERT(decoder.addFEC(parity, true) == (index < 256));
            TSUNIT_ASSERT(decoder.bufferedFEC() <= 256);
        }
        _send(decoder, small, {1});
        _checkOutput(decoder, small);
        TSUNIT_EQUAL(1, decoder.recoveredPackets()); // A singleton recovery must be counted exactly once.
    }

    TSUNIT_DEFINE_TEST(History)
    {
        // Playout completion is different from history expiry for later parity.
        Decoder decoder;
        decoder.reset(cn::milliseconds(100));
        Media media(_makePackets(4));
        media[2].setArrival(media[2].arrival() + cn::milliseconds(100)); // Later media establish a later deadline for the missing gap.
        media[3].setArrival(media[3].arrival() + cn::milliseconds(100)); // The first played packet must remain available as history.
        _send(decoder, media, {1}); // A single gap whose expiry can be checked precisely.
        Datagram output;
        // Packet zero leaves playout before column parity arrives, but must remain
        // in history so that packet one can still be recovered before the gap expires.
        TSUNIT_ASSERT(decoder.getDatagram(output, _now() + cn::milliseconds(100)));
        TSUNIT_ASSERT(output.data() == media[0].data()); // The first original has already left playout.
        TSUNIT_ASSERT(!decoder.getDatagram(output, _now() + cn::milliseconds(150)));
        TSUNIT_ASSERT(decoder.addFEC(_makeParity(media, 0, 1, 4), false));
        TSUNIT_ASSERT(decoder.getDatagram(output, _now() + cn::milliseconds(150)));
        TSUNIT_ASSERT(output.data() == media[1].data()); // The next output must be the recovered missing member.
        TSUNIT_EQUAL(0, decoder.lostPackets()); // Recovery or buffer pressure must not silently lose media.
    }

    TSUNIT_DEFINE_TEST(LongRun)
    {
        // Exercise ongoing recovery over more than a full sequence cycle.
        Decoder decoder;
        decoder.reset(cn::milliseconds(10), 256); // Continuously drain a small window over multiple RTP cycles.
        size_t emitted = 0;
        const size_t groups = 17500; // Seventy thousand media starting near the wrap boundary.
        auto drain = [&](const Decoder::TimePoint& now) {
            Datagram output;
            while (decoder.getDatagram(output, now)) {
                // More than a full sequence cycle must preserve exact packet order.
                TSUNIT_ASSERT(output.data() == _makePacket(static_cast<uint16_t>(65000 + emitted)).data());
                ++emitted;
            }
        };
        for (size_t group = 0; group < groups; ++group) {
            Media media(_makePackets(4, static_cast<uint16_t>(65000 + 4 * group)));
            for (size_t index = 0; index < 4; ++index) {
                media[index].setArrival(media[index].arrival() + cn::milliseconds(4 * group + index));
            }
            const bool lost = group % 8 == 0; // A regular sparse loss pattern with a known recovery count.
            _send(decoder, media, lost ? std::set<size_t> {1} : std::set<size_t> {});
            auto parity = _makeParity(media, 0, 1, 4, true);
            parity.setArrival(_now() + cn::milliseconds(4 * group + 4));
            TSUNIT_ASSERT(decoder.addFEC(parity, true));
            drain(parity.arrival()); // Advance monotonic time while continuing to receive media.
            TSUNIT_ASSERT(decoder.bufferedMedia() <= 256);
        }
        drain(_now() + cn::seconds(1000)); // Drain the finite tail without sleeping.
        TSUNIT_EQUAL(4 * groups, emitted); // Every original sequence must appear exactly once.
        TSUNIT_EQUAL((groups + 7) / 8, decoder.recoveredPackets()); // One recovery for each selected group.
        TSUNIT_EQUAL(0, decoder.lostPackets()); // Recovery or buffer pressure must not silently lose media.
    }

    TSUNIT_DEFINE_TEST(DuplicateEquations)
    {
        // Concurrent singleton equations must not duplicate their common solution.
        Decoder decoder;
        const Media media(_makePackets(4));
        _send(decoder, media, {1, 2}); // Both equations initially have two unknown members.
        // Two overlapping equations become solvable on the same media insertion.
        // They must count and insert their common recovered packet only once.
        TSUNIT_ASSERT(decoder.addFEC(_makeParity(media, 0, 1, 4), false));
        TSUNIT_ASSERT(decoder.addFEC(_makeParity(media, 0, 1, 4, true), true));
        auto original = media[1]; // One known member makes both pending equations solvable.
        TSUNIT_ASSERT(decoder.addMedia(original));
        TSUNIT_EQUAL(1, decoder.recoveredPackets()); // A singleton recovery must be counted exactly once.
        TSUNIT_EQUAL(0, decoder.bufferedFEC()); // Completed, expired or reset equations release their storage.
        _checkOutput(decoder, media);
    }

    TSUNIT_DEFINE_TEST(RejectedParity)
    {
        // Deep validation must be transactional with respect to startup ordering.
        Decoder decoder;
        const Media media(_makePackets(8));
        const Media tail(media.begin() + 4, media.end());
        auto invalid_clock = media[0];
        invalid_clock.setArrival(Decoder::TimePoint::max()); // A deadline must not overflow the clock representation.
        TSUNIT_ASSERT(!decoder.addMedia(invalid_clock));
        auto invalid_parity_clock = _makeParity(media, 0, 1, 8, true);
        invalid_parity_clock.setArrival(Decoder::TimePoint::max());
        TSUNIT_ASSERT(!decoder.addFEC(invalid_parity_clock, true));
        _send(decoder, tail); // Startup is anchored by actual media, not unsupported parity.
        auto bad = _makeParity(media, 0, 1, 8, true);
        bad.data().resize(28 + 100); // Shorter than each known associated media payload.
        TSUNIT_ASSERT(!decoder.addFEC(bad, true));
        _checkOutput(decoder, tail);
        TSUNIT_EQUAL(0, decoder.lostPackets()); // Rejection must not invent leading losses.
        TSUNIT_EQUAL(0, decoder.bufferedFEC()); // Nor may it leave partial reverse references.
    }

    TSUNIT_DEFINE_TEST(Pressure)
    {
        // A finite window must shorten delay before overwriting pending originals.
        Decoder decoder;
        decoder.reset(cn::seconds(60), 256); // Large latency forces the sequence-window limit to take effect.
        const Media media(_makePackets(1024)); // Four complete sequence windows with no network loss.
        size_t emitted = 0;
        Datagram output;
        for (const auto& packet : media) {
            auto copy = packet;
            TSUNIT_ASSERT(decoder.addMedia(copy));
            // Drain at the window boundary before new input can overwrite history.
            // A too-small buffer shortens latency without losing original media.
            while (decoder.getDatagram(output, _now())) {
                TSUNIT_ASSERT(emitted < media.size());
                TSUNIT_ASSERT(output.data() == media[emitted].data());
                ++emitted;
            }
            TSUNIT_ASSERT(!decoder.isPlayoutBufferFull()); // Playout must release enough space before the next insertion.
            TSUNIT_ASSERT(decoder.bufferedMedia() <= 256);
        }
        while (decoder.getDatagram(output, _now() + cn::seconds(61))) {
            TSUNIT_ASSERT(emitted < media.size());
            TSUNIT_ASSERT(output.data() == media[emitted].data());
            ++emitted;
        }
        TSUNIT_EQUAL(media.size(), emitted); // Pressure may shorten latency, but must preserve all originals.
        TSUNIT_EQUAL(0, decoder.lostPackets()); // Recovery or buffer pressure must not silently lose media.
    }
}
