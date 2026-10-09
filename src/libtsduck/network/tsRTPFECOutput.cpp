//----------------------------------------------------------------------------
//
// TSDuck - The MPEG Transport Stream Toolkit
// Copyright (c) 2026, Jason Chua
// BSD-2-Clause license, see LICENSE.txt file or https://tsduck.io/license
//
//----------------------------------------------------------------------------
// FEC is opt-in and uses the media socket, as required by ST 2022-1 section 9.

#include "tsRTPFECOutput.h"
#include "tsArgs.h"
#include "tsIPProtocols.h"

namespace ts {
    namespace {
        constexpr size_t MAX_STREAMS = 2; // One column stream and one optional row stream.
        constexpr uint16_t PORT_STEP = 2; // Leave each RTP stream's adjacent RTCP port available.
        constexpr uint16_t MAX_PORT = 65535; // Validate before narrowing destination port arithmetic.
        constexpr int32_t DISABLED = 0; // Match the established dektec default, explicit "none".
        constexpr int32_t COLUMN = 1; // Staggered column-only FEC, ST 2022-1 Level A.
        constexpr int32_t COLUMN_BLOCK = 2; // The -b suffix has the same alignment meaning as dektec.
        constexpr int32_t COLUMN_ROW = 3; // Staggered columns plus consecutive rows, Level B.
        constexpr int32_t COLUMN_ROW_BLOCK = 4; // Block-aligned columns plus consecutive rows.
    }
}

//----------------------------------------------------------------------------
// The owning datagram output provides the error sink and the already-open socket.
//----------------------------------------------------------------------------

ts::RTPFECOutput::RTPFECOutput(Report& report) :
    _report(report)
{
}

// The owning library provides one vtable definition to shared-library callers.
ts::RTPFECOutput::~RTPFECOutput() = default;

void ts::RTPFECOutput::defineArgs(Args& args) const
{
    // A supplied selector requires a value, just like dektec output; omission disables FEC.
    args.option(u"smpte-2022-fec", 0, Names({
        {u"none", DISABLED},
        {u"1d", COLUMN},
        {u"1d-b", COLUMN_BLOCK},
        {u"2d", COLUMN_ROW},
        {u"2d-b", COLUMN_ROW_BLOCK},
    }));
    args.help(u"smpte-2022-fec", u"mode",
              u"With --rtp, generate SMPTE ST 2022-1 column FEC (1d) or column and row FEC (2d). "
              u"The suffix -b selects block alignment; otherwise columns are staggered. The default is none. "
              u"FEC uses the media destination port +2 and +4, with the same source port. "
              u"Requires an even media port, payload type 33 and at most seven TS packets per datagram. "
              u"Use --enforce-burst for a constant media packet count.");
    args.option(u"smpte-2022-l", 0, Args::INTEGER, 0, 1, 1, RTPFECEncoder::MAX_COLUMNS); // Reuse dektec's L: columns.
    args.help(u"smpte-2022-l", u"count",
              u"Number of FEC columns L, from 1 to 20 (at least 4 with 2D FEC). The default is 4. "
              u"Requires enabled --smpte-2022-fec. Columns times rows must not exceed 100.");
    args.option(u"smpte-2022-d", 0, Args::INTEGER, 0, 1, RTPFECEncoder::MIN_ROWS, RTPFECEncoder::MAX_ROWS); // Reuse dektec's D: rows.
    args.help(u"smpte-2022-d", u"count",
              u"Number of FEC rows D, from 4 to 20. The default is 4. Requires enabled --smpte-2022-fec. "
              u"Columns times rows must not exceed 100. Parity is interleaved with subsequent media.");
}

//----------------------------------------------------------------------------
// Reject configuration incompatibilities before opening or transmitting media.
//----------------------------------------------------------------------------

bool ts::RTPFECOutput::loadArgs(Args& args, const IPSocketAddress& destination, bool rtp, uint8_t payload_type, size_t burst)
{
    // Named modes select the wire dimension and alignment, independently of vendor pacing labels.
    const int32_t mode = args.intValue<int32_t>(u"smpte-2022-fec", DISABLED);
    if (mode < DISABLED || mode > COLUMN_ROW_BLOCK) {
        args.error(u"invalid SMPTE-2022 FEC mode"); // Reject numeric enumeration values outside our supported modes too.
        return false;
    }
    _streams = mode == DISABLED ? 0 : (mode == COLUMN || mode == COLUMN_BLOCK ? 1 : MAX_STREAMS);
    _block_aligned = mode == COLUMN_BLOCK || mode == COLUMN_ROW_BLOCK;
    // Matrix parameters count RTP datagrams, not individual transport packets.
    args.getIntValue(_columns, u"smpte-2022-l", RTPFECEncoder::DEFAULT_COLUMNS);
    args.getIntValue(_rows, u"smpte-2022-d", RTPFECEncoder::DEFAULT_ROWS);
    // A matrix without an enabled encoder is almost certainly a command-line mistake.
    if (_streams == 0 && (args.present(u"smpte-2022-l") || args.present(u"smpte-2022-d"))) {
        args.error(u"--smpte-2022-l and --smpte-2022-d require enabled --smpte-2022-fec"); // Do not ignore a requested matrix.
        return false;
    }
    // The profile restrictions let missing RTP header fields be reconstructed uniquely.
    if (_streams > 0 && (!rtp || payload_type != RTP_PT_MP2T || burst > RTPFECEncoder::MAX_TS_PACKETS)) {
        args.error(u"--smpte-2022-fec requires --rtp, payload type 33 and --packet-burst at most 7");
        return false; // The parity profile cannot protect arbitrary RTP header/payload formats.
    }
    // Bound fields before multiplying; public callers may supply their own Args schema.
    if (_streams > 0 && (_streams > MAX_STREAMS || _columns == 0 || _columns > RTPFECEncoder::MAX_COLUMNS ||
        _rows < RTPFECEncoder::MIN_ROWS || _rows > RTPFECEncoder::MAX_ROWS ||
        _columns * _rows > RTPFECEncoder::MAX_MATRIX || (_streams == MAX_STREAMS && _columns < RTPFECEncoder::MIN_2D_COLUMNS))) {
        args.error(u"FEC requires columns times rows at most 100, and at least 4 columns in 2D mode");
        return false; // These are the ST 2022-2 mandatory matrix combinations.
    }
    // Destination arithmetic is checked before conversion back to a 16-bit port.
    if (_streams > 0 && (destination.port() == 0 || destination.port() % 2 != 0 || destination.port() > MAX_PORT - PORT_STEP * _streams)) {
        args.error(u"--smpte-2022-fec requires an even media destination port with room for its parity ports");
        return false; // Avoid zero ports, odd RTP ports and UDP port wraparound.
    }
    _column_destination = _row_destination = destination; // Keep the exact IP version and multicast/unicast address.
    if (_streams > 0) {
        _column_destination.setPort(destination.port() + PORT_STEP); // The first stream is always column parity.
        if (_streams == MAX_STREAMS) {
            _row_destination.setPort(destination.port() + MAX_STREAMS * PORT_STEP); // Never compute an unused, overflowing 1D row port.
        }
    }
    return true;
}

bool ts::RTPFECOutput::reset()
{
    clear(); // A restarted or newly disabled plugin must not retain the last run's parity.
    // No storage is allocated when FEC is disabled.
    // Stream numbering starts independently from the media's chosen initial sequence.
    return _streams == 0 || _encoder.reset(_columns, _rows, _streams == MAX_STREAMS, _block_aligned);
}

void ts::RTPFECOutput::clear()
{
    _encoder.clear(); // Drop partial equations and delayed final columns instead of violating emission timing.
    _output.clear(); // Release parity that could not be sent after an output error.
}

//----------------------------------------------------------------------------
// Media is sent first. Explicit parity destinations leave the media default intact.
// A single socket guarantees matching source ports and all socket options.
//----------------------------------------------------------------------------

bool ts::RTPFECOutput::send(UDPSocket& socket, const void* address, size_t size)
{
    if (_streams == 0) {
        return true; // Ordinary UDP/RTP output performs no encoding or additional network operation.
    }
    if (!_encoder.addMedia(address, size, _output)) {
        _report.error(u"invalid or non-consecutive ST 2022-2 media for FEC output");
        return false; // Stop instead of producing parity from a broken media session.
    }
    // A row and a shaped column may coincide, but there are never more than two outputs.
    // The encoder owns no socket; a send failure ends the same session as a media failure.
    for (const auto& parity : _output) {
        if (!socket.send(parity.data().data(), parity.data().size(), parity.isRow() ? _row_destination : _column_destination)) {
            return false; // Propagate the real socket error through the existing output plugin path.
        }
    }
    return true;
}
