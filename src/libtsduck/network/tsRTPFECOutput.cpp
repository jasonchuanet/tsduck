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
    }
}

//----------------------------------------------------------------------------
// The owning datagram output provides the error sink and the already-open socket.
//----------------------------------------------------------------------------

ts::RTPFECOutput::RTPFECOutput(Report& report) :
    _report(report)
{
}

void ts::RTPFECOutput::defineArgs(Args& args) const
{
    args.option(u"fec", 0, Args::INTEGER, 0, 1, 1, 2, true); // An omitted value enables the two-stream profile.
    args.help(u"fec", u"1|2",
              u"With --rtp, generate SMPTE ST 2022-1 column FEC (1) or column and row FEC (2, the default). "
              u"FEC uses the media destination port +2 and +4, with the same source port. "
              u"Requires an even media port, payload type 33 and at most seven TS packets per datagram.");
    args.option(u"fec-columns", 0, Args::INTEGER, 0, 1, 1, RTPFECEncoder::MAX_COLUMNS); // L controls burst-loss protection.
    args.help(u"fec-columns", u"count",
              u"Number of FEC columns L, from 1 to 20 (at least 4 with 2D FEC). The default is 4. "
              u"Requires --fec. Columns times rows must not exceed 100.");
    args.option(u"fec-rows", 0, Args::INTEGER, 0, 1, RTPFECEncoder::MIN_ROWS, RTPFECEncoder::MAX_ROWS); // D controls column overhead.
    args.help(u"fec-rows", u"count",
              u"Number of FEC rows D, from 4 to 20. The default is 4. Requires --fec. "
              u"Columns times rows must not exceed 100. Parity is interleaved with subsequent media.");
}

//----------------------------------------------------------------------------
// Reject configuration incompatibilities before opening or transmitting media.
//----------------------------------------------------------------------------

bool ts::RTPFECOutput::loadArgs(Args& args, const IPSocketAddress& destination, bool rtp, uint8_t payload_type, size_t burst)
{
    // Optional integer values distinguish an absent flag from bare --fec.
    // Configuration stays disabled for every pre-existing output path.
    _streams = args.present(u"fec") ? args.intValue<size_t>(u"fec", 2) : 0;
    // Matrix parameters count RTP datagrams, not individual transport packets.
    args.getIntValue(_columns, u"fec-columns", RTPFECEncoder::DEFAULT_COLUMNS);
    args.getIntValue(_rows, u"fec-rows", RTPFECEncoder::DEFAULT_ROWS);
    // A matrix without an enabled encoder is almost certainly a command-line mistake.
    if (_streams == 0 && (args.present(u"fec-columns") || args.present(u"fec-rows"))) {
        args.error(u"--fec-columns and --fec-rows require --fec"); // Do not silently ignore a requested matrix.
        return false;
    }
    // The profile restrictions let missing RTP header fields be reconstructed uniquely.
    if (_streams > 0 && (!rtp || payload_type != RTP_PT_MP2T || burst > RTPFECEncoder::MAX_TS_PACKETS)) {
        args.error(u"--fec requires --rtp, payload type 33 and --packet-burst at most 7");
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
        args.error(u"--fec requires an even media destination port with room for its parity ports");
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
    return _streams == 0 || _encoder.reset(_columns, _rows, _streams == MAX_STREAMS);
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
