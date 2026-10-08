//----------------------------------------------------------------------------
//
// TSDuck - The MPEG Transport Stream Toolkit
// Copyright (c) 2026, Jason Chua
// BSD-2-Clause license, see LICENSE.txt file or https://tsduck.io/license
//
//----------------------------------------------------------------------------
//
// Original implementation from SMPTE ST 2022-1:2007 sections 7 and 8 and the
// MPEG-TS RTP constraints in ST 2022-2:2007. No third-party FEC source is used.
// The standards are available at https://pub.smpte.org/pub/st2022-1/ and
// https://pub.smpte.org/pub/st2022-2/. They are not redistributed here.
//
//----------------------------------------------------------------------------

#include "tsRTPFECDecoder.h"
#include "tsIPProtocols.h"
#include "tsMemory.h"
#include "tsTS.h"


namespace ts {
    namespace {
        // Fixed wire fields from ST 2022-1/2; offsets are bytes, not native structures.
        constexpr uint8_t RTP_FIXED_FLAGS = 0x80; // ST 2022-2 forbids padding, extensions and contributing sources.
        constexpr size_t RTP_SEQUENCE_OFFSET = 2; // Read network byte order without overlaying a native structure.
        constexpr size_t RTP_TIMESTAMP_OFFSET = 4; // Media time is independent of parity RTP time.
        constexpr size_t RTP_SSRC_OFFSET = 8; // Zero for parity; a media change starts a new session.
        constexpr int64_t RTP_SEQUENCE_CYCLE = 65536; // Extend wire sequences without changing their encoding.
        constexpr int64_t RTP_SEQUENCE_HALF = RTP_SEQUENCE_CYCLE / 2; // A shorter window gives one nearest cycle.
        constexpr int64_t RTP_SEQUENCE_MASK = RTP_SEQUENCE_CYCLE - 1; // Also works for negative startup extensions.
        constexpr size_t FEC_LENGTH_OFFSET = 2; // Recovery length excludes the fixed RTP header.
        constexpr size_t FEC_PT_OFFSET = 4; // PT shares this byte with the required E bit.
        constexpr size_t FEC_MASK_OFFSET = 5; // Older mask mode is outside the extended XOR profile.
        constexpr size_t FEC_TIMESTAMP_OFFSET = 8; // Protect media timestamps, not parity timestamps.
        constexpr size_t FEC_FLAGS_OFFSET = 12; // D selects direction; N, type and index are unsupported.
        constexpr size_t FEC_STRIDE_OFFSET = 13; // Explicit stride permits independently staggered columns.
        constexpr size_t FEC_COUNT_OFFSET = 14; // NA counts members, not their final index.
        constexpr size_t FEC_SNBASE_EXT_OFFSET = 15; // This extension byte must be zero in ST 2022-1.
        constexpr uint8_t FEC_EXTENSION_BIT = 0x80; // Extended fields require E before they can be read.
        constexpr uint8_t FEC_PT_MASK = 0x7F; // Strip E before reducing the recovery PT.
        constexpr uint8_t FEC_ROW_BIT = 0x40; // Wire direction must match the receiving parity stream.
        // Normal 2D FEC has at most one row and one column reference per media.
        constexpr size_t MAX_REFERENCES_PER_MEDIA = 2;
    }
}



//----------------------------------------------------------------------------
// Reset configuration, session and statistics.
//----------------------------------------------------------------------------

void ts::RTPFECDecoder::reset(const cn::milliseconds& latency, size_t max_packets)
{
    _latency = std::clamp(latency, MIN_LATENCY, MAX_LATENCY);    // Never use a negative playout interval.
    _max_packets = std::clamp(max_packets, MIN_BUFFER_SIZE, MAX_BUFFER_SIZE); // Keep sequence extension unambiguous.
    _initialized = _emitted = false;                    // A new session has no sequence anchor.
    _ssrc = 0;                                         // Taken from the next accepted media packet.
    _highest = _next = 0;                               // Extended receive and playout positions.
    _recovered = _lost = 0;                             // Statistics belong to this input run.
    _media.clear();                                    // Discard both pending playout and played media history.
    _fec.clear();                                      // Release the owned parity payloads.
    _waiting.clear();                                  // Remove all references to the discarded equations.
    _missing_references = 0;                            // Payload and reverse-index bounds are independent.
    _early_fec.clear();                                // A new media anchor must not reinterpret old parity.
}


//----------------------------------------------------------------------------
// ST 2022-2 uses a fixed RTP header and an integral number of TS packets.
// Accept 1..7 packets, including the mandatory 1, 4 and 7 operating points.
//----------------------------------------------------------------------------

bool ts::RTPFECDecoder::_isMedia(const ByteBlock& data) const
{
    if (data.size() <= RTP_HEADER_SIZE || data[0] != RTP_FIXED_FLAGS || data[1] != RTP_PT_MP2T) {
        return false;
    }
    const size_t length = data.size() - RTP_HEADER_SIZE; // Padding and extensions have already been rejected.
    for (const size_t& packet_size : {PKT_SIZE, PKT_RS_SIZE}) {
        if (length % packet_size == 0 && length <= MAX_TS_PACKETS * packet_size) {
            bool valid = true;
            for (size_t offset = RTP_HEADER_SIZE; valid && offset < data.size(); offset += packet_size) {
                valid = data[offset] == SYNC_BYTE;      // Check every TS boundary, including RS204.
            }
            if (valid) {
                return true;
            }
        }
    }
    return false;
}


//----------------------------------------------------------------------------
// Interpret a 16-bit sequence in the nearest cycle around the highest media.
// The configured history is strictly shorter than half the sequence space.
//----------------------------------------------------------------------------

int64_t ts::RTPFECDecoder::_extendSequence(uint16_t sequence) const
{
    const int64_t difference = static_cast<int64_t>(sequence) - (_highest & RTP_SEQUENCE_MASK); // Difference within the current cycle.
    return _highest + difference + (difference < -RTP_SEQUENCE_HALF ? RTP_SEQUENCE_CYCLE : difference >= RTP_SEQUENCE_HALF ? -RTP_SEQUENCE_CYCLE : 0);
}


//----------------------------------------------------------------------------
// Reduce the XOR equation by a known packet. Short payloads are zero-extended.
// ST 2022-2 fixes all RTP fields not carried by the FEC recovery header.
//----------------------------------------------------------------------------

void ts::RTPFECDecoder::_reduce(FEC& fec, const Datagram& media) const
{
    const size_t length = media.data().size() - RTP_HEADER_SIZE;
    fec._length ^= static_cast<uint16_t>(length);                     // Length recovery excludes the fixed RTP header.
    fec._payload_type ^= media.data()[1];                  // ST 2022-2 fixes the marker to zero.
    fec._timestamp ^= GetUInt32(media.data().data() + RTP_TIMESTAMP_OFFSET);  // Preserve the exact 32-bit RTP timestamp.
    fec._arrival = std::min(fec._arrival, media.arrival());   // Recovery must not add another playout delay.
    for (size_t index = 0; index < std::min(length, fec._payload.size()); ++index) {
        fec._payload[index] ^= media.data()[RTP_HEADER_SIZE + index];
    }
}


//----------------------------------------------------------------------------
// Media ingestion. A changed SSRC starts a fresh session, retaining statistics.
// Early parity is interpreted only after the first media anchors its sequence.
//----------------------------------------------------------------------------

bool ts::RTPFECDecoder::addMedia(Datagram& datagram)
{
    if (!_isMedia(datagram.data()) || datagram.arrival() > TimePoint::max() - _latency) {
        return false;
    }
    const uint32_t ssrc = GetUInt32(datagram.data().data() + RTP_SSRC_OFFSET); // Media SSRC is independent of parity's zero SSRC.
    if (_initialized && ssrc != _ssrc) {
        const uint64_t recovered = _recovered;
        const uint64_t lost = _lost;
        reset(_latency, _max_packets);                  // Old equations cannot protect the new session.
        _recovered = recovered;
        _lost = lost;
    }
    if (!_initialized) {
        _initialized = true;
        _ssrc = ssrc;
        _highest = _next = GetUInt16(datagram.data().data() + RTP_SEQUENCE_OFFSET); // Initial RTP sequence need not be zero.
    }
    const int64_t sequence = _extendSequence(GetUInt16(datagram.data().data() + RTP_SEQUENCE_OFFSET)); // Reorder across wrap.
    if (sequence < _highest - static_cast<int64_t>(_max_packets) + 1) {
        return false;
    }
    if (!_emitted) {
        _next = std::min(_next, sequence);              // Allow startup reordering before any output.
    }
    // Late media can still help pending parity, but is never played twice.
    _insertMedia(sequence, datagram);
    while (!_early_fec.empty()) {
        auto early = std::move(_early_fec.front());     // Move the buffered wire packet, not its sequence anchor.
        _early_fec.pop_front();
        if (!addFEC(early.first, early.second)) {
            continue; // Rejected parity is discarded without ending valid media input.
        }
    }
    return true;
}


//----------------------------------------------------------------------------
// FEC ingestion. Use each packet's SNBase, Offset and NA; both staggered and
// block-aligned columns work. Reject unsupported headers before storing data.
//----------------------------------------------------------------------------

bool ts::RTPFECDecoder::addFEC(const Datagram& datagram, bool row)
{
    const ByteBlock& data = datagram.data();
    if (datagram.arrival() > TimePoint::max() - _latency ||
        data.size() <= RTP_HEADER_SIZE + FEC_HEADER_SIZE || data.size() > RTP_HEADER_SIZE + FEC_HEADER_SIZE + MAX_TS_PACKETS * PKT_RS_SIZE ||
        data[0] != RTP_FIXED_FLAGS || data[1] != FEC_PAYLOAD_TYPE || GetUInt32(data.data() + RTP_SSRC_OFFSET) != 0) {
        return false;
    }
    const uint8_t* const header = data.data() + RTP_HEADER_SIZE;
    const uint8_t offset = header[FEC_STRIDE_OFFSET];                  // Period between protected media sequences.
    const uint8_t count = header[FEC_COUNT_OFFSET];                   // NA: number of associated media packets.
    // E=1, mask=0, N/type/index/SNBase extension=0, and D matches its stream.
    // Rows protect consecutive media. Reject groups larger than our history.
    if ((header[FEC_PT_OFFSET] & FEC_EXTENSION_BIT) == 0 || GetUInt24(header + FEC_MASK_OFFSET) != 0 ||
        header[FEC_FLAGS_OFFSET] != (row ? FEC_ROW_BIT : 0) || header[FEC_SNBASE_EXT_OFFSET] != 0 || offset == 0 || count == 0 ||
        (row && offset != 1) || static_cast<size_t>(count - 1) * offset >= _max_packets) {
        return false;
    }
    if (!_initialized) {
        if (_early_fec.size() >= _max_packets) {
            return false;
        }
        _early_fec.emplace_back(datagram, row);         // Retain direction together with the unanchored parity.
        return true;
    }
    return _addEquation(datagram, row);
}


//----------------------------------------------------------------------------
// Anchor a validated wire equation and index only its still-unknown members.
// Splitting parsing from reduction keeps each operation's invariants local.
//----------------------------------------------------------------------------

bool ts::RTPFECDecoder::_addEquation(const Datagram& datagram, bool row)
{
    const ByteBlock& data = datagram.data();
    const uint8_t* const header = data.data() + RTP_HEADER_SIZE;
    const uint8_t offset = header[FEC_STRIDE_OFFSET];
    const uint8_t count = header[FEC_COUNT_OFFSET];
    const int64_t base = _extendSequence(GetUInt16(header)); // SNBase is a media sequence, not a parity sequence.
    const int64_t end = base + static_cast<int64_t>(count - 1) * offset;  // Last member of this particular equation.
    if (base < _highest - static_cast<int64_t>(_max_packets) + 1 || end > _highest + static_cast<int64_t>(_max_packets) - 1) {
        return false;
    }
    const FECKey key(base, offset, count, row);          // Independent equations may overlap in media space.
    if (_fec.contains(key)) {
        return true;
    }
    FEC fec;
    fec._length = GetUInt16(header + FEC_LENGTH_OFFSET);                 // XOR of protected payload lengths.
    fec._payload_type = header[FEC_PT_OFFSET] & FEC_PT_MASK;                // Strip E; these are recovery bits, not RTP's parity PT.
    fec._timestamp = GetUInt32(header + FEC_TIMESTAMP_OFFSET);             // Ignore the timestamp in the parity RTP header.
    fec._payload.assign(header + FEC_HEADER_SIZE, data.data() + data.size()); // Only parity payload participates in XOR.
    fec._arrival = datagram.arrival();                     // Can be earlier than any associated media arrival.
    if (!_reduceEquation(fec, base, offset, count)) {
        return false;
    }
    if (fec._missing.empty()) {
        return true;
    }
    if (_fec.size() >= _max_packets || _missing_references + fec._missing.size() > MAX_REFERENCES_PER_MEDIA * _max_packets) {
        return false;
    }
    // At startup a parity group can reveal losses preceding the first media.
    if (!_emitted) {
        _next = std::min(_next, base);
    }
    // Index only missing packets so ingestion touches the affected equations.
    for (const int64_t& sequence : fec._missing) {
        _waiting[sequence].insert(key);
        ++_missing_references;                         // Bound overlapping equations under malformed traffic.
    }
    _fec.emplace(key, std::move(fec));                   // One owned copy of each incomplete equation.
    ReadyMedia ready;
    _recover(key, ready);                               // An arriving equation may already have a single loss.
    while (!ready.empty()) {
        auto recovered = std::move(ready.front());
        ready.pop_front();
        _insertMedia(recovered.first, recovered.second); // Newly recovered media can unlock other equations.
    }
    return true;
}


//----------------------------------------------------------------------------
// Reduce known members; played history can still help later column parity.
// A missing member already emitted is final and cannot be reintroduced.
//----------------------------------------------------------------------------

bool ts::RTPFECDecoder::_reduceEquation(FEC& fec, int64_t base, uint8_t offset, uint8_t count)
{
    const int64_t end = base + static_cast<int64_t>(count - 1) * offset;
    for (int64_t sequence = base; sequence <= end; sequence += offset) {
        const auto media = _media.find(sequence);
        if (media != _media.end()) {
            // A parity payload must cover the longest associated media payload.
            if (media->second.data().size() - RTP_HEADER_SIZE > fec._payload.size()) {
                return false;
            }
            _reduce(fec, media->second);                // Played media still count while retained in history.
        }
        else if (_emitted && sequence < _next) {
            // This missing packet has already passed its deadline.
            return false;
        }
        else {
            fec._missing.insert(sequence);              // Track losses and media which have not arrived yet.
        }
    }
    return true;
}


//----------------------------------------------------------------------------
// Insert originals and recovered media through the same path. A work queue
// avoids recursive row/column recovery and bounds the stack usage.
//----------------------------------------------------------------------------

void ts::RTPFECDecoder::_insertMedia(int64_t sequence, Datagram& datagram)
{
    ReadyMedia ready;
    ready.emplace_back(sequence, std::move(datagram));  // The same bounded work path handles all media.
    while (!ready.empty()) {
        auto item = std::move(ready.front());
        ready.pop_front();
        _highest = std::max(_highest, item.first);      // Out-of-order media cannot move the anchor backwards.
        if (item.first < _highest - static_cast<int64_t>(_max_packets) + 1) {
            continue;
        }
        const auto [media, inserted] = _media.emplace(item.first, std::move(item.second)); // Discard duplicates.
        if (!inserted) {
            continue;
        }
        if (media->second.isRecovered()) {
            ++_recovered;                             // Count only the first successful insertion of a recovery.
        }
        // Copy the references because solving an equation removes its index.
        const auto waiting = _waiting.find(item.first);
        if (waiting != _waiting.end()) {
            const auto keys = waiting->second;
            for (const auto& key : keys) {
                auto fec = _fec.find(key);
                if (fec != _fec.end()) {
                    if (media->second.data().size() - RTP_HEADER_SIZE > fec->second._payload.size()) {
                        _dropFEC(key);
                    }
                    else {
                        _reduce(fec->second, media->second);
                        fec->second._missing.erase(item.first); // This equation no longer waits for this media.
                        assert(_missing_references > 0); // Every indexed member owns one reference.
                        --_missing_references;        // Keep the reverse-index bound accurate.
                        _recover(key, ready);
                    }
                }
            }
            _waiting.erase(item.first);               // All references to the newly known media are satisfied.
        }
    }
    _prune();
}


//----------------------------------------------------------------------------
// Solve an equation only when exactly one packet is missing. Validate the
// reconstructed RTP payload before allowing it to participate in more FEC.
//----------------------------------------------------------------------------

void ts::RTPFECDecoder::_recover(const FECKey& key, ReadyMedia& ready)
{
    const auto it = _fec.find(key);
    if (it == _fec.end() || it->second._missing.size() > 1) {
        return;
    }
    const FEC& fec = it->second;
    if (!fec._missing.empty() && fec._length > 0 && fec._length <= fec._payload.size() && fec._payload_type == RTP_PT_MP2T) {
        const int64_t sequence = *fec._missing.begin();
        Datagram media;
        media.data().resize(RTP_HEADER_SIZE + fec._length);
        media.data()[0] = RTP_FIXED_FLAGS;                          // V=2, P=X=CC=0 in the ST 2022-2 profile.
        media.data()[1] = fec._payload_type;               // Profile validation has also guaranteed marker zero.
        PutUInt16(media.data().data() + RTP_SEQUENCE_OFFSET, static_cast<uint16_t>(sequence)); // Put the extended sequence back on the wire.
        PutUInt32(media.data().data() + RTP_TIMESTAMP_OFFSET, fec._timestamp); // Restore timing instead of copying parity RTP time.
        PutUInt32(media.data().data() + RTP_SSRC_OFFSET, _ssrc);          // SSRC is not part of the parity recovery fields.
        MemCopy(media.data().data() + RTP_HEADER_SIZE, fec._payload.data(), fec._length);
        media._arrival = fec._arrival;                   // Use the oldest participating receive time for playout.
        media._recovered = true;
        if (_isMedia(media.data())) {
            ready.emplace_back(sequence, std::move(media)); // No kernel receive timestamp exists for this packet.
        }
    }
    // Completed and invalid equations cannot contribute further recovery.
    _dropFEC(key);
}


//----------------------------------------------------------------------------
// Remove a parity group and its reverse references, including empty entries.
//----------------------------------------------------------------------------

void ts::RTPFECDecoder::_dropFEC(const FECKey& key)
{
    const auto fec = _fec.find(key);
    if (fec != _fec.end()) {
        for (const int64_t& sequence : fec->second._missing) {
            assert(_missing_references > 0);           // Removing an equation must not underflow its index.
            --_missing_references;
            const auto waiting = _waiting.find(sequence);
            if (waiting != _waiting.end()) {
                waiting->second.erase(key);
                if (waiting->second.empty()) {
                    _waiting.erase(waiting);
                }
            }
        }
        _fec.erase(fec);                               // Payload destruction follows removal of its references.
    }
}


//----------------------------------------------------------------------------
// Bound media and parity history to one sequence window. Keep played media
// within that window, since later column parity may still need those packets.
//----------------------------------------------------------------------------

void ts::RTPFECDecoder::_prune()
{
    const int64_t first = _highest - static_cast<int64_t>(_max_packets) + 1;
    _media.erase(_media.begin(), _media.lower_bound(first)); // History has a strict sequence-space bound.
    // The reverse index is ordered by missing sequence. Visit only equations
    // with expired members, rather than scanning all parity for every packet.
    // Already reduced parity can outlive its earliest known media in history.
    const int64_t expired = std::max(first, _next);     // A missing packet outside history or playout is final.
    while (!_waiting.empty() && _waiting.begin()->first < expired) {
        const FECKey key(*_waiting.begin()->second.begin());
        _dropFEC(key);
    }
}


//----------------------------------------------------------------------------
// Playout. All returned data are in RTP sequence order. A hole uses the next
// available packet's deadline, so uncorrectable losses do not stall forever.
// Window pressure advances playout even if the requested latency is too large.
//----------------------------------------------------------------------------

bool ts::RTPFECDecoder::getDatagram(Datagram& datagram, const TimePoint& now)
{
    auto media = _media.lower_bound(_next);
    if (media == _media.end() || (media->second.arrival() + _latency > now && !isPlayoutBufferFull())) {
        return false;
    }
    assert(media->first >= _next);                    // lower_bound must never rewind playout.
    _lost += static_cast<uint64_t>(media->first - _next);             // Count the gap once, regardless of parity loss.
    _next = media->first + 1;                           // Played packets must never be returned again.
    _emitted = true;                                   // Startup reordering is no longer allowed to rewind.
    // Retain media for late parity, including its original RTP receive metadata.
    datagram = media->second;
    _prune();
    return true;
}

cn::milliseconds ts::RTPFECDecoder::timeToNextDatagram(const TimePoint& now) const
{
    const auto media = _media.lower_bound(_next);
    if (media == _media.end()) {
        return cn::milliseconds::max();
    }
    if (isPlayoutBufferFull() || media->second.arrival() + _latency <= now) {
        return cn::milliseconds::zero();
    }
    return cn::ceil<cn::milliseconds>(media->second.arrival() + _latency - now); // Avoid a busy loop below one ms.
}
