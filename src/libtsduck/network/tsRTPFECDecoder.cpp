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


//----------------------------------------------------------------------------
// Reset configuration, session and statistics.
//----------------------------------------------------------------------------

void ts::RTPFECDecoder::reset(cn::milliseconds latency, size_t max_packets)
{
    _latency = std::max(cn::milliseconds(1), latency);    // Never use a negative playout interval.
    _max_packets = std::clamp<size_t>(max_packets, 256, 32767); // Keep sequence extension unambiguous.
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

bool ts::RTPFECDecoder::IsMedia(const ByteBlock& data)
{
    if (data.size() <= RTP_HEADER_SIZE || data[0] != 0x80 || data[1] != RTP_PT_MP2T) {
        return false;
    }
    const size_t length = data.size() - RTP_HEADER_SIZE; // Padding and extensions have already been rejected.
    for (const size_t packet_size : {PKT_SIZE, PKT_RS_SIZE}) {
        if (length % packet_size == 0 && length <= 7 * packet_size) {
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

int64_t ts::RTPFECDecoder::extendSequence(uint16_t sequence) const
{
    const int64_t difference = int64_t(sequence) - (_highest & 0xFFFF); // Difference within the current cycle.
    return _highest + difference + (difference < -32768 ? 65536 : difference > 32767 ? -65536 : 0);
}


//----------------------------------------------------------------------------
// Reduce the XOR equation by a known packet. Short payloads are zero-extended.
// ST 2022-2 fixes all RTP fields not carried by the FEC recovery header.
//----------------------------------------------------------------------------

void ts::RTPFECDecoder::Reduce(FEC& fec, const Datagram& media)
{
    const size_t length = media.data.size() - RTP_HEADER_SIZE;
    fec.length ^= uint16_t(length);                     // Length recovery excludes the fixed RTP header.
    fec.payload_type ^= media.data[1];                  // ST 2022-2 fixes the marker to zero.
    fec.timestamp ^= GetUInt32(media.data.data() + 4);  // Preserve the exact 32-bit RTP timestamp.
    fec.arrival = std::min(fec.arrival, media.arrival);   // Recovery must not add another playout delay.
    for (size_t index = 0; index < std::min(length, fec.payload.size()); ++index) {
        fec.payload[index] ^= media.data[RTP_HEADER_SIZE + index];
    }
}


//----------------------------------------------------------------------------
// Media ingestion. A changed SSRC starts a fresh session, retaining statistics.
// Early parity is interpreted only after the first media anchors its sequence.
//----------------------------------------------------------------------------

bool ts::RTPFECDecoder::addMedia(Datagram& datagram)
{
    if (!IsMedia(datagram.data)) {
        return false;
    }
    const uint32_t ssrc = GetUInt32(datagram.data.data() + 8); // Media SSRC is independent of parity's zero SSRC.
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
        _highest = _next = GetUInt16(datagram.data.data() + 2); // Initial RTP sequence need not be zero.
    }
    const int64_t sequence = extendSequence(GetUInt16(datagram.data.data() + 2)); // Reorder across wrap.
    if (sequence < _highest - int64_t(_max_packets) + 1) {
        return false;
    }
    if (!_emitted) {
        _next = std::min(_next, sequence);              // Allow startup reordering before any output.
    }
    // Late media can still help pending parity, but is never played twice.
    insertMedia(sequence, datagram);
    while (!_early_fec.empty()) {
        auto early = std::move(_early_fec.front());     // Move the buffered wire packet, not its sequence anchor.
        _early_fec.pop_front();
        addFEC(early.first, early.second);
    }
    return true;
}


//----------------------------------------------------------------------------
// FEC ingestion. Use each packet's SNBase, Offset and NA; both staggered and
// block-aligned columns work. Reject unsupported headers before storing data.
//----------------------------------------------------------------------------

bool ts::RTPFECDecoder::addFEC(const Datagram& datagram, bool row)
{
    const ByteBlock& data = datagram.data;
    if (data.size() <= RTP_HEADER_SIZE + 16 || data.size() > RTP_HEADER_SIZE + 16 + 7 * PKT_RS_SIZE ||
        data[0] != 0x80 || data[1] != 96 || GetUInt32(data.data() + 8) != 0) {
        return false;
    }
    const uint8_t* const header = data.data() + RTP_HEADER_SIZE;
    const uint8_t offset = header[13];                  // Period between protected media sequences.
    const uint8_t count = header[14];                   // NA: number of associated media packets.
    // E=1, mask=0, N/type/index/SNBase extension=0, and D matches its stream.
    // Rows protect consecutive media. Reject groups larger than our history.
    if ((header[4] & 0x80) == 0 || GetUInt24(header + 5) != 0 ||
        header[12] != (row ? 0x40 : 0) || header[15] != 0 || offset == 0 || count == 0 ||
        (row && offset != 1) || size_t(count - 1) * offset >= _max_packets) {
        return false;
    }
    if (!_initialized) {
        if (_early_fec.size() >= _max_packets) {
            return false;
        }
        _early_fec.emplace_back(datagram, row);         // Retain direction together with the unanchored parity.
        return true;
    }
    const int64_t base = extendSequence(GetUInt16(header)); // SNBase is a media sequence, not a parity sequence.
    const int64_t end = base + int64_t(count - 1) * offset;  // Last member of this particular equation.
    if (base < _highest - int64_t(_max_packets) + 1 || end > _highest + int64_t(_max_packets) - 1) {
        return false;
    }
    const FECKey key(base, offset, count, row);          // Independent equations may overlap in media space.
    if (_fec.contains(key)) {
        return true;
    }
    // At startup a parity group can reveal losses preceding the first media.
    if (!_emitted) {
        _next = std::min(_next, base);
    }
    FEC fec;
    fec.length = GetUInt16(header + 2);                 // XOR of protected payload lengths.
    fec.payload_type = header[4] & 0x7F;                // Strip E; these are recovery bits, not RTP's parity PT.
    fec.timestamp = GetUInt32(header + 8);             // Ignore the timestamp in the parity RTP header.
    fec.payload.assign(header + 16, data.data() + data.size()); // Only parity payload participates in XOR.
    fec.arrival = datagram.arrival;                     // Can be earlier than any associated media arrival.
    for (int64_t sequence = base; sequence <= end; sequence += offset) {
        const auto media = _media.find(sequence);
        if (media != _media.end()) {
            // A parity payload must cover the longest associated media payload.
            if (media->second.data.size() - RTP_HEADER_SIZE > fec.payload.size()) {
                return false;
            }
            Reduce(fec, media->second);                // Played media still count while retained in history.
        }
        else if (sequence < _next) {
            // This missing packet has already passed its deadline.
            return false;
        }
        else {
            fec.missing.insert(sequence);              // Track losses and media which have not arrived yet.
        }
    }
    if (fec.missing.empty()) {
        return true;
    }
    if (_fec.size() >= _max_packets || _missing_references + fec.missing.size() > 2 * _max_packets) {
        return false;
    }
    // Index only missing packets so ingestion touches the affected equations.
    for (const int64_t sequence : fec.missing) {
        _waiting[sequence].insert(key);
        ++_missing_references;                         // Bound overlapping equations under malformed traffic.
    }
    _fec.emplace(key, std::move(fec));                   // One owned copy of each incomplete equation.
    std::deque<std::pair<int64_t, Datagram>> ready;
    recover(key, ready);                               // An arriving equation may already have a single loss.
    while (!ready.empty()) {
        auto recovered = std::move(ready.front());
        ready.pop_front();
        insertMedia(recovered.first, recovered.second); // Newly recovered media can unlock other equations.
    }
    return true;
}


//----------------------------------------------------------------------------
// Insert originals and recovered media through the same path. A work queue
// avoids recursive row/column recovery and bounds the stack usage.
//----------------------------------------------------------------------------

void ts::RTPFECDecoder::insertMedia(int64_t sequence, Datagram& datagram)
{
    std::deque<std::pair<int64_t, Datagram>> ready;
    ready.emplace_back(sequence, std::move(datagram));  // The same bounded work path handles all media.
    while (!ready.empty()) {
        auto item = std::move(ready.front());
        ready.pop_front();
        _highest = std::max(_highest, item.first);      // Out-of-order media cannot move the anchor backwards.
        if (item.first < _highest - int64_t(_max_packets) + 1) {
            continue;
        }
        const auto [media, inserted] = _media.emplace(item.first, std::move(item.second)); // Discard duplicates.
        if (!inserted) {
            continue;
        }
        if (media->second.recovered) {
            ++_recovered;                             // Count only the first successful insertion of a recovery.
        }
        // Copy the references because solving an equation removes its index.
        const auto waiting = _waiting.find(item.first);
        if (waiting != _waiting.end()) {
            const auto keys = waiting->second;
            for (const auto& key : keys) {
                auto fec = _fec.find(key);
                if (fec != _fec.end()) {
                    if (media->second.data.size() - RTP_HEADER_SIZE > fec->second.payload.size()) {
                        dropFEC(key);
                    }
                    else {
                        Reduce(fec->second, media->second);
                        fec->second.missing.erase(item.first); // This equation no longer waits for this media.
                        --_missing_references;                 // Keep the reverse-index bound accurate.
                        recover(key, ready);
                    }
                }
            }
            _waiting.erase(item.first);               // All references to the newly known media are satisfied.
        }
    }
    prune();
}


//----------------------------------------------------------------------------
// Solve an equation only when exactly one packet is missing. Validate the
// reconstructed RTP payload before allowing it to participate in more FEC.
//----------------------------------------------------------------------------

void ts::RTPFECDecoder::recover(const FECKey& key, std::deque<std::pair<int64_t, Datagram>>& ready)
{
    const auto it = _fec.find(key);
    if (it == _fec.end() || it->second.missing.size() > 1) {
        return;
    }
    const FEC& fec = it->second;
    if (!fec.missing.empty() && fec.length > 0 && fec.length <= fec.payload.size() && fec.payload_type == RTP_PT_MP2T) {
        const int64_t sequence = *fec.missing.begin();
        Datagram media;
        media.data.resize(RTP_HEADER_SIZE + fec.length);
        media.data[0] = 0x80;                          // V=2, P=X=CC=0 in the ST 2022-2 profile.
        media.data[1] = fec.payload_type;               // Profile validation has also guaranteed marker zero.
        PutUInt16(media.data.data() + 2, uint16_t(sequence)); // Put the extended sequence back on the wire.
        PutUInt32(media.data.data() + 4, fec.timestamp); // Restore timing instead of copying parity RTP time.
        PutUInt32(media.data.data() + 8, _ssrc);          // SSRC is not part of the parity recovery fields.
        MemCopy(media.data.data() + RTP_HEADER_SIZE, fec.payload.data(), fec.length);
        media.arrival = fec.arrival;                   // Use the oldest participating receive time for playout.
        media.recovered = true;
        if (IsMedia(media.data)) {
            ready.emplace_back(sequence, std::move(media)); // No kernel receive timestamp exists for this packet.
        }
    }
    // Completed and invalid equations cannot contribute further recovery.
    dropFEC(key);
}


//----------------------------------------------------------------------------
// Remove a parity group and its reverse references, including empty entries.
//----------------------------------------------------------------------------

void ts::RTPFECDecoder::dropFEC(const FECKey& key)
{
    const auto fec = _fec.find(key);
    if (fec != _fec.end()) {
        for (const int64_t sequence : fec->second.missing) {
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

void ts::RTPFECDecoder::prune()
{
    const int64_t first = _highest - int64_t(_max_packets) + 1;
    _media.erase(_media.begin(), _media.lower_bound(first)); // History has a strict sequence-space bound.
    // The reverse index is ordered by missing sequence. Visit only equations
    // with expired members, rather than scanning all parity for every packet.
    // Already reduced parity can outlive its earliest known media in history.
    const int64_t expired = std::max(first, _next);     // A missing packet outside history or playout is final.
    while (!_waiting.empty() && _waiting.begin()->first < expired) {
        const FECKey key(*_waiting.begin()->second.begin());
        dropFEC(key);
    }
}


//----------------------------------------------------------------------------
// Playout. All returned data are in RTP sequence order. A hole uses the next
// available packet's deadline, so uncorrectable losses do not stall forever.
// Window pressure advances playout even if the requested latency is too large.
//----------------------------------------------------------------------------

bool ts::RTPFECDecoder::getDatagram(Datagram& datagram, TimePoint now)
{
    auto media = _media.lower_bound(_next);
    if (media == _media.end() || (media->second.arrival + _latency > now && !isPlayoutBufferFull())) {
        return false;
    }
    _lost += uint64_t(media->first - _next);             // Count the gap once, regardless of parity loss.
    _next = media->first + 1;                           // Played packets must never be returned again.
    _emitted = true;                                   // Startup reordering is no longer allowed to rewind.
    // Retain media for late parity, including its original RTP receive metadata.
    datagram = media->second;
    prune();
    return true;
}

cn::milliseconds ts::RTPFECDecoder::timeToNextDatagram(TimePoint now) const
{
    const auto media = _media.lower_bound(_next);
    if (media == _media.end()) {
        return cn::milliseconds::max();
    }
    if (isPlayoutBufferFull() || media->second.arrival + _latency <= now) {
        return cn::milliseconds::zero();
    }
    return cn::ceil<cn::milliseconds>(media->second.arrival + _latency - now); // Avoid a busy loop below one ms.
}
