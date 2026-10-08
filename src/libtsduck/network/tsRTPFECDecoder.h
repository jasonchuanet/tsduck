//----------------------------------------------------------------------------
//
// TSDuck - The MPEG Transport Stream Toolkit
// Copyright (c) 2026, Jason Chua
// BSD-2-Clause license, see LICENSE.txt file or https://tsduck.io/license
//
//----------------------------------------------------------------------------
//!
//! @file
//! SMPTE ST 2022-1 parity recovery for ST 2022-2 MPEG-TS over RTP.
//!
//----------------------------------------------------------------------------

#pragma once
#include "tsByteBlock.h"
#include "tsUDPSocket.h"

namespace ts {
    //!
    //! Recover and reorder MPEG-TS RTP datagrams using column and row XOR FEC.
    //! @ingroup libtsduck net
    //!
    //! This is an original implementation of SMPTE ST 2022-1:2007, sections 7
    //! and 8, for the RTP profile in ST 2022-2:2007. No external FEC code is used.
    //! All calls must be made from the same thread. A steady clock is used for
    //! playout so that RTP timestamp wrapping and wall clock changes are harmless.
    //!
    class TSDUCKDLL RTPFECDecoder
    {
        TS_NOCOPY(RTPFECDecoder);
    public:
        using Clock = cn::steady_clock;      //!< Clock used for receive deadlines.
        using TimePoint = Clock::time_point; //!< Receive time or playout deadline.

        //!
        //! One complete RTP datagram and its receive metadata.
        //! Recovered datagrams have an RTP timestamp but no kernel timestamp.
        //!
        struct Datagram
        {
            ByteBlock data {};                      //!< RTP header and payload.
            TimePoint arrival {};                   //!< Monotonic receive time.
            cn::microseconds timestamp {-1};        //!< Kernel timestamp, if available.
            UDPSocket::TimeStampType timestamp_type = UDPSocket::TimeStampType::NONE; //!< Kernel timestamp type.
            bool recovered = false;                 //!< True when reconstructed from parity.
        };

        //!
        //! Constructor, with default buffering parameters.
        //!
        RTPFECDecoder() = default;

        //!
        //! Discard all buffered data and reset statistics.
        //! @param [in] latency Time to retain media before playout.
        //! @param [in] max_packets Maximum sequence window, clamped to 256..32767.
        //! Both media history and pending parity are bounded by this value.
        //!
        void reset(cn::milliseconds latency = cn::milliseconds(1000), size_t max_packets = 4096);

        //!
        //! Insert a media datagram. Duplicates are discarded.
        //! Only the ST 2022-2 profile is accepted: fixed 12-byte RTP header,
        //! payload type 33, no padding, extension, CSRC, or marker, and up to
        //! seven 188-byte or 204-byte TS packets. A new SSRC resets the session.
        //! @param [in,out] datagram Datagram whose data may be moved into the decoder.
        //! @return True for supported media, false for malformed or stale media.
        //!
        bool addMedia(Datagram& datagram);

        //!
        //! Insert an extended XOR FEC datagram (RTP payload type 96).
        //! Parity may arrive before the protected media. No block alignment is assumed.
        //! @param [in] datagram FEC datagram, including its RTP and 16-byte FEC headers.
        //! @param [in] row True for the row stream, false for the column stream.
        //! @return True for supported parity, false for malformed, stale, or excess parity.
        //!
        bool addFEC(const Datagram& datagram, bool row);

        //!
        //! Get the next media datagram in sequence order after its playout deadline.
        //! Unrecoverable gaps are skipped at the deadline or when the sequence window fills.
        //! @param [out] datagram Next media datagram, including its original or reconstructed RTP header.
        //! @param [in] now Current monotonic time.
        //! @return True when a datagram is available, false otherwise.
        //!
        bool getDatagram(Datagram& datagram, TimePoint now);

        //!
        //! Time until the next playout deadline, rounded up to milliseconds.
        //! @param [in] now Current monotonic time.
        //! @return Delay, or milliseconds::max() if no media is buffered.
        //!
        cn::milliseconds timeToNextDatagram(TimePoint now) const;

        //!
        //! Check whether more media would overwrite pending playout history.
        //! The caller must drain media before inserting another datagram when full.
        //! @return True when the oldest pending media must be released early.
        //!
        bool isPlayoutBufferFull() const { return _initialized && _highest - _next >= int64_t(_max_packets) - 1; }

        uint64_t recoveredPackets() const { return _recovered; } //!< Number of recovered RTP datagrams.
        uint64_t lostPackets() const { return _lost; }           //!< Number of expired missing RTP datagrams.
        size_t bufferedMedia() const { return _media.size(); }   //!< Number of retained media datagrams, including history.
        size_t bufferedFEC() const { return _fec.size() + _early_fec.size(); } //!< Number of retained parity datagrams.

    private:
        // Extended media sequence numbers remove ambiguity around 16-bit wrap.
        // The key also distinguishes independent row and column parity groups.
        using FECKey = std::tuple<int64_t, uint8_t, uint8_t, bool>;

        // Partially reduced parity. Only missing media need further XOR operations.
        struct FEC
        {
            uint16_t length = 0;
            uint8_t payload_type = 0;
            uint32_t timestamp = 0;
            ByteBlock payload {};
            TimePoint arrival {};
            std::set<int64_t> missing {};
        };

        cn::milliseconds _latency {1000};
        size_t _max_packets = 4096;
        bool _initialized = false;
        bool _emitted = false;
        uint32_t _ssrc = 0;
        int64_t _highest = 0;
        int64_t _next = 0;
        uint64_t _recovered = 0;
        uint64_t _lost = 0;
        std::map<int64_t, Datagram> _media {};                // Bounded media history and playout buffer.
        std::map<FECKey, FEC> _fec {};                      // Incomplete parity groups.
        std::map<int64_t, std::set<FECKey>> _waiting {};      // Missing media -> affected parity groups.
        size_t _missing_references = 0;                      // Bound the reverse index as well as the payloads.
        std::deque<std::pair<Datagram, bool>> _early_fec {};   // Parity received before the first media.

        // Validate a complete media datagram, including the TS packet boundaries.
        static bool IsMedia(const ByteBlock& data);

        // Extend a wire sequence number using the most recent media as reference.
        int64_t extendSequence(uint16_t sequence) const;

        // Reduce parity with one known media packet, including length, PT and timestamp.
        static void Reduce(FEC& fec, const Datagram& media);

        // Insert media and cascade newly solvable parity groups without recursion.
        void insertMedia(int64_t sequence, Datagram& datagram);
        void recover(const FECKey& key, std::deque<std::pair<int64_t, Datagram>>& ready);

        // Remove parity and all of its reverse references, or prune expired history.
        void dropFEC(const FECKey& key);
        void prune();
    };
}
