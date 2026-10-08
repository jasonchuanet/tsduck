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

        static constexpr size_t DEFAULT_BUFFER_SIZE = 4096; //!< Default media sequence window.
        static constexpr size_t MIN_BUFFER_SIZE = 256;      //!< Smallest supported sequence window.
        static constexpr size_t MAX_BUFFER_SIZE = 32767;    //!< Largest unambiguous sequence window.
        static constexpr cn::milliseconds DEFAULT_LATENCY {1000}; //!< Default playout delay.
        static constexpr cn::milliseconds MIN_LATENCY {1};        //!< Smallest playout delay.
        static constexpr cn::milliseconds MAX_LATENCY {60000};    //!< Largest configurable playout delay.
        static constexpr size_t MAX_TS_PACKETS = 7;         //!< Largest ST 2022-2 media payload in TS packets.
        static constexpr size_t FEC_HEADER_SIZE = 16;       //!< Extended XOR FEC header size in bytes.
        static constexpr uint8_t FEC_PAYLOAD_TYPE = 96;     //!< ST 2022-1 RTP payload type.

        //!
        //! One complete RTP datagram and its receive metadata.
        //! Recovered datagrams have an RTP timestamp but no kernel timestamp.
        //!
        class TSDUCKDLL Datagram
        {
        public:
            //! Default constructor.
            Datagram() = default;
            //! Copy constructor.
            //! @param [in] other Datagram to copy, including its receive metadata.
            Datagram(const Datagram& other) = default;
            //! Move constructor.
            //! @param [in,out] other Datagram whose storage is transferred.
            Datagram(Datagram&& other) noexcept = default;
            //! Destructor.
            virtual ~Datagram() = default;
            //! Copy assignment.
            //! @param [in] other Datagram to copy.
            //! @return A reference to this object.
            Datagram& operator=(const Datagram& other) = default;
            //! Move assignment.
            //! @param [in,out] other Datagram whose storage is transferred.
            //! @return A reference to this object.
            Datagram& operator=(Datagram&& other) noexcept = default;

            //! Access the complete RTP datagram for ingestion or inspection.
            //! @return Mutable data, valid for the lifetime of this datagram.
            ByteBlock& data() { return _data; }
            //! Inspect the complete RTP datagram.
            //! @return Read-only data, valid for the lifetime of this datagram.
            const ByteBlock& data() const { return _data; }
            //! Get the monotonic receive time used for playout.
            //! @return Receive time, valid for the lifetime of this datagram.
            const TimePoint& arrival() const { return _arrival; }
            //! Set the monotonic receive time.
            //! @param [in] value Receive time from Clock, independent of RTP and kernel clocks.
            void setArrival(const TimePoint& value) { _arrival = value; }
            //! Get the original kernel receive timestamp, if available.
            //! @return Kernel timestamp, or -1 microsecond when unavailable.
            cn::microseconds receiveTimestamp() const { return _timestamp; }
            //! Get the original receive timestamp's clock type.
            //! @return Kernel timestamp type, or NONE for reconstructed media.
            UDPSocket::TimeStampType receiveTimestampType() const { return _timestamp_type; }
            //! Store the original datagram's kernel receive metadata.
            //! @param [in] value Kernel receive timestamp, or a negative value if unavailable.
            //! @param [in] type Clock type reported by the UDP socket.
            void setReceiveTimestamp(const cn::microseconds& value, UDPSocket::TimeStampType type)
            {
                _timestamp_type = value < cn::microseconds::zero() ? UDPSocket::TimeStampType::NONE : type;
                _timestamp = _timestamp_type == UDPSocket::TimeStampType::NONE ? cn::microseconds(-1) : value;
            }
            //! Check whether parity reconstructed this datagram.
            //! Only the decoder can mark a datagram as recovered.
            //! @return True for reconstructed media, false for an original datagram.
            bool isRecovered() const { return _recovered; }

        private:
            friend class RTPFECDecoder;
            ByteBlock _data {};                      //!< Complete wire data; always validated at the decoder boundary.
            TimePoint _arrival {};                   //!< Steady time, unrelated to the kernel timestamp's origin.
            cn::microseconds _timestamp {-1};        //!< Recovery never invents a kernel receive event.
            UDPSocket::TimeStampType _timestamp_type = UDPSocket::TimeStampType::NONE; //!< Original kernel timestamp clock, or NONE.
            bool _recovered = false;                 //!< Written only by successful parity reconstruction.
        };

        //!
        //! Constructor, with default buffering parameters.
        //!
        RTPFECDecoder() = default;

        //! Destructor; owned containers release media, parity and reverse references.
        virtual ~RTPFECDecoder() = default;

        //!
        //! Discard all buffered data and reset statistics.
        //! @param [in] latency Time to retain media before playout, clamped to 1..60000 milliseconds.
        //! @param [in] max_packets Maximum sequence window, clamped to 256..32767.
        //! Both media history and pending parity are bounded by this value.
        //!
        void reset(const cn::milliseconds& latency = DEFAULT_LATENCY, size_t max_packets = DEFAULT_BUFFER_SIZE);

        //!
        //! Insert a media datagram. Duplicates are discarded.
        //! Only the ST 2022-2 profile is accepted: fixed 12-byte RTP header,
        //! payload type 33, no padding, extension, CSRC, or marker, and up to
        //! seven 188-byte or 204-byte TS packets. A new SSRC resets the session.
        //! @param [in,out] datagram Datagram whose data may be moved into the decoder.
        //! @return True for supported media, false for malformed, stale or unrepresentable receive times.
        //!
        bool addMedia(Datagram& datagram);

        //!
        //! Insert an extended XOR FEC datagram (RTP payload type 96).
        //! Parity may arrive before the protected media. No block alignment is assumed.
        //! @param [in] datagram FEC datagram, including its RTP and 16-byte FEC headers.
        //! @param [in] row True for the row stream, false for the column stream.
        //! @return True for supported parity, false for malformed, stale, excess or unrepresentable receive times.
        //!
        bool addFEC(const Datagram& datagram, bool row);

        //!
        //! Get the next media datagram in sequence order after its playout deadline.
        //! Unrecoverable gaps are skipped at the deadline or when the sequence window fills.
        //! @param [out] datagram Next media datagram, including its original or reconstructed RTP header.
        //! @param [in] now Current monotonic time.
        //! @return True when a datagram is available, false otherwise.
        //!
        bool getDatagram(Datagram& datagram, const TimePoint& now);

        //!
        //! Time until the next playout deadline, rounded up to milliseconds.
        //! @param [in] now Current monotonic time.
        //! @return Delay, or milliseconds::max() if no media is buffered.
        //!
        cn::milliseconds timeToNextDatagram(const TimePoint& now) const;

        //!
        //! Check whether more media would overwrite pending playout history.
        //! The caller must drain media before inserting another datagram when full.
        //! @return True when the oldest pending media must be released early.
        //!
        bool isPlayoutBufferFull() const { return _initialized && _highest - _next >= int64_t(_max_packets) - 1; }

        //! Count successfully reconstructed media.
        //! @return Number of recovered RTP datagrams since reset().
        uint64_t recoveredPackets() const { return _recovered; }
        //! Count media gaps finalized by playout.
        //! @return Number of expired missing RTP datagrams since reset().
        uint64_t lostPackets() const { return _lost; }
        //! Inspect retained media storage.
        //! @return Number of buffered datagrams, including already played history.
        size_t bufferedMedia() const { return _media.size(); }
        //! Inspect retained parity storage.
        //! @return Number of incomplete equations and unanchored FEC datagrams.
        size_t bufferedFEC() const { return _fec.size() + _early_fec.size(); }

    private:
        //! Identify parity by extended base sequence, stride, count and direction.
        using FECKey = std::tuple<int64_t, uint8_t, uint8_t, bool>;

        //! Partially reduced parity; only missing media need further XOR operations.
        class FEC
        {
            friend class RTPFECDecoder;
        public:
            //! Default constructor.
            FEC() = default;
            //! Copy constructor.
            //! @param [in] other Equation to copy.
            FEC(const FEC& other) = default;
            //! Move constructor.
            //! @param [in,out] other Equation whose storage is transferred.
            FEC(FEC&& other) noexcept = default;
            //! Destructor.
            virtual ~FEC() = default;
            //! Copy assignment.
            //! @param [in] other Equation to copy.
            //! @return A reference to this object.
            FEC& operator=(const FEC& other) = default;
            //! Move assignment.
            //! @param [in,out] other Equation whose storage is transferred.
            //! @return A reference to this object.
            FEC& operator=(FEC&& other) noexcept = default;
        private:
            uint16_t _length = 0;                    //!< XOR length excludes the fixed media RTP header.
            uint8_t _payload_type = 0;               //!< E is removed before reducing the recovery PT.
            uint32_t _timestamp = 0;                 //!< Restore media time, never use parity's RTP time.
            ByteBlock _payload {};                  //!< Partially reduced, zero-extended XOR payload.
            TimePoint _arrival {};                  //!< Oldest participant prevents extra recovery latency.
            std::set<int64_t> _missing {};           //!< Only unknown members need reverse references.
        };

        cn::milliseconds _latency {DEFAULT_LATENCY}; //!< Playout delay, independent of RTP and wall clocks.
        size_t _max_packets = DEFAULT_BUFFER_SIZE; //!< Maximum retained media sequence window.
        bool _initialized = false; //!< A valid media source and sequence anchor are known.
        bool _emitted = false; //!< Playout has committed its first sequence position.
        uint32_t _ssrc = 0; //!< Accepted media source identifier.
        int64_t _highest = 0; //!< Highest accepted extended media sequence.
        int64_t _next = 0; //!< Next extended sequence considered for playout.
        uint64_t _recovered = 0; //!< Successfully reconstructed media datagrams.
        uint64_t _lost = 0; //!< Missing media finalized by playout.
        std::map<int64_t, Datagram> _media {};                //!< Bounded media history and playout buffer.
        std::map<FECKey, FEC> _fec {};                      //!< Incomplete parity groups.
        std::map<int64_t, std::set<FECKey>> _waiting {};      //!< Missing media -> affected parity groups.
        size_t _missing_references = 0;                      //!< Bound the reverse index as well as the payloads.
        std::deque<std::pair<Datagram, bool>> _early_fec {};   //!< Parity received before the first media.

        //! Validate a complete media datagram, including TS boundaries.
        //! @param [in] data Complete RTP wire data.
        //! @return True for a supported ST 2022-2 media packet.
        bool _isMedia(const ByteBlock& data) const;

        //! Extend a wire sequence number using the most recent media as reference.
        //! @param [in] sequence Wire sequence, including 16-bit wrap.
        //! @return Closest extended sequence within half a cycle.
        int64_t _extendSequence(uint16_t sequence) const;

        //! Reduce parity with one known media packet.
        //! @param [in,out] fec Equation reduced by the known media.
        //! @param [in] media Validated media, including its fixed RTP header.
        void _reduce(FEC& fec, const Datagram& media) const;

        //! Pending reconstructed media, processed iteratively without recursion.
        using ReadyMedia = std::deque<std::pair<int64_t, Datagram>>;
        //! Insert media and cascade newly solvable parity groups.
        //! @param [in] sequence Extended media sequence.
        //! @param [in,out] datagram Validated media whose storage is transferred.
        void _insertMedia(int64_t sequence, Datagram& datagram);
        //! Reconstruct a singleton equation without recursively inserting media.
        //! @param [in] key Equation to inspect.
        //! @param [out] ready Queue receiving successfully reconstructed media.
        void _recover(const FECKey& key, ReadyMedia& ready);
        //! Anchor and retain a validated parity datagram.
        //! @param [in] datagram Parity with validated RTP and FEC headers.
        //! @param [in] row True for row parity, false for column parity.
        //! @return True for a retained, completed or duplicate equation; false on rejection.
        bool _addEquation(const Datagram& datagram, bool row);
        //! Reduce an equation against known media and identify its missing members.
        //! @param [in,out] fec Equation receiving reduced fields and missing sequences.
        //! @param [in] base Extended base media sequence.
        //! @param [in] offset Media sequence stride.
        //! @param [in] count Number of protected media datagrams.
        //! @return True if every known media length fits and no missing member is finalized.
        bool _reduceEquation(FEC& fec, int64_t base, uint8_t offset, uint8_t count);

        //! Remove parity and all its reverse references.
        //! @param [in] key Equation to remove; an absent key is harmless.
        void _dropFEC(const FECKey& key);
        //! Remove media and parity history outside the configured sequence window.
        void _prune();
    };
}
