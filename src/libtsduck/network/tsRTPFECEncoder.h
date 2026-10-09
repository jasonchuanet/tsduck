//----------------------------------------------------------------------------
//
// TSDuck - The MPEG Transport Stream Toolkit
// Copyright (c) 2026, Jason Chua
// BSD-2-Clause license, see LICENSE.txt file or https://tsduck.io/license
//
//----------------------------------------------------------------------------
//!
//! @file
//! Original SMPTE ST 2022-1 parity generation for ST 2022-2 RTP media.
//!
//----------------------------------------------------------------------------

#pragma once
#include "tsByteBlock.h"

namespace ts {
    //!
    //! Generate column and optional row FEC, using ST 2022-1 Annex B or C shaping.
    //! @ingroup libtsduck net
    //! All calls belong to one thread. Media are never buffered or modified.
    //! Rows are emitted immediately; columns honor the required transmission delay.
    //! Only complete groups are protected. clear() discards pending parity at shutdown,
    //! without manufacturing media or sending columns before their required delay.
    //!
    class TSDUCKDLL RTPFECEncoder
    {
        TS_NOCOPY(RTPFECEncoder);
    public:
        static constexpr size_t DEFAULT_COLUMNS = 4; //!< Default L, the number of columns.
        static constexpr size_t DEFAULT_ROWS = 4;    //!< Default D, the number of rows.
        static constexpr size_t MAX_COLUMNS = 20;   //!< Largest mandatory ST 2022-2 L.
        static constexpr size_t MIN_ROWS = 4;       //!< Smallest mandatory ST 2022-2 D.
        static constexpr size_t MAX_ROWS = 20;      //!< Largest mandatory ST 2022-2 D.
        static constexpr size_t MAX_MATRIX = 100;   //!< Largest mandatory L * D.
        static constexpr size_t MIN_2D_COLUMNS = 4; //!< Two streams require L >= 4.
        static constexpr size_t MAX_TS_PACKETS = 7; //!< Largest supported RTP media payload.
        static constexpr size_t FEC_HEADER_SIZE = 16; //!< Extended XOR header size.
        static constexpr uint8_t FEC_PAYLOAD_TYPE = 96; //!< First dynamic RTP payload type.

        //! One emitted parity datagram, including its RTP and extended FEC headers.
        class TSDUCKDLL Datagram
        {
        public:
            //! Default constructor.
            Datagram() = default;
            //! Copy constructor.
            //! @param [in] other Parity to copy.
            Datagram(const Datagram& other) = default;
            //! Move constructor.
            //! @param [in,out] other Parity whose storage is transferred.
            Datagram(Datagram&& other) noexcept = default;
            //! Destructor.
            virtual ~Datagram();
            //! Copy assignment.
            //! @param [in] other Parity to copy.
            //! @return A reference to this object.
            Datagram& operator=(const Datagram& other) = default;
            //! Move assignment.
            //! @param [in,out] other Parity whose storage is transferred.
            //! @return A reference to this object.
            Datagram& operator=(Datagram&& other) noexcept = default;
            //! Inspect the complete wire datagram.
            //! @return Data owned by this object, valid for its lifetime.
            const ByteBlock& data() const
            {
                return _data;
            }
            //! Identify the destination parity stream.
            //! @return True for row parity, false for column parity.
            bool isRow() const
            {
                return _row;
            }
        private:
            friend class RTPFECEncoder;
            ByteBlock _data {}; //!< Complete parity wire bytes, owned independently of media.
            bool _row = false;  //!< Stream direction; column uses media port +2, row uses +4.
        };
        using Datagrams = std::vector<Datagram>; //!< At most two datagrams emitted per media packet.

        //! Default constructor; call reset() before adding media.
        RTPFECEncoder() = default;
        //! Destructor; owned containers release all pending parity.
        virtual ~RTPFECEncoder();
        //! Validate and configure a new session, discarding old parity.
        //! @param [in] columns L, in 1..20 (4..20 with rows enabled).
        //! @param [in] rows D, in 4..20; L * D must not exceed 100.
        //! @param [in] two_dimensional Enable row parity in addition to columns.
        //! @param [in] block_aligned Align column starts in a block (Annex C), otherwise stagger them (Annex B).
        //! @return True on success; invalid configuration leaves the session unchanged.
        bool reset(size_t columns = DEFAULT_COLUMNS, size_t rows = DEFAULT_ROWS, bool two_dimensional = true, bool block_aligned = true);
        //! Discard configuration and pending parity. reset() is required before reuse.
        void clear();
        //! Consume one consecutive MPEG-TS RTP media packet without modifying it.
        //! Fixed RTP header, PT 33 and 1..7 TS188/RS204 packets are required.
        //! Variable payload lengths are supported, including short final bursts.
        //! Sequence numbers must be consecutive modulo 65536 and SSRC must stay fixed.
        //! @param [in] address Complete media wire data; null is rejected.
        //! @param [in] size Size of @a address in bytes, validated before access.
        //! @param [out] output Cleared on every call; receives zero, one or two parity packets.
        //! @return True on success; rejected input does not change the encoder session.
        bool addMedia(const void* address, size_t size, Datagrams& output);

    private:
        using _Media = std::span<const uint8_t>; //!< Borrowed wire bytes, valid only during ingestion.
        size_t _columns_count = 0; //!< L; zero means unconfigured.
        size_t _rows_count = 0;    //!< D, independent of the row-stream enable flag.
        bool _two_dimensional = false; //!< Row parity is optional; column parity is always generated.
        bool _block_aligned = true; //!< Column starts share a row only with Annex C shaping.
        bool _initialized = false; //!< Media sequence and SSRC are anchored.
        uint16_t _expected_sequence = 0; //!< Next required media wire sequence, modulo 65536.
        uint32_t _ssrc = 0; //!< Media session identifier; parity SSRC is always zero.
        size_t _position = 0; //!< Media position within the current L * D matrix.
        size_t _next_column = 0; //!< Next completed column due in the following matrix.
        uint16_t _column_sequence = 0; //!< Independent column RTP sequence, modulo 65536.
        uint16_t _row_sequence = 0; //!< Independent row RTP sequence, modulo 65536.
        ByteBlock _row {}; //!< Incrementally reduced parity for the current row.
        std::vector<ByteBlock> _columns {}; //!< L incremental column equations; media are not retained.
        std::vector<ByteBlock> _pending {}; //!< Completed columns waiting for traffic shaping.

        //! Validate the media profile and the current session without changing state.
        //! @param [in] media Borrowed wire packet with its size already bounded.
        //! @return True for supported, consecutive media in the configured session.
        bool _validMedia(const _Media& media) const;
        //! XOR known media into a wire-format equation, zero-extending shorter payloads.
        //! @param [in,out] parity Equation, initialized when empty.
        //! @param [in] media Validated media.
        //! @param [in] row True for a consecutive row, false for an interleaved column.
        void _accumulate(ByteBlock& parity, const _Media& media, bool row) const;
        //! Move one completed equation into the caller's output, assigning its RTP sequence.
        //! @param [in,out] parity Completed equation, emptied by the move.
        //! @param [in] row Parity stream whose sequence is advanced.
        //! @param [out] output Destination for the emitted parity.
        void _emit(ByteBlock& parity, bool row, Datagrams& output);
        //! Accumulate one media packet into staggered columns, emitting complete equations after L intervals.
        //! @param [in] media Validated media at the current matrix position.
        //! @param [out] output Destination for a column whose required delay has elapsed.
        void _staggeredColumn(const _Media& media, Datagrams& output);
    };
}
