//----------------------------------------------------------------------------
//
// TSDuck - The MPEG Transport Stream Toolkit
// Copyright (c) 2026, Jason Chua
// BSD-2-Clause license, see LICENSE.txt file or https://tsduck.io/license
//
//----------------------------------------------------------------------------
//!
//! @file
//! Configure and send SMPTE RTP parity using an existing media UDP socket.
//!
//----------------------------------------------------------------------------

#pragma once
#include "tsRTPFECEncoder.h"
#include "tsUDPSocket.h"

namespace ts {
    class Args;
    //! FEC configuration and transmission for RTP UDP output, confined to its owner thread.
    //! @ingroup libtsduck net
    //! The media socket supplies source port, interface, TTL, TOS and multicast settings
    //! to every parity packet. This object owns no socket and creates no worker thread.
    class TSDUCKDLL RTPFECOutput
    {
        TS_NOBUILD_NOCOPY(RTPFECOutput);
    public:
        //! Constructor.
        //! @param [in,out] report Destination for configuration and transmission errors.
        explicit RTPFECOutput(Report& report);
        //! Destructor; releases parity without closing the borrowed media socket.
        virtual ~RTPFECOutput() = default;
        //! Define optional FEC and matrix command-line arguments.
        //! @param [in,out] args Argument definitions to extend.
        void defineArgs(Args& args) const;
        //! Load and validate FEC options against the media configuration.
        //! @param [in,out] args Parsed arguments; receives errors for incompatible options.
        //! @param [in] destination Media destination, used for parity ports +2 and +4.
        //! @param [in] rtp Whether RTP encapsulation is enabled.
        //! @param [in] payload_type Media RTP payload type, required to be 33 with FEC.
        //! @param [in] burst Maximum media TS packet count, required to be at most seven.
        //! @return True on success, false on an incompatible or invalid FEC configuration.
        bool loadArgs(Args& args, const IPSocketAddress& destination, bool rtp, uint8_t payload_type, size_t burst);
        //! Start a new session after media output has been configured.
        //! @return True on success; enabled FEC initializes bounded parity storage.
        bool reset();
        //! Discard partial groups and columns whose emission would need more media.
        //! No synthetic TS or prematurely transmitted parity is produced.
        void clear();
        //! Protect and transmit parity after the owner has successfully sent the media.
        //! @param [in,out] socket Media socket; parity uses explicit destinations, preserving its default.
        //! @param [in] address Complete media RTP datagram; borrowed during this call only.
        //! @param [in] size Size in bytes of @a address.
        //! @return True on success or when disabled; false on invalid media or transmission failure.
        bool send(UDPSocket& socket, const void* address, size_t size);
    private:
        Report& _report; //!< Error sink; outlives this configuration object.
        size_t _streams = 0; //!< Disabled, columns only, or columns and rows.
        size_t _columns = RTPFECEncoder::DEFAULT_COLUMNS; //!< Matrix width L.
        size_t _rows = RTPFECEncoder::DEFAULT_ROWS; //!< Matrix depth D.
        IPSocketAddress _column_destination {}; //!< Same media destination IP, UDP port N+2.
        IPSocketAddress _row_destination {}; //!< Same media destination IP, UDP port N+4.
        RTPFECEncoder _encoder {}; //!< Single-thread encoder; no media copies are retained.
        RTPFECEncoder::Datagrams _output {}; //!< At most two parity packets waiting for this call to send.
    };
}
