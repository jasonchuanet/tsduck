//----------------------------------------------------------------------------
//
// TSDuck - The MPEG Transport Stream Toolkit
// Copyright (c) 2005-2026, Thierry Lelegard, Jason Chua
// BSD-2-Clause license, see LICENSE.txt file or https://tsduck.io/license
//
//----------------------------------------------------------------------------
//!
//!  @file
//!  IP input plugin for tsp.
//!
//----------------------------------------------------------------------------

#pragma once
#include "tsAbstractDatagramInputPlugin.h"
#include "tsUDPReceiver.h"
#include "tsRTPFECDecoder.h"
#include "tsMessageQueue.h"
#include "tsThread.h"

namespace ts {
    //!
    //! IP input plugin for tsp.
    //! @ingroup libtsduck plugin
    //!
    class TSDUCKDLL IPInputPlugin: public AbstractDatagramInputPlugin
    {
        TS_PLUGIN_CONSTRUCTORS(IPInputPlugin);
    public:
        //! Destructor.
        virtual ~IPInputPlugin() override;

        // Implementation of plugin API.
        virtual bool getOptions() override;
        virtual bool start() override;
        virtual bool stop() override;
        virtual bool abortInput() override;
        virtual bool setReceiveTimeout(cn::milliseconds timeout) override;

    protected:
        // Implementation of AbstractDatagramInputPlugin.
        virtual bool receiveDatagram(uint8_t* buffer, size_t buffer_size, size_t& ret_size, cn::microseconds& timestamp, TimeSource& timesource) override;

    private:
        UDPReceiverArgs _sock_args {};
        UDPReceiver     _sock {this};

        // FEC is optional. With FEC disabled, the original single-socket path is used.
        size_t _fec_streams = 0;                 // Disabled, columns only, or columns and rows.
        size_t _fec_buffer_size = 4096;           // Separate from the socket's byte buffer option.
        cn::milliseconds _fec_latency {1000};    // Wait for parity and reorder media before playout.
        UDPReceiver _column_sock {this};
        UDPReceiver _row_sock {this};
        RTPFECDecoder _fec {};
        std::atomic<bool> _closing {false};       // Accessed by the input, abort and receive threads.
        std::atomic<uint64_t> _queue_overflows {0}; // Updated by all enabled receive threads.
        bool _input_ended = false;                // Only accessed by the input consumer.
        IPSocketAddress _media_source {};  // One media sender; parity must originate from the same IP address.

        // One bounded queue merges media and parity. Blocking receive threads preserve
        // UDPReceiver's source, multicast, timeout and kernel timestamp handling on all OS's.
        struct Datagram: public RTPFECDecoder::Datagram
        {
            size_t stream = 0;       // 0: media, 1: column parity, 2: row parity.
            IPSocketAddress sender {};
            bool end = false;       // Wake the consumer after a receive error or abort.
        };
        MessageQueue<Datagram> _queue {};
        std::deque<std::shared_ptr<Datagram>> _early_fec {};

        // Each enabled stream has one receiver. The decoder itself stays on the input thread.
        class Receiver: public Thread
        {
            TS_NOCOPY(Receiver);
        public:
            Receiver(IPInputPlugin& plugin, UDPReceiver& socket, size_t stream);
            virtual ~Receiver() override;
        private:
            IPInputPlugin& _plugin;
            UDPReceiver& _socket;
            size_t _stream;
            virtual void main() override;
        };
        Receiver _media_receiver {*this, _sock, 0};
        Receiver _column_receiver {*this, _column_sock, 1};
        Receiver _row_receiver {*this, _row_sock, 2};

        // Close all sockets to interrupt receives, and then join the receiver threads.
        void closeReceivers();

        // Map UDP receive metadata consistently on both the direct and FEC paths.
        static TimeSource TimestampSource(UDPSocket::TimeStampType type);
    };
}
