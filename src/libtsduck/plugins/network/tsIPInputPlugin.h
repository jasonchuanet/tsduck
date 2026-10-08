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
        //! Superclass for initialization and lifecycle delegation.
        using SuperClass = AbstractDatagramInputPlugin;

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
        UDPReceiverArgs _sock_args {}; //!< Common media socket options.
        UDPReceiver     _sock {this}; //!< Media UDP receiver.

        // FEC is optional. With FEC disabled, the original single-socket path is used.
        size_t _fec_streams = 0;                 //!< Disabled, columns only, or columns and rows.
        size_t _fec_buffer_size = RTPFECDecoder::DEFAULT_BUFFER_SIZE; //!< A datagram window, not the socket's byte buffer.
        cn::milliseconds _fec_latency {RTPFECDecoder::DEFAULT_LATENCY}; //!< Covers parity transmission and network jitter.
        UDPReceiver _column_sock {this}; //!< Column parity receiver on media port plus two.
        UDPReceiver _row_sock {this}; //!< Optional row parity receiver on media port plus four.
        RTPFECDecoder _fec {}; //!< Decoder accessed exclusively by the input thread.
        std::atomic<bool> _closing {false};       //!< Accessed by the input, abort and receive threads.
        std::atomic<uint64_t> _queue_overflows {0}; //!< Updated by all enabled receive threads.
        bool _input_ended = false;                //!< Only accessed by the input consumer.
        IPSocketAddress _media_source {};  //!< One media sender; parity must originate from the same IP address.

        // One bounded queue merges media and parity. Blocking receive threads preserve
        // UDPReceiver's source, multicast, timeout and kernel timestamp handling on all OS's.
        // Strong stream identifiers prevent mixing direction with queue capacity.
        //! Identify the receiving socket independently of parity header contents.
        enum class Stream {
            MEDIA, //!< Original media stream.
            COLUMN, //!< Column parity stream.
            ROW, //!< Row parity stream.
        };
        //! Queued RTP data with the socket source and end-of-input marker.
        class Datagram: public RTPFECDecoder::Datagram
        {
            friend class IPInputPlugin;
        public:
            //! Media wire data and receive metadata.
            using SuperClass = RTPFECDecoder::Datagram;
            //! Default constructor.
            Datagram() = default;
            //! Copy constructor.
            //! @param [in] other Queued datagram to copy.
            Datagram(const Datagram& other) = default;
            //! Destructor.
            virtual ~Datagram() override = default;
            //! Copy assignment.
            //! @param [in] other Queued datagram to copy.
            //! @return A reference to this object.
            Datagram& operator=(const Datagram& other) = default;
        private:
            Stream _stream = Stream::MEDIA; //!< Parity direction is independent of FEC's media sequence.
            IPSocketAddress _sender {};    //!< Source port pins media; only the IP address pins parity.
            bool _end = false;             //!< A unique end marker wakes a waiting input consumer.
        };
        MessageQueue<Datagram> _queue {}; //!< Bounded shared queue merging enabled receive streams.
        std::deque<std::shared_ptr<Datagram>> _early_fec {}; //!< Bounded parity received before media source association.

        //! One socket worker; the decoder remains exclusively on the input thread.
        class Receiver: public Thread
        {
            TS_NOCOPY(Receiver);
        public:
            //! Thread lifecycle implementation.
            using SuperClass = Thread;
            //! Default construction is disabled: a worker requires its owning plugin.
            Receiver() = delete;
            //! Constructor; the explicit stack size leaves the large receive buffer on the heap.
            //! @param [in,out] plugin Plugin whose queue receives datagrams.
            //! @param [in,out] socket Socket to receive from until closed.
            //! @param [in] stream Identity of the receiving socket.
            Receiver(IPInputPlugin& plugin, UDPReceiver& socket, Stream stream);
            //! Destructor; joins the worker before its fields are destroyed.
            virtual ~Receiver() override;
        private:
            IPInputPlugin& _plugin; //!< Owning plugin, which outlives this worker.
            UDPReceiver& _socket; //!< Receiver socket closed before joining this worker.
            const Stream _stream; //!< Identity of the worker socket.
            //! Receive bounded wire datagrams and retain their kernel metadata.
            virtual void main() override;
        };
        Receiver _media_receiver {*this, _sock, Stream::MEDIA}; //!< Media UDP receiver.
        Receiver _column_receiver {*this, _column_sock, Stream::COLUMN}; //!< Column parity receiver on media port plus two.
        Receiver _row_receiver {*this, _row_sock, Stream::ROW}; //!< Optional row parity receiver on media port plus four.

        //! Close open sockets, join workers, and release queued storage; repeatable.
        void _closeReceivers();

        //! Wait for buffered playout, retaining kernel metadata for the superclass.
        //! @param [out] buffer Destination for the complete media RTP datagram.
        //! @param [in] buffer_size Capacity of @a buffer in bytes.
        //! @param [out] ret_size Number of bytes copied on success.
        //! @param [out] timestamp Original kernel timestamp, or -1 after recovery.
        //! @param [out] timesource Origin of the kernel timestamp, when available.
        //! @return True on a complete datagram, false on abort, end or a buffer error.
        bool _receiveFEC(uint8_t* buffer, size_t buffer_size, size_t& ret_size, cn::microseconds& timestamp, TimeSource& timesource);
        //! Associate queued data with its media session, then ingest it.
        //! @param [in] datagram Queued wire data or an end marker; ownership remains shared.
        void _processDatagram(const std::shared_ptr<Datagram>& datagram);

        //! Map UDP receive metadata consistently on direct and FEC input.
        //! @param [in] type Kernel timestamp clock type.
        //! @return Corresponding plugin time source, or UNDEFINED when unavailable.
        TimeSource _timestampSource(UDPSocket::TimeStampType type) const;
    };
}
