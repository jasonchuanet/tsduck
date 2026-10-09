//----------------------------------------------------------------------------
//
// TSDuck - The MPEG Transport Stream Toolkit
// Copyright (c) 2005-2026, Thierry Lelegard, Jason Chua
// BSD-2-Clause license, see LICENSE.txt file or https://tsduck.io/license
//
//----------------------------------------------------------------------------

#include "tsIPInputPlugin.h"
#include "tsPluginRepository.h"
#include "tsIPProtocols.h"
#include "tsMemory.h"

namespace ts {
    namespace {
        // Receive buffers live on the heap; keep generous headroom for socket
        // filtering, report formatting and their platform-dependent call stacks.
        constexpr size_t RECEIVER_STACK_SIZE = 128 * 1024;
        constexpr cn::milliseconds ABORT_POLL_INTERVAL {100};
        constexpr uint16_t COLUMN_PORT_OFFSET = 2; // Standard RTP/RTCP port pairs reserve the adjacent odd port.
        constexpr uint16_t ROW_PORT_OFFSET = 4; // Both parity sockets are opened; a 1D sender leaves the row stream quiet.
        constexpr uint16_t MAX_PORT = 65535; // Check destination arithmetic before narrowing back to a port.
    }
}

TS_REGISTER_INPUT_PLUGIN(u"ip", ts::IPInputPlugin);


//----------------------------------------------------------------------------
// Input constructor
//----------------------------------------------------------------------------

ts::IPInputPlugin::IPInputPlugin(TSP* tsp_) :
    SuperClass(tsp_, IP_MAX_PACKET_SIZE, u"Receive TS packets from UDP/IP, multicast or unicast", u"[options] [address:]port",
                                u"kernel", u"A kernel-provided timestamp for the packet, when available (Linux only)",
                                TSDatagramInputOptions::REAL_TIME | TSDatagramInputOptions::ALLOW_RS204)
{
    // Reuse the common argument definitions so FEC inherits the established
    // multicast, source filtering and SSM behavior of direct UDP reception.
    _sock_args.defineArgs(*this, true, true);

    option(u"smpte-2022-fec"); // Match dektec input: a receiver discovers geometry from the parity headers.
    help(u"smpte-2022-fec",
         u"Enable SMPTE ST 2022-1 FEC recovery for ST 2022-2 MPEG-TS over RTP. "
         u"Receive column FEC on media port +2 and optional row FEC on media port +4. "
         u"Both 1D/2D and block/non-block arrangements are detected from the headers. "
         u"All streams use the same destination address and local interface. "
         u"FEC is disabled by default.");

    option<cn::milliseconds>(u"smpte-2022-fec-latency", 0, 0, 1, RTPFECDecoder::MIN_LATENCY.count(), RTPFECDecoder::MAX_LATENCY.count());
    help(u"smpte-2022-fec-latency",
         u"Specify the FEC playout latency in milliseconds. The default is 1000. "
         u"Allow time for the sender's FEC matrix and network jitter. "
         u"Unrecoverable losses are skipped when this delay expires.");

    option(u"smpte-2022-fec-buffer-size", 0, INTEGER, 0, 1, RTPFECDecoder::MIN_BUFFER_SIZE, RTPFECDecoder::MAX_BUFFER_SIZE);
    help(u"smpte-2022-fec-buffer-size", u"datagrams",
         u"Bound the FEC media sequence window, parity buffer and receive queue. "
         u"The default is 4096 RTP datagrams. Increase it for high bitrates or long latency. "
         u"When the sequence window fills, older media are released early.");
}


//----------------------------------------------------------------------------
// Input command line options method
//----------------------------------------------------------------------------

bool ts::IPInputPlugin::getOptions()
{
    // Get command line arguments for superclass and socket.
    const bool ok = SuperClass::getOptions() && _sock_args.loadArgs(*this, _sock.parameters().receive_timeout);
    _sock.setParameters(_sock_args);                    // Media keeps all existing UDP receiver options.
    _fec_enabled = present(u"smpte-2022-fec"); // Row silence is normal for a column-only sender.
    getIntValue(_fec_buffer_size, u"smpte-2022-fec-buffer-size", RTPFECDecoder::DEFAULT_BUFFER_SIZE); // A datagram window, not bytes.
    getChronoValue(_fec_latency, u"smpte-2022-fec-latency", RTPFECDecoder::DEFAULT_LATENCY); // Monotonic playout delay.
    if (!_fec_enabled && (present(u"smpte-2022-fec-latency") || present(u"smpte-2022-fec-buffer-size"))) {
        error(u"--smpte-2022-fec-latency and --smpte-2022-fec-buffer-size require --smpte-2022-fec");
        return false;                                  // Reject options that would otherwise be silently ignored.
    }
    if (_fec_enabled && _sock_args.destination.port() > MAX_PORT - ROW_PORT_OFFSET) {
        error(u"media port is too high for the FEC ports");
        return false;                                  // Prevent wrapping a parity destination back to port zero.
    }
    return ok;
}


//----------------------------------------------------------------------------
// Input start method
//----------------------------------------------------------------------------

bool ts::IPInputPlugin::start()
{
    // Initialize superclass and media socket, retaining the direct UDP path.
    if (!SuperClass::start() || !_sock.open()) {
        return false;
    }
    if (!_fec_enabled) {
        return true;                                   // No workers or parity sockets on ordinary UDP input.
    }
    _closing = false;                                  // Receiver objects can be reused after plugin restart.
    _queue_overflows = 0;                               // Queue loss is reported separately from media loss.
    _input_ended = false;                               // A receive timeout still permits buffered playout.
    _media_source.clear();                             // Pin the first valid media sender for this run.
    _early_fec.clear();                                // No old parity may join the new media session.
    _queue.clear();                                    // Restart must not replay the previous run's queue.
    _queue.setMaxMessages(_fec_buffer_size);            // Producers never grow the queue without a bound.
    _fec.reset(_fec_latency, _fec_buffer_size);          // Reset history, equations and recovery statistics.

    // Preserve destination, local interface, multicast membership and explicit
    // source IP filtering. FEC source ports differ from the media source port.
    // Inactivity of a parity stream is normal, especially with lost FEC packets.
    UDPReceiverArgs args(_sock_args);                  // Inherit multicast, SSM, interface and source IP.
    args.receive_timeout = cn::milliseconds(-1);        // Absence of parity must not stop the media input.
    args.use_first_source = false;                     // Parity is pinned to the accepted media IP instead.
    args.source.setPort(IPSocketAddress::AnyPort);       // Interoperate with senders using separate FEC source ports.
    args.destination.setPort(_sock_args.destination.port() + COLUMN_PORT_OFFSET); // First parity stream: columns.
    _column_sock.setParameters(args);
    args.destination.setPort(_sock_args.destination.port() + ROW_PORT_OFFSET); // Receive either wire dimension without a mode option.
    _row_sock.setParameters(args);
    if (!_column_sock.open() || !_row_sock.open() ||
        !_media_receiver.start() || !_column_receiver.start() || !_row_receiver.start()) {
        _closeReceivers();                             // Undo even a partially successful socket/thread startup.
        return false;
    }
    return true;
}


//----------------------------------------------------------------------------
// Input stop method
//----------------------------------------------------------------------------

bool ts::IPInputPlugin::stop()
{
    _closeReceivers();                                 // Join before reading statistics written during reception.
    if (_fec_enabled) {
        verbose(u"FEC recovered %'d RTP datagrams, lost %'d RTP datagrams", _fec.recoveredPackets(), _fec.lostPackets());
        if (_queue_overflows.load() > 0) {
            warning(u"FEC receive queue dropped %'d datagrams; increase --smpte-2022-fec-buffer-size", _queue_overflows.load());
        }
    }
    return SuperClass::stop();         // Release the superclass's datagram state as usual.
}


//----------------------------------------------------------------------------
// Input abort method
//----------------------------------------------------------------------------

bool ts::IPInputPlugin::abortInput()
{
    debug(u"aborting IP input");
    // Do not join here: abortInput can run while the input thread is waiting.
    const bool already_closing = _closing.exchange(true); // Only one abort marker may overflow the queue.
    // A closed socket is already clean; close() reports false in that state.
    const bool media_closed = already_closing || !_sock.isOpen() || _sock.close(true); // Interrupt blocking media reception.
    const bool column_closed = already_closing || !_column_sock.isOpen() || _column_sock.close(true); // Parity can remain idle.
    const bool row_closed = already_closing || !_row_sock.isOpen() || _row_sock.close(true); // Row traffic may be absent.
    if (_fec_enabled && !already_closing) {
        auto end = std::make_shared<_Datagram>();
        end->_end = true; // An abort marker is unique; normal producers remain strictly bounded.
        _queue.forceEnqueue(end);                      // Wake a consumer even when the ordinary queue is full.
    }
    return media_closed && column_closed && row_closed;
}


//----------------------------------------------------------------------------
// Set receive timeout from tsp.
//----------------------------------------------------------------------------

bool ts::IPInputPlugin::setReceiveTimeout(cn::milliseconds timeout)
{
    if (timeout > cn::milliseconds::zero()) {
        _sock.setReceiveTimeoutArg(timeout);            // Only media inactivity determines the receive timeout.
    }
    return true;
}


//----------------------------------------------------------------------------
// Datagram reception method.
//----------------------------------------------------------------------------

bool ts::IPInputPlugin::receiveDatagram(uint8_t* buffer, size_t buffer_size, size_t& ret_size, cn::microseconds& timestamp, TimeSource& timesource)
{
    if (_fec_enabled) {
        return _receiveFEC(buffer, buffer_size, ret_size, timestamp, timesource);
    }

    IPSocketAddress sender;
    IPSocketAddress destination;
    UDPSocket::TimeStampType ts_type = UDPSocket::TimeStampType::NONE;
    const bool ok = _sock.receive(buffer, buffer_size, ret_size, sender, destination, tsp, &timestamp, &ts_type);
    timesource = _timestampSource(ts_type);             // Same metadata mapping on direct and buffered input.
    return ok;
}


//----------------------------------------------------------------------------
// FEC scheduling waits for either queued input or the next playout deadline.
// It drains buffered media on receive timeout, and promptly responds to abort.
//----------------------------------------------------------------------------

bool ts::IPInputPlugin::_receiveFEC(uint8_t* buffer, size_t buffer_size, size_t& ret_size, cn::microseconds& timestamp, TimeSource& timesource)
{
    // A timed dequeue also drives playout when all senders are quiet.
    // A media receive timeout ends input after buffered media have drained.
    size_t received = 0;                          // Bound queue draining between two playout operations.
    while (!_closing.load()) {
        RTPFECDecoder::Datagram output;
        // Apply queued parity before expiring gaps, but limit each batch
        // so continuous traffic cannot starve playout.
        if ((received >= _fec_buffer_size || _queue.currentQueueSize() == 0 || _fec.isPlayoutBufferFull()) &&
            _fec.getDatagram(output, RTPFECDecoder::Clock::now())) {
            if (buffer == nullptr || output.data().size() > buffer_size) {
                error(u"FEC datagram exceeds the receive buffer");
                return false;                         // Never silently truncate a reconstructed RTP packet.
            }
            ret_size = output.data().size();
            MemCopy(buffer, output.data().data(), ret_size);      // Include RTP for superclass timestamp extraction.
            timestamp = output.receiveTimestamp();                      // Originals retain their kernel receive time.
            timesource = _timestampSource(output.receiveTimestampType()); // Recoveries have no kernel timestamp.
            return true;
        }
        cn::milliseconds delay = _fec.timeToNextDatagram(RTPFECDecoder::Clock::now());
        if (_input_ended && delay == cn::milliseconds::max() && _queue.currentQueueSize() == 0) {
            return false;                         // Timeout/end only finishes after queue and playout drain.
        }
        // Periodically check abort even without any traffic or buffered media.
        delay = std::min(delay, ABORT_POLL_INTERVAL);   // Make quiet-stream aborts prompt on every platform.
        std::shared_ptr<_Datagram> datagram;
        if (!_queue.dequeue(datagram, delay)) {
            continue;                             // A dequeue timeout may simply mean media is ready to play.
        }
        ++received;
        _processDatagram(datagram);
    }
    return false;
}


//----------------------------------------------------------------------------
// Associate parity with the accepted media source before it reaches recovery.
// Malformed or unrelated datagrams are discarded without ending the stream.
//----------------------------------------------------------------------------

void ts::IPInputPlugin::_processDatagram(const std::shared_ptr<_Datagram>& datagram)
{
    if (datagram->_end) {
        _input_ended = true;                   // Do not discard buffered data after a socket error.
    }
    else if (datagram->_stream == _Stream::MEDIA) {
        // Pin parity to the first accepted media sender. Explicit UDP
        // filtering is already applied by UDPReceiver on each socket.
        if ((!_media_source.hasAddress() || _media_source == datagram->_sender) && _fec.addMedia(*datagram)) {
            _media_source = datagram->_sender;  // Remember the media port as well as its IP address.
            for (const auto& early : _early_fec) {
                if (IPAddress(_media_source) == IPAddress(early->_sender)) {
                    if (!_fec.addFEC(*early, early->_stream == _Stream::ROW)) {
                        continue;                      // Unsupported parity is discarded without ending valid input.
                    }
                }
            }
            _early_fec.clear();                // Unrelated early sources must not consume further storage.
        }
    }
    else if (!_media_source.hasAddress()) {
        if (_early_fec.size() < _fec_buffer_size) {
            _early_fec.push_back(datagram);    // Parity may beat the first media packet over the network.
        }
    }
    else if (IPAddress(_media_source) == IPAddress(datagram->_sender)) {
        if (!_fec.addFEC(*datagram, datagram->_stream == _Stream::ROW)) {
            return;                                    // Rejection intentionally discards unsupported parity.
        }
    }
}


//----------------------------------------------------------------------------
// Socket workers are only started when FEC is enabled. Queue overflow drops
// datagrams rather than blocking shutdown or allocating unbounded memory.
//----------------------------------------------------------------------------

// Keep the private queued value's vtable in the plugin library too.
ts::IPInputPlugin::_Datagram::~_Datagram() = default;

ts::IPInputPlugin::_Receiver::_Receiver(IPInputPlugin& plugin, UDPReceiver& socket, _Stream stream) :
    SuperClass(ThreadAttributes().setStackSize(RECEIVER_STACK_SIZE)),
    _plugin(plugin),
    _socket(socket),
    _stream(stream)
{
}

ts::IPInputPlugin::_Receiver::~_Receiver()
{
    if (!waitForTermination()) {
        _plugin.error(u"cannot join IP FEC receiver");
        std::terminate();                              // Destruction cannot safely continue with a live worker.
    }
}

void ts::IPInputPlugin::_Receiver::main()
{
    ByteBlock buffer(IP_MAX_PACKET_SIZE);              // A reusable receive buffer avoids truncating bad datagrams.
    while (!_plugin._closing.load()) {
        auto datagram = std::make_shared<_Datagram>();
        datagram->_stream = _stream;                    // End markers also identify which receiver stopped.
        IPSocketAddress destination;
        size_t size = 0; // Never allocate from an earlier receive length after a socket error.
        cn::microseconds timestamp {-1}; // A platform without kernel timestamping must not inherit old metadata.
        UDPSocket::TimeStampType timestamp_type = UDPSocket::TimeStampType::NONE; // Unavailable receive clocks stay explicitly unavailable.
        if (!_socket.receive(buffer.data(),
                             buffer.size(),
                             size,
                             datagram->_sender,
                             destination,
                             _plugin.tsp,
                             &timestamp,
                             &timestamp_type)) {
            if (!_plugin._closing.load()) {
                datagram->_end = true;
                _plugin._queue.forceEnqueue(datagram); // Ensure an error reaches a waiting input consumer.
            }
            break;
        }
        datagram->setArrival(RTPFECDecoder::Clock::now());
        datagram->setReceiveTimestamp(timestamp, timestamp_type); // Deadline clock is independent of kernel timestamp origin.
        // Reject oversized profile packets before allocating queued storage.
        if (size > RTP_HEADER_SIZE + (_stream == _Stream::MEDIA ? 0 : RTPFECDecoder::FEC_HEADER_SIZE) + RTPFECDecoder::MAX_TS_PACKETS * PKT_RS_SIZE) {
            continue;
        }
        datagram->data().assign(buffer.begin(), buffer.begin() + size); // Queue only the received payload bytes.
        if (!_plugin._queue.enqueue(datagram, cn::milliseconds::zero())) {
            ++_plugin._queue_overflows;                // Never block shutdown behind a full producer queue.
        }
    }
}


//----------------------------------------------------------------------------
// Shutdown order: interrupt receives, join workers, then discard queued data.
//----------------------------------------------------------------------------

ts::IPInputPlugin::~IPInputPlugin()
{
    _closeReceivers();
}

void ts::IPInputPlugin::_closeReceivers()
{
    _closing = true; // Publish shutdown before closing sockets or waiting for workers.
    // Test isOpen() so repeated cleanup and unused parity sockets stay successful.
    const bool media_closed = !_sock.isOpen() || _sock.close(true); // Interrupt every socket before joining any worker.
    const bool column_closed = !_column_sock.isOpen() || _column_sock.close(true); // Parity inactivity must not obstruct shutdown.
    const bool row_closed = !_row_sock.isOpen() || _row_sock.close(true); // Row traffic may be absent.
    if (!media_closed || !column_closed || !row_closed) {
        error(u"error closing IP input sockets");        // Continue cleanup even if a close reports failure.
    }
    const bool media_joined = _media_receiver.waitForTermination(); // Never-started receivers are safe here too.
    const bool column_joined = _column_receiver.waitForTermination(); // Wait before destroying storage shared with this producer.
    const bool row_joined = _row_receiver.waitForTermination(); // Join the row socket even when the sender emits only columns.
    if (!media_joined || !column_joined || !row_joined) {
        error(u"cannot join IP FEC receivers");
        std::terminate();                              // Never free storage still reachable by a live worker.
    }
    _queue.clear();                                   // Release datagrams still pending after abort.
    _early_fec.clear();                               // Release parity which never found its media source.
}

ts::TimeSource ts::IPInputPlugin::_timestampSource(UDPSocket::TimeStampType type) const
{
    switch (type) {
        case UDPSocket::TimeStampType::SOFTWARE:
            return TimeSource::KERNEL;
        case UDPSocket::TimeStampType::HARDWARE:
            return TimeSource::HARDWARE;
        case UDPSocket::TimeStampType::NONE:
        default:
            return TimeSource::UNDEFINED;
    }
}
