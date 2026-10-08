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

TS_REGISTER_INPUT_PLUGIN(u"ip", ts::IPInputPlugin);


//----------------------------------------------------------------------------
// Input constructor
//----------------------------------------------------------------------------

ts::IPInputPlugin::IPInputPlugin(TSP* tsp_) :
    AbstractDatagramInputPlugin(tsp_, IP_MAX_PACKET_SIZE, u"Receive TS packets from UDP/IP, multicast or unicast", u"[options] [address:]port",
                                u"kernel", u"A kernel-provided timestamp for the packet, when available (Linux only)",
                                TSDatagramInputOptions::REAL_TIME | TSDatagramInputOptions::ALLOW_RS204)
{
    // Add UDP receiver common options.
    _sock_args.defineArgs(*this, true, true);

    option(u"fec", 0, INTEGER, 0, 1, 1, 2, true);
    help(u"fec", u"1|2",
         u"Enable SMPTE ST 2022-1 FEC recovery for ST 2022-2 MPEG-TS over RTP. "
         u"Use 1 for column FEC (media port +2), or 2 for column and row FEC "
         u"(media ports +2 and +4). The default with no value is 2. "
         u"All streams use the same destination address and local interface. "
         u"FEC is disabled by default.");

    option<cn::milliseconds>(u"fec-latency", 0, 0, 1, 1, 60000);
    help(u"fec-latency",
         u"Specify the FEC playout latency in milliseconds. The default is 1000. "
         u"Allow time for the sender's FEC matrix and network jitter. "
         u"Unrecoverable losses are skipped when this delay expires.");

    option(u"fec-buffer-size", 0, INTEGER, 0, 1, 256, 32767);
    help(u"fec-buffer-size", u"datagrams",
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
    const bool ok = AbstractDatagramInputPlugin::getOptions() && _sock_args.loadArgs(*this, _sock.parameters().receive_timeout);
    _sock.setParameters(_sock_args);                    // Media keeps all existing UDP receiver options.
    _fec_streams = present(u"fec") ? intValue<size_t>(u"fec", 2) : 0; // Bare --fec enables both streams.
    getIntValue(_fec_buffer_size, u"fec-buffer-size", 4096); // A sequence window, not a byte buffer size.
    getChronoValue(_fec_latency, u"fec-latency", cn::milliseconds(1000)); // Monotonic playout delay.
    if (_fec_streams == 0 && (present(u"fec-latency") || present(u"fec-buffer-size"))) {
        error(u"--fec-latency and --fec-buffer-size require --fec");
        return false;                                  // Reject options that would otherwise be silently ignored.
    }
    if (_fec_streams > 0 && _sock_args.destination.port() > 65535 - 2 * _fec_streams) {
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
    if (!AbstractDatagramInputPlugin::start() || !_sock.open()) {
        return false;
    }
    if (_fec_streams == 0) {
        return true;                                   // No workers or parity sockets on ordinary UDP input.
    }
    _closing = false;                                  // Receiver objects can be reused after plugin restart.
    _queue_overflows = 0;                               // Queue loss is reported separately from media loss.
    _input_ended = false;                               // A receive timeout still permits buffered playout.
    _media_source.clear();                             // Pin the first valid media sender for this run.
    _early_fec.clear();
    _queue.clear();
    _queue.setMaxMessages(_fec_buffer_size);            // Producers never grow the queue without a bound.
    _fec.reset(_fec_latency, _fec_buffer_size);          // Reset history, equations and recovery statistics.

    // Preserve destination, local interface, multicast membership and explicit
    // source IP filtering. FEC source ports differ from the media source port.
    // Inactivity of a parity stream is normal, especially with lost FEC packets.
    UDPReceiverArgs args(_sock_args);                  // Inherit multicast, SSM, interface and source IP.
    args.receive_timeout = cn::milliseconds(-1);        // Absence of parity must not stop the media input.
    args.use_first_source = false;                     // Parity is pinned to the accepted media IP instead.
    args.source.setPort(IPSocketAddress::AnyPort);       // Interoperate with senders using separate FEC source ports.
    args.destination.setPort(_sock_args.destination.port() + 2); // First parity stream: columns.
    _column_sock.setParameters(args);
    if (_fec_streams == 2) {
        args.destination.setPort(_sock_args.destination.port() + 4); // Second parity stream: rows.
        _row_sock.setParameters(args);
    }
    if (!_column_sock.open() || (_fec_streams == 2 && !_row_sock.open()) ||
        !_media_receiver.start() || !_column_receiver.start() || (_fec_streams == 2 && !_row_receiver.start())) {
        closeReceivers();                             // Undo even a partially successful socket/thread startup.
        return false;
    }
    return true;
}


//----------------------------------------------------------------------------
// Input stop method
//----------------------------------------------------------------------------

bool ts::IPInputPlugin::stop()
{
    closeReceivers();                                 // Join before reading statistics written during reception.
    if (_fec_streams > 0) {
        verbose(u"FEC recovered %'d RTP datagrams, lost %'d RTP datagrams", _fec.recoveredPackets(), _fec.lostPackets());
        if (_queue_overflows > 0) {
            warning(u"FEC receive queue dropped %'d datagrams; increase --fec-buffer-size", _queue_overflows.load());
        }
    }
    return AbstractDatagramInputPlugin::stop();         // Release the superclass's datagram state as usual.
}


//----------------------------------------------------------------------------
// Input abort method
//----------------------------------------------------------------------------

bool ts::IPInputPlugin::abortInput()
{
    debug(u"aborting IP input");
    // Do not join here: abortInput can run while the input thread is waiting.
    _closing = true;                                  // Publish shutdown before interrupting any socket.
    _sock.close(true);                                // Closing a socket interrupts its blocking receiver.
    _column_sock.close(true);                          // A parity worker can be idle for the entire stream.
    _row_sock.close(true);                             // Also safe when the optional row socket was never opened.
    if (_fec_streams > 0) {
        auto end = std::make_shared<Datagram>();
        end->end = true;
        _queue.forceEnqueue(end);                      // Wake a consumer even when the ordinary queue is full.
    }
    return true;
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
    if (_fec_streams > 0) {
        // A timed dequeue also drives playout when all senders are quiet.
        // A media receive timeout ends input after buffered media have drained.
        size_t received = 0;                          // Bound queue draining between two playout operations.
        while (!_closing) {
            RTPFECDecoder::Datagram output;
            // Apply queued parity before expiring gaps, but limit each batch
            // so continuous traffic cannot starve playout.
            if ((received >= _fec_buffer_size || _queue.currentQueueSize() == 0 || _fec.isPlayoutBufferFull()) &&
                _fec.getDatagram(output, RTPFECDecoder::Clock::now())) {
                ret_size = std::min(buffer_size, output.data.size()); // Respect the datagram API's buffer bound.
                MemCopy(buffer, output.data.data(), ret_size);      // Include RTP for superclass timestamp extraction.
                timestamp = output.timestamp;                      // Originals retain their kernel receive time.
                timesource = TimestampSource(output.timestamp_type); // Recoveries have no kernel timestamp.
                return true;
            }
            cn::milliseconds delay = _fec.timeToNextDatagram(RTPFECDecoder::Clock::now());
            if (_input_ended && delay == cn::milliseconds::max() && _queue.currentQueueSize() == 0) {
                return false;                         // Timeout/end only finishes after queue and playout drain.
            }
            // Periodically check abort even without any traffic or buffered media.
            delay = std::min(delay, cn::milliseconds(100)); // Make quiet-stream aborts prompt on every platform.
            std::shared_ptr<Datagram> datagram;
            if (!_queue.dequeue(datagram, delay)) {
                continue;                             // A dequeue timeout may simply mean media is ready to play.
            }
            ++received;
            if (datagram->end) {
                _input_ended = true;                   // Do not discard buffered data after a socket error.
            }
            else if (datagram->stream == 0) {
                // Pin parity to the first accepted media sender. Explicit UDP
                // filtering is already applied by UDPReceiver on each socket.
                if ((!_media_source.hasAddress() || _media_source == datagram->sender) && _fec.addMedia(*datagram)) {
                    _media_source = datagram->sender;  // Remember the media port as well as its IP address.
                    for (const auto& early : _early_fec) {
                        if (IPAddress(_media_source) == IPAddress(early->sender)) {
                            _fec.addFEC(*early, early->stream == 2); // Replay only parity from the accepted media IP.
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
            else if (IPAddress(_media_source) == IPAddress(datagram->sender)) {
                _fec.addFEC(*datagram, datagram->stream == 2); // Reject malformed parity inside the decoder.
            }
        }
        return false;
    }

    IPSocketAddress sender;
    IPSocketAddress destination;
    UDPSocket::TimeStampType ts_type = UDPSocket::TimeStampType::NONE;
    const bool ok = _sock.receive(buffer, buffer_size, ret_size, sender, destination, tsp, &timestamp, &ts_type);
    timesource = TimestampSource(ts_type);             // Same metadata mapping on direct and buffered input.
    return ok;
}


//----------------------------------------------------------------------------
// Socket workers are only started when FEC is enabled. Queue overflow drops
// datagrams rather than blocking shutdown or allocating unbounded memory.
//----------------------------------------------------------------------------

ts::IPInputPlugin::Receiver::Receiver(IPInputPlugin& plugin, UDPReceiver& socket, size_t stream) :
    _plugin(plugin),
    _socket(socket),
    _stream(stream)
{
}

ts::IPInputPlugin::Receiver::~Receiver()
{
    waitForTermination();                             // No worker may outlive its socket or owning plugin.
}

void ts::IPInputPlugin::Receiver::main()
{
    ByteBlock buffer(IP_MAX_PACKET_SIZE);              // A reusable receive buffer avoids truncating bad datagrams.
    while (!_plugin._closing) {
        auto datagram = std::make_shared<Datagram>();
        datagram->stream = _stream;                    // End markers also identify which receiver stopped.
        IPSocketAddress destination;
        size_t size = 0;
        if (!_socket.receive(buffer.data(), buffer.size(), size, datagram->sender, destination, _plugin.tsp,
                             &datagram->timestamp, &datagram->timestamp_type)) {
            if (!_plugin._closing) {
                datagram->end = true;
                _plugin._queue.forceEnqueue(datagram); // Ensure an error reaches a waiting input consumer.
            }
            break;
        }
        datagram->arrival = RTPFECDecoder::Clock::now(); // Deadline clock is independent of kernel timestamp origin.
        // Reject oversized profile packets before allocating queued storage.
        if (size > RTP_HEADER_SIZE + (_stream == 0 ? 0 : 16) + 7 * PKT_RS_SIZE) {
            continue;
        }
        datagram->data.assign(buffer.begin(), buffer.begin() + size); // Queue only the received payload bytes.
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
    closeReceivers();
}

void ts::IPInputPlugin::closeReceivers()
{
    _closing = true;
    _sock.close(true);
    _column_sock.close(true);
    _row_sock.close(true);
    _media_receiver.waitForTermination();              // No producer may access the queue after it is cleared.
    _column_receiver.waitForTermination();             // Safe for a receiver that failed to start.
    _row_receiver.waitForTermination();                // Safe in column-only mode as well.
    _queue.clear();                                   // Release datagrams still pending after abort.
    _early_fec.clear();                               // Release parity which never found its media source.
}

ts::TimeSource ts::IPInputPlugin::TimestampSource(UDPSocket::TimeStampType type)
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
