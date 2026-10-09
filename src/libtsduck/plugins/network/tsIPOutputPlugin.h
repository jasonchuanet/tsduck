//----------------------------------------------------------------------------
//
// TSDuck - The MPEG Transport Stream Toolkit
// Copyright (c) 2005-2026, Thierry Lelegard, Jason Chua
// BSD-2-Clause license, see LICENSE.txt file or https://tsduck.io/license
//
//----------------------------------------------------------------------------
//!
//!  @file
//!  IP output plugin for tsp.
//!
//----------------------------------------------------------------------------

#pragma once
#include "tsOutputPlugin.h"
#include "tsTSDatagramOutput.h"

namespace ts {
    //!
    //! IP output plugin for tsp.
    //! @ingroup libtsduck plugin
    //! Optional ST 2022-1 FEC protects ST 2022-2 RTP media without modifying TS payloads.
    //! Media and parity share their UDP source port and all configured socket settings.
    //! Matrix geometry and profile validation belong to the datagram output's FEC helper.
    //! This plugin adds no receive socket or worker thread.
    //!
    class TSDUCKDLL IPOutputPlugin: public OutputPlugin
    {
        TS_PLUGIN_CONSTRUCTORS(IPOutputPlugin);
    public:
        // Implementation of plugin API.
        virtual bool getOptions() override;
        virtual bool start() override;
        virtual bool stop() override;
        virtual bool isRealTime() override;
        virtual bool send(const TSPacket*, const TSPacketMetadata*, size_t) override;

    private:
        using SuperClass = OutputPlugin; //!< Plugin superclass.
        TSDatagramOutput _datagram {*this, TSDatagramOutputOptions::ALLOW_RTP | TSDatagramOutputOptions::ALLOW_RS204 | TSDatagramOutputOptions::ALLOW_FEC}; //!< Media and optional FEC use one UDP socket.
    };
}
