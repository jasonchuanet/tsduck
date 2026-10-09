//----------------------------------------------------------------------------
//
// TSDuck - The MPEG Transport Stream Toolkit
// Copyright (c) 2026, Jason Chua
// BSD-2-Clause license, see LICENSE.txt file or https://tsduck.io/license
//
//----------------------------------------------------------------------------
// Check FEC parser diagnostics without depending on process-exit logging.

#include "tsIPInputPlugin.h"
#include "tsIPOutputPlugin.h"
#include "tsReportBuffer.h"
#include "tsunit.h"

// These tests only construct and parse plugins; they never start a socket or
// processing thread. A report buffer captures diagnostics before any owner can
// exit, independently of the asynchronous logger's startup behavior.
class RTPFECOptionsTest: public tsunit::Test
{
    TSUNIT_DECLARE_TEST(InputParsing);
    TSUNIT_DECLARE_TEST(OutputParsing);

private:
    // Each rejected command gets a fresh plugin, with no previous parsed state.
    // The report outlives the plugin, including its ordinary destructor cleanup.
    template <typename PLUGIN>
    void _reject(const ts::UStringVector& arguments, const ts::UString& message)
    {
        ts::ReportBuffer<> report;
        // No TSP callback is needed for argument parsing alone.
        PLUGIN plugin(nullptr);
        plugin.delegateReport(&report);
        // Return parsing errors to the test rather than terminating its process.
        plugin.setFlags(plugin.getFlags() | ts::Args::NO_EXIT_ON_ERROR);
        // Use the real CLI option definitions without constructing PluginThread.
        // Invalid syntax must fail before option loading or any socket setup.
        TSUNIT_ASSERT(!plugin.analyze(u"ip", arguments, false));
        // Rejection alone could hide an unrelated error; check its explanation.
        TSUNIT_ASSERT(report.messages().find(message) != ts::NPOS);
    }
};

TSUNIT_REGISTER(RTPFECOptionsTest);

// Input uses a flag, not an output-style dimension selector.
TSUNIT_DEFINE_TEST(InputParsing)
{
    _reject<ts::IPInputPlugin>({u"5000", u"--smpte-2022-fec=2d"}, u"no value allowed");
    // The former development spelling must not become an undocumented alias.
    _reject<ts::IPInputPlugin>({u"5000", u"--fec=2"}, u"unknown option");
}

// Output needs a mode and accepts only the documented software arrangements.
TSUNIT_DEFINE_TEST(OutputParsing)
{
    _reject<ts::IPOutputPlugin>({u"127.0.0.1:5000", u"--smpte-2022-fec"}, u"missing value");
    _reject<ts::IPOutputPlugin>({u"127.0.0.1:5000", u"--fec=2"}, u"unknown option");
    // Vendor packet-insertion timing names are not additional wire codes.
    // The CLI must reject them instead of silently approximating their timing.
    for (const auto* mode : {u"2d-m1", u"2d-m1-b", u"2d-m2", u"2d-m2-b"}) {
        _reject<ts::IPOutputPlugin>({u"127.0.0.1:5000", u"--rtp", u"--smpte-2022-fec", mode}, mode);
    }
}
