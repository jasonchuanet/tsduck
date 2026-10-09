//----------------------------------------------------------------------------
//
// TSDuck - The MPEG Transport Stream Toolkit
// Copyright (c) 2005-2026, Thierry Lelegard, Jason Chua
// BSD-2-Clause license, see LICENSE.txt file or https://tsduck.io/license
//
//----------------------------------------------------------------------------

#include "tsPluginThread.h"
#include "tsPluginRepository.h"
#include "tsEnvironment.h"


//----------------------------------------------------------------------------
// Constructor
//----------------------------------------------------------------------------

ts::PluginThread::PluginThread(Report* report, const UString& appName, PluginType type, const PluginOptions& options, const ThreadAttributes& attributes) :
    Thread(),
    TSP(report->maxSeverity(), options.name + u": ", report),
    _name(options.name),
    _plugin(nullptr)
{
    const UChar* shell_opt = nullptr;

    // Create the plugin instance object
    switch (type) {
        case PluginType::INPUT: {
            PluginRepository::InputPluginFactory allocator = PluginRepository::Instance().getInput(_name, *report);
            if (allocator != nullptr) {
                _plugin = allocator(this);
                shell_opt = u" -I";
            }
            break;
        }
        case PluginType::OUTPUT: {
            PluginRepository::OutputPluginFactory allocator = PluginRepository::Instance().getOutput(_name, *report);
            if (allocator != nullptr) {
                _plugin = allocator(this);
                shell_opt = u" -O";
            }
            break;
        }
        case PluginType::PROCESSOR: {
            PluginRepository::ProcessorPluginFactory allocator = PluginRepository::Instance().getProcessor(_name, *report);
            if (allocator != nullptr) {
                _plugin = allocator(this);
               shell_opt = u" -P";
            }
            break;
        }
        default:
            assert(false);
    }

    if (_plugin == nullptr || !_analyzeOptions(appName + shell_opt, options, report->maxSeverity())) {
        // Filtered parser diagnostics may not reach the owner's report. Mark
        // failure there even when its severity threshold suppresses output.
        // The UString overload records errors before applying that threshold.
        if (!report->gotErrors()) {
            report->log(Severity::Error, UString::Format(u"error initializing plugin %s", _name));
        }
        // The owner checks report errors and cleans up the partial chain.
        return;
    }

    // Get non-default thread stack size.
    size_t stackSize = 0;
    if (!GetEnvironment(u"TSPLUGINS_STACK_SIZE").toInteger(stackSize, UString::DEFAULT_THOUSANDS_SEPARATOR) || stackSize == 0) {
        // Use default value.
        stackSize = STACK_SIZE_OVERHEAD + _plugin->stackUsage();
    }

    // Define thread name and stack size.
    // Exit application when a thread terminates on an exception. This is required because
    // a dead plugin thread will block the processing chain and the application will hang.
    ThreadAttributes attr(attributes);
    attr.setName(_name);
    attr.setStackSize(stackSize);
    attr.setExitOnException(true);
    Thread::setAttributes(attr);
}


//----------------------------------------------------------------------------
// Configure and analyze the created plugin without exiting the calling process.
//----------------------------------------------------------------------------

bool ts::PluginThread::_analyzeOptions(const UString& shell, const PluginOptions& options, int max_severity)
{
    // Match the owner's command name and reporting severity before parsing.
    _plugin->setShell(shell);
    _plugin->setMaxSeverity(max_severity);
    // std::exit would bypass destruction of an asynchronous report and could
    // discard the diagnostic before its thread displays it. Temporarily return
    // argument errors to the owner, preserving all other flags and help behavior.
    const int flags = _plugin->getFlags();
    _plugin->setFlags(flags | Args::NO_EXIT_ON_ERROR);
    // Argument redirection was already processed at the command level.
    const bool valid = _plugin->analyze(options.name, options.args, false);
    _plugin->setFlags(flags); // Later use retains the plugin's original error policy.
    return valid;
}


//----------------------------------------------------------------------------
// Destructor
//----------------------------------------------------------------------------

ts::PluginThread::~PluginThread()
{
    // Deallocate plugin instance, if allocated.
    if (_plugin != nullptr) {
        delete _plugin;
        _plugin = nullptr;
    }
}


//----------------------------------------------------------------------------
// Implementation of TSP interface.
//----------------------------------------------------------------------------

ts::UString ts::PluginThread::pluginName() const
{
    return _name;
}

ts::Plugin* ts::PluginThread::plugin() const
{
    return _plugin;
}

//----------------------------------------------------------------------------
// Set the plugin name as displayed in log messages.
//----------------------------------------------------------------------------

void ts::PluginThread::setLogName(const UString& name)
{
    setReportPrefix((name.empty() ? _name : name) + u": ");
}
