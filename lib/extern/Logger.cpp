#include "Logger.hpp"

static constexpr bool kIsDebugBuild =
#ifndef RELEASE
    true;
#else
    false;
#endif

sas::Logger *sas::BaseLogger::getLogger(std::string_view loggerName) noexcept
{
    auto it = loggers.find(std::string(loggerName));
    if (it != loggers.end())
    {
        return it->second.get();
    }

    mainLogger.warn("The requested logger does not exist. Returning a fallback");
    return &mainLogger;
}

sas::Logger *sas::BaseLogger::addLogger(std::string_view loggerName, const LoggerSettings &settings) noexcept
{

    std::string nameStr{loggerName};

    auto [it, inserted] = loggers.try_emplace(nameStr, std::make_unique<Logger>(loggerName, settings));
    if (!inserted)
    {
        mainLogger.warn("Logger with this name already exists");
    }

    return it->second.get();
}


#define ANSI_COLOR_RESET "\033[0m"
#define ANSI_COLOR_YELLOW "\033[1;33m"
#define ANSI_COLOR_RED "\033[1;31m"

sas::Logger::Logger(std::string_view loggerName, const LoggerSettings &optionalSettings) noexcept
    : name(loggerName),
      settings(optionalSettings),
      logPreMessage(name + " Log: "),
      warningPreMessage(name + ANSI_COLOR_YELLOW + " Warning! " + ANSI_COLOR_RESET),
      errorPreMessage(name + ANSI_COLOR_RED + " !!!ERROR!!!" + ANSI_COLOR_RESET)
{
}

static std::string getFormattedTimestamp() noexcept
{
    auto now = std::chrono::system_clock::now();

    return std::format("[{:%H:%M:%S}] ", std::chrono::floor<std::chrono::milliseconds>(now));
}


void sas::Logger::log(std::string_view message) noexcept
{
    if constexpr (kIsDebugBuild)
    {
        if (settings.inactive || settings.settings.ingoreLogs)
        {
            return;
        }

        const std::string preMessageTransform = settings.settings.timeStamp ? getFormattedTimestamp() + logPreMessage : logPreMessage;

        for (auto logerSink : settings.loggerSinks)
        {
            logerSink->startLogging(preMessageTransform, message, postMessage);
        }
    }
}

void sas::Logger::warn(std::string_view message) noexcept
{
    if constexpr (kIsDebugBuild)
    {
        if (settings.inactive || settings.settings.ingoreWarns)
        {
            return;
        }

        const std::string preMessageTransform = settings.settings.timeStamp ? getFormattedTimestamp() + logPreMessage : warningPreMessage;


        for (auto logerSink : settings.loggerSinks)
        {
            logerSink->startLogging(preMessageTransform, message, postMessage);
        }
    }
}

void sas::Logger::error(std::string_view message) noexcept
{
    if constexpr (kIsDebugBuild)
    {
        if (settings.inactive || settings.settings.ignoreErrors)
        {
            return;
        }
        const std::string preMessageTransform = settings.settings.timeStamp ? getFormattedTimestamp() + logPreMessage : errorPreMessage;

        for (auto logerSink : settings.loggerSinks)
        {
            logerSink->startLogging(preMessageTransform, message, postMessage);
        }
    }
}

void sas::LoggerSettingsInternal::applySettings(const LoggerSettings &newSettings) noexcept
{
    inactive = false;
    settings = newSettings;
    loggerSinks.clear();

    if (settings.outFile)
    {
        if (settings.filePaths.empty())
        {
            std::cerr << "Logger configured with a file but no files provided\n";
        }

        for (const auto &path : settings.filePaths)
        {
            auto fileLoggerSink = std::make_shared<FileSink>(path);

            if (!fileLoggerSink->isFileOpen())
            {
                std::cerr << "Logger file not found\n";
            }
            else
            {
                loggerSinks.push_back(fileLoggerSink);
            }
        }

        if (loggerSinks.empty())
        {
            std::cerr << "Warning! All logger file paths are empty\n";
        }
    }

    if (settings.strOut)
    {
        loggerSinks.push_back(std::make_shared<StdoutSink>());
    }

    if (settings.ignoreAll || (settings.ingoreLogs && settings.ingoreWarns && settings.ignoreErrors) || (!settings.strOut && !settings.outFile) || (!settings.strOut && loggerSinks.empty()))
    {
        std::cout << "inactive is on\n";
        inactive = true;
    }
}

void sas::ILogSink::ringBuffers() noexcept
{
    using namespace std::chrono_literals;

    while (true)
    {
        std::unique_lock lock(mtx);

        consumer.wait_for(lock, 250ms, [this]
                          { return flushFull; });

        if (!flushFull && front.size > 0)
        {
            std::swap(front, back);
            flushFull = true;
        }

        if (flushFull)
        {
            lock.unlock();

            if (back.size > 0)
            {
                write(back.data.data(), back.size);
                flush();
                back.reset();
            }

            lock.lock();
            flushFull = false;
            producer.notify_one();
        }

        if (stopRequested && front.size == 0 && back.size == 0)
        {
            break;
        }
    }
}

void sas::ILogSink::startLogging(std::string_view preMessage, std::string_view message, std::string_view endMessage) noexcept
{
    std::unique_lock lock(mtx);
    const size_t messageSize = preMessage.size() + message.size() + endMessage.size();
    
    if(messageSize > bufferSize)
    {
        std::cerr << "Warning! Log bigger than max capacity\n";
        return;
    }

    if (!front.hasSpace(messageSize))
    {

        producer.wait(lock, [this]
                      { return !flushFull; });

        std::swap(front, back);
        flushFull = true;

        consumer.notify_one();
    }
    front.addMessage(preMessage, message, endMessage);
}
