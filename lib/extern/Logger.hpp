#pragma once

#include <iostream>
#include <unordered_map>
#include <array>
#include <chrono>
#include <condition_variable>
#include <utility>
#include <thread>
#include <csignal>
#include <cstring>
#include <fstream>

namespace sas
{

    static constexpr size_t bufferSize = 64 * 1024;

    class ILogSink;

    struct LoggerBuffer
    {
        std::array<char, bufferSize> data;

        size_t size = 0;

        bool hasSpace(size_t bytes) const noexcept
        {
            return (bufferSize - size) >= bytes;
        }

        void addMessage(std::string_view preMessage, std::string_view message, std::string_view postMessage) noexcept
        {
            const std::size_t messageSize = preMessage.size() + message.size() + postMessage.size();

            auto dest = data.begin() + size;

            std::memcpy(dest, preMessage.data(), preMessage.size());
            dest += preMessage.size();

            std::memcpy(dest, message.data(), message.size());
            dest += message.size();

            std::memcpy(dest, postMessage.data(), postMessage.size());

            size += messageSize;
        }

        void reset() noexcept
        {
            size = 0;
        }
    };

    class ILogSink
    {
    protected:
        LoggerBuffer front;
        LoggerBuffer back;

        std::thread flushThread;
        std::mutex mtx;
        std::condition_variable producer;
        std::condition_variable consumer;
        bool flushFull = false;
        bool stopRequested = false;

        void ringBuffers() noexcept;

        void stopLogging() noexcept
        {
            if (flushThread.joinable())
            {
                {
                    std::lock_guard lock(mtx);
                    stopRequested = true;
                }
                consumer.notify_all();
                flushThread.join();
            }
        }

    public:
        ILogSink() noexcept
            : flushThread(&ILogSink::ringBuffers, this)
        {
        }

        void startLogging(std::string_view preMessage, std::string_view message, std::string_view endMessage) noexcept;

        virtual void write(const char *data, size_t size) noexcept = 0;
        virtual void flush() noexcept = 0;

        ILogSink(const ILogSink &log) noexcept = delete;
        ILogSink(ILogSink &&log) noexcept = delete;

        ILogSink &operator=(const ILogSink &) noexcept = delete;
        ILogSink &operator=(ILogSink &&) noexcept = delete;

        virtual ~ILogSink() noexcept = default;

        void crashFlush() noexcept
        {
            if (front.size > 0)
            {
#if defined(_WIN32)
                HANDLE hStderr = GetStdHandle(STD_ERROR_HANDLE);
                DWORD written;
                WriteFile(hStderr, front.data.data(), static_cast<DWORD>(front.size), &written, NULL);
#else
                ::write(STDERR_FILENO, front.data.data(), front.size);
#endif
                front.size = 0;
            }
        }
    };

    class StdoutSink : public ILogSink
    {
    public:
        void write(const char *data, size_t size) noexcept override
        {
            std::cout.write(data, size);
        }

        void flush() noexcept override
        {
            std::cout.flush();
        }

        ~StdoutSink() noexcept
        {
            stopLogging();
        }
    };

    class FileSink : public ILogSink
    {
    private:
        std::ofstream file;

    public:
        explicit FileSink(const std::string &filename)
            : file(filename, std::ios::out | std::ios::app | std::ios::binary)
        {
        }

        [[nodiscard]] bool isFileOpen() const noexcept
        {
            return file.is_open();
        }

        void write(const char *data, size_t size) noexcept override
        {
            if (file.is_open())
            {
                file.write(data, size);
            }
        }

        void flush() noexcept override
        {
            if (file.is_open())
            {
                file.flush();
            }
        }

        ~FileSink() override
        {
            stopLogging();
        }
    };

    struct LoggerSettings
    {
        bool ingoreLogs = false;
        bool ingoreWarns = false;
        bool ignoreErrors = false;
        bool ignoreAll = false;
        bool timeStamp = true;

        bool strOut = true;
        bool outFile = false;
        std::vector<std::string> filePaths;



        [[nodiscard]] LoggerSettings &setOutFile(bool enable) noexcept
        {
            outFile = enable;
            return *this;
        }
        [[nodiscard]] LoggerSettings &setFilePaths(std::vector<std::string> paths) noexcept
        {
            filePaths = std::move(paths);
            return *this;
        }
        [[nodiscard]] LoggerSettings &setIgnoreErrors(bool ignore) noexcept
        {
            ignoreErrors = ignore;
            return *this;
        }

        [[nodiscard]] LoggerSettings &setIgnoreAll(bool enable) noexcept
        {
            ignoreAll = enable;
            return *this;
        }
        [[nodiscard]] LoggerSettings &setStandardOut(bool enable) noexcept
        {
            strOut = enable;
            return *this;
        }
        [[nodiscard]] LoggerSettings &setIgnoreLogs(bool ignore) noexcept
        {
            ingoreLogs = ignore;
            return *this;
        }

        [[nodiscard]] LoggerSettings &setIgnoreWarns(bool enable) noexcept
        {
            ingoreWarns = enable;
            return *this;
        }

        [[nodiscard]] LoggerSettings& setTimeStamp(bool enable) noexcept
        {
            timeStamp = enable;
            return *this;
        }
    };

    struct LoggerSettingsInternal
    {
        LoggerSettings settings;

        bool inactive = false;
        std::vector<std::shared_ptr<ILogSink>> loggerSinks;

        void applySettings(const LoggerSettings &newSettings) noexcept;

        LoggerSettingsInternal(const LoggerSettings &newSettings) noexcept
        {
            applySettings(newSettings);
        }
    };

    class Logger
    {
    private:
        std::string name;
        LoggerSettingsInternal settings;
        std::string logPreMessage;
        std::string warningPreMessage;
        std::string errorPreMessage;
        std::string_view postMessage = "\n";

    public:
        Logger(std::string_view loggerName, const LoggerSettings &optionalSettings = {}) noexcept;

        void log(std::string_view message) noexcept;
        void warn(std::string_view message) noexcept;        
        void error(std::string_view message) noexcept;

        void changeSettings(const LoggerSettings &newSettings) noexcept
        {
            settings.applySettings(newSettings);
        }
    };

    class BaseLogger
    {
    private:
        static inline Logger mainLogger{"mainLogger"};
        static inline std::unordered_map<std::string, std::unique_ptr<Logger>> loggers;

    public:

        static void log(std::string_view message) noexcept
        {
            mainLogger.log(message);
        }
        static void warn(std::string_view message) noexcept
        {
            mainLogger.warn(message);
        }
        static void error(std::string_view message) noexcept
        {
            mainLogger.error(message);
        }

        [[nodiscard]] static Logger *MainLogger() noexcept
        {
            return &mainLogger;
        }

        [[nodiscard]] static Logger *getLogger(std::string_view loggerName) noexcept;

        static Logger *addLogger(std::string_view loggerName, const LoggerSettings &settings = {}) noexcept;
    };
} //namespace sas