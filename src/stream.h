// Fern-Airspy, an Airspy input module for FernSDR.
// SPDX-License-Identifier: GPL-2.0-or-later
//
// Moves samples from the vendor library's thread to fd 1. The library calls
// back with each block; the callback converts it, copies it into a ring and
// never blocks. A writer thread empties the ring into fd 1 with blocking
// writes. When the ring is full the newest samples are dropped and counted.
#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <thread>

namespace fern {

// Single producer, single consumer. The capacity is a power of two, so that
// the free-running positions can wrap around.
class RingBuffer {
public:
    explicit RingBuffer(size_t capacity);

    size_t capacity() const { return mask_ + 1; }
    size_t size() const;

    // Producer: stores all of data, or nothing when it does not fit.
    bool push(const uint8_t* data, size_t len);

    // Consumer: the longest contiguous readable span, then release part of it.
    size_t peek(const uint8_t** data) const;
    void consume(size_t len);

private:
    std::unique_ptr<uint8_t[]> buf_;
    size_t mask_;
    std::atomic<size_t> head_{0};  // written by the producer
    std::atomic<size_t> tail_{0};  // written by the consumer
};

class Stream;

// An opened Airspy that delivers samples to a stream.
class Source {
public:
    virtual ~Source() = default;
    // Starts delivering through stream.deliver(), on a thread of the vendor
    // library. 0, or the library's error, after which nothing runs.
    virtual int start(Stream& stream) = 0;
    // Stops the library's threads and waits for them: no deliver() runs
    // once it returns.
    virtual int stop() = 0;
    // False once the library saw USB fail.
    virtual bool streaming() = 0;
};

class Stream {
public:
    enum class WriterEnd { running, stopped, host_gone, failed };

    // notify_fd, an eventfd, is signalled when the writer ends. sample_bytes
    // is the size of one I/Q pair in the output format.
    Stream(Source& source, int samples_fd, int notify_fd, size_t ring_bytes, size_t sample_bytes);
    ~Stream();
    Stream(const Stream&) = delete;
    Stream& operator=(const Stream&) = delete;

    // Starts the writer and then the source. 0, or the source's error; a
    // negative errno-like -1 when a thread could not be created. Nothing is
    // left running after a failure.
    int start();
    // Stops taking samples and starts stopping the source, in a thread of
    // its own because the vendor library joins its threads; does not wait.
    void request_stop();
    // Keeps nudging the writer until it and the source's stop have finished
    // or the deadline passes. Returns whether both finished; only then may
    // the device be closed.
    bool wait(std::chrono::steady_clock::time_point deadline);

    // Called by the source, on the library's thread. lost counts the
    // samples the library itself lost before this block.
    void deliver(const void* data, size_t bytes, uint64_t lost);
    // Called by an R2's or Mini's source with what the converter saw: real
    // samples, those of them clipped, and the largest distance of one from
    // the midpoint, 0 to 2048.
    void note_levels(uint64_t real_samples, uint64_t clipped, unsigned peak);

    bool stop_requested() const { return stop_requested_.load(); }
    // The library stopped on its own: USB failed or the device went.
    bool source_failed();
    WriterEnd writer_end() const { return writer_end_.load(); }
    int writer_errno() const { return writer_errno_.load(); }

    uint64_t samples_received() const { return bytes_received_.load() / sample_bytes_; }
    uint64_t samples_delivered() const { return bytes_written_.load() / sample_bytes_; }
    uint64_t samples_dropped() const { return bytes_dropped_.load() / sample_bytes_ + lost_.load(); }
    uint64_t real_samples() const { return real_samples_.load(); }
    uint64_t samples_clipped() const { return samples_clipped_.load(); }
    // The largest distance from the midpoint since the last call, 0 to 2048.
    unsigned take_peak() { return peak_.exchange(0); }
    // steady_clock time of the last block, or of start() before one.
    std::chrono::steady_clock::time_point last_data() const;

private:
    void writer_main();
    void notify(int fd);

    Source& source_;
    const int samples_fd_;
    const int notify_fd_;
    const size_t sample_bytes_;
    RingBuffer ring_;
    int data_event_ = -1;

    std::thread writer_;
    std::thread stopper_;
    bool source_started_ = false;
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> source_stopped_{false};
    std::atomic<bool> writer_stop_{false};
    std::atomic<WriterEnd> writer_end_{WriterEnd::running};
    std::atomic<int> writer_errno_{0};

    std::atomic<uint64_t> bytes_received_{0};
    std::atomic<uint64_t> bytes_dropped_{0};
    std::atomic<uint64_t> bytes_written_{0};
    std::atomic<uint64_t> lost_{0};
    std::atomic<uint64_t> real_samples_{0};
    std::atomic<uint64_t> samples_clipped_{0};
    std::atomic<unsigned> peak_{0};
    std::atomic<int64_t> last_data_ns_{0};
};

}  // namespace fern
