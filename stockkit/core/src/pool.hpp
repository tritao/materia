#pragma once

#include <condition_variable>
#include <exception>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace stockkit {

/**
 * A fixed set of worker threads for fork-join work: `run(n, job)` calls
 * job(0) .. job(n - 1) once each, job 0 on the calling thread, and returns
 * when all are done. The first exception a job throws is rethrown by `run`
 * after every job has finished.
 */
class Pool {
public:
    Pool() = default;
    Pool(const Pool &) = delete;
    Pool &operator=(const Pool &) = delete;
    ~Pool();

    /** Threads available to `run`, including the caller's. */
    unsigned size() const { return static_cast<unsigned>(workers_.size()) + 1; }
    /** Starts or stops workers so `size()` is `threads` (at least 1). */
    void resize(unsigned threads);
    void run(unsigned jobs, const std::function<void(unsigned)> &job);

    /** Threads the platform can run at once; 1 where threads are unavailable. */
    static unsigned hardware();

private:
    void stop();
    void work(unsigned index);

    std::vector<std::thread> workers_;
    std::mutex mutex_;
    std::condition_variable start_, done_;
    const std::function<void(unsigned)> *job_ = nullptr;
    unsigned jobs_ = 0;
    unsigned pending_ = 0;
    unsigned long long round_ = 0;
    bool stopping_ = false;
    std::exception_ptr error_;
};

} // namespace stockkit
