#include "pool.hpp"

namespace stockkit {

#if defined(__EMSCRIPTEN__) && !defined(__EMSCRIPTEN_PTHREADS__)
constexpr bool kThreads = false;
#else
constexpr bool kThreads = true;
#endif

Pool::~Pool() { stop(); }

unsigned Pool::hardware() {
    if (!kThreads) return 1;
    unsigned count = std::thread::hardware_concurrency();
    return count == 0 ? 1 : count;
}

void Pool::stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
    }
    start_.notify_all();
    for (std::thread &worker : workers_) worker.join();
    workers_.clear();
    stopping_ = false;
}

void Pool::resize(unsigned threads) {
    if (!kThreads || threads == 0) threads = 1;
    if (threads == size()) return;
    stop();
    for (unsigned k = 1; k < threads; ++k) workers_.emplace_back([this, k] { work(k); });
}

void Pool::run(unsigned jobs, const std::function<void(unsigned)> &job) {
    if (jobs == 0) return;
    if (jobs > size()) jobs = size();
    if (jobs == 1) {
        job(0);
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        job_ = &job;
        jobs_ = jobs;
        pending_ = jobs - 1;
        error_ = nullptr;
        ++round_;
    }
    start_.notify_all();
    std::exception_ptr own;
    try {
        job(0);
    } catch (...) {
        own = std::current_exception();
    }
    std::unique_lock<std::mutex> lock(mutex_);
    done_.wait(lock, [this] { return pending_ == 0; });
    job_ = nullptr;
    if (own) std::rethrow_exception(own);
    if (error_) std::rethrow_exception(error_);
}

void Pool::work(unsigned index) {
    unsigned long long seen = 0;
    for (;;) {
        const std::function<void(unsigned)> *job;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            start_.wait(lock, [&] { return stopping_ || round_ != seen; });
            if (stopping_) return;
            seen = round_;
            if (index >= jobs_) continue; // not needed this round
            job = job_;
        }
        std::exception_ptr error;
        try {
            (*job)(index);
        } catch (...) {
            error = std::current_exception();
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (error && !error_) error_ = error;
            if (--pending_ == 0) done_.notify_one();
        }
    }
}

} // namespace stockkit
