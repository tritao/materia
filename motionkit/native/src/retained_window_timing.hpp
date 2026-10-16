#ifndef MOTIONKIT_RETAINED_WINDOW_TIMING_HPP
#define MOTIONKIT_RETAINED_WINDOW_TIMING_HPP

#include "planning_profile.hpp"
#include <toppra/algorithm/toppra.hpp>
#include <toppra/solver/seidel.hpp>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace motionkit {
// Worker-owned, immutable problems. No algorithm/constraint may be shared with
// another caller. Existing windows stay immutable; queue growth only appends
// new owned problems, and retirement only removes an externally certified prefix.
// Each window retains compact backward state; the owner keeps one reusable solver
// table, leased to at most one algorithm at a time.
class RetainedWindowTiming {
public:
    using Algorithm = toppra::algorithm::TOPPRA;
    explicit RetainedWindowTiming(std::vector<std::unique_ptr<Algorithm>> windows)
        : windows_(std::move(windows)), intervals_(windows_.size() + 1),
          workspace_(std::make_shared<toppra::solver::Seidel>()) {
        if (windows_.empty()) throw std::invalid_argument("Timing needs a window");
        for (auto& window : windows_) {
            if (!window) throw std::invalid_argument("Timing needs owned window problems");
            window->releaseSolverWorkspace();
        }
    }
    RetainedWindowTiming(const RetainedWindowTiming&) = delete;
    RetainedWindowTiming& operator=(const RetainedWindowTiming&) = delete;

    std::size_t windowCount() const { return windows_.size(); }

    // Old windows retain their backward data. No preview is current until the
    // changed terminal suffix has been propagated through a successful backward pass.
    void append(std::unique_ptr<Algorithm> window) {
        if (!window) throw std::invalid_argument("Timing needs an owned window problem");
        window->releaseSolverWorkspace();
        intervals_.emplace_back();
        try { windows_.push_back(std::move(window)); }
        catch (...) { intervals_.pop_back(); throw; }
        valid_ = false;
    }

    // The caller certifies/publishes the first result and retains its reached
    // boundary state. This method only retires solver state; it is not certification.
    // Dispose the owner after the final window rather than keeping an empty queue.
    void retireFirst() {
        if (!valid_) throw std::logic_error("Retirement needs current backward state");
        if (windows_.size() < 2) throw std::logic_error("Dispose timing after its final window");
        if (leased_ == windows_.front().get()) releaseWorkspace();
        windows_.erase(windows_.begin());
        intervals_.erase(intervals_.begin());
        --cached_windows_;
    }

    toppra::ReturnCode backward(const toppra::Bound& terminal) {
        PathProfilePhase profile("stream_toppra_backward");
        const std::size_t reusable = cached_windows_;
        cached_windows_ = 0;
        valid_ = false;
        if (!terminal.allFinite() || terminal[0] < 0 || terminal[0] > terminal[1])
            throw std::invalid_argument("Terminal squared speeds must be finite, nonnegative and ordered");
        error_.clear();
        intervals_.back() = terminal;
        for (std::size_t i = windows_.size(); i-- > 0;) {
            auto& algorithm = windows_[i];
            leaseWorkspace(algorithm.get());
            const bool reuse = i < reusable;
            toppra::ReturnCode status;
            try {
                if (reuse) status = algorithm->updateControllableSetsFromSquaredSpeed(intervals_[i + 1], workspace_);
                else {
                    algorithm->solver(workspace_);
                    status = algorithm->computeControllableSetsFromSquaredSpeed(intervals_[i + 1]);
                }
            } catch (...) { releaseWorkspace(); throw; }
            if (status != toppra::ReturnCode::OK) {
                error_ = algorithm->getErrorMessage();
                return status;
            }
            const toppra::Bound updated = algorithm->getParameterizationData().controllable_sets.row(0);
            const bool unchanged = reuse && (updated.array() == intervals_[i].array()).all();
            intervals_[i] = updated;
            if (unchanged) break;
        }
        cached_windows_ = windows_.size();
        valid_ = true;
        return toppra::ReturnCode::OK;
    }

    // Publish copied profiles atomically. Older results never alias mutable solver data.
    toppra::ReturnCode forward(double start_squared, std::vector<toppra::Vector>& result) {
        return forwardPrefix(start_squared, windows_.size(), result);
    }

    // Preview only a bounded pending prefix; returned data is owned and does
    // not alias a future backward update or retirement. Publication remains external.
    toppra::ReturnCode forwardPrefix(double start_squared, std::size_t count,
            std::vector<toppra::Vector>& result, std::vector<toppra::Vector>* controls = nullptr) {
        PathProfilePhase profile("stream_toppra_forward");
        if (!valid_) throw std::logic_error("Timing needs a successful backward pass");
        if (!std::isfinite(start_squared) || start_squared < 0)
            throw std::invalid_argument("Initial squared speed must be finite and nonnegative");
        if (count == 0 || count > windows_.size()) throw std::invalid_argument("Invalid timing prefix size");
        std::vector<toppra::Vector> profiles,forward_controls;
        profiles.reserve(count);if (controls) forward_controls.reserve(count);
        for (std::size_t i = 0; i < count; ++i) {
            auto& algorithm = windows_[i];
            leaseWorkspace(algorithm.get());
            toppra::ReturnCode status;
            try { status = algorithm->computeForwardFromControllableSets(start_squared, workspace_); }
            catch (...) { releaseWorkspace(); throw; }
            if (status != toppra::ReturnCode::OK) {
                error_ = algorithm->getErrorMessage();
                return status;
            }
            profiles.push_back(algorithm->getParameterizationData().parametrization);
            if (controls) forward_controls.push_back(algorithm->getParameterizationData().parametrization_controls);
            start_squared = profiles.back()[profiles.back().size() - 1];
        }
        result.swap(profiles);if (controls) controls->swap(forward_controls);
        return toppra::ReturnCode::OK;
    }
    toppra::Bound boundarySquaredBounds(std::size_t boundary) const {
        if (!valid_) throw std::logic_error("Timing needs a successful backward pass");
        if (boundary >= intervals_.size()) throw std::out_of_range("Invalid timing boundary");
        return intervals_[boundary];
    }
    toppra::Bound entrySquaredBounds() const { return boundarySquaredBounds(0); }
    const std::string& error() const { return error_; }

private:
    // Keep the immutable head problem initialized across adjacent backward /
    // forward previews. Switching windows still releases the single workspace;
    // the fork's generation check protects any subsequent reinitialization.
    void releaseWorkspace() {
        if (leased_) leased_->releaseSolverWorkspace();
        leased_ = nullptr;
    }
    void leaseWorkspace(Algorithm* algorithm) {
        if (leased_ != algorithm) { releaseWorkspace();leased_ = algorithm; }
    }
    Algorithm* leased_ = nullptr;
    std::vector<std::unique_ptr<Algorithm>> windows_;
    std::vector<toppra::Bound> intervals_;
    std::shared_ptr<toppra::solver::Seidel> workspace_;
    bool valid_ = false;
    std::size_t cached_windows_ = 0;
    std::string error_;
};
}
#endif
