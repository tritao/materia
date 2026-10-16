#ifndef MOTIONKIT_WINDOW_TIMING_STREAM_HPP
#define MOTIONKIT_WINDOW_TIMING_STREAM_HPP

#include "retained_window_timing.hpp"
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace motionkit {
// One worker's bounded pending timing queue. Existing windows are immutable.
// While open, future tails must admit rest at their entry: their controllable
// interval lies between {0} and [0, maximum_squared_speed]. This is the ordinary
// rest-feasible path model, not a promise for arbitrary minimum-speed processes.
// This class proves timing-profile stability and guards preview identity only.
// The compiler still owns task/clearance/drive/seam/event certification.
class WindowTimingStream {
    struct Revision {};
public:
    using Algorithm = RetainedWindowTiming::Algorithm;
    class Preview {
        friend class WindowTimingStream;
        std::shared_ptr<const Revision> revision_;
        std::vector<toppra::Vector> profiles_,controls_;
        double end_squared_;
        toppra::Bound successor_bounds_;
        Preview(std::shared_ptr<const Revision> revision, std::vector<toppra::Vector> profiles,
                std::vector<toppra::Vector> controls, const toppra::Bound& successor_bounds)
            : revision_(std::move(revision)), profiles_(std::move(profiles)),controls_(std::move(controls)),
              end_squared_(profiles_.back()[profiles_.back().size() - 1]),
              successor_bounds_(successor_bounds) {}
    public:
        const std::vector<toppra::Vector>& profiles() const { return profiles_; }
        const std::vector<toppra::Vector>& controls() const { return controls_; }
        double endSquaredSpeed() const { return end_squared_; }
        toppra::Bound successorSquaredBounds() const { return successor_bounds_; }
    };

    WindowTimingStream(double start_squared, std::size_t max_pending_windows, double maximum_squared_speed)
        : entry_squared_(start_squared), maximum_squared_(maximum_squared_speed), capacity_(max_pending_windows) {
        if (!std::isfinite(maximum_squared_) || maximum_squared_ <= 0 || capacity_ == 0 ||
                !std::isfinite(entry_squared_) || entry_squared_ < 0 || entry_squared_ > maximum_squared_)
            throw std::invalid_argument("Timing stream needs finite speed bounds and a positive queue budget");
    }
    WindowTimingStream(const WindowTimingStream&) = delete;
    WindowTimingStream& operator=(const WindowTimingStream&) = delete;
    WindowTimingStream(WindowTimingStream&&) = delete;
    WindowTimingStream& operator=(WindowTimingStream&&) = delete;

    std::size_t pendingWindows() const { return pending_ ? pending_->windowCount() : 0; }
    double entrySquaredSpeed() const { return entry_squared_; }
    bool finished() const { return closed_ && !pending_ && !cancelled_; }

    void append(std::unique_ptr<Algorithm> window) {
        requireLive();
        if (closed_) throw std::logic_error("Cannot append to a finished timing input");
        if (!window) throw std::invalid_argument("Timing stream needs an owned window");
        if (pendingWindows() == capacity_) throw std::length_error("Timing look-ahead window budget exhausted");
        // Enforce the domain used by the optimistic prefix proof on every window.
        window->setInitialXBounds(toppra::Bound{0, maximum_squared_});
        if (pending_) pending_->append(std::move(window));
        else {
            std::vector<std::unique_ptr<Algorithm>> first;
            first.push_back(std::move(window));
            pending_ = std::make_unique<RetainedWindowTiming>(std::move(first));
        }
        revision_.reset();
    }

    // All geometry is now known. An explicit endpoint update affects only the
    // pending suffix, whose entry is fixed by previously certified retirements.
    void finish(double end_squared = 0) {
        requireLive();
        if (!std::isfinite(end_squared) || end_squared < 0 || end_squared > maximum_squared_)
            throw std::invalid_argument("Timing stream terminal speed is outside its domain");
        if (!pending_ && end_squared != entry_squared_)
            throw std::invalid_argument("An empty timing stream cannot change boundary speed");
        terminal_squared_ = end_squared;
        closed_ = true;
        revision_.reset();
    }

    std::optional<Preview> preview() {
        requireLive();
        if (!pending_) return std::nullopt;
        // A fresh identity invalidates older candidates even if this solve fails.
        // Tokens retain only this tiny identity and their copied profile data.
        auto next = std::make_shared<const Revision>();
        revision_ = std::move(next);
        std::vector<toppra::Vector> safe,safe_controls;
        if (closed_) {
            if (!solve(toppra::Bound{terminal_squared_, terminal_squared_}, safe,safe_controls)) fail();
            return Preview(revision_, std::move(safe),std::move(safe_controls),pending_->boundarySquaredBounds(1));
        }
        const bool can_stop = solve(toppra::Bound{0,0}, safe,safe_controls);
        // Capture the rest-feasible suffix before the optimistic solve changes it.
        const toppra::Bound successor = can_stop ? pending_->boundarySquaredBounds(1) : toppra::Bound{0,0};
        std::vector<toppra::Vector> optimistic,optimistic_controls;
        if (!solve(toppra::Bound{0,maximum_squared_}, optimistic,optimistic_controls)) fail();
        // Compare the entire head profile, not just its entry interval. A
        // future rest-feasible terminal interval is nested between these two.
        if (!can_stop || (!identical(safe, optimistic) || !identical(safe_controls,optimistic_controls))) return std::nullopt;
        return Preview(revision_, std::move(safe),std::move(safe_controls),successor);
    }

    // Call only after the compiler has certified this candidate in context,
    // including shared boundary state, and has taken ownership for publication.
    // No allocation follows validation, and no old result data is mutated.
    void retireCertified(const Preview& preview) {
        retireCertified(preview, preview.end_squared_);
    }
    // The caller certifies the lowered prefix itself. Here we additionally
    // ensure its actual clock-boundary speed admits the captured safe suffix.
    void retireCertified(const Preview& preview, double actual_end_squared) {
        requireLive();
        if (!pending_ || !revision_ || preview.revision_ != revision_)
            throw std::logic_error("Timing preview is stale or belongs to another stream");
        if (!std::isfinite(actual_end_squared) ||
                actual_end_squared < preview.successor_bounds_[0] ||
                actual_end_squared > preview.successor_bounds_[1])
            throw std::invalid_argument("Actual boundary speed cannot enter the certified timing suffix");
        if (pending_->windowCount() > 1) pending_->retireFirst();
        else pending_.reset();
        entry_squared_ = actual_end_squared;
        revision_.reset();
    }

    void cancel() {
        revision_.reset();
        pending_.reset();
        cancelled_ = true;
    }

private:
    void requireLive() const {
        if (cancelled_) throw std::logic_error("Timing stream was cancelled");
    }
    bool solve(const toppra::Bound& terminal, std::vector<toppra::Vector>& profile,
            std::vector<toppra::Vector>& controls) {
        return pending_->backward(terminal) == toppra::ReturnCode::OK &&
            pending_->forwardPrefix(entry_squared_, 1, profile,&controls) == toppra::ReturnCode::OK;
    }
    [[noreturn]] void fail() const {
        throw std::runtime_error("Timing stream has no feasible pending prefix: " + pending_->error());
    }
    static bool identical(const std::vector<toppra::Vector>& a, const std::vector<toppra::Vector>& b) {
        if (a.size() != b.size()) return false;
        for (std::size_t i = 0; i < a.size(); ++i)
            if (a[i].size() != b[i].size() || !(a[i].array() == b[i].array()).all()) return false;
        return true;
    }
    double entry_squared_;
    double maximum_squared_;
    std::size_t capacity_;
    double terminal_squared_ = 0;
    bool closed_ = false;
    bool cancelled_ = false;
    std::unique_ptr<RetainedWindowTiming> pending_;
    std::shared_ptr<const Revision> revision_;
};
}
#endif
