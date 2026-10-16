#include "../src/retained_window_timing.hpp"
#include <toppra/constraint/linear_joint_acceleration.hpp>
#include <toppra/constraint/linear_joint_velocity.hpp>
#include <toppra/geometric_path/piecewise_poly_path.hpp>
#include <cassert>
#include <cmath>
#include <limits>

using motionkit::RetainedWindowTiming;
using toppra::ReturnCode;

static std::unique_ptr<RetainedWindowTiming::Algorithm> window(int i, double length,
        toppra::DiscretizationType scheme) {
    toppra::Matrix coefficient(2, 1);
    coefficient << 1, i * length;
    auto path = std::make_shared<toppra::PiecewisePolyPath>(toppra::Matrices{coefficient},
        std::vector<double>{i * length, (i + 1) * length});
    toppra::LinearConstraintPtrs constraints{
        std::make_shared<toppra::constraint::LinearJointVelocity>(
            -2 * toppra::Vector::Ones(1), 2 * toppra::Vector::Ones(1)),
        std::make_shared<toppra::constraint::LinearJointAcceleration>(
            -toppra::Vector::Ones(1), toppra::Vector::Ones(1))};
    for (auto& constraint : constraints) constraint->discretizationType(scheme);
    auto algorithm = std::make_unique<RetainedWindowTiming::Algorithm>(constraints, path);
    algorithm->setGridpoints(toppra::Vector::LinSpaced(51, i * length, (i + 1) * length));
    return algorithm;
}

static std::unique_ptr<RetainedWindowTiming> problem(double length,
        toppra::DiscretizationType scheme, int window_count = 3) {
    std::vector<std::unique_ptr<RetainedWindowTiming::Algorithm>> windows;
    for (int i = 0; i < window_count; ++i) windows.push_back(window(i, length, scheme));
    return std::make_unique<RetainedWindowTiming>(std::move(windows));
}

static void equivalent(const std::vector<toppra::Vector>& a, const std::vector<toppra::Vector>& b) {
    assert(a.size() == b.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        assert(a[i].size() == b[i].size());
        assert((a[i] - b[i]).cwiseAbs().maxCoeff() < 1e-12);
        if (i) assert(a[i][0] == a[i - 1][a[i - 1].size() - 1]);
    }
}

static void growing_windows(toppra::DiscretizationType scheme) {
    auto stream = problem(4, scheme, 1);
    assert(stream->backward(toppra::Bound{0,0}) == ReturnCode::OK);
    std::vector<toppra::Vector> old_preview;
    assert(stream->forwardPrefix(0, 1, old_preview) == ReturnCode::OK);
    stream->append(window(1, 4, scheme));
    assert(stream->windowCount() == 2);
    bool refused = false;
    try { std::vector<toppra::Vector> out; stream->forwardPrefix(0, 1, out); }
    catch (const std::logic_error&) { refused = true; }
    assert(refused);
    refused = false;
    try { stream->retireFirst(); } catch (const std::logic_error&) { refused = true; }
    assert(refused);
    assert(stream->backward(toppra::Bound{0,0}) == ReturnCode::OK);
    std::vector<toppra::Vector> first, all, reference;
    assert(stream->forwardPrefix(0, 1, first) == ReturnCode::OK);
    assert(old_preview[0][50] == 0 && first[0][50] > 0);
    auto fresh = problem(4, scheme, 2);
    assert(fresh->backward(toppra::Bound{0,0}) == ReturnCode::OK);
    assert(fresh->forward(0, reference) == ReturnCode::OK);
    assert(stream->forward(0, all) == ReturnCode::OK);
    equivalent(all, reference);
    equivalent(first, std::vector<toppra::Vector>{reference[0]});
    stream->append(window(2, 4, scheme));
    assert(stream->backward(toppra::Bound{0,0}) == ReturnCode::OK);
    std::vector<toppra::Vector> with_tail;
    assert(stream->forward(0, with_tail) == ReturnCode::OK);
    // A later saturated tail can leave this head profile unchanged, but
    // appending the earlier braking window changed it; previews are snapshots.
    equivalent(first, std::vector<toppra::Vector>{with_tail[0]});
    const double first_end = first[0][50];
    stream->retireFirst();
    assert(stream->windowCount() == 2);
    std::vector<toppra::Vector> second;
    assert(stream->forwardPrefix(first_end, 1, second) == ReturnCode::OK);
    equivalent(second, std::vector<toppra::Vector>{with_tail[1]});
    stream->retireFirst();
    assert(stream->windowCount() == 1);
    const double second_end = second[0][50];
    assert(stream->backward(toppra::Bound{0.25,0.25}) == ReturnCode::OK);
    std::vector<toppra::Vector> last;
    assert(stream->forward(second_end, last) == ReturnCode::OK);
    std::vector<std::unique_ptr<RetainedWindowTiming::Algorithm>> remaining;
    remaining.push_back(window(2, 4, scheme));
    RetainedWindowTiming fresh_last(std::move(remaining));
    assert(fresh_last.backward(toppra::Bound{0.25,0.25}) == ReturnCode::OK);
    assert(fresh_last.forward(second_end, reference) == ReturnCode::OK);
    equivalent(last, reference);
    refused = false;
    try { stream->retireFirst(); } catch (const std::logic_error&) { refused = true; }
    assert(refused && stream->windowCount() == 1);
    refused = false;
    try { stream->forwardPrefix(second_end, 0, last); }
    catch (const std::invalid_argument&) { refused = true; }
    assert(refused);
    equivalent(last, reference);
    assert(old_preview[0][50] == 0 && first[0][50] == first_end);

    auto rolling = problem(4, scheme, 2);
    assert(rolling->backward(toppra::Bound{0,0}) == ReturnCode::OK);
    std::vector<toppra::Vector> published;
    assert(rolling->forwardPrefix(0, 1, published) == ReturnCode::OK);
    const double published_end = published[0][50];
    rolling->retireFirst();
    rolling->append(window(2, 4, scheme));
    // A failed update after growth invalidates all reused state, including
    // the retained suffix that has shifted to the front after retirement.
    assert(rolling->backward(toppra::Bound{100,100}) != ReturnCode::OK);
    assert(rolling->backward(toppra::Bound{0,0}) == ReturnCode::OK);
    std::vector<toppra::Vector> suffix;
    assert(rolling->forward(published_end, suffix) == ReturnCode::OK);
    std::vector<std::unique_ptr<RetainedWindowTiming::Algorithm>> tail;
    tail.push_back(window(1, 4, scheme)); tail.push_back(window(2, 4, scheme));
    RetainedWindowTiming fresh_tail(std::move(tail));
    assert(fresh_tail.backward(toppra::Bound{0,0}) == ReturnCode::OK);
    assert(fresh_tail.forward(published_end, reference) == ReturnCode::OK);
    equivalent(suffix, reference);
    assert(published[0][50] == published_end);
}

int main() {
    for (auto scheme : {toppra::DiscretizationType::Collocation, toppra::DiscretizationType::Interpolation}) {
        growing_windows(scheme);
        // Equal entry intervals permit backward-cache reuse; they do not
        // certify that a previously previewed full window is unchanged.
        auto preview = problem(4.0, scheme, 1);
        assert(preview->backward(toppra::Bound{0,0}) == ReturnCode::OK);
        const auto stopped_entry = preview->entrySquaredBounds();
        std::vector<toppra::Vector> stopped, moving;
        assert(preview->forward(0, stopped) == ReturnCode::OK);
        assert(preview->backward(toppra::Bound{1,1}) == ReturnCode::OK);
        const auto moving_entry = preview->entrySquaredBounds();
        assert((stopped_entry.array() == moving_entry.array()).all());
        assert(preview->forward(0, moving) == ReturnCode::OK);
        assert(stopped[0][50] == 0 && std::abs(moving[0][50] - 1) < 1e-12);
        assert((stopped[0] - moving[0]).cwiseAbs().maxCoeff() > 0.5);
        for (double length : {0.05, 4.0}) {
            auto retained = problem(length, scheme);
            std::vector<toppra::Vector> old_result;
            for (const toppra::Bound terminal : {toppra::Bound{0, 0}, toppra::Bound{0, 1},
                    toppra::Bound{0.04, 0.09}, toppra::Bound{0, 1}, toppra::Bound{0, 1}, toppra::Bound{0, 0}}) {
                auto fresh = problem(length, scheme);
                assert(retained->backward(terminal) == ReturnCode::OK);
                assert(fresh->backward(terminal) == ReturnCode::OK);
                std::vector<toppra::Vector> updated, reference;
                assert(retained->forward(0, updated) == ReturnCode::OK);
                assert(fresh->forward(0, reference) == ReturnCode::OK);
                equivalent(updated, reference);
                if (!old_result.empty()) assert(old_result.back()[50] == 0);
                if (old_result.empty()) old_result = updated;
                // Infeasible entry must not replace an existing published result.
                auto preserved = updated;
                assert(retained->forward(100, updated) != ReturnCode::OK);
                equivalent(updated, preserved);
            }
            assert(retained->backward(toppra::Bound{100, 100}) != ReturnCode::OK);
            bool rejected = false;
            try { std::vector<toppra::Vector> out; retained->forward(0, out); }
            catch (const std::logic_error&) { rejected = true; }
            assert(rejected);
            assert(retained->backward(toppra::Bound{0, 0}) == ReturnCode::OK);
            auto fresh = problem(length, scheme);
            assert(fresh->backward(toppra::Bound{0, 0}) == ReturnCode::OK);
            std::vector<toppra::Vector> restored, reference;
            assert(retained->forward(0, restored) == ReturnCode::OK);
            assert(fresh->forward(0, reference) == ReturnCode::OK);
            equivalent(restored, reference);
            retained.reset();
            equivalent(old_result, reference);
        }
    }
}
