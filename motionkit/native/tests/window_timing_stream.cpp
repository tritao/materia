#include "../src/window_timing_stream.hpp"
#include "../src/clock_velocity_profile.hpp"
#include "../src/clock_profile_cursor.hpp"
#include <toppra/constraint/linear_joint_acceleration.hpp>
#include <toppra/constraint/linear_joint_velocity.hpp>
#include <toppra/geometric_path/piecewise_poly_path.hpp>
#include <cassert>
#include <limits>

using motionkit::WindowTimingStream;
using motionkit::RetainedWindowTiming;

static std::unique_ptr<RetainedWindowTiming::Algorithm> window(int i, toppra::DiscretizationType scheme, double velocity = 2) {
    toppra::Matrix coefficients(2,1);
    coefficients << 1, 4*i;
    auto path = std::make_shared<toppra::PiecewisePolyPath>(toppra::Matrices{coefficients},
        std::vector<double>{4.0*i,4.0*(i+1)});
    toppra::LinearConstraintPtrs constraints{
        std::make_shared<toppra::constraint::LinearJointVelocity>(-velocity*toppra::Vector::Ones(1),velocity*toppra::Vector::Ones(1)),
        std::make_shared<toppra::constraint::LinearJointAcceleration>(-toppra::Vector::Ones(1),toppra::Vector::Ones(1))};
    for (auto& constraint : constraints) constraint->discretizationType(scheme);
    auto result = std::make_unique<RetainedWindowTiming::Algorithm>(constraints,path);
    result->setGridpoints(toppra::Vector::LinSpaced(51,4*i,4*(i+1)));
    return result;
}
static void equivalent(const toppra::Vector& a, const toppra::Vector& b) {
    assert(a.size()==b.size() && (a-b).cwiseAbs().maxCoeff()<1e-12);
}
template<class Error, class Action> static void refuses(Action action) {
    bool failed=false;
    try { action(); } catch (const Error&) { failed=true; }
    assert(failed);
}
static void clockProfiles() {
    // Dense knots alternate around their ideal times. Keeping raw speeds on
    // this clock violates acceleration; the envelope must recover feasible travel.
    std::vector<int64_t> ticks;
    std::vector<double> reference;
    for (int i=0; i<=10000; ++i) {
        ticks.push_back(100000LL*i + (i>0 && i<10000 ? (i%2 ? 1 : -1) : 0));
        reference.push_back(i/10000.0);
    }
    ticks.back()+=2; // A representable horizon above the continuous optimum.
    auto verify=[&](double distance,double start,double lo,double hi,const std::vector<double>& raw) {
        std::vector<double> result;
        assert(motionkit::clockVelocityProfile(ticks,raw,distance,start,lo,hi,2,1,result));
        assert(result.front()==start && result.back()>=lo && result.back()<=hi);
        long double area=0;
        for (std::size_t i=1;i<result.size();++i) {
            const long double dt=(ticks[i]-ticks[i-1])*1e-9L;
            assert(result[i]>=0 && result[i]<=2);
            assert(std::abs(result[i]-result[i-1])<=static_cast<double>(dt)+2e-16);
            area+=(result[i]+result[i-1])*dt/2;
        }
        assert(std::abs(area-distance)<2e-16L);
    };
    verify(0.5,0,0,2,reference);
    std::reverse(reference.begin(),reference.end());
    verify(0.500000001,1,0,0,reference);
    verify(1,1,1,1,std::vector<double>(ticks.size(),1.5));
    // A physically saturated braking interface need not be realizable on an
    // integer clock. T=1-sqrt(0.5) is strictly between these two ticks: the
    // shorter horizon cannot reach the endpoint speed; the longer one cannot
    // cover only 0.25 m under the acceleration bound. This requires upstream
    // boundary adaptation, not repeated local horizon extension.
    const double end=std::sqrt(0.5);
    std::vector<double> candidate;
    for (int64_t duration : {292893218LL,292893219LL})
        assert(!motionkit::clockVelocityProfile({0,duration},{1,end},0.25,1,end,end,2,1,candidate));
    assert(motionkit::clockVelocityProfile({0,146455188,292910377},
        {0.9999,(0.9999+end)/2,end},0.25,0.9999,end,end,2,1,candidate));
    std::vector<double> unchanged{42};
    assert(!motionkit::clockVelocityProfile(ticks,reference,3,0,0,0,2,1,unchanged));
    assert(unchanged==std::vector<double>{42});
    assert(!motionkit::clockVelocityProfile({0,0},{0,0},1,0,0,0,2,1,unchanged));
}
static void clockCursor() {
    // A triangular profile split at arbitrary geometry distances, including
    // saturated braking windows which cannot be independently rounded.
    motionkit::ClockProfileCursor cursor(0,0);
    double last_position=0,last_velocity=0;
    int64_t elapsed=0;
    for (int window=0;window<40;++window) {
        std::vector<double> positions,velocities;
        for (int i=0;i<=100;++i) {
            const double s=(window*100+i)/1000.0;
            positions.push_back(s);
            velocities.push_back(std::sqrt(2*std::min(s,4-s)));
        }
        const auto before=cursor;
        const auto stages=cursor.append(positions,velocities,window==39);
        assert(stages.front().position==last_position && stages.front().velocity==last_velocity);
        int64_t duration=0;
        for (const auto& stage:stages) {
            assert(stage.start_tick==duration && stage.duration>0);
            assert(std::abs(stage.acceleration)<=1+1e-10);
            duration+=stage.duration;
        }
        elapsed+=duration;
        assert(cursor.tick()==elapsed && cursor.tick()>before.tick());
        last_position=cursor.position(); last_velocity=cursor.velocity();
        const double time=elapsed*1e-9;
        const double expected=time<=2 ? time*time/2 : 4-(4-time)*(4-time)/2;
        assert(std::abs(last_position-expected)<1e-10);
    }
    assert(last_velocity==0 && std::abs(last_position-4)<1e-10);
    const auto finished=cursor;
    refuses<std::invalid_argument>([&]{cursor.append({4,5},{0,1},true);});
    assert(cursor.tick()==finished.tick() && cursor.position()==finished.position() && cursor.velocity()==finished.velocity());
    refuses<std::invalid_argument>([]{motionkit::ClockProfileCursor invalid(0,-1);});
    // Discarding a speculative cursor must not advance the worker's clock.
    motionkit::ClockProfileCursor owner(0,0);
    auto speculative=owner;
    speculative.append({0,0.1},{0,std::sqrt(0.2)},false);
    assert(owner.tick()==0 && owner.position()==0 && owner.velocity()==0);

}
int main() {
    clockCursor();
    clockProfiles();
    refuses<std::invalid_argument>([]{WindowTimingStream invalid(0,0,4);});
    refuses<std::invalid_argument>([]{WindowTimingStream invalid(-1,2,4);});
    refuses<std::invalid_argument>([]{WindowTimingStream invalid(5,2,4);});

    for (auto scheme : {toppra::DiscretizationType::Collocation,toppra::DiscretizationType::Interpolation}) {
        WindowTimingStream stream(0,3,4);
        assert(!stream.preview());
        stream.append(window(0,scheme));
        assert(!stream.preview()); // Its braking tail still depends on future geometry.
        stream.append(window(1,scheme));
        auto old=stream.preview();
        assert(old && old->profiles().size()==1 && old->endSquaredSpeed()>0);
        auto current=stream.preview();
        assert(current);
        refuses<std::logic_error>([&]{stream.retireCertified(*old);});
        assert(stream.pendingWindows()==2 && stream.entrySquaredSpeed()==0);

        WindowTimingStream foreign(0,1,4);
        foreign.append(window(0,scheme)); foreign.finish();
        auto foreign_preview=foreign.preview();
        assert(foreign_preview);
        refuses<std::logic_error>([&]{stream.retireCertified(*foreign_preview);});
        stream.append(window(2,scheme));
        refuses<std::logic_error>([&]{stream.retireCertified(*current);});
        current=stream.preview(); assert(current);

        std::vector<std::unique_ptr<RetainedWindowTiming::Algorithm>> windows;
        for(int i=0;i<3;++i) windows.push_back(window(i,scheme));
        RetainedWindowTiming fresh(std::move(windows));
        assert(fresh.backward(toppra::Bound{0,0})==toppra::ReturnCode::OK);
        std::vector<toppra::Vector> reference;
        assert(fresh.forward(0,reference)==toppra::ReturnCode::OK);
        equivalent(current->profiles()[0],reference[0]);
        equivalent(old->profiles()[0],reference[0]);
        stream.retireCertified(*current);
        assert(stream.pendingWindows()==2 && stream.entrySquaredSpeed()==current->endSquaredSpeed());
        refuses<std::logic_error>([&]{stream.retireCertified(*current);});
        stream.finish();
        auto middle=stream.preview(); assert(middle);
        equivalent(middle->profiles()[0],reference[1]);
        stream.retireCertified(*middle);
        auto last=stream.preview(); assert(last);
        equivalent(last->profiles()[0],reference[2]);
        stream.retireCertified(*last);
        assert(stream.finished() && stream.pendingWindows()==0 && stream.entrySquaredSpeed()==0);
        refuses<std::logic_error>([&]{stream.append(window(3,scheme));});
        equivalent(old->profiles()[0],reference[0]);

        WindowTimingStream budget(0,1,4);
        budget.append(window(0,scheme)); assert(!budget.preview());
        refuses<std::length_error>([&]{budget.append(window(1,scheme));});
        assert(budget.pendingWindows()==1);
        budget.finish();
        auto bounded=budget.preview(); assert(bounded);
        budget.retireCertified(*bounded); assert(budget.finished());

        WindowTimingStream cancelled(0,2,4);
        cancelled.append(window(0,scheme)); cancelled.append(window(1,scheme));
        auto snapshot=cancelled.preview(); assert(snapshot);
        cancelled.cancel(); cancelled.cancel();
        assert(cancelled.pendingWindows()==0 && !cancelled.finished());
        refuses<std::logic_error>([&]{cancelled.retireCertified(*snapshot);});
        refuses<std::logic_error>([&]{cancelled.preview();});
        equivalent(snapshot->profiles()[0],reference[0]);

        WindowTimingStream retarget(0,2,4);
        retarget.append(window(0,scheme)); retarget.append(window(1,scheme));
        auto fixed_head=retarget.preview(); assert(fixed_head);
        retarget.retireCertified(*fixed_head);
        retarget.finish(1);
        auto changed_tail=retarget.preview(); assert(changed_tail);
        std::vector<std::unique_ptr<RetainedWindowTiming::Algorithm>> tail_window;
        tail_window.push_back(window(1,scheme));
        RetainedWindowTiming fresh_tail(std::move(tail_window));
        assert(fresh_tail.backward(toppra::Bound{1,1})==toppra::ReturnCode::OK);
        std::vector<toppra::Vector> tail_reference;
        assert(fresh_tail.forward(fixed_head->endSquaredSpeed(),tail_reference)==toppra::ReturnCode::OK);
        equivalent(changed_tail->profiles()[0],tail_reference[0]);
        equivalent(fixed_head->profiles()[0],reference[0]);
        retarget.retireCertified(*changed_tail); assert(retarget.finished());

        WindowTimingStream failed_update(0,2,4);
        failed_update.append(window(0,scheme,0.2)); failed_update.append(window(1,scheme,0.2));
        auto before_failure=failed_update.preview(); assert(before_failure);
        failed_update.finish(4); // The actual motor limit excludes this endpoint.
        refuses<std::runtime_error>([&]{failed_update.preview();});
        refuses<std::logic_error>([&]{failed_update.retireCertified(*before_failure);});
        failed_update.finish(0);
        auto recovered=failed_update.preview(); assert(recovered);
        equivalent(recovered->profiles()[0],before_failure->profiles()[0]);

        // Clock lowering may reach a slower interface. The next solve must use
        // that actual state, while rejecting invalid feedback atomically.
        WindowTimingStream feedback(0,2,4);
        feedback.append(window(0,scheme)); feedback.append(window(1,scheme));
        auto candidate=feedback.preview(); assert(candidate);
        const auto bounds=candidate->successorSquaredBounds();
        assert(bounds[0]==0 && bounds[1]>=candidate->endSquaredSpeed());
        for (double invalid : {-1.0, 5.0, std::numeric_limits<double>::infinity(),
                std::numeric_limits<double>::quiet_NaN()}) {
            refuses<std::invalid_argument>([&]{feedback.retireCertified(*candidate,invalid);});
            assert(feedback.pendingWindows()==2 && feedback.entrySquaredSpeed()==0);
        }
        const double reached=candidate->endSquaredSpeed()*0.999;
        feedback.retireCertified(*candidate,reached);
        assert(feedback.entrySquaredSpeed()==reached);
        feedback.finish();
        auto suffix=feedback.preview(); assert(suffix);
        assert(suffix->profiles()[0][0]==reached);
        std::vector<std::unique_ptr<RetainedWindowTiming::Algorithm>> feedback_windows;
        feedback_windows.push_back(window(1,scheme));
        RetainedWindowTiming feedback_fresh(std::move(feedback_windows));
        assert(feedback_fresh.backward(toppra::Bound{0,0})==toppra::ReturnCode::OK);
        std::vector<toppra::Vector> feedback_reference;
        assert(feedback_fresh.forward(reached,feedback_reference)==toppra::ReturnCode::OK);
        equivalent(suffix->profiles()[0],feedback_reference[0]);
        refuses<std::invalid_argument>([&]{feedback.retireCertified(*suffix,0.001);});
        feedback.retireCertified(*suffix,0); assert(feedback.finished());

        WindowTimingStream moving(0,1,4);
        moving.append(window(0,scheme)); moving.finish(0.25);
        auto move=moving.preview(); assert(move && std::abs(move->endSquaredSpeed()-0.25)<1e-12);
        assert(move->successorSquaredBounds()[0]==0.25 && move->successorSquaredBounds()[1]==0.25);
        refuses<std::invalid_argument>([&]{moving.retireCertified(*move,0);});
        assert(moving.pendingWindows()==1 && moving.entrySquaredSpeed()==0);
        moving.retireCertified(*move); assert(moving.finished());
        refuses<std::invalid_argument>([&]{moving.finish(0);});
        assert(moving.finished());
    }
}
