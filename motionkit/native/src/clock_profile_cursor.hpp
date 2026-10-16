#ifndef MOTIONKIT_CLOCK_PROFILE_CURSOR_HPP
#define MOTIONKIT_CLOCK_PROFILE_CURSOR_HPP

#include "clock_velocity_profile.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace motionkit {
struct ClockProfileStage {
    int64_t start_tick, duration;
    double position, velocity, acceleration;
};
// A solver window is a numerical partition, not a physical clock anchor.
// Preserve the fractional tail when publishing at floor(nominal end time).
// Copy this small cursor for a preview; install it only after certification.
class ClockProfileCursor {
public:
    explicit ClockProfileCursor(double position, double velocity)
        : nominal_position_(position), nominal_velocity_(velocity),
          clock_position_(position), clock_velocity_(velocity) {
        if (!std::isfinite(position) || !std::isfinite(velocity) || velocity<0)
            throw std::invalid_argument("Clock cursor needs finite position and nonnegative speed");
    }

    int64_t tick() const { return tick_; }
    double position() const { return static_cast<double>(clock_position_); }
    double velocity() const { return static_cast<double>(clock_velocity_); }

    // A moving final endpoint uses a bounded envelope to preserve its authored
    // speed and position. Supplied bounds construct a candidate only; lowering
    // must still validate the joint trajectory continuously.
    std::vector<ClockProfileStage> append(const std::vector<double>& positions,
            const std::vector<double>& velocities, bool final,
            double final_velocity_bound=0, double final_acceleration_bound=0,
            const std::vector<double>* reference_controls=nullptr) {
        if (positions.size()<2 || positions.size()!=velocities.size() ||
                (reference_controls && reference_controls->size()+1!=positions.size()) ||
                positions.front()!=nominal_position_ || velocities.front()!=nominal_velocity_ ||
                (final && velocities.back()!=0 && (!(final_velocity_bound>0) || !(final_acceleration_bound>0))))
            throw std::invalid_argument("Clock cursor needs contiguous reference windows and compatible final bounds");
        const long double begin=static_cast<long double>(tick_)*1e-9L;
        std::vector<long double> times{begin};
        std::vector<long double> controls=tail_controls_;
        times.insert(times.end(),tail_times_.begin(),tail_times_.end());
        long double end=nominal_time_;
        for (size_t i=1;i<positions.size();++i) {
            const long double distance=static_cast<long double>(positions[i])-positions[i-1];
            const long double sum=static_cast<long double>(velocities[i])+velocities[i-1];
            if (!(distance>0) || !(sum>0) || !std::isfinite(distance) || !std::isfinite(sum) || velocities[i]<0 || velocities[i-1]<0)
                throw std::invalid_argument("Clock cursor needs finite increasing travel and nonnegative speed");
            const long double duration=2*distance/sum;
            const long double control=reference_controls ? (*reference_controls)[i-1] :
                (static_cast<long double>(velocities[i])-velocities[i-1])/duration;
            if (!std::isfinite(control)) throw std::invalid_argument("Clock control must be finite");
            controls.push_back(control);
            end+=duration;
            times.push_back(end);
        }
        if (!(end*1e9L<INT64_MAX/2)) throw std::length_error("Clock duration is out of range");
        const int64_t finish=static_cast<int64_t>(final ? std::ceil(end*1e9L) : std::floor(end*1e9L));
        if (finish<=tick_) throw std::length_error("Window contains no publishable clock tick");
        if (final && end<finish*1e-9L) { times.push_back(finish*1e-9L); controls.push_back(0); }
        std::vector<int64_t> knots{tick_};
        // The carried solver interface and final reference endpoint are not
        // clock anchors. Rounding either again can manufacture a one-tick stage.
        const size_t first=tail_times_.size()+1;
        const size_t after=times.size()-(final && times.back()>end ? 2 : 1);
        for (size_t i=first;i<after;++i) {
            const int64_t knot=std::llround(times[i]*1e9L);
            if (knot>knots.back() && knot<finish) knots.push_back(knot);
        }
        knots.push_back(finish);
        // Carry the reference controls themselves. Reconstructing a carried
        // sub-tick tail's slope from nearly equal endpoint speeds amplifies
        // their rounding even when wider intermediates are used.
        size_t interval=0;
        std::vector<long double> clock_speeds{clock_velocity_},clock_controls;
        clock_speeds.reserve(knots.size());clock_controls.reserve(knots.size()-1);
        for (size_t i=1;i<knots.size();++i) {
            long double cursor=knots[i-1]*1e-9L;
            const long double right=knots[i]*1e-9L,dt=(knots[i]-knots[i-1])*1e-9L;
            long double change=0;
            while (cursor<right) {
                while (interval+2<times.size() && times[interval+1]<=cursor) ++interval;
                const long double next=std::min(right,times[interval+1]);
                if (!(next>cursor)) throw std::runtime_error("Clock reference support is incomplete");
                change+=controls[interval]*(next-cursor);cursor=next;
            }
            long double acceleration=change/dt;
            long double next=clock_speeds.back()+change;
            if (next<0 || (final && velocities.back()==0 && i+1==knots.size())) {
                next=0;acceleration=-clock_speeds.back()/dt;
            }
            clock_controls.push_back(acceleration);clock_speeds.push_back(next);
        }
        if (final && velocities.back()>0) {
            std::vector<double> corrected;
            const std::vector<double> candidate(clock_speeds.begin(),clock_speeds.end());
            if (!clockVelocityProfile(knots,candidate,positions.back()-static_cast<double>(clock_position_),
                    clock_velocity_,velocities.back(),velocities.back(),final_velocity_bound,final_acceleration_bound,corrected))
                throw std::runtime_error("Moving final clock boundary is infeasible");
            clock_speeds.assign(corrected.begin(),corrected.end());
            for (size_t i=1;i<knots.size();++i)
                clock_controls[i-1]=(clock_speeds[i]-clock_speeds[i-1])/((knots[i]-knots[i-1])*1e-9L);
        }
        std::vector<ClockProfileStage> result;
        long double position=clock_position_;
        long double velocity=clock_velocity_;
        for (size_t i=1;i<knots.size();++i) {
            const long double next=clock_speeds[i];
            const long double dt=(knots[i]-knots[i-1])*1e-9L;
            result.push_back({knots[i-1]-tick_,knots[i]-knots[i-1],static_cast<double>(position),
                static_cast<double>(velocity),static_cast<double>(clock_controls[i-1])});
            position+=(static_cast<long double>(velocity)+next)*dt/2;
            velocity=next;
        }
        std::vector<long double> tail_times;
        std::vector<long double> tail_controls;
        const long double clock_end=finish*1e-9L;
        for (size_t i=1;i<times.size();++i) if (times[i]>clock_end) {
            tail_times.push_back(times[i]);tail_controls.push_back(controls[i-1]);
        }
        // Atomic update after all allocations and validation have completed.
        tail_times_.swap(tail_times);tail_controls_.swap(tail_controls);
        nominal_time_=end; nominal_position_=positions.back(); nominal_velocity_=velocities.back();
        tick_=finish; clock_position_=position; clock_velocity_=velocity;
        return result;
    }
private:
    long double nominal_time_=0;
    double nominal_position_, nominal_velocity_;
    int64_t tick_=0;
    long double clock_position_;
    long double clock_velocity_;
    std::vector<long double> tail_times_;
    std::vector<long double> tail_controls_;
};
}
#endif
