#include "merge_cases.hpp"
int main() {
    auto merge = [](const std::vector<mk_segment> &segments, const motionkit::MergeOptions &options, bool position_budget = false) {
        motionkit::Trajectory trajectory(1);
        for (const auto &s : segments) assert(trajectory.append(s));
        motionkit::PolynomialSource source{&trajectory, trajectory.segment_count(), 1,
            [](const void *context, uint32_t i) {
                const auto &t = *static_cast<const motionkit::Trajectory *>(context);
                return motionkit::PolynomialSegmentView{t.segment_start(i), t.segment_duration(i), t.segment_degree(i),
                    t.joint_count(), t.segment_coefficients(i), sizeof(mk_joint_coefficients)};
            }};
        std::vector<mk_segment> out;
        auto function = position_budget ? motionkit::merge_polynomials_position_budget : motionkit::merge_polynomials;
        assert(function(source, options, {&out,
            [](void *context, const mk_segment &s, uint32_t, uint32_t) -> mk_result {
                static_cast<std::vector<mk_segment> *>(context)->push_back(s); return MK_OK;
            }}) == MK_OK);
        return out;
    };
    merge_cases([&](const auto &segments, const auto &options) { return merge(segments, options); });
    // Short candidates spend the position budget over their own duration, rather
    // than inheriting unnecessarily tight derivative errors from a long window.
    std::vector<mk_segment> source(2);
    for(unsigned i=0;i<2;++i){
        auto &s=source[i];
        s.struct_size=sizeof(s);s.joint_count=1;s.degree=2;
        s.t0_ns=i*1000000;s.duration_ns=1000000;
        s.coefficients[0].value[0]=i ? 0.5e-6 : 0;
        s.coefficients[0].value[1]=i ? 0.001 : 0;
        s.coefficients[0].value[2]=i ? -0.5 : 0.5;
    }
    const double p[]={1e-6},v[]={2e-5},a[]={0.0008};
    const double vl[]={1},al[]={10},lo[]={-1},hi[]={1};
    motionkit::MergeOptions options{p,v,a,vl,al,lo,hi,0.1};
    assert(merge(source,options).size()==2);
    options.velocity_tolerance=options.acceleration_tolerance=nullptr;
    auto out=merge(source,options,true);
    assert(out.size()==1);
    for(unsigned i=0;i<=2000;++i){
        double t=i*1e-6;
        unsigned piece=t<0.001 ? 0 : 1;
        mk_trajectory_state actual{},reference{};
        motionkit::evaluate_segment(out[0],t,actual);
        motionkit::evaluate_segment(source[piece],t-piece*0.001,reference);
        assert(std::abs(actual.position[0]-reference.position[0])<=1e-6);
        assert(std::abs(actual.velocity[0]-reference.velocity[0])<=0.001);
        assert(std::abs(actual.acceleration[0]-reference.acceleration[0])<=2);
        assert(std::abs(actual.velocity[0])<=1 && std::abs(actual.acceleration[0])<=10);
    }
    options.max_duration_seconds=0.001;
    assert(merge(source,options,true).size()==2);
    const double tight[]={1e-12};options.position_tolerance=tight;options.max_duration_seconds=0.1;
    assert(merge(source,options,true).size()==2);
}
