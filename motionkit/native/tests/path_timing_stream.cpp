#include "motionkit.h"
#include <cassert>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

struct Result {
    mk_path_timing_preview_handle preview{};
    mk_time_law_handle law{};
    mk_trajectory_handle trajectory{};
    ~Result() { mk_path_timing_preview_destroy(preview); mk_time_law_destroy(law); mk_trajectory_destroy(trajectory); }
};
static mk_path_handle line(int spans, double length) {
    std::vector<mk_path_sample> samples(spans+1);
    for(int i=0;i<=spans;++i) {
        auto& p=samples[i]; p.struct_size=sizeof(p); p.joint_count=1;
        p.s=p.position[0]=length*i/spans; p.first[0]=1;
    }
    mk_path_handle result{};
    assert(mk_path_create(samples.data(),samples.size(),&result)==MK_OK);
    return result;
}
static void run(int spans, double length) {
    auto path=line(spans,length);
    const double velocity=2,acceleration=1;
    mk_path_timing_stream_handle stream{};
    assert(mk_path_timing_stream_create(path,&velocity,&acceleration,1,nullptr,0,
        0,0,1e-6,0,32,&stream)==MK_OK);
    mk_path_destroy(path);
    double distance=0,total_time=0,last_speed=0;
    uint32_t windows=0,total=0;
    for(int step=0;step<100;++step) {
        Result result;
        mk_path_timing_stream_progress progress{};
        mk_timing_report report{};
        const auto status=mk_path_timing_stream_step(stream,&progress,&result.preview,&report,&result.law,&result.trajectory);
        if(status!=MK_OK) {
            assert(!result.preview.id && !result.law.id && !result.trajectory.id);
            std::fprintf(stderr,"stream failed status=%d spans=%d window=%u derivative=%u value=%.17g limit=%.17g\n",
                status,spans,windows,report.derivative,report.value,report.limit);
        }
        assert(status==MK_OK);
        if(progress.state==MK_TIMING_STREAM_WAITING) {
            assert(!result.preview.id && !result.law.id && !result.trajectory.id); continue;
        }
        if(progress.state==MK_TIMING_STREAM_DONE) {
            assert(!result.preview.id && !result.law.id && !result.trajectory.id); break;
        }
        assert(progress.state==MK_TIMING_STREAM_READY && result.preview.id && result.law.id && result.trajectory.id);
        assert(progress.window_index==windows && progress.start_s==distance && progress.span_count>0);
        total=progress.total_windows;
        int64_t duration=0;
        assert(mk_trajectory_duration_ns(result.trajectory,&duration)==MK_OK);
        mk_trajectory_state a{},b{}; a.struct_size=sizeof(a); b.struct_size=sizeof(b);
        assert(mk_trajectory_evaluate(result.trajectory,0,&a)==MK_OK);
        assert(mk_trajectory_evaluate(result.trajectory,duration,&b)==MK_OK);
        assert(std::abs(a.position[0]-progress.start_s)<1e-7);
        assert(std::abs(b.position[0]-progress.end_s)<1e-7);
        assert(std::abs(a.velocity[0]-last_speed)<1e-8);
        assert(std::abs(b.velocity[0]-progress.end_speed)<1e-8);
        double begin=-1,end=-1;
        assert(mk_path_distance_to_time(result.law,progress.start_s,&begin)==MK_OK && begin==0);
        assert(mk_path_distance_to_time(result.law,progress.end_s,&end)==MK_OK);
        assert(std::abs(end-duration*1e-9)<2e-9);
        assert(mk_path_timing_stream_commit(stream,result.preview)==MK_OK);
        assert(mk_path_timing_stream_commit(stream,result.preview)==MK_ERROR_INVALID_ARGUMENT);
        distance=progress.end_s; last_speed=progress.end_speed; total_time+=duration*1e-9; ++windows;
    }
    assert(windows==total && windows>0 && std::abs(distance-length)<1e-10 && last_speed==0);
    if(spans>1000) assert(windows>1);
    mk_path_timing_stream_destroy(stream);
    path=line(spans,length);
    Result reference;
    assert(mk_time_path(path,&velocity,&acceleration,1,nullptr,0,0,0,1e-6,0,nullptr,&reference.law,&reference.trajectory)==MK_OK);
    int64_t duration=0;
    assert(mk_trajectory_duration_ns(reference.trajectory,&duration)==MK_OK);
    // The legacy formatter stretches stopped paths after clock-induced
    // overshoot. A carried clock should instead attain the analytic triangular
    // profile without that fallback; raw solver equivalence is tested separately.
    assert(total_time<=duration*1e-9+2e-9);
    assert(std::abs(total_time-2*std::sqrt(length))<1e-8);
    mk_path_destroy(path);
}
static void ownership() {
    auto path=line(2,1);
    double velocity=2,acceleration=1;
    mk_path_timing_stream_handle a{},b{};
    assert(mk_path_timing_stream_create(path,&velocity,&acceleration,1,nullptr,0,0,0,1e-6,0,2,&a)==MK_OK);
    assert(mk_path_timing_stream_create(path,&velocity,&acceleration,1,nullptr,0,0,0,1e-6,0,2,&b)==MK_OK);
    velocity=acceleration=0.001; mk_path_destroy(path);
    Result first,foreign,current;
    mk_path_timing_stream_progress progress{};
    assert(mk_path_timing_stream_step(a,&progress,&first.preview,nullptr,&first.law,&first.trajectory)==MK_OK);
    assert(progress.state==MK_TIMING_STREAM_READY);
    assert(mk_path_timing_stream_step(b,&progress,&foreign.preview,nullptr,&foreign.law,&foreign.trajectory)==MK_OK);
    assert(mk_path_timing_stream_commit(a,foreign.preview)==MK_ERROR_INVALID_ARGUMENT);
    assert(mk_path_timing_stream_step(a,&progress,&current.preview,nullptr,&current.law,&current.trajectory)==MK_OK);
    assert(mk_path_timing_stream_commit(a,first.preview)==MK_ERROR_INVALID_ARGUMENT);
    assert(mk_path_timing_stream_commit(a,current.preview)==MK_OK);
    mk_path_timing_stream_destroy(a); mk_path_timing_stream_destroy(b);
    assert(mk_path_timing_stream_commit(a,current.preview)==MK_ERROR_INVALID_HANDLE);
    uint32_t count=0;
    assert(mk_time_law_binding_count(first.law,&count)==MK_OK && count>0);
    mk_timing_binding binding{}; binding.struct_size=sizeof(binding);
    assert(mk_time_law_get_binding(first.law,count-1,&binding)==MK_OK);
}
static void movingEndpoint(double start) {
    const double length=1.000000000123;
    auto path=line(4,length);
    const double velocity=start>0 ? 0.5/0.98 : 2,acceleration=1;
    mk_path_timing_stream_handle stream{};
    assert(mk_path_timing_stream_create(path,&velocity,&acceleration,1,nullptr,0,start,0.5,1e-6,0,4,&stream)==MK_OK);
    mk_path_destroy(path);
    Result result;
    mk_path_timing_stream_progress progress{};
    assert(mk_path_timing_stream_step(stream,&progress,&result.preview,nullptr,&result.law,&result.trajectory)==MK_OK);
    assert(progress.state==MK_TIMING_STREAM_READY && progress.end_speed==0.5);
    assert(std::abs(progress.end_s-length)<1e-10);
    assert(mk_path_timing_stream_commit(stream,result.preview)==MK_OK);
    mk_path_timing_stream_destroy(stream);
}
static void curved(double endSpeed) {
    std::vector<mk_path_sample> samples(8001);
    for (size_t i=0;i<samples.size();++i) {
        auto& p=samples[i]; p.struct_size=sizeof(p); p.joint_count=1;
        p.s=static_cast<double>(i)/8000; p.position[0]=p.s+0.1*p.s*p.s;
        p.first[0]=1+0.2*p.s; p.second[0]=p.second_before[0]=0.2;
    }
    mk_path_handle path{};
    assert(mk_path_create(samples.data(),samples.size(),&path)==MK_OK);
    const double velocity=2,acceleration=1;
    mk_path_timing_stream_handle stream{};
    assert(mk_path_timing_stream_create(path,&velocity,&acceleration,1,nullptr,0,0,endSpeed,1e-6,0,32,&stream)==MK_OK);
    mk_path_destroy(path);
    double position=0,speed=0;
    int windows=0;
    for (int step=0;step<100;++step) {
        Result result; mk_path_timing_stream_progress progress{}; mk_timing_report report{};
        const auto status=mk_path_timing_stream_step(stream,&progress,&result.preview,&report,&result.law,&result.trajectory);
        if (status!=MK_OK) std::fprintf(stderr,"curved status=%d window=%d derivative=%u value=%.17g limit=%.17g\n",
            status,windows,report.derivative,report.value,report.limit);
        assert(status==MK_OK);
        if (progress.state==MK_TIMING_STREAM_WAITING) continue;
        if (progress.state==MK_TIMING_STREAM_DONE) break;
        int64_t duration=0;
        assert(mk_trajectory_duration_ns(result.trajectory,&duration)==MK_OK);
        mk_trajectory_state a{},b{}; a.struct_size=sizeof(a); b.struct_size=sizeof(b);
        assert(mk_trajectory_evaluate(result.trajectory,0,&a)==MK_OK);
        assert(mk_trajectory_evaluate(result.trajectory,duration,&b)==MK_OK);
        assert(std::abs(a.position[0]-position)<1e-9 && std::abs(a.velocity[0]-speed)<1e-8);
        assert(std::abs(b.position[0]-(progress.end_s+0.1*progress.end_s*progress.end_s))<1e-8);
        assert(std::abs(b.velocity[0]-(1+0.2*progress.end_s)*progress.end_speed)<1e-8);
        position=b.position[0]; speed=b.velocity[0]; ++windows;
        assert(mk_path_timing_stream_commit(stream,result.preview)==MK_OK);
    }
    if (windows<2) std::fprintf(stderr,"curved windows=%d position=%.17g speed=%.17g\n",windows,position,speed);
    assert(windows>1 && std::abs(position-1.1)<1e-8 && std::abs(speed-1.2*endSpeed)<1e-8);
    mk_path_timing_stream_destroy(stream);
}
static void advanceContext() {
    mk_trajectory_handle source{},full{};
    assert(mk_trajectory_create(1,&source)==MK_OK);
    mk_segment segment{}; segment.struct_size=sizeof(segment); segment.joint_count=1;
    segment.duration_ns=2'000'000'000; segment.degree=5;
    segment.coefficients[0].value[3]=1.25; segment.coefficients[0].value[4]=-0.9375;
    segment.coefficients[0].value[5]=0.1875;
    assert(mk_trajectory_append_segment(source,&segment)==MK_OK);
    assert(mk_advance_joint(source,0,0.03,0.04,&full)==MK_OK);
    for (int range=0;range<3;++range) {
        const int64_t begin=range==0 ? 0 : 500'000'000;
        const int64_t end=range==2 ? 2'000'000'000 : 1'500'000'000;
        mk_trajectory_handle input{},advanced{};
        assert(mk_trajectory_slice(source,begin,end,&input)==MK_OK);
        assert(mk_advance_joint_context(input,0,0.03,0.04,begin,range==2,&advanced)==MK_OK);
        const int64_t first=range==0 ? 0 : 20'000'000;
        const int64_t after=end-begin-(range==2 ? 0 : 20'000'000);
        for (int64_t time=first;time<=after;time+=1'000'000) {
            mk_trajectory_state a{},b{}; a.struct_size=sizeof(a); b.struct_size=sizeof(b);
            assert(mk_trajectory_evaluate(full,begin+time,&a)==MK_OK);
            assert(mk_trajectory_evaluate(advanced,time,&b)==MK_OK);
            assert(std::abs(a.position[0]-b.position[0])<1e-10);
            assert(std::abs(a.velocity[0]-b.velocity[0])<1e-8);
            assert(std::abs(a.acceleration[0]-b.acceleration[0])<1e-5);
        }
        mk_trajectory_destroy(input); mk_trajectory_destroy(advanced);
    }
    mk_trajectory_handle rejected{123};
    assert(mk_advance_joint_context(source,0,0.03,0.04,-1,0,&rejected)==MK_ERROR_INVALID_ARGUMENT && rejected.id==0);
    rejected.id=123;
    assert(mk_advance_joint_context(source,0,0.03,0.04,0,2,&rejected)==MK_ERROR_INVALID_ARGUMENT && rejected.id==0);
    mk_trajectory_destroy(full); mk_trajectory_destroy(source);
}
// Captured from the cube's 1.267664675 mm stopped travel. The generic
// collocation result exceeded Y acceleration by 2.1857e-6 m/s^2.
static void affineCubeRegression() {
    constexpr unsigned joints=18;
    const double q[2][joints]={{0.0091643803931303405,-2.8033410894803898,-2.8033410894803898,-2.8033410894803898,0.075733922295673092,0.075733922295673092,0.075733922295673092,-1.3638035835923588,1.3638035835923588,-1.3638035835923588,1.3638035835923588,-0.0086822432694065917,0,0,-0.0097999999999999997,-7.6969020012949922,-7.6969020012949922,-7.6969020012949922},{0.0097750000000000059,-3.0709068188840263,-3.0709068188840263,-3.0709068188840263,0,0,0,-1.5354534094420125,1.5354534094420125,-1.5354534094420125,1.5354534094420125,-0.0097750000000000059,0,0,-0.0096000000000000009,-7.5398223686155035,-7.5398223686155035,-7.5398223686155035}};
    const double rate[joints]={0.48168846301614426,-211.06979995353586,-211.06979995353586,-211.06979995353586,-59.742906280488086,-59.742906280488086,-59.742906280488086,-135.40635311701197,135.40635311701197,-135.40635311701197,135.40635311701197,-0.86202361698476504,0,0,0.15777042928777157,123.91260540104277,123.91260540104277,123.91260540104277};
    const double velocity[joints]={0.50000001080835477,157.07963607503444,78.539818037517222,157.07963607503444,157.07963607503444,78.539818037517222,157.07963607503444,78.539818037517222,78.539818037517222,78.539818037517222,78.539818037517222,0.50000001080835477,26.179939345839074,78.539818037517222,0.10000000216167096,78.539818037517222,78.539818037517222,78.539818037517222};
    const double acceleration[joints]={12.542575411039081,5754.8621251168342,5754.8621251168342,5754.8621251168342,5754.8621251168342,5754.8621251168342,5754.8621251168342,3784.6789866960162,3784.6789866960162,3784.6789866960162,3784.6789866960162,24.094014749947863,9407.2153443972802,28221.646033191842,18.059129597335712,14183.607218303967,14183.607218303967,14183.607218303967};
    const double length=0.0012676646752047694, cap=0.14999999999999999;
    mk_path_sample samples[2]{};
    for (unsigned i=0;i<2;++i) {
        samples[i].struct_size=sizeof(mk_path_sample);samples[i].joint_count=joints;samples[i].s=i*length;
        for (unsigned joint=0;joint<joints;++joint) { samples[i].position[joint]=q[i][joint];samples[i].first[joint]=rate[joint]; }
    }
    mk_path_handle path{};
    assert(mk_path_create(samples,2,&path)==MK_OK);
    mk_path_timing_stream_handle a{},b{};
    assert(mk_path_timing_stream_create(path,velocity,acceleration,joints,&cap,1,0,0,1e-6,0,1,&a)==MK_OK);
    assert(mk_path_timing_stream_create(path,velocity,acceleration,joints,&cap,1,0,0,1e-6,0,1,&b)==MK_OK);
    mk_path_destroy(path);
    Result stale,foreign,current;
    mk_path_timing_stream_progress progress{};
    assert(mk_path_timing_stream_step(a,&progress,&stale.preview,nullptr,&stale.law,&stale.trajectory)==MK_OK);
    assert(mk_path_timing_stream_step(b,&progress,&foreign.preview,nullptr,&foreign.law,&foreign.trajectory)==MK_OK);
    assert(mk_path_timing_stream_step(a,&progress,&current.preview,nullptr,&current.law,&current.trajectory)==MK_OK);
    assert(progress.state==MK_TIMING_STREAM_READY && progress.total_windows==1);
    assert(mk_path_timing_stream_commit(a,foreign.preview)==MK_ERROR_INVALID_ARGUMENT);
    assert(mk_path_timing_stream_commit(a,stale.preview)==MK_ERROR_INVALID_ARGUMENT);
    assert(mk_path_timing_stream_commit(a,current.preview)==MK_OK);
    mk_path_timing_stream_destroy(a);mk_path_timing_stream_destroy(b);
    int64_t duration=0;uint32_t segments=0;
    assert(mk_trajectory_duration_ns(current.trajectory,&duration)==MK_OK);
    assert(mk_trajectory_segment_count(current.trajectory,&segments)==MK_OK && segments==3);
    long double A=1e100L,V=cap;
    for (unsigned joint=0;joint<joints;++joint) {
        const long double delta=std::abs(static_cast<long double>(q[1][joint])-q[0][joint]);
        if (delta!=0) { A=std::min(A,acceleration[joint]*length/delta);V=std::min(V,velocity[joint]*length/delta); }
    }
    const long double ideal=V*V<A*length ? length/V+V/A : 2*std::sqrt(length/A);
    assert(duration*1e-9L>=ideal-1e-12L && duration*1e-9L<=ideal+3e-9L);
    mk_limits limits{};limits.struct_size=sizeof(limits);limits.joint_count=joints;
    for (unsigned joint=0;joint<joints;++joint) {
        limits.derivative_claimed[joint]=3;limits.max_velocity[joint]=velocity[joint];limits.max_acceleration[joint]=acceleration[joint];
    }
    mk_validation_report report{};report.struct_size=sizeof(report);
    assert(mk_validate(current.trajectory,&limits,&report)==MK_OK);
    assert(report.checks[MK_CHECK_ACCELERATION].status==MK_CHECK_PASSED);
    assert(report.checks[MK_CHECK_VELOCITY].status==MK_CHECK_PASSED);
    for (unsigned i=0;i<=100;++i) {
        mk_trajectory_state state{};state.struct_size=sizeof(state);
        assert(mk_trajectory_evaluate(current.trajectory,duration*i/100,&state)==MK_OK);
        const double progress=(state.position[0]-q[0][0])/(q[1][0]-q[0][0]);
        for (unsigned joint=0;joint<joints;++joint) {
            assert(std::abs(state.position[joint]-(q[0][joint]+(q[1][joint]-q[0][joint])*progress))<1e-11);
            assert(std::abs(state.velocity[joint])<=velocity[joint]*(1+1e-12));
            assert(std::abs(state.acceleration[joint])<=acceleration[joint]*(1+1e-12));
        }
    }
}

// A redundant one-tick cut in a constant-speed source cannot manufacture
// pressure-advance acceleration from the rounding of its absolute position.
static void pressureAdvanceClockSliver() {
    mk_trajectory_handle source{},advanced{};
    assert(mk_trajectory_create(1,&source)==MK_OK);
    const int64_t knots[]={0,1'000'000'000,1'000'000'001,2'000'000'000};
    for (unsigned i=0;i<3;++i) {
        mk_segment segment{};segment.struct_size=sizeof(segment);segment.joint_count=1;segment.degree=5;
        segment.t0_ns=knots[i];segment.duration_ns=knots[i+1]-knots[i];
        segment.coefficients[0].value[0]=100+knots[i]*1e-9;
        segment.coefficients[0].value[1]=1;
        assert(mk_trajectory_append_segment(source,&segment)==MK_OK);
    }
    assert(mk_advance_joint(source,0,0.03,0.04,&advanced)==MK_OK);
    mk_trajectory_destroy(source);
    for (int64_t shift : {-20'000'000LL,0LL,20'000'000LL}) for (int64_t delta=-2;delta<=3;++delta) {
        mk_trajectory_state state{};state.struct_size=sizeof(state);
        const int64_t time=1'000'000'000+shift+delta;
        assert(mk_trajectory_evaluate(advanced,time,&state)==MK_OK);
        assert(std::abs(state.position[0]-(100+time*1e-9+0.03))<1e-10);
        assert(std::abs(state.velocity[0]-1)<1e-12);
        assert(std::abs(state.acceleration[0])<1e-9);
    }
    mk_limits limits{};limits.struct_size=sizeof(limits);limits.joint_count=1;
    limits.derivative_claimed[0]=2;limits.max_acceleration[0]=1000;
    mk_validation_report report{};report.struct_size=sizeof(report);
    assert(mk_validate(advanced,&limits,&report)==MK_OK);
    // The smooth end tapers are below 200 m/s^2. Continuous validation must
    // also inspect the sliver's fractional-tick interior, not just its knots.
    assert(report.checks[MK_CHECK_ACCELERATION].status==MK_CHECK_PASSED);
    mk_trajectory_destroy(advanced);
}

// A two-axis tangent, reduced from the cube. Backward rounding of an
// almost-zero tangent coefficient must not become an extreme control bound.
static void tangentBoundRounding() {
    const double data[][9]={
        {0.0,0.00916438055098076,-0.0031743122048878275,0,-1,0,0,-1598.8857396310748,-9.79035151620772e-14},
        {0.00019690518461240591,0.00916438055098076,-0.0033712173895002195,6.123233995736766e-17,-1,-20164.074121909372,-1.2346934415583145e-12,0,0},
        {0.00020859029584513467,0.009163010288233492,-0.0033827946809962123,-0.23344536385616185,-0.972369920397615,-19606.939148812628,4707.209620211751,-19606.939148812628,4707.209620211751},
        {0.00022027540707783566,0.009158975220929275,-0.0033937322095209527,-0.4539904997395182,-0.8910065241883824,-17966.32159683938,9154.298087390323,-17966.32159683938,9154.298087390323}
    };
    const double velocity[]={0.5000000108083548,0.5000000108083548},
        acceleration[]={12.542575411039081,24.094014749947863},
        caps[]={0.15,0.06,0.06};
    std::vector<mk_path_sample> samples(4);
    for (unsigned i=0;i<4;++i) {
        auto& p=samples[i];p.struct_size=sizeof(p);p.joint_count=2;p.s=data[i][0];
        for (unsigned j=0;j<2;++j) {
            p.position[j]=data[i][1+j];p.first[j]=data[i][3+j];
            p.second[j]=data[i][5+j];p.second_before[j]=data[i][7+j];
        }
    }
    mk_path_handle path{};assert(mk_path_create(samples.data(),samples.size(),&path)==MK_OK);
    mk_path_timing_stream_handle stream{};
    assert(mk_path_timing_stream_create(path,velocity,acceleration,2,caps,3,0,0,1e-6,0,2,&stream)==MK_OK);
    mk_path_destroy(path);
    Result result;mk_path_timing_stream_progress progress{};mk_timing_report timing{};
    assert(mk_path_timing_stream_step(stream,&progress,&result.preview,&timing,&result.law,&result.trajectory)==MK_OK);
    assert(progress.state==MK_TIMING_STREAM_READY);
    mk_limits limits{};limits.struct_size=sizeof(limits);limits.joint_count=2;
    for (unsigned j=0;j<2;++j) {
        limits.derivative_claimed[j]=3;limits.max_velocity[j]=velocity[j];limits.max_acceleration[j]=acceleration[j];
    }
    mk_validation_report report{};report.struct_size=sizeof(report);
    assert(mk_validate(result.trajectory,&limits,&report)==MK_OK);
    assert(report.checks[MK_CHECK_ACCELERATION].status==MK_CHECK_PASSED);
    assert(report.checks[MK_CHECK_VELOCITY].status==MK_CHECK_PASSED);
    mk_path_timing_stream_destroy(stream);
}

// Reduced from the cube's curvature-to-line transition. Nominal collocation
// and continuous lowering agreed on an unsafe 13.533144729 m/s^2 peak: the
// rounded clock can switch acceleration on the other side of a curvature join.
static void curvatureClockJoin() {
    const double data[][5]={
        {0.0,-0.009715947898776875,0.06258427672040515,12599.775816303621,12599.775816303621},
        {1.848354322167567e-05,-0.009712659034968929,0.2916688986230861,12075.601295784285,12075.601295784285},
        {3.6967086443379094e-05,-0.009705263331928046,0.5049439862867875,0,10896.884473887903}
    };
    std::vector<mk_path_sample> samples(3);
    for (unsigned i=0;i<3;++i) {
        auto& p=samples[i];p.struct_size=sizeof(p);p.joint_count=1;
        p.s=data[i][0];p.position[0]=data[i][1];p.first[0]=data[i][2];
        p.second[0]=data[i][3];p.second_before[0]=data[i][4];
    }
    const double velocity=0.5000000108083548, acceleration=12.542575411039081;
    const double caps[]={0.02,0.02};
    mk_path_handle path{};assert(mk_path_create(samples.data(),samples.size(),&path)==MK_OK);
    mk_path_timing_stream_handle stream{};
    assert(mk_path_timing_stream_create(path,&velocity,&acceleration,1,caps,2,0,0,1e-6,0,2,&stream)==MK_OK);
    mk_path_destroy(path);
    Result result;mk_path_timing_stream_progress progress{};mk_timing_report timing{};
    assert(mk_path_timing_stream_step(stream,&progress,&result.preview,&timing,&result.law,&result.trajectory)==MK_OK);
    assert(progress.state==MK_TIMING_STREAM_READY);
    mk_limits limits{};limits.struct_size=sizeof(limits);limits.joint_count=1;
    limits.derivative_claimed[0]=3;limits.max_velocity[0]=velocity;limits.max_acceleration[0]=acceleration;
    mk_validation_report report{};report.struct_size=sizeof(report);
    assert(mk_validate(result.trajectory,&limits,&report)==MK_OK);
    assert(report.checks[MK_CHECK_ACCELERATION].status==MK_CHECK_PASSED);
    assert(report.checks[MK_CHECK_VELOCITY].status==MK_CHECK_PASSED);
    assert(mk_path_timing_stream_commit(stream,result.preview)==MK_OK);
    mk_path_timing_stream_destroy(stream);
}

static void continuousVelocityBound() {
    mk_path_sample samples[2]{};
    const double q[]={0,1.5707963267948966};
    const double first[]={200.1956649789314,198.89389758376342};
    const double second[]={137277.02741412443,-137797.73437219165};
    for (int i=0;i<2;++i) {
        auto& point=samples[i];point.struct_size=sizeof(point);point.joint_count=1;
        point.s=i*0.005000000000000003;point.position[0]=q[i];point.first[0]=first[i];
        point.second[0]=point.second_before[0]=second[i];
    }
    mk_path_handle path{};assert(mk_path_create(samples,2,&path)==MK_OK);
    const double velocity=1,acceleration=2,cap=0.1;
    mk_path_timing_stream_handle stream{};
    assert(mk_path_timing_stream_create(path,&velocity,&acceleration,1,&cap,1,0,0,1e-6,1,1,&stream)==MK_OK);
    mk_path_destroy(path);
    Result result;mk_path_timing_stream_progress progress{};
    assert(mk_path_timing_stream_step(stream,&progress,&result.preview,nullptr,&result.law,&result.trajectory)==MK_OK);
    assert(progress.state==MK_TIMING_STREAM_READY);
    mk_limits limits{};limits.struct_size=sizeof(limits);limits.joint_count=1;
    limits.derivative_claimed[0]=3;limits.max_velocity[0]=velocity;limits.max_acceleration[0]=acceleration;
    mk_validation_report report{};report.struct_size=sizeof(report);
    assert(mk_validate(result.trajectory,&limits,&report)==MK_OK);
    assert(report.checks[MK_CHECK_VELOCITY].status==MK_CHECK_PASSED);
    assert(report.checks[MK_CHECK_ACCELERATION].status==MK_CHECK_PASSED);
    mk_path_timing_stream_destroy(stream);
}

static void uncappedStationaryEnd() {
    mk_path_sample samples[2]{};
    const double q[]={0,0.1};
    const double first[]={1,1e-9};
    const double second[]={10,10};
    for (int i=0;i<2;++i) {
        auto& point=samples[i];point.struct_size=sizeof(point);point.joint_count=1;
        point.s=i*0.1;point.position[0]=q[i];point.first[0]=first[i];
        point.second[0]=point.second_before[0]=second[i];
    }
    mk_path_handle path{};assert(mk_path_create(samples,2,&path)==MK_OK);
    const double velocity=1,acceleration=2;
    mk_path_timing_stream_handle stream{};
    assert(mk_path_timing_stream_create(path,&velocity,&acceleration,1,nullptr,0,0,0,1e-6,1,1,&stream)==MK_OK);
    mk_path_destroy(path);
    Result result;mk_path_timing_stream_progress progress{};
    assert(mk_path_timing_stream_step(stream,&progress,&result.preview,nullptr,&result.law,&result.trajectory)==MK_OK);
    assert(progress.state==MK_TIMING_STREAM_READY);
    mk_limits limits{};limits.struct_size=sizeof(limits);limits.joint_count=1;
    limits.derivative_claimed[0]=3;limits.max_velocity[0]=velocity;limits.max_acceleration[0]=acceleration;
    mk_validation_report report{};report.struct_size=sizeof(report);
    assert(mk_validate(result.trajectory,&limits,&report)==MK_OK);
    assert(report.checks[MK_CHECK_VELOCITY].status==MK_CHECK_PASSED);
    assert(report.checks[MK_CHECK_ACCELERATION].status==MK_CHECK_PASSED);
    mk_path_timing_stream_destroy(stream);
}

static void derivativeCertifiedLowering() {
    mk_path_sample samples[3]{};
    const double q[]={0,0.04999500049995001,0.09999500000004999};
    const double x[]={0.1,0.10000000000000006,0.1000000000000001};
    for (int i=0;i<3;++i) {
        auto& sample=samples[i];sample.struct_size=sizeof(sample);sample.joint_count=4;
        sample.s=i*0.005000000000000001;
        sample.position[0]=x[i];sample.position[1]=sample.position[2]=0.1;sample.position[3]=q[i];
        sample.first[0]=9.714451465460403e-15;sample.first[3]=9.999999999989997;
    }
    mk_path_handle path{};assert(mk_path_create(samples,3,&path)==MK_OK);
    const double velocity[]={1,1,1,1},acceleration[]={2,2,2,2},caps[]={0.1,0.1};
    mk_path_timing_stream_handle stream{};
    assert(mk_path_timing_stream_create(path,velocity,acceleration,4,caps,2,0,0,1e-6,1,2,&stream)==MK_OK);
    mk_path_destroy(path);
    Result result;mk_path_timing_stream_progress progress{};
    assert(mk_path_timing_stream_step(stream,&progress,&result.preview,nullptr,&result.law,&result.trajectory)==MK_OK);
    assert(progress.state==MK_TIMING_STREAM_READY);
    int64_t duration=0;assert(mk_trajectory_duration_ns(result.trajectory,&duration)==MK_OK);
    assert(duration==447230419); // Representation refinement preserves the original time law.
    mk_limits limits{};limits.struct_size=sizeof(limits);limits.joint_count=4;
    for (int i=0;i<4;++i) {limits.derivative_claimed[i]=3;limits.max_velocity[i]=1;limits.max_acceleration[i]=2;}
    mk_validation_report report{};report.struct_size=sizeof(report);
    assert(mk_validate(result.trajectory,&limits,&report)==MK_OK);
    assert(report.checks[MK_CHECK_ACCELERATION].status==MK_CHECK_PASSED);
    assert(report.checks[MK_CHECK_VELOCITY].status==MK_CHECK_PASSED);
    mk_path_timing_stream_destroy(stream);
}

int main() { uncappedStationaryEnd(); continuousVelocityBound(); derivativeCertifiedLowering(); pressureAdvanceClockSliver(); tangentBoundRounding(); curvatureClockJoin(); affineCubeRegression(); advanceContext(); ownership(); run(2,1); run(20000,4); movingEndpoint(0); movingEndpoint(0.5); curved(0); curved(0.1); }
