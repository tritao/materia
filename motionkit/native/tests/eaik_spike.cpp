#include "EAIK.h"
#include "eaik_fixtures.h"
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>
namespace {
void require(bool value,const char *message){if(!value)throw std::runtime_error(message);}
mk_analytic_pose readPose(std::istream &in) {
    mk_analytic_pose p{};p.struct_size=sizeof(p);
    in>>p.position[0]>>p.position[1]>>p.position[2];for(double &x:p.quaternion)in>>x;return p;
}
EAIK::Robot direct(const mk_eaik_model &m) {
    return EAIK::Robot(Eigen::MatrixXd(Eigen::Map<const eaik_fixtures::H>(m.axes)),
        Eigen::MatrixXd(Eigen::Map<const eaik_fixtures::P>(m.displacements)),
        Eigen::Matrix3d(Eigen::Map<const Eigen::Matrix3d>(m.terminal_rotation)));
}
volatile size_t benchmarkSink=0;
void benchmark(const std::string &name,const mk_eaik_model &model,const double *lower,const double *upper) {
    auto robot=direct(model);mk_eaik_handle solver{};require(mk_eaik_create(&model,&solver)==MK_OK,"Benchmark model rejected");
    std::mt19937_64 rng(0x50503161);std::uniform_real_distribution<double> fraction(.05,.95);
    std::vector<mk_analytic_pose> targets;std::vector<Eigen::Matrix4d> matrices;
    for(unsigned i=0;i<100000;++i){double q[6];for(unsigned j=0;j<6;++j)q[j]=lower[j]+fraction(rng)*(upper[j]-lower[j]);const auto t=eaik_fixtures::forward(model,q);matrices.push_back(t);targets.push_back(eaik_fixtures::pose(t));}
    std::vector<double> coreTimes,adapterTimes;coreTimes.reserve(targets.size());adapterTimes.reserve(targets.size());
    const auto core=[&](size_t i){const auto result=robot.calculate_IK(matrices[i]);benchmarkSink=result.Q.size();};
    const auto adapter=[&](size_t i){mk_analytic_solution out[32];uint32_t count=0;require(mk_eaik_inverse(solver,&targets[i],out,32,&count)==MK_OK,"Benchmark inverse failed");benchmarkSink=count;};
    for(unsigned i=0;i<1000;++i){core(i);adapter(i);}
    const auto measure=[](const auto &call,size_t i){const auto start=std::chrono::steady_clock::now();call(i);return std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();};
    for(size_t i=0;i<targets.size();++i){if(i%2){adapterTimes.push_back(measure(adapter,i));coreTimes.push_back(measure(core,i));}else{coreTimes.push_back(measure(core,i));adapterTimes.push_back(measure(adapter,i));}}
    std::sort(coreTimes.begin(),coreTimes.end());std::sort(adapterTimes.begin(),adapterTimes.end());
    std::cout<<"NON_GATING_EAIK_TIMING "<<name<<" targets=100000 coreMedianUs="<<coreTimes[50000]<<" coreP95Us="<<coreTimes[95000]<<" adapterMedianUs="<<adapterTimes[50000]<<" adapterP95Us="<<adapterTimes[95000]<<"\n";
    mk_eaik_destroy(solver);
}
void checkModels(const char *file,bool measure) {
    std::ifstream in(file);require(bool(in),"Cannot open compiled model fixtures");unsigned models;in>>models;require(models==7,"Expected seven compiled models");
    for(unsigned model=0;model<models;++model){std::string name;in>>name;mk_eaik_model geometry{};geometry.struct_size=sizeof(geometry);
        for(double &x:geometry.axes)in>>x;for(double &x:geometry.displacements)in>>x;for(double &x:geometry.terminal_rotation)in>>x;
        double lower[6],upper[6];for(double &x:lower)in>>x;for(double &x:upper)in>>x;unsigned retired;in>>retired;require(retired==0,"Regenerate the retired-solver fixture metadata");
        unsigned samples;in>>samples;require(samples==1005,"Expected 1000 regular and five singular samples");
        mk_eaik_handle solver{};require(mk_eaik_create(&geometry,&solver)==MK_OK,"Compiled model rejected");auto robot=direct(geometry);
        for(unsigned sample=0;sample<samples;++sample){double q[6];for(double &x:q)in>>x;const auto target=readPose(in);require(bool(in),"Malformed compiled model fixture");
            const auto expected=eaik_fixtures::forward(geometry,q);const auto record=eaik_fixtures::pose(expected);
            for(unsigned j=0;j<3;++j)require(std::abs(record.position[j]-target.position[j])<1e-9,"Independent compiled FK position");
            const Eigen::Quaterniond actual(record.quaternion[3],record.quaternion[0],record.quaternion[1],record.quaternion[2]);
            const Eigen::Quaterniond desired(target.quaternion[3],target.quaternion[0],target.quaternion[1],target.quaternion[2]);
            require(Eigen::AngleAxisd(actual.conjugate()*desired).angle()<1e-9,"Independent compiled FK orientation");
            mk_analytic_solution out[32],repeat[32];uint32_t count=0,repeated=0;
            require(mk_eaik_inverse(solver,&target,out,32,&count)==MK_OK && count>0,"Compiled inverse exact solutions");
            require(mk_eaik_inverse(solver,&target,repeat,32,&repeated)==MK_OK && count==repeated,"Deterministic compiled inverse");
            bool original=false;
            for(unsigned i=0;i<count;++i){double delta=0;for(unsigned j=0;j<6;++j){require(out[i].joints[j]==repeat[i].joints[j],"Deterministic joints");delta=std::max(delta,std::abs(std::remainder(out[i].joints[j]-q[j],2*std::acos(-1.0))));}original=original || delta<1e-9;}
            if(sample<1000)require(original,"Compiled inverse lost original branch");
        }
        std::cout<<"Production EAIK compiled model "<<name<<" 1005 samples, family="<<robot.get_kinematic_family()<<"\n";mk_eaik_destroy(solver);
        if(measure && name=="RobotArm900")benchmark(name,geometry,lower,upper);
    }
    if(measure){double lower[6],upper[6];for(unsigned j=0;j<6;++j){lower[j]=-2.8;upper[j]=2.8;}benchmark("IRB2400",eaik_fixtures::published()[0],lower,upper);}
}
}
int main(int argc,char **argv) {
    try {if(argc>1)checkModels(argv[1],argc>2 && std::string(argv[2])=="--benchmark");else std::cout<<"Pass regenerated compiled-model fixtures; timings are non-gating.\n";return 0;}
    catch(const std::exception &e){std::cerr<<e.what()<<"\n";return 1;}
}
