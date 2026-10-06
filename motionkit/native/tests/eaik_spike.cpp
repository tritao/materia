#include "EAIK.h"
#include <opw_kinematics/opw_parameters_examples.h>
#include <opw_kinematics/opw_utilities.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>
#include <fstream>
#include <chrono>

volatile std::size_t benchmarkSink=0;

namespace {
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

bool samePose(const Eigen::Matrix4d& actual, const Eigen::Matrix4d& expected) {
    const Eigen::Matrix3d relative = actual.block<3, 3>(0, 0).transpose() * expected.block<3, 3>(0, 0);
    return (actual.block<3, 1>(0, 3) - expected.block<3, 1>(0, 3)).norm() < 1e-9
        && Eigen::AngleAxisd(relative).angle() < 1e-9;
}

// Canonical OPW fixture vectors. Authored RobotModel vectors are a separate
// acceptance check; these fixtures include signed joints and zero offsets.
void check(const char* name, const opw_kinematics::Parameters<double>& p) {
    Eigen::Matrix<double, 3, 6> h;
    h << 0, 0, 0, 0, 0, 0,
         0, 1, 1, 0, 1, 0,
         1, 0, 0, 1, 0, 1;
    Eigen::Matrix<double, 3, 7> positions = Eigen::Matrix<double, 3, 7>::Zero();
    positions.col(0) = Eigen::Vector3d(0, 0, p.c1);
    positions.col(1) = Eigen::Vector3d(p.a1, p.b, 0);
    positions.col(2) = Eigen::Vector3d(0, 0, p.c2);
    positions.col(3) = Eigen::Vector3d(p.a2, 0, p.c3);
    positions.col(6) = Eigen::Vector3d(0, 0, p.c4);
    EAIK::Robot robot(h, positions);
    require(robot.has_known_decomposition(), "EAIK lacks a fixture decomposition");
    std::mt19937_64 random(0x50503161);
    std::uniform_real_distribution<double> angle(-2.8, 2.8);
    std::size_t matched = 0;
    for (int sample = 0; sample < 1000; ++sample) {
        std::array<double, 6> physical;
        std::vector<double> canonical(6);
        for (int j = 0; j < 6; ++j) {
            physical[j] = angle(random);
            canonical[j] = p.sign_corrections[j] * physical[j] - p.offsets[j];
        }
        auto target = opw_kinematics::forward(p, physical);
        require(samePose(robot.fwdkin(canonical), target.matrix()),
            "Canonical fixture FK differs from OPW");
        const auto solutions = robot.calculate_IK(target.matrix());
        const auto repeated = robot.calculate_IK(target.matrix());
        require(solutions.Q == repeated.Q && solutions.is_LS_vec == repeated.is_LS_vec,
            "EAIK all-solutions calls are not deterministic");
        for (std::size_t i = 0; i < solutions.Q.size(); ++i) {
            if (solutions.is_LS_vec[i]) continue;
            require(samePose(robot.fwdkin(solutions.Q[i]), target.matrix()),
                "EAIK fixture FK round trip failed");
        }
        for (const auto& expected : opw_kinematics::inverse(p, target)) {
            if (!opw_kinematics::isValid(expected)) continue;
            bool found = false;
            for (std::size_t i = 0; i < solutions.Q.size(); ++i) {
                if (solutions.is_LS_vec[i]) continue;
                const auto& solution = solutions.Q[i];
                double error = 0;
                for (int j = 0; j < 6; ++j)
                    error = std::max(error, std::abs(std::remainder(
                        solution[j] - (p.sign_corrections[j] * expected[j] - p.offsets[j]),
                        2 * std::acos(-1.0))));
                found = found || error < 1e-9;
            }
            require(found, "EAIK does not contain every OPW solution within 1e-9 rad");
            ++matched;
        }
    }
    std::cout << name << ": 1000 deterministic targets, " << matched
              << " OPW solutions contained; " << robot.get_kinematic_family() << '\n';
}

void benchmark(const std::string& name,const EAIK::Robot& robot,
    const opw_kinematics::Parameters<double>& parameters,const Eigen::Matrix4d& base,
    const Eigen::Matrix4d& tool,const std::array<double,6>& lower,const std::array<double,6>& upper) {
    constexpr std::size_t count=100000;
    std::mt19937_64 random(0x50503161);
    std::vector<Eigen::Matrix4d> targets;
    std::vector<opw_kinematics::Transform<double>> local;
    targets.reserve(count);local.reserve(count);
    const auto inverseBase=base.inverse().eval(),inverseTool=tool.inverse().eval();
    for(std::size_t i=0;i<count;++i){
        std::vector<double> q(6);
        for(int j=0;j<6;++j)q[j]=lower[j]+std::generate_canonical<double,53>(random)*(upper[j]-lower[j]);
        targets.push_back(robot.fwdkin(q));
        local.emplace_back(inverseBase*targets.back()*inverseTool);
    }
    auto opwCall=[&](std::size_t i){const auto result=opw_kinematics::inverse(parameters,local[i]);
        std::size_t valid=0;for(const auto& q:result)valid+=opw_kinematics::isValid(q);benchmarkSink=valid;};
    auto eaikCall=[&](std::size_t i){const auto result=robot.calculate_IK(targets[i]);benchmarkSink=result.Q.size();};
    for(std::size_t i=0;i<1000;++i){opwCall(i);eaikCall(i);}
    std::vector<double> opwTimes(count),eaikTimes(count);
    auto time=[](auto&& call,std::size_t i){auto start=std::chrono::steady_clock::now();call(i);
        return std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();};
    for(std::size_t i=0;i<count;++i){
        if(i%2==0){opwTimes[i]=time(opwCall,i);eaikTimes[i]=time(eaikCall,i);}
        else{eaikTimes[i]=time(eaikCall,i);opwTimes[i]=time(opwCall,i);}
    }
    std::sort(opwTimes.begin(),opwTimes.end());std::sort(eaikTimes.begin(),eaikTimes.end());
    std::cout << "benchmark " << name << " targets=" << count << " OPW median_us=" << opwTimes[count/2]
      << " p95_us=" << opwTimes[count*95/100] << " EAIK median_us=" << eaikTimes[count/2]
      << " p95_us=" << eaikTimes[count*95/100] << " R=" << eaikTimes[count/2]/opwTimes[count/2] << '\n';
}

Eigen::Matrix4d readPose(std::istream& input) {
    double x,y,z,qx,qy,qz,qw;
    input >> x >> y >> z >> qx >> qy >> qz >> qw;
    Eigen::Matrix4d result=Eigen::Matrix4d::Identity();
    result.block<3,3>(0,0)=Eigen::Quaterniond(qw,qx,qy,qz).normalized().toRotationMatrix();
    result.block<3,1>(0,3)=Eigen::Vector3d(x,y,z);
    return result;
}

void checkModels(const char* path,bool measure) {
    std::ifstream input(path);
    require(bool(input), "Cannot open compiled-model fixtures");
    int fixtures=0; input >> fixtures;
    require(fixtures==7,"Expected seven compiled-model fixtures");
    for(int fixture=0;fixture<fixtures;++fixture) {
        std::string name; input >> name;
        Eigen::Matrix<double,3,6> h;
        Eigen::Matrix<double,3,7> p;
        Eigen::Matrix3d rotation;
        for(int col=0;col<6;++col)for(int row=0;row<3;++row)input >> h(row,col);
        for(int col=0;col<7;++col)for(int row=0;row<3;++row)input >> p(row,col);
        for(int col=0;col<3;++col)for(int row=0;row<3;++row)input >> rotation(row,col);
        std::array<double,6> lower,upper;
        for(auto& value:lower)input >> value;
        for(auto& value:upper)input >> value;
        int spherical;input >> spherical;
        opw_kinematics::Parameters<double> opw;
        Eigen::Matrix4d base=Eigen::Matrix4d::Identity(),tool=base;
        if(spherical){
            input >> opw.a1 >> opw.a2 >> opw.b >> opw.c1 >> opw.c2 >> opw.c3 >> opw.c4;
            for(auto& value:opw.offsets)input >> value;
            for(auto& value:opw.sign_corrections){int sign;input >> sign;value=sign;}
            base=readPose(input);tool=readPose(input);
        }
        EAIK::Robot robot(h,p,rotation);
        require(robot.has_known_decomposition(), "Compiled model has no EAIK decomposition");
        int samples=0;input >> samples;
        require(samples==1005,"Expected 1000 regular and five independently compiled singular targets");
        for(int sample=0;sample<samples;++sample){
            std::vector<double> q(6);for(auto& value:q)input >> value;
            auto target=readPose(input);
            require(bool(input), "Malformed compiled-model fixture");
            require(samePose(robot.fwdkin(q),target), "Compiled-model EAIK FK disagrees with KinematicGroup");
            const auto solutions=robot.calculate_IK(target);
            const auto repeated=robot.calculate_IK(target);
            require(solutions.Q==repeated.Q && solutions.is_LS_vec==repeated.is_LS_vec,"Compiled-model EAIK is not deterministic");
            bool found=false;
            for(std::size_t i=0;i<solutions.Q.size();++i){
                if(solutions.is_LS_vec[i])continue;
                require(samePose(robot.fwdkin(solutions.Q[i]),target),"Compiled-model inverse FK failed");
                found=true;
            }
            require(found,"Compiled reachable target has no exact EAIK solution");
            if(spherical && sample<1000){
                opw_kinematics::Transform<double> local(base.inverse()*target*tool.inverse());
                for(const auto& expected:opw_kinematics::inverse(opw,local)){
                    if(!opw_kinematics::isValid(expected))continue;
                    bool contained=false;
                    for(std::size_t i=0;i<solutions.Q.size();++i){
                        if(solutions.is_LS_vec[i])continue;
                        double error=0;
                        for(int j=0;j<6;++j)error=std::max(error,std::abs(std::remainder(solutions.Q[i][j]-expected[j],2*std::acos(-1.0))));
                        contained=contained || error<1e-9;
                    }
                    require(contained,"Compiled-model EAIK does not contain every OPW solution");
                }
            }
        }
        for(double wrist:{0.0,1e-12,-1e-12,1e-7,std::acos(-1.0)}){
            std::vector<double> q={0.2,-0.3,0.4,0.5,wrist,0.7};
            const auto target=robot.fwdkin(q);
            const auto solutions=robot.calculate_IK(target);
            bool found=false;
            for(std::size_t i=0;i<solutions.Q.size();++i){
                if(solutions.is_LS_vec[i])continue;
                require(samePose(robot.fwdkin(solutions.Q[i]),target),"Singular solution FK failed");found=true;
            }
            require(found,"Singular reachable target has no exact EAIK solution");
        }
        std::cout << name << ": " << samples << " compiled FK/IK targets and wrist singularities passed; " << robot.get_kinematic_family() << '\n';
        if(measure && name=="RobotArm900")benchmark(name,robot,opw,base,tool,lower,upper);
    }
    if(measure){
        const auto parameters=opw_kinematics::makeIrb2400_10<double>();
        Eigen::Matrix<double,3,6> h;h << 0,0,0,0,0,0, 0,1,1,0,1,0, 1,0,0,1,0,1;
        Eigen::Matrix<double,3,7> p=Eigen::Matrix<double,3,7>::Zero();
        p.col(0)=Eigen::Vector3d(0,0,parameters.c1);p.col(1)=Eigen::Vector3d(parameters.a1,parameters.b,0);
        p.col(2)=Eigen::Vector3d(0,0,parameters.c2);p.col(3)=Eigen::Vector3d(parameters.a2,0,parameters.c3);
        p.col(6)=Eigen::Vector3d(0,0,parameters.c4);
        // EAIK's fixture coordinates are canonical; OPW's fixture offset must
        // be removed here so both timed solvers receive the same coordinates.
        auto canonical=parameters;canonical.offsets.fill(0);
        EAIK::Robot robot(h,p);
        std::array<double,6> lower,upper;lower.fill(-2.8);upper.fill(2.8);
        benchmark("IRB2400",robot,canonical,Eigen::Matrix4d::Identity(),Eigen::Matrix4d::Identity(),lower,upper);
    }
}

void checkUnsupported() {
    Eigen::Matrix<double,3,6> h;
    h << 1,0,0,1,1,0, 0,1,0,1,0,1, 0,0,1,0,1,1;
    for(int i=0;i<6;++i)h.col(i).normalize();
    Eigen::Matrix<double,3,7> p;
    p << .1,.2,.3,.4,.5,.6,.7, .7,.3,.5,.2,.6,.1,.4, .2,.6,.1,.7,.3,.4,.5;
    EAIK::Robot robot(h,p);
    require(!robot.has_known_decomposition(),"Generic 6R unexpectedly has a closed-form decomposition");
    bool refused=false;
    try{robot.calculate_IK(robot.fwdkin({.1,.2,.3,.4,.5,.6}));}
    catch(const std::runtime_error&){refused=true;}
    require(refused,"Unknown decomposition must refuse instead of returning an incomplete solution set");
    std::cout << "Unknown 6R decomposition refused; pinned EAIK provides no general 1-D search backend\n";
}
}

int main(int argc,char** argv) {
    try {
        check("IRB2400", opw_kinematics::makeIrb2400_10<double>());
        check("KR6", opw_kinematics::makeKukaKR6_R700_sixx<double>());
        check("R2000", opw_kinematics::makeFanucR2000iB_200R<double>());
        check("TX40", opw_kinematics::makeStaubliTX40<double>());
        checkUnsupported();
        if(argc>1)checkModels(argv[1],argc>2 && std::string(argv[2])=="--benchmark");
    } catch (const std::exception& error) {
        std::cerr << "EAIK spike failed: " << error.what() << '\n';
        return 1;
    }
}
