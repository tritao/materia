#include "motionkit.h"
#include <Eigen/Sparse>
#include <proxsuite/proxqp/sparse/sparse.hpp>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <vector>

namespace {
using Term=std::pair<int,double>;
using Row=std::vector<Term>;
using Sparse=Eigen::SparseMatrix<double,Eigen::ColMajor,int>;
struct Rows {
    std::vector<Eigen::Triplet<double>> terms;
    std::vector<double> lower,upper;
    void add(const Row &row,double lo,double hi) {
        const int i=static_cast<int>(lower.size());
        for(auto [j,x]:row)if(x!=0)terms.emplace_back(i,j,x);
        lower.push_back(lo);upper.push_back(hi);
    }
    Sparse matrix(int width)const {
        Sparse m(static_cast<int>(lower.size()),width);m.setFromTriplets(terms.begin(),terms.end());return m;
    }
};
Row combine(const Row &a,double x,const Row &b,double y) {
    Row r;for(auto [i,v]:a)r.emplace_back(i,v*x);for(auto [i,v]:b)r.emplace_back(i,v*y);return r;
}
struct Cubic {
    Row position[4],first[3],second[2];
    Cubic(int a,int b,double h) {
        position[0]={{a,1}};position[1]={{a,1},{a+1,h/3}};
        position[2]={{b,1},{b+1,-h/3}};position[3]={{b,1}};
        first[0]={{a+1,1}};first[1]={{a,-3/h},{b,3/h},{a+1,-1},{b+1,-1}};first[2]={{b+1,1}};
        second[0]={{a,-6/(h*h)},{b,6/(h*h)},{a+1,-4/h},{b+1,-2/h}};
        second[1]={{a,6/(h*h)},{b,-6/(h*h)},{a+1,2/h},{b+1,4/h}};
    }
};
double evaluate(const Row &r,const Eigen::VectorXd &x) {
    double v=0;for(auto [i,a]:r)v+=a*x[i];return v;
}
}
extern "C" mk_result MK_CALL mk_refine_redundancy(const mk_refinement_limits *l,
    const mk_refinement_sample *s,uint32_t n,const mk_refinement_bound *bounds,uint32_t nb,
    mk_refinement_solution *out) {
    if(!l || l->struct_size<sizeof(*l) || !s || !out || n<2 || n>10001 ||
        !l->coordinate_count || l->coordinate_count>MK_MAX_JOINTS || (!bounds && nb))return MK_ERROR_INVALID_ARGUMENT;
    const uint32_t d=l->coordinate_count;
    if(!std::isfinite(l->route_weight) || l->route_weight<=0 || !std::isfinite(l->seed_weight) || l->seed_weight<0 ||
        !std::isfinite(l->curvature_weight) || l->curvature_weight<0)return MK_ERROR_INVALID_ARGUMENT;
    for(uint32_t j=0;j<d;++j)if(!std::isfinite(l->max_velocity[j]) || l->max_velocity[j]<=0 ||
        !std::isfinite(l->max_acceleration[j]) || l->max_acceleration[j]<=0)return MK_ERROR_INVALID_ARGUMENT;
    for(uint32_t i=0;i<n;++i) {
        if(s[i].struct_size<sizeof(s[i]) || !std::isfinite(s[i].s) || (i && s[i].s<=s[i-1].s) ||
            !std::isfinite(s[i].feed) || s[i].feed<=0 || !std::isfinite(s[i].feed_gradient) || s[i].feed_gradient<0)return MK_ERROR_INVALID_ARGUMENT;
        for(uint32_t j=0;j<d;++j)if(!std::isfinite(s[i].route[j]) || !std::isfinite(s[i].seed[j]) ||
            !std::isfinite(s[i].gradient[j]) || !std::isfinite(s[i].lower[j]) || !std::isfinite(s[i].upper[j]) || s[i].lower[j]>s[i].upper[j])return MK_ERROR_INVALID_ARGUMENT;
    }
    for(uint32_t k=0;k<nb;++k) {
        if(bounds[k].struct_size<sizeof(bounds[k]) || bounds[k].sample>=n || bounds[k].derivative>2 ||
            !std::isfinite(bounds[k].lower) || !std::isfinite(bounds[k].upper) || bounds[k].lower>bounds[k].upper)return MK_ERROR_INVALID_ARGUMENT;
        for(uint32_t j=0;j<d;++j)if(!std::isfinite(bounds[k].coefficients[j]))return MK_ERROR_INVALID_ARGUMENT;
    }
    try {
        // Propagate scalar position boxes through the drive's maximum travel.
        // This proves simple infeasibility before asking the iterative solver.
        std::vector<double> knotLow(n*d),knotHigh(n*d);
        for(uint32_t i=0;i<n;++i)for(uint32_t j=0;j<d;++j){knotLow[i*d+j]=s[i].lower[j];knotHigh[i*d+j]=s[i].upper[j];}
        for(uint32_t k=0;k<nb;++k)if(bounds[k].derivative==0){
            int coordinate=-1;double coefficient=0;
            for(uint32_t j=0;j<d;++j)if(bounds[k].coefficients[j]!=0){
                if(coordinate>=0){coordinate=-2;break;}
                coordinate=static_cast<int>(j);coefficient=bounds[k].coefficients[j];
            }
            if(coordinate>=0){const auto i=bounds[k].sample*d+static_cast<uint32_t>(coordinate);
                double low=bounds[k].lower/coefficient,high=bounds[k].upper/coefficient;if(coefficient<0)std::swap(low,high);
                knotLow[i]=std::max(knotLow[i],low);knotHigh[i]=std::min(knotHigh[i],high);
            }
        }
        for(uint32_t j=0;j<d;++j){double low=knotLow[j],high=knotHigh[j];
            if(low>high+1e-12)return MK_ERROR_GENERATION;
            for(uint32_t i=1;i<n;++i){const double travel=l->max_velocity[j]*(s[i].s-s[i-1].s)/s[i-1].feed;
                low=std::max(low-travel,knotLow[i*d+j]);high=std::min(high+travel,knotHigh[i*d+j]);
                if(low>high+1e-12)return MK_ERROR_GENERATION;
            }
        }
        const int width=static_cast<int>(2*n*d);
        auto index=[d](uint32_t i,uint32_t j){return static_cast<int>(2*(i*d+j));};
        std::vector<Eigen::Triplet<double>> hessian;
        Eigen::VectorXd gradient=Eigen::VectorXd::Zero(width);
        Rows equality,inequality;
        for(uint32_t i=0;i<n;++i) {
            const double weight=((i ? s[i].s-s[i-1].s : 0)+(i+1<n ? s[i+1].s-s[i].s : 0))/2;
            for(uint32_t j=0;j<d;++j) {
                const int a=index(i,j);
                hessian.emplace_back(a,a,2*weight*(l->route_weight+l->seed_weight));
                hessian.emplace_back(a+1,a+1,1e-12); // Unique derivatives even with zero curvature weight.
                gradient[a]=weight*(-2*l->route_weight*s[i].route[j]-2*l->seed_weight*s[i].seed[j]+s[i].gradient[j]);
                inequality.add({{a,1}},s[i].lower[j],s[i].upper[j]);
            }
        }
        for(uint32_t i=0;i+1<n;++i) {
            const double h=s[i+1].s-s[i].s,feed=s[i].feed;
            for(uint32_t j=0;j<d;++j) {
                Cubic c(index(i,j),index(i+1,j),h);
                // A cubic lies in its Bernstein polygon. Limit the whole span,
                // not only a knot grid, using the intersection of endpoint bounds.
                const double lo=std::max(s[i].lower[j],s[i+1].lower[j]);
                const double hi=std::min(s[i].upper[j],s[i+1].upper[j]);
                if(lo>hi)return MK_ERROR_GENERATION;
                for(const auto &row:c.position)inequality.add(row,lo,hi);
                const double vmax=l->max_velocity[j]/feed;
                const double available=l->max_acceleration[j]-l->max_velocity[j]*s[i].feed_gradient;
                if(available<0)return MK_ERROR_GENERATION;
                const double amax=available/(feed*feed);
                for(const auto &row:c.first)inequality.add(row,-vmax,vmax);
                for(const auto &row:c.second)inequality.add(row,-amax,amax);
                if(i+2<n) {
                    Cubic next(index(i+1,j),index(i+2,j),s[i+2].s-s[i+1].s);
                    equality.add(combine(c.second[1],1,next.second[0],-1),0,0);
                }
                // Integral curvature energy: h/3 * (a0^2+a0*a1+a1^2).
                const double factor=2*l->curvature_weight*h/3;
                for(int u=0;u<2;++u)for(int v=0;v<2;++v)
                    for(auto [a,x]:c.second[u])for(auto [b,y]:c.second[v])
                        hessian.emplace_back(a,b,factor*(u==v ? 1 : .5)*x*y);
            }
        }
        for(uint32_t k=0;k<nb;++k) {
            const auto &bound=bounds[k];Row row;
            for(uint32_t j=0;j<d;++j) {
                const double w=bound.coefficients[j];if(w==0)continue;
                const uint32_t i=bound.sample;
                if(bound.derivative<2)row.emplace_back(index(i,j)+static_cast<int>(bound.derivative),w);
                else {
                    const uint32_t span=i+1<n ? i : i-1;
                    Cubic c(index(span,j),index(span+1,j),s[span+1].s-s[span].s);
                    for(auto [a,x]:c.second[i+1<n ? 0 : 1])row.emplace_back(a,w*x);
                }
            }
            inequality.add(row,bound.lower,bound.upper);
        }
        Sparse H(width,width);H.setFromTriplets(hessian.begin(),hessian.end());
        const auto A=equality.matrix(width),C=inequality.matrix(width);
        const Eigen::VectorXd b=Eigen::Map<const Eigen::VectorXd>(equality.lower.data(),equality.lower.size());
        const Eigen::VectorXd low=Eigen::Map<const Eigen::VectorXd>(inequality.lower.data(),inequality.lower.size());
        const Eigen::VectorXd high=Eigen::Map<const Eigen::VectorXd>(inequality.upper.data(),inequality.upper.size());
        proxsuite::proxqp::sparse::QP<double,int> qp(width,A.rows(),C.rows());
        qp.settings.eps_abs=1e-8;qp.settings.eps_rel=0;qp.settings.max_iter=500;qp.settings.max_iter_in=50;
        qp.settings.sparse_backend=proxsuite::proxqp::SparseBackend::SparseCholesky;
        qp.settings.verbose=false;qp.settings.compute_timings=false;
        const auto began=std::chrono::steady_clock::now();
        qp.init(H,gradient,A,b,C,low,high);qp.solve();
        const char *profile=std::getenv("PROCESS_PATH_PROFILE");
        if(profile && std::strcmp(profile,"1")==0)std::fprintf(stderr,
            "PROCESS_PATH_REFINEMENT_QP {\"variables\":%d,\"equalities\":%d,\"inequalities\":%d,\"seconds\":%.9g,\"status\":%d,\"iterations\":%lld,\"primalResidual\":%.9g,\"dualResidual\":%.9g}\n",
            width,static_cast<int>(A.rows()),static_cast<int>(C.rows()),
            std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count(),static_cast<int>(qp.results.info.status),static_cast<long long>(qp.results.info.iter),qp.results.info.pri_res,qp.results.info.dua_res);
        if(qp.results.info.status==proxsuite::proxqp::QPSolverOutput::PROXQP_MAX_ITER_REACHED)return MK_ERROR_LIMIT;
        if(qp.results.info.status!=proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED || !qp.results.x.allFinite())return MK_ERROR_GENERATION;
        const auto &x=qp.results.x;
        // Recheck in the caller's units rather than trusting a scaled residual.
        const Eigen::VectorXd checkedEquality=A*x;
        for(int i=0;i<A.rows();++i)if(std::abs(checkedEquality[i]-b[i])>1e-6){
            if(profile && std::strcmp(profile,"1")==0)std::fprintf(stderr,"PROCESS_PATH_QP_RECHECK equality=%d residual=%.12g\n",i,checkedEquality[i]-b[i]);
            return MK_ERROR_GENERATION;
        }
        const Eigen::VectorXd checked=C*x;
        for(int i=0;i<C.rows();++i)if(checked[i]<low[i]-1e-7 || checked[i]>high[i]+1e-7){
            if(profile && std::strcmp(profile,"1")==0)std::fprintf(stderr,"PROCESS_PATH_QP_RECHECK inequality=%d value=%.12g lower=%.12g upper=%.12g\n",i,checked[i],low[i],high[i]);
            return MK_ERROR_GENERATION;
        }
        for(uint32_t i=0;i<n;++i) {
            out[i]={};out[i].struct_size=sizeof(out[i]);
            const uint32_t span=i+1<n ? i : i-1;
            for(uint32_t j=0;j<d;++j) {
                out[i].value[j]=x[index(i,j)];out[i].first[j]=x[index(i,j)+1];
                Cubic c(index(span,j),index(span+1,j),s[span+1].s-s[span].s);
                out[i].second[j]=evaluate(c.second[i+1<n ? 0 : 1],x);
            }
        }
        return MK_OK;
    } catch(const std::bad_alloc &) {return MK_ERROR_OUT_OF_MEMORY;}
      catch(...) {return MK_ERROR_GENERATION;}
}
