#ifndef MOTIONKIT_OFFSET_WRIST_POLYNOMIAL_H
#define MOTIONKIT_OFFSET_WRIST_POLYNOMIAL_H

// Internal coefficient prototype. Not an inverse solver or registered family.
// Coefficients are indexed [base half-angle power][final-wrist power].
#include <array>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motionkit_offset {
struct Polynomial {
    static constexpr int extent = 9;
    std::array<std::array<long double, extent>, extent> coefficients{};
    int base_degree = 0, wrist_degree = 0;
    Polynomial(long double scalar = 0) { coefficients[0][0] = scalar; }
    static Polynomial monomial(int base, int wrist, long double scalar = 1) {
        if (base < 0 || wrist < 0 || base >= extent || wrist >= extent)
            throw std::overflow_error("Offset polynomial degree exceeds storage");
        Polynomial result;
        result.base_degree = base; result.wrist_degree = wrist;
        result.coefficients[base][wrist] = scalar;
        return result;
    }
};
inline Polynomial operator+(const Polynomial &a, const Polynomial &b) {
    Polynomial result;
    result.base_degree = a.base_degree > b.base_degree ? a.base_degree : b.base_degree;
    result.wrist_degree = a.wrist_degree > b.wrist_degree ? a.wrist_degree : b.wrist_degree;
    for (int i=0;i<Polynomial::extent;++i)
        for (int j=0;j<Polynomial::extent;++j)
            result.coefficients[i][j]=a.coefficients[i][j]+b.coefficients[i][j];
    return result;
}
inline Polynomial operator-(const Polynomial &a, const Polynomial &b) {
    Polynomial negative=b;
    for (auto &row:negative.coefficients) for (auto &value:row) value=-value;
    return a+negative;
}
inline Polynomial operator*(const Polynomial &a, const Polynomial &b) {
    Polynomial result;
    for (int i=0;i<=a.base_degree;++i) for(int j=0;j<=a.wrist_degree;++j) {
        if(a.coefficients[i][j]==0) continue;
        for(int k=0;k<=b.base_degree;++k) for(int l=0;l<=b.wrist_degree;++l) {
            if(b.coefficients[k][l]==0) continue;
            if(i+k>=Polynomial::extent || j+l>=Polynomial::extent)
                throw std::overflow_error("Offset polynomial degree exceeds storage");
            result.coefficients[i+k][j+l]+=a.coefficients[i][j]*b.coefficients[k][l];
            if(i+k>result.base_degree) result.base_degree=i+k;
            if(j+l>result.wrist_degree) result.wrist_degree=j+l;
        }
    }
    return result;
}
using Vector = std::array<Polynomial,3>;
inline Polynomial dot(const Vector &a,const Vector &b) {
    Polynomial result;
    for(int i=0;i<3;++i) result=result+a[i]*b[i];
    return result;
}
struct Constraints { Polynomial lateral, length; };

// Dimensions: a1,a2,b,c1,c2,c3,c4,middle-axis offset. Rotation: row-major.
inline Constraints constraints(const std::array<long double,9> &rotation,
                               const std::array<long double,3> &position,
                               const std::array<long double,8> &dimensions,
                               bool reciprocal_base=false,bool reciprocal_wrist=false) {
    for(auto value:rotation) if(!std::isfinite(value))
        throw std::invalid_argument("Offset rotation must be finite");
    for(auto value:position) if(!std::isfinite(value))
        throw std::invalid_argument("Offset position must be finite");
    for(auto value:dimensions) if(!std::isfinite(value))
        throw std::invalid_argument("Offset dimensions must be finite");
    if(dimensions[4]<=0 || dimensions[5]<=0 || dimensions[6]<0)
        throw std::invalid_argument("Offset arm lengths must be positive and flange length nonnegative");
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) {
        long double product=0;
        for(int k=0;k<3;++k) product+=rotation[3*k+i]*rotation[3*k+j];
        if(!std::isfinite(product) || std::abs(product-(i==j ? 1.L : 0.L))>1e-10L)
            throw std::invalid_argument("Offset rotation must be orthonormal");
    }
    const auto determinant=rotation[0]*(rotation[4]*rotation[8]-rotation[5]*rotation[7])
        -rotation[1]*(rotation[3]*rotation[8]-rotation[5]*rotation[6])
        +rotation[2]*(rotation[3]*rotation[7]-rotation[4]*rotation[6]);
    if(!std::isfinite(determinant) || std::abs(determinant-1)>1e-10L)
        throw std::invalid_argument("Offset rotation must be proper");
    const auto a1=dimensions[0],a2=dimensions[1],b=dimensions[2],c1=dimensions[3];
    const auto c2=dimensions[4],c3=dimensions[5],c4=dimensions[6],d=dimensions[7];
    const auto u=Polynomial::monomial(1,0),t=Polynomial::monomial(0,1);
    const auto D=Polynomial(1)+u*u,E=Polynomial(1)+t*t;
    const auto base_cos=reciprocal_base ? u*u-Polynomial(1) : Polynomial(1)-u*u;
    const auto wrist_cos=reciprocal_wrist ? t*t-Polynomial(1) : Polynomial(1)-t*t;
    const Vector xn={base_cos,Polynomial(2)*u,Polynomial(0)};
    const Vector yn={Polynomial(0)-Polynomial(2)*u,base_cos,Polynomial(0)};
    Vector mn,origin,wn;
    for(int i=0;i<3;++i) {
        mn[i]=Polynomial(rotation[3*i])*Polynomial(2)*t+Polynomial(rotation[3*i+1])*wrist_cos;
        origin[i]=Polynomial(position[i]-c4*rotation[3*i+2]);
        wn[i]=E*origin[i]-Polynomial(d)*mn[i];
    }
    const auto wxn=dot(wn,xn);
    const auto Xn=wxn-Polynomial(a1)*D*E,Zn=D*(wn[2]-Polynomial(c1)*E);
    const auto Mn=dot(mn,xn),Nn=D*mn[2];
    const auto Ln=D*(E*(dot(origin,origin)+Polynomial(d*d-b*b+a1*a1+c1*c1+a2*a2+c3*c3-c2*c2))
                        -Polynomial(2*d)*dot(origin,mn)-Polynomial(2*c1)*wn[2])-Polynomial(2*a1)*wxn;
    const auto Kn=(Polynomial(a2)*Xn+Polynomial(c3)*Zn)*Mn-(Polynomial(c3)*Xn-Polynomial(a2)*Zn)*Nn;
    Constraints result{dot(wn,yn)-Polynomial(b)*D*E,Ln*Ln*(Mn*Mn+Nn*Nn)-Polynomial(4)*Kn*Kn};
    for(const auto *poly:{&result.lateral,&result.length})
        for(const auto &row:poly->coefficients) for(auto value:row)
            if(!std::isfinite(value)) throw std::overflow_error("Offset coefficient arithmetic overflow");
    return result;
}

inline std::array<long double,9> at_wrist(const Polynomial &polynomial,long double wrist) {
    if(!std::isfinite(wrist)) throw std::invalid_argument("Offset wrist coordinate must be finite");
    std::array<long double,9> result{};
    for(int i=0;i<9;++i) {
        for(int j=8;j>=0;--j) result[i]=result[i]*wrist+polynomial.coefficients[i][j];
        if(!std::isfinite(result[i])) throw std::overflow_error("Offset evaluation arithmetic overflow");
    }
    return result;
}

// Evaluate the formal degree-(2,8) resultant. A leading-degree drop can make
// this zero without a common finite root; root recovery must check that case.
inline long double resultant_at_wrist(const Constraints &constraints,long double wrist) {
    auto f=at_wrist(constraints.lateral,wrist),g=at_wrist(constraints.length,wrist);
    std::array<std::array<long double,10>,10> matrix{};
    long double f_scale=0,g_scale=0;
    for(int i=0;i<=2;++i) f_scale=std::max(f_scale,std::abs(f[i]));
    for(int i=0;i<=8;++i) g_scale=std::max(g_scale,std::abs(g[i]));
    if(f_scale==0 || g_scale==0) return 0;
    for(int row=0;row<8;++row) for(int i=0;i<=2;++i) matrix[row][row+i]=f[2-i]/f_scale;
    for(int row=0;row<2;++row) for(int i=0;i<=8;++i) matrix[8+row][row+i]=g[8-i]/g_scale;
    long double determinant=1;
    for(int column=0;column<10;++column) {
        int pivot=column;
        for(int row=column+1;row<10;++row)
            if(std::abs(matrix[row][column])>std::abs(matrix[pivot][column])) pivot=row;
        if(matrix[pivot][column]==0) return 0;
        if(pivot!=column) {std::swap(matrix[pivot],matrix[column]);determinant=-determinant;}
        determinant*=matrix[column][column];
        for(int row=column+1;row<10;++row) {
            const auto multiplier=matrix[row][column]/matrix[column][column];
            for(int i=column+1;i<10;++i) matrix[row][i]-=multiplier*matrix[column][i];
        }
    }
    for(int i=0;i<8;++i) determinant*=f_scale;
    determinant*=g_scale*g_scale;
    if(!std::isfinite(determinant)) throw std::overflow_error("Offset resultant arithmetic overflow");
    return determinant;
}
} // namespace motionkit_offset
#endif
