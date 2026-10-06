"""Exact-rational offset-wrist elimination experiment, not production IK.

Run with Python and SymPy. Avoid general rational-expression cancellation by
constructing the common-denominator numerators directly. See the derivation in
motionkit/plans/OFFSET_WRIST_INVERSE.md for reconstruction and degeneracies.
"""
import json
import math
import time
import sympy as s


def constraints(rotation, position, dimensions):
    u, t = s.symbols('u t')
    a1, a2, b, c1, c2, c3, c4, d = dimensions
    D, E = 1 + u*u, 1 + t*t
    xn = s.Matrix([1-u*u, 2*u, 0])
    yn = s.Matrix([-2*u, 1-u*u, 0])
    mn = rotation*s.Matrix([2*t, 1-t*t, 0])
    origin = position-c4*rotation[:, 2]
    wn = E*origin-d*mn
    wxn = wn.dot(xn)
    Xn, Zn = wxn-a1*D*E, D*(wn[2]-c1*E)
    Mn, Nn = mn.dot(xn), D*mn[2]
    # |w|^2 = |origin|^2 + d^2 - 2*d*origin.dot(m), since |m|=1.
    Ln = D*(E*(origin.dot(origin)+d*d-b*b+a1*a1+c1*c1+a2*a2+c3*c3-c2*c2)
            -2*d*origin.dot(mn)-2*c1*wn[2])-2*a1*wxn
    Kn = (a2*Xn+c3*Zn)*Mn-(c3*Xn-a2*Zn)*Nn
    lateral = s.Poly(wn.dot(yn)-b*D*E, u, t)
    length = s.Poly(Ln*Ln*(Mn*Mn+Nn*Nn)-4*Kn*Kn, u, t)
    return u, t, lateral, length


def reconstruct(rotation, position, dimensions, alpha, gamma):
    a1,a2,b,c1,c2,c3,c4,d = map(float, dimensions)
    def rz(angle):
        c,k=math.cos(angle),math.sin(angle)
        return s.Matrix([[c,-k,0],[k,c,0],[0,0,1]])
    def ry(angle):
        c,k=math.cos(angle),math.sin(angle)
        return s.Matrix([[c,0,k],[0,1,0],[-k,0,c]])
    x=s.Matrix([math.cos(alpha),math.sin(alpha),0]); z=s.Matrix([0,0,1])
    m=rotation*s.Matrix([math.sin(gamma),math.cos(gamma),0])
    w=position-c4*rotation[:,2]-d*m
    M,N=float(m.dot(x)),float(m[2])
    if math.hypot(M,N)<1e-10:
        raise ValueError('Degenerate projected axis requires separate beta isolation')
    result=[]
    for sign in [-1,1]:
        beta=math.atan2(-sign*N,sign*M)
        f=(a2*math.cos(beta)+c3*math.sin(beta))*x+(-a2*math.sin(beta)+c3*math.cos(beta))*z
        v=w-a1*x-c1*z-f
        if abs(float(v.dot(v))-b*b-c2*c2)>1e-8:
            continue
        q2=math.atan2(float(v.dot(x)),float(v[2]));q3=beta-q2
        arm=rz(alpha)*ry(beta);h=arm.T*m
        q4=math.atan2(-float(h[0]),float(h[1]))
        B=rz(-q4)*arm.T*rotation*rz(-gamma)
        q5=math.atan2(float(B[0,2]),float(B[0,0]))
        actual_r=arm*rz(q4)*ry(q5)*rz(gamma)
        local=s.Matrix([a1+c2*math.sin(q2)+a2*math.cos(beta)+c3*math.sin(beta),b,
                        c1+c2*math.cos(q2)-a2*math.sin(beta)+c3*math.cos(beta)])
        actual_p=rz(alpha)*local+c4*actual_r[:,2]+d*(actual_r*s.Matrix([math.sin(gamma),math.cos(gamma),0]))
        error=max(max(abs(float(e)) for e in actual_r-rotation),max(abs(float(e)) for e in actual_p-position))
        if error<1e-8:
            result.append({'q':[alpha,q2,q3,q4,q5,gamma],'fk_error':error})
    return result


def main():
    rotation = s.Matrix([[s.Rational(2,15), -s.Rational(2,3), s.Rational(11,15)],
                         [s.Rational(14,15), s.Rational(1,3), s.Rational(2,15)],
                         [-s.Rational(1,3), s.Rational(2,3), s.Rational(2,3)]])
    assert rotation.T*rotation == s.eye(3) and rotation.det() == 1
    position = s.Matrix([s.Rational(3,5), s.Rational(1,5), s.Rational(7,10)])
    dimensions = list(map(s.Rational, ['.17','-.09','.08','.4','.6','.5','.12','.035']))
    started = time.monotonic()
    u,t,F,G = constraints(rotation, position, dimensions)
    resultant = s.Poly(s.resultant(F.as_expr(), G.as_expr(), u), t)
    factors = []
    solutions = []
    for expression, multiplicity in s.factor_list(resultant.as_expr())[1]:
        factor = s.Poly(expression, t)
        for interval, multiplicity_root in factor.intervals(eps=s.Rational(1,10**20)):
            t_value=float((interval[0]+interval[1])/2)
            coefficients=[float(c.subs(t,t_value)) for c in s.Poly(F.as_expr(),u).all_coeffs()]
            a,bq,c=coefficients
            discriminant=bq*bq-4*a*c
            if discriminant<0:
                continue
            if abs(a)<1e-12:
                raise ValueError('Affine base boundary requires complementary chart')
            for u_value in [(-bq-math.sqrt(discriminant))/(2*a),(-bq+math.sqrt(discriminant))/(2*a)]:
                solutions.extend(reconstruct(rotation,position,dimensions,2*math.atan(u_value),2*math.atan(t_value)))
        factors.append({'degree': factor.degree(), 'multiplicity': multiplicity,
                        'real_roots': int(factor.count_roots(-s.oo, s.oo)),
                        'coefficients': [str(c) for c in factor.all_coeffs()]})
    print(json.dumps({'lateral_degrees':[F.degree(u),F.degree(t)],
                      'length_degrees':[G.degree(u),G.degree(t)],
                      'resultant_degree':resultant.degree(), 'factors':factors,
                      'solutions':solutions, 'elapsed_seconds':time.monotonic()-started}, indent=2))


if __name__ == '__main__':
    main()
