"""Exact-rational offset-wrist elimination experiment, not production IK.

Run with Python and SymPy. Avoid general rational-expression cancellation by
constructing the common-denominator numerators directly. See the derivation in
motionkit/plans/OFFSET_WRIST_INVERSE.md for reconstruction and degeneracies.
"""
import json
import math
import time
import sys
import sympy as s


def constraints(rotation, position, dimensions, inverse_base=False, inverse_wrist=False):
    u, t = s.symbols('u t')
    a1, a2, b, c1, c2, c3, c4, d = dimensions
    D, E = 1 + u*u, 1 + t*t
    base_cos = u*u-1 if inverse_base else 1-u*u
    wrist_cos = t*t-1 if inverse_wrist else 1-t*t
    xn = s.Matrix([base_cos, 2*u, 0])
    yn = s.Matrix([-2*u, base_cos, 0])
    mn = rotation*s.Matrix([2*t, wrist_cos, 0])
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
    projected_degenerate=math.hypot(M,N)<1e-10
    if projected_degenerate:
        X,Z=float(w.dot(x))-a1,float(w[2])-c1
        A,B=a2*X+c3*Z,c3*X-a2*Z
        L=X*X+Z*Z+a2*a2+c3*c3-c2*c2
        amplitude=math.hypot(A,B)
        if amplitude<1e-12:
            if abs(L)<1e-10:
                raise ValueError('Continuous beta solution family requires parameterized output')
            return []
        ratio=L/(2*amplitude)
        if abs(ratio)>1+1e-10:
            return []
        phase=math.atan2(B,A)
        displacement=math.acos(max(-1,min(1,ratio)))
        betas=[phase-displacement,phase+displacement]
    else:
        betas=[math.atan2(-sign*N,sign*M) for sign in [-1,1]]
    result=[]
    for beta in betas:
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
            result.append({'q':[alpha,q2,q3,q4,q5,gamma],'fk_error':error,
                           'projected_degenerate':projected_degenerate})
    return result


def solve(rotation, position, dimensions):
    solutions=[]
    charts=[]
    for inverse_base in [False,True]:
        for inverse_wrist in [False,True]:
            u,t,F,G=constraints(rotation,position,dimensions,inverse_base,inverse_wrist)
            resultant=s.Poly(s.resultant(F.as_expr(),G.as_expr(),u),t)
            if resultant.is_zero:
                raise ValueError('Identically zero resultant requires degenerate-system isolation')
            factors=[]
            for expression,multiplicity in s.factor_list(resultant.as_expr())[1]:
                factor=s.Poly(expression,t)
                factors.append({'degree':factor.degree(),'multiplicity':multiplicity,
                                'real_roots':int(factor.count_roots(-s.oo,s.oo))})
                for interval,_ in factor.intervals(eps=s.Rational(1,10**20)):
                    t_value=float((interval[0]+interval[1])/2)
                    if abs(t_value)>1+1e-12:
                        continue
                    coefficients=[float(c.subs(t,t_value)) for c in s.Poly(F.as_expr(),u).all_coeffs()]
                    while len(coefficients)>1 and abs(coefficients[0])<1e-12:
                        coefficients.pop(0)
                    if len(coefficients)==3:
                        a,bq,c=coefficients
                        discriminant=bq*bq-4*a*c
                        if discriminant<0:
                            continue
                        roots=[(-bq-math.sqrt(discriminant))/(2*a),(-bq+math.sqrt(discriminant))/(2*a)]
                    elif len(coefficients)==2:
                        roots=[-coefficients[1]/coefficients[0]]
                    elif abs(coefficients[0])<1e-12:
                        raise ValueError('Vanishing lateral equation requires joint-system back-substitution')
                    else:
                        roots=[]
                    for u_value in roots:
                        if abs(u_value)>1+1e-12:
                            continue
                        alpha=2*math.atan2(1,u_value) if inverse_base else 2*math.atan(u_value)
                        gamma=2*math.atan2(1,t_value) if inverse_wrist else 2*math.atan(t_value)
                        for solution in reconstruct(rotation,position,dimensions,alpha,gamma):
                            if any(all(abs(math.atan2(math.sin(a-b),math.cos(a-b)))<1e-7
                                       for a,b in zip(solution['q'],old['q'])) for old in solutions):
                                continue
                            solutions.append(solution)
            charts.append({'inverse_base':inverse_base,'inverse_wrist':inverse_wrist,
                           'resultant_degree':resultant.degree(),'factors':factors})
    return {'solutions':solutions,'charts':charts}


def exact_target(tangents, dimensions):
    def trig(tangent):
        if tangent is None:
            return s.Integer(0),s.Integer(-1)
        value=s.Rational(tangent)
        return 2*value/(1+value*value),(1-value*value)/(1+value*value)
    def rotation(axis,tangent):
        sn,cs=trig(tangent)
        if axis=='z':
            return s.Matrix([[cs,-sn,0],[sn,cs,0],[0,0,1]])
        return s.Matrix([[cs,0,sn],[0,1,0],[-sn,0,cs]])
    matrices=[rotation(axis,t) for axis,t in zip('zyyzyz',tangents)]
    R=s.eye(3)
    for matrix in matrices:
        R=R*matrix
    s2,c2q=trig(tangents[1]);s3,c3q=trig(tangents[2])
    sb,cb=s2*c3q+c2q*s3,c2q*c3q-s2*s3
    a1,a2,b,c1,c2,c3,c4,d=dimensions
    local=s.Matrix([a1+c2*s2+a2*cb+c3*sb,b,c1+c2*c2q-a2*sb+c3*cb])
    sg,cg=trig(tangents[5])
    position=matrices[0]*local+c4*R[:,2]+d*(R*s.Matrix([sg,cg,0]))
    original=[math.pi if t is None else 2*math.atan(float(t)) for t in tangents]
    return R,position,original


def boundary_checks(dimensions):
    records=[]
    for base,wrist,middle_start in [(None,s.Rational(1,5),s.Rational(2,5)),
                                     (s.Rational(1,7),None,s.Rational(2,5)),
                                     (None,None,s.Rational(2,5)),
                                     (s.Rational(1,7),s.Rational(1,5),s.Integer(0)),
                                     (s.Rational(1,7),s.Rational(1,5),None)]:
        tangents=[base,s.Rational(1,3),s.Rational(-1,4),middle_start,s.Rational(1,2),wrist]
        R,position,original=exact_target(tangents,dimensions)
        result=solve(R,position,dimensions)
        recovered=any(all(abs(math.atan2(math.sin(a-b),math.cos(a-b)))<1e-7
                          for a,b in zip(original,item['q'])) for item in result['solutions'])
        assert recovered, {'original':original,'result':result}
        records.append({'base_pi':base is None,'wrist_pi':wrist is None, 'q4_tangent':str(middle_start),
                        'degenerate_reconstructions':sum(item['projected_degenerate'] for item in result['solutions']),
                        'solutions':len(result['solutions']),'original_recovered':recovered,
                        'max_fk_error':max(item['fk_error'] for item in result['solutions'])})
    continuum_dimensions=list(map(s.Rational,['.17','.3','.08','.4','.5','.4','.12','.035']))
    a1,a2,b,c1,c2,c3,c4,d=continuum_dimensions
    continuum_position=s.Matrix([a1,b+d,c1+c4])
    try:
        reconstruct(s.eye(3),continuum_position,continuum_dimensions,0,0)
    except ValueError as error:
        assert 'Continuous beta' in str(error)
        records.append({'continuous_beta_diagnostic':str(error)})
    else:
        raise AssertionError('Continuous solution family was silently converted to finite roots')
    return records


def random_checks(dimensions, count):
    import random
    rng=random.Random(712019)
    records=[]
    for sample in range(count):
        tangents=[s.Rational(rng.randint(-9,9),rng.randint(1,9)) for _ in range(6)]
        R,position,original=exact_target(tangents,dimensions)
        result=solve(R,position,dimensions)
        recovered=any(all(abs(math.atan2(math.sin(a-b),math.cos(a-b)))<1e-7
                          for a,b in zip(original,item['q'])) for item in result['solutions'])
        assert recovered, {'sample':sample,'tangents':list(map(str,tangents)),'result':result}
        records.append({'sample':sample,'tangents':list(map(str,tangents)),
                        'solutions':len(result['solutions']),'original_recovered':recovered,
                        'max_fk_error':max(item['fk_error'] for item in result['solutions'])})
        print('offset-polynomial sample',sample,'solutions',len(result['solutions']),flush=True,file=sys.stderr)
    return records


def main():
    rotation=s.Matrix([[s.Rational(2,15),-s.Rational(2,3),s.Rational(11,15)],
                       [s.Rational(14,15),s.Rational(1,3),s.Rational(2,15)],
                       [-s.Rational(1,3),s.Rational(2,3),s.Rational(2,3)]])
    assert rotation.T*rotation==s.eye(3) and rotation.det()==1
    position=s.Matrix([s.Rational(3,5),s.Rational(1,5),s.Rational(7,10)])
    dimensions=list(map(s.Rational,['.17','-.09','.08','.4','.6','.5','.12','.035']))
    started=time.monotonic()
    result=solve(rotation,position,dimensions)
    if '--check-boundaries' in sys.argv:
        result['boundary_checks']=boundary_checks(dimensions)
    for argument in sys.argv[1:]:
        if argument.startswith('--check-random='):
            result['random_checks']=random_checks(dimensions,int(argument.split('=',1)[1]))
    result['elapsed_seconds']=time.monotonic()-started
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    main()
