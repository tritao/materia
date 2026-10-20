"""Independent SDOF step and input-shaper cancellation fixtures."""
import math
from dynamic_error import run_response,shaper


def main():
    zeta=.01;frequency=10.;w=2*math.pi*frequency
    basis=[dict(frequencyHz=frequency,accelerationResidueXY=[[0,0],[0,1]])]
    equilibrium=[[0,0],[0,-1000/w**2]]
    profile=dict(duration=1.,distance=.5,v0=0.,v1=1.,times=[0.,1.],rows=[[0,0,0,1,0],[1,.5,1,1,0]])
    result=run_response(basis,equilibrium,[profile],'none',1,zeta,.0001,count=1)
    expected=1000/w**2*(1+math.exp(-math.pi*zeta/math.sqrt(1-zeta*zeta)))
    if abs(result['peakElasticFollowingMm']/expected-1)>1e-4:raise ValueError('SDOF acceleration step response differs from analytical overshoot')
    for kind in ('ZV','ZVD'):
        pulses=shaper(kind,frequency,zeta);wd=w*math.sqrt(1-zeta*zeta);tail=pulses[-1][0]
        response=sum(weight*complex(math.cos(wd*(tail-t)),math.sin(wd*(tail-t)))*math.exp(-zeta*w*(tail-t)) for t,weight in pulses)
        if abs(response)>1e-12 or abs(sum(weight for _,weight in pulses)-1)>1e-12:raise ValueError('Shaper does not preserve DC and cancel its tuned oscillator')
    print('Dynamic fixtures passed: analytical SDOF step, ZV/ZVD cancellation and unity DC gain')


if __name__=='__main__':main()
