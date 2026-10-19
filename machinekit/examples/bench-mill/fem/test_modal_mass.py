import unittest
from modal_mass import constellation, distributing_equations, aggregate
from frame_solver import cross

class ModalMassChecks(unittest.TestCase):
    def test_constellation_reproduces_full_inertia(self):
        mass=12;centre=[13,-7,31];inertia=[[90,7,-5],[7,80,3],[-5,3,70]]
        points=constellation(mass,centre,inertia)
        self.assertAlmostEqual(sum(m for m,_ in points),mass)
        for i in range(3):
            self.assertAlmostEqual(sum(m*p[i] for m,p in points)/mass,centre[i])
            for j in range(3):
                actual=0
                for m,p in points:
                    r=[p[k]-centre[k] for k in range(3)]
                    actual+=m*((sum(x*x for x in r) if i==j else 0)-r[i]*r[j])
                self.assertAlmostEqual(actual,inertia[i][j],places=9)

    def test_distributing_motion_recovers_rigid_field_without_locking_patch(self):
        nodes={1:(-2,-3,0),2:(2,-3,0),3:(2,3,0),4:(-2,3,0)}
        point=(5,7,11);translation=(.1,.2,-.3);rotation=(.01,-.02,.03)
        values={}
        for n,p in nodes.items():
            spin=cross(rotation,[p[i]-point[i] for i in range(3)])
            for i in range(3): values[n,i+1]=translation[i]+spin[i]
        for n,vector in ((5,translation),(6,rotation)):
            for i in range(3):values[n,i+1]=vector[i]
        equations=distributing_equations(nodes,nodes,point,5,6)
        self.assertEqual(len(equations),6)
        for equation in equations:
            self.assertAlmostEqual(sum(values[n,d]*v for n,d,v in equation),0,places=10)
        self.assertEqual({e[0][0] for e in equations},{5,6})

    def test_nonphysical_inertia_rejected(self):
        with self.assertRaisesRegex(ValueError,'triangle'):
            constellation(1,[0,0,0],[[10,0,0],[0,1,0],[0,0,1]])

if __name__=='__main__': unittest.main()
