import unittest
from leveling import offsets_from_tables


class LevelingTests(unittest.TestCase):
    def test_coupled_flexible_seats(self):
        # Four coupled compliant seats; S is a known symmetric positive matrix.
        s = [[2,.2,.1,0],[.2,3,0,.1],[.1,0,2.5,.3],[0,.1,.3,2]]
        loads = [10,20,30,40]
        model = {'supportConstraints':dict.fromkeys('abcd'), 'springNPerMm':100000,
                 'references':{'foot-'+c:{'translation':i+1} for i,c in enumerate('abcd')}}
        tables = {(1,'displacements','SUPPORTS'):{i+1:(0,0,-sum(s[i][j]*loads[j] for j in range(4))*1e-5) for i in range(4)}}
        for j in range(4):
            tables[j+2,'displacements','SUPPORTS']={i+1:(0,0,s[i][j]*1e-5) for i in range(4)}
        offsets, check = offsets_from_tables(model,tables)
        for c, load in zip('abcd',loads):self.assertAlmostEqual(offsets[c],load/100000)
        self.assertEqual(check['symmetryRelativeError'],0)
        tables[2,'displacements','SUPPORTS'][2]=(0,0,1)
        with self.assertRaisesRegex(ValueError,'reciprocity'):offsets_from_tables(model,tables)
