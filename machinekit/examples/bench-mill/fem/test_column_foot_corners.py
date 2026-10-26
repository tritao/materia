import unittest

from column_foot_corners import read_stresses, stress_matrix
from compliance import symmetric_eigensystem


class ColumnCornerChecks(unittest.TestCase):
    def test_tensor_components_preserve_principal_stress(self):
        for tensor, expected in [((0, 0, 0, 5, 0, 0), (5, 0, -5)),
                                 ((5, 5, 0, 5, 0, 0), (10, 0, 0)),
                                 ((-4, -4, -4, 0, 0, 0), (-4, -4, -4))]:
            values = [mode['value'] for mode in symmetric_eigensystem(stress_matrix(tensor))]
            for value, wanted in zip(values, expected):
                self.assertAlmostEqual(value, wanted, places=10)

    def table(self):
        return '\n'.join(f'''stresses (elem, integ.pnt.,sxx,syy,szz,sxy,sxz,syz) for set CORNER and time {step}
 17 1 1 2 3 4 5 6

''' for step in (1, 2, 3))

    def test_three_linear_bases_are_complete_and_finite(self):
        tables = read_stresses(self.table())
        self.assertEqual(tables[1][(17, 1)], (1, 2, 3, 4, 5, 6))
        with self.assertRaisesRegex(ValueError, 'Invalid integration-point stress'):
            read_stresses(self.table().replace('1 2 3 4 5 6', 'nan 2 3 4 5 6'))
        with self.assertRaisesRegex(ValueError, 'Unknown stress tensor column order'):
            read_stresses(self.table().replace('sxz,syz', 'syz,sxz'))
        with self.assertRaisesRegex(ValueError, 'Incomplete three-basis'):
            read_stresses(self.table().replace(' 17 1 1 2 3 4 5 6', '', 1))


if __name__ == '__main__':
    unittest.main()
