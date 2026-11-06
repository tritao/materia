"""Independent parallel-axis and non-physical tensor checks."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mount_mass import aggregate, tensor


def body(name, mass, centre, inertia=None):
    return dict(id=name, massKg=mass, centreOfMassMm=dict(zip('xyz', centre)),
                centroidalInertiaKgMm2World=inertia or dict(xx=1., yy=1., zz=1., xy=0., xz=0., yz=0.))


class MountMassTests(unittest.TestCase):
    def test_spatial_mass_parallel_axis_and_products(self):
        result = aggregate([body('a', 2., (1., 2., 0.)), body('b', 2., (-1., -2., 0.))])
        self.assertEqual(result['massKg'], 4.)
        self.assertEqual(result['centreOfMassMm'], dict(x=0., y=0., z=0.))
        self.assertEqual(result['centroidalInertiaKgMm2World'],
                         [[18., -8., 0.], [-8., 6., 0.], [0., 0., 22.]])

    def test_positive_diagonal_does_not_make_a_valid_tensor(self):
        with self.assertRaises(ValueError):
            tensor(body('nonphysical', 1., (0., 0., 0.),
                        dict(xx=1., yy=1., zz=1., xy=2., xz=0., yz=0.)))
        with self.assertRaises(ValueError):
            aggregate([body('negative mass', -1., (0., 0., 0.))])


if __name__ == '__main__':
    unittest.main()
