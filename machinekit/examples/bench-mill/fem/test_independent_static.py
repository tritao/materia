import unittest
from independent_static import split_steps


class IndependentStaticTests(unittest.TestCase):
    def test_loads_are_isolated_with_original_time_labels(self):
        header='*HEADING\nfixture\n*NSET,NSET=OBS\n1\n'
        text=header+'*STEP\n** gravity\n*STATIC\n*CLOAD\n1,3,-1000\n*END STEP\n*STEP\n** x\n*STATIC\n*CLOAD,OP=NEW\n1,1,1\n*END STEP\n'
        a,b=split_steps(text)
        self.assertTrue(a.startswith(header));self.assertTrue(b.startswith(header))
        self.assertEqual(a.count('*STEP\n'),1);self.assertEqual(b.count('*STEP\n'),1)
        self.assertNotIn('TOTAL TIME',a+b)
        self.assertIn('*STATIC\n',a)
        self.assertNotIn('-1000',b);self.assertIn('1,1,1',b)
        with self.assertRaisesRegex(ValueError,'ordinary static'):split_steps('*STEP\n*FREQUENCY\n6\n*END STEP\n')
