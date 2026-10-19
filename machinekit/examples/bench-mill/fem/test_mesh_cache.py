import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from mesh_cache import cached_mesh

class MeshCacheChecks(unittest.TestCase):
    def test_exact_round_trip_and_corruption_gate(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);manifest=root/'manifest.json';manifest.write_text('{}')
            library=root/'gmsh.so';library.write_text('pinned-library')
            gmsh=SimpleNamespace(__version__='4.12.2',libpath=str(library),logger=SimpleNamespace(get=lambda:[]))
            mesh={'nodes':{1:(1.123456789012345,2.0,3.0)},'elements':{'casting':[(7,(1,)*10)]},'bodyNodes':{'casting':{1}}}
            def build(*args):
                output=args[-1];output.mkdir();(output/'frame.msh').write_text('exact mesh bytes')
                return mesh
            cache=root/'cache'
            with patch('mesh_cache.build_mesh',side_effect=build) as builder:
                first=cached_mesh(gmsh,{},manifest,{'a':1},40,root/'first',cache)
                second=cached_mesh(gmsh,{},manifest,{'a':1},40,root/'second',cache)
                self.assertEqual(builder.call_count,1)
                self.assertEqual(first['nodes'],second['nodes'])
                self.assertEqual(first['bodyNodes'],second['bodyNodes'])
                self.assertEqual((root/'first/frame.msh').read_bytes(),(root/'second/frame.msh').read_bytes())
                data=next(cache.glob('*/mesh.json'));data.write_text(data.read_text()+' ')
                with self.assertRaisesRegex(ValueError,'Corrupted'):
                    cached_mesh(gmsh,{},manifest,{'a':1},40,root/'third',cache)

if __name__=='__main__':unittest.main()
