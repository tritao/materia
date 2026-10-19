package machinekit.hobbing;

import cadkit.Shape;
import machinekit.component.ComponentDisplayMesh.ComponentDisplayMeshData;

/** Display tessellation of the same fused solid used for CAD export.
    No independent tooth, core or end-cap construction lives in the renderer. */
class HobMesh {
  public static function build(hob:Hob):ComponentDisplayMeshData {
    var solid=HobSolid.build(hob);
    try {
      var result=fromShape(solid.shape,hob.parameterNumber("surfaceTolerance"));
      solid.close();return result;
    }catch(error:Dynamic){solid.close();throw error;}
  }

  /** Keep CAD face normals at sharp edges. Coincident boundary positions are
      checked geometrically by the mesh tests; normal seams need separate vertices. */
  public static function fromShape(shape:Shape,tolerance:Float):ComponentDisplayMeshData {
    var mesh=shape.tessellate(tolerance);
    return {
      positions:[for(i in 0...mesh.vertexCount*3)mesh.vertices.getDouble(i*8)],
      normals:[for(i in 0...mesh.vertexCount*3)mesh.normals.getDouble(i*8)],
      indices:[for(i in 0...mesh.indexCount)mesh.indices.getInt32(i*4)]
    };
  }
}
