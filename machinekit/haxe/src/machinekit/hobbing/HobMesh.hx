package machinekit.hobbing;

import cadkit.Shape;
import machinekit.component.ComponentDisplayMesh.ComponentDisplayMeshData;

/** Display tessellation of the same fused solid used for CAD export.
    No independent tooth, core or end-cap construction lives in the renderer. */
class HobMesh {
  /** Persistent: building the detailed mesh costs about a minute and depends only on the hob's parameters. */
  static final meshes:machinekit.cache.Memo<ComponentDisplayMeshData> = new machinekit.cache.Memo<ComponentDisplayMeshData>("hob-display-mesh", 16, copyMesh)
    .persist(machinekit.cache.CodeIdentity.of("machinekit.hobbing.HobMesh"), machinekit.cache.Codecs.displayMesh,
      machinekit.cache.Codecs.undisplayMesh);

  static function copyMesh(mesh:ComponentDisplayMeshData):ComponentDisplayMeshData return {
    positions: mesh.positions.copy(),
    normals: mesh.normals.copy(),
    indices: mesh.indices.copy()
  };

  public static function build(hob:Hob):ComponentDisplayMeshData return meshes.get(Hob.recipeType().key(hob.values()), () -> compute(hob));

  static function compute(hob:Hob):ComponentDisplayMeshData {
    var solid = HobSolid.build(hob);
    try {
      var result = fromShape(solid.shape, hob.parameterNumber("surfaceTolerance"));
      solid.close();
      return result;
    } catch (error:Dynamic) {
      solid.close();
      throw error;
    }
  }

  /** Keep CAD face normals at sharp edges. Coincident boundary positions are
      checked geometrically by the mesh tests; normal seams need separate vertices. */
  public static function fromShape(shape:Shape, tolerance:Float):ComponentDisplayMeshData {
    var mesh = shape.tessellate(tolerance);
    return {
      positions: [for (i in 0...mesh.vertexCount * 3) mesh.vertices.getDouble(i * 8)],
      normals: [for (i in 0...mesh.vertexCount * 3) mesh.normals.getDouble(i * 8)],
      indices: [for (i in 0...mesh.indexCount) mesh.indices.getInt32(i * 4)]
    };
  }
}
