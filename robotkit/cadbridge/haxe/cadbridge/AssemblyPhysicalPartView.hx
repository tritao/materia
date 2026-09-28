package cadbridge;

import cadkit.ConvexHullVertices;
import materia.project.SceneArtifact.SceneArtifactData;
import cadbridge.AssemblySimulationBridge.AssemblyPhysicalData;
import cadbridge.AssemblySimulationBridge.AssemblyPhysicalPart;

/** One physical-part view shared by simulation and future collision planning. */
class AssemblyPhysicalPartView {
  public static function fromSceneArtifact(scene:SceneArtifactData):AssemblyPhysicalData {
    if (scene == null || !Math.isFinite(scene.metresPerUnit) ||
        scene.metresPerUnit <= 0.0 || scene.parts == null)
      throw "Physical-part view needs a valid scene artifact";
    var parts:Array<AssemblyPhysicalPart> = [];
    for (part in scene.parts) {
      if (part.materialId == null || part.materialDensity == null ||
          part.volume == null || part.centerOfMass == null || part.inertia == null)
        throw 'Physical-part view needs resolved material and mass for "${part.id}"';
      var hull = ConvexHullVertices.fromMesh(part.vertices, part.vertexCount);
      parts.push({id: part.id, materialId: part.materialId, volume: cast part.volume,
        centerOfMass: part.centerOfMass.copy(), inertia: part.inertia.copy(),
        density: cast part.materialDensity, collisionHull: hull});
    }
    return {metresPerUnit: scene.metresPerUnit, parts: parts};
  }
}
