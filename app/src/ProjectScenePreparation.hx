package app;

import materia.project.SceneArtifact.SceneArtifactPart;
import materia.project.MaterialLibrary;
import materia.project.MeshMassProperties;
import cadbridge.AssemblySimulationBridge.AssemblyPhysicalPart;

/** Portable render and physics preparation; owns no files or native geometry handles. */
class ProjectScenePreparation {
  public static function prepare(components:Array<SceneArtifactPart>, scale:Float,
      ?control:ProjectLoadControl):Array<PreparedProjectPart> {
    var prepared:Array<PreparedProjectPart> = [];
    var physicsSeconds = 0.0;
    for (component in components) {
      if (control != null) control.throwIfCancelled();
      var label = component.name;
      var minimum = [1e300, 1e300, 1e300], maximum = [-1e300, -1e300, -1e300];
      for (vertex in 0...component.vertexCount) for (axis in 0...3) {
        var coordinate = component.vertices.getDouble(vertex * 24 + axis * 8);
        if (!Math.isFinite(coordinate)) throw 'Project component "$label" has a non-finite vertex';
        minimum[axis] = Math.min(minimum[axis], coordinate);
        maximum[axis] = Math.max(maximum[axis], coordinate);
      }
      var geometry = PortableCadGeometry.prepare(component, minimum, maximum, scale);
      var physicsStarted = Sys.time();
      var properties = MeshMassProperties.compute(component.vertices, component.indices);
      var materialId = component.materialId == null ? "neutral" : component.materialId;
      var collision = cadkit.ConvexHullVertices.safeFromMesh(component.vertices,
        component.vertexCount, 0.0005 / scale);
      var physical:AssemblyPhysicalPart = {id: component.id, materialId: materialId,
        collisionHull: collision.vertices, collisionWarning: collision.warning,
        collisionErrorRatio: collision.errorRatio,
        volume: component.volume == null ? properties.volume : component.volume,
        centerOfMass: component.centerOfMass == null ? properties.centerOfMass : component.centerOfMass.copy(),
        inertia: component.inertia == null ? properties.inertia : component.inertia.copy(),
        density: component.materialDensity == null
          ? MaterialLibrary.require(materialId).physical.density : component.materialDensity};
      physicsSeconds += Sys.time() - physicsStarted;
      prepared.push({id: component.id, geometry: geometry, physical: physical});
    }
    if (control != null) control.measure("Physics preparation (within geometry preparation)", physicsSeconds);
    return prepared;
  }
}
