package cadbridge;

import cadkit.ConvexHullVertices;
import materia.project.SceneArtifact.SceneArtifactData;
import cadbridge.AssemblySimulationBridge.AssemblyPhysicalData;
import cadbridge.AssemblySimulationBridge.AssemblyPhysicalPart;

/** One physical-part view shared by simulation and future collision planning. */
class AssemblyPhysicalPartView {
  /** Resolved homogeneous-part mass properties in SI, preserving declared physical overrides. */
  public static function massProperties(part:AssemblyPhysicalPart,metresPerUnit:Float):robotkit.tool.MassProperties {
    if(part==null || !Math.isFinite(metresPerUnit) || metresPerUnit<=0 ||
      !Math.isFinite(part.volume) || part.volume<=0 || !Math.isFinite(part.density) || part.density<=0 ||
      part.centerOfMass==null || part.centerOfMass.length!=3 || part.inertia==null || part.inertia.length!=9)
      throw "Physical mass properties require a valid solid and unit scale";
    var mass=part.physicalMassKg==null ? part.volume*part.density*Math.pow(metresPerUnit,3) : (cast part.physicalMassKg:Float);
    if(!Math.isFinite(mass) || mass<0)throw "Invalid resolved physical mass";
    var center=new robotkit.spatial.Vec3(part.centerOfMass[0]*metresPerUnit,part.centerOfMass[1]*metresPerUnit,part.centerOfMass[2]*metresPerUnit);
    if(mass==0)return new robotkit.tool.MassProperties(0,center,robotkit.spatial.Inertia3.zero());
    var tensor=part.physicalInertiaKgM2;
    if(tensor==null)tensor=[for(value in part.inertia)value*(mass/part.volume)*metresPerUnit*metresPerUnit];
    if(tensor.length!=9)throw "Physical inertia requires nine centroidal tensor entries";
    for(value in tensor)if(!Math.isFinite(value))throw "Physical inertia must be finite";
    for(pair in [[1,3],[2,6],[5,7]])if(Math.abs(tensor[pair[0]]-tensor[pair[1]])>1e-12)throw "Physical inertia must be symmetric";
    return new robotkit.tool.MassProperties(mass,center,
      new robotkit.spatial.Inertia3(tensor[0],tensor[1],tensor[2],tensor[4],tensor[5],tensor[8]));
  }

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
        density: cast part.materialDensity, physicalMassKg: part.physicalMassKg,
        physicalMassDeclared: part.physicalMassDeclared,
        physicalInertiaKgM2: part.physicalInertiaKgM2 == null ? null : part.physicalInertiaKgM2.copy(),
        collisionHull: hull});
    }
    return {metresPerUnit: scene.metresPerUnit, parts: parts};
  }
}
