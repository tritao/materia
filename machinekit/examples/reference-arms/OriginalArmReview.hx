import materia.assembly.AssemblyDefinition;
import materia.project.SceneArtifact;
import cadbridge.AssemblySimulationBridge;
import cadbridge.AssemblyPhysicalPartView;
import robotkit.manipulation.Manipulator;
import robotkit.collision.CollisionSceneBuilder;

/** Equal-pose hull diagnostics; synthetic travel is not vendor commissioned travel. */
class OriginalArmReview {
  public static function run():Void {
    var root=ReferenceAssetLocation.DIRECTORY+"/../../zeroerr-arm";
    var probePath=root+"/build/straight/required-probes.json";
    var probes:Array<Array<Float>> = haxe.Json.parse(sys.io.File.getContent(probePath));
    var loadProbePath=root+"/build/original-evaluation/load-probes.json";
    var loadProbes:Array<Array<Float>> = haxe.Json.parse(sys.io.File.getContent(loadProbePath));
    var reports:Array<Dynamic> = [];
    for(id in ["zeroerr-original","zeroerr"]) {
      var source=ComparisonProject.source(id),fingerprint=ComparisonReview.fingerprint(source);
      var definition:AssemblyDefinition=source.assemblyDefinition;
      var moving=[for(j in definition.joints) if(j.type=="revolute") j];
      if(moving.length!=6) throw "Expected six source joints";
      for(j in moving) {
        if(j.passive!=true) throw "Reference arm must remain passive";
        if(id=="zeroerr-original" && (j.limits.lower!=null || j.limits.upper!=null)) throw "Original reference inherited a synthetic restriction";
        j.limits.lower=null;j.limits.upper=null;
      }
      var converted=AssemblySimulationBridge.toRobotModel(definition,AssemblyPhysicalPartView.fromSceneArtifact(source));
      var flange=[for(f in converted.model.frames) if(f.name=="toolFlange robot flange") f][0];
      var arm=new Manipulator(converted.model,converted.model.links[0].id,flange.id);
      var solver=new ReferenceKinematics(arm);
      var builder=new CollisionSceneBuilder(arm,probes[0],id,0.0005);
      var clearance=builder.clearance(() -> new collisionkit.native.NativeCollisionWorld());
      var observations:Array<Dynamic> = [];
      var loadObservations:Array<Dynamic> = [];
      try {
        for(q in probes) {
          var witness=clearance.violation(q);
          observations.push({joints:q,clear:witness==null,witness:witness});
        }
        for(q in loadProbes) {
          var witness=clearance.violation(q);
          loadObservations.push({joints:q,clear:witness==null,witness:witness});
        }
      } catch(error:Dynamic) {clearance.world.dispose();throw error;}
      clearance.world.dispose();
      reports.push({model:id,sourceFingerprint:fingerprint,poses:observations.length,
        ikBackend:solver.exact==null ? "numeric_fallback" : "analytic_available",unsupportedDiagnostic:solver.unsupportedDiagnostic,
        clearPoses:[for(r in observations) if(r.clear==true) r].length,observations:observations,
        loadObservations:loadObservations});
    }
    var result={status:"equal_pose_hull_diagnostics_not_hardware_limits",marginMetres:0.0005,
      probeSha256:haxe.crypto.Sha256.encode(sys.io.File.getContent(probePath)),models:reports,
      loadProbeSha256:haxe.crypto.Sha256.encode(sys.io.File.getContent(loadProbePath)),
      producerSha256:haxe.crypto.Sha256.encode(sys.io.File.getContent(ReferenceAssetLocation.DIRECTORY+"/../OriginalArmReview.hx")),
      scope:"Same 60 source-zero joint configurations; enclosing visual hulls and standard adjacent-body exclusions. Modified-arm restrictions removed only for this diagnostic. No tool or fixtures."};
    sys.io.File.saveContent(root+"/build/original-evaluation/hull-comparison.json",haxe.Json.stringify(result,null,"  "));
    for(r in reports) Sys.println(r.model+": "+r.clearPoses+"/"+r.poses+" hull-clear poses");
  }
}
