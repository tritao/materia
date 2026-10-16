import materia.project.SceneArtifact;
import cadbridge.AssemblySimulationBridge;
import cadbridge.AssemblyPhysicalPartView;
import robotkit.manipulation.Manipulator;
import motionkit.planner.JointPathSamples;
import motionkit.robot.JointCurveClearance;

/** Continuous synthetic-cycle hull review; no cell or cable qualification. */
class StraightCurveReview {
  static function digest(bytes:haxe.io.Bytes):String {
    var hashed = haxe.crypto.Sha256.make(bytes), result = new StringBuf();
    for (i in 0...hashed.length) result.add(StringTools.hex(hashed.get(i),2).toLowerCase());
    return result.toString();
  }
  public static function run():Void {
    var directory = ReferenceAssetLocation.DIRECTORY + "/../../zeroerr-arm/build/straight";
    var artifactPath = directory + "/zeroerr5-straight80.mtrg";
    var bytes = sys.io.File.getBytes(artifactPath);
    var scene = SceneArtifact.decode(bytes);
    // Omitting savedState deliberately compiles source-zero coordinates, matching
    // the load study. No powered actuator or commissioned joint limit is added.
    var converted = AssemblySimulationBridge.toRobotModel(scene.assemblyDefinition,AssemblyPhysicalPartView.fromSceneArtifact(scene));
    var frames = [for (frame in converted.model.frames) if (frame.name == "toolFlange robot flange") frame];
    if (frames.length != 1) throw "Straight arm needs one semantic flange frame";
    var arm = new Manipulator(converted.model,converted.model.links[0].id,frames[0].id);
    if (arm.group.count() != 6) throw "Straight arm needs six joints";
    for (i in 0...6) if (arm.group.jointIds[i] != 'j${i+1}') throw "Unexpected cycle joint order";
    var configPath = directory + "/../../revised-engineering-config.json";
    var config:Dynamic = haxe.Json.parse(sys.io.File.getContent(configPath));
    var rows:Array<Array<Float>> = config.missionStudy.waypointsRad;
    var speed:Float = config.application.jointSpeedDegS;
    var acceleration:Float = config.application.jointAccelerationDegS2;
    speed *= Math.PI/180; acceleration *= Math.PI/180;
    var knots = [0.0];
    for (i in 1...rows.length) {
      var delta = 0.0;
      for (j in 0...6) delta = Math.max(delta,Math.abs(rows[i][j]-rows[i-1][j]));
      knots.push(knots[i-1]+Math.max(1.0,Math.max(1.875*delta/speed,Math.sqrt(5.773502691896258*delta/acceleration))));
    }
    var zeros = [for (_ in rows) [for (_ in 0...6) 0.0]];
    var curve = new JointPathSamples(knots,rows,zeros,zeros);
    var builder = new robotkit.collision.CollisionSceneBuilder(arm,rows[0],"straight-cycle",0.0005);
    var clearance = builder.clearance(() -> new collisionkit.native.NativeCollisionWorld());
    var proof:JointCurveClearance = null;
    var clear = false;
    var error:Null<String> = null;
    try {
      proof = new JointCurveClearance(clearance,curve,1e-6,false);
      clear = proof.check();
    } catch (failure:Dynamic) { error = Std.string(failure); }
    clearance.world.dispose();
    var report = {status:error != null ? "indeterminate" : clear ? "clear_within_stated_hull_policy" : "clearance_not_proven",
      artifactSha256:digest(bytes),
      configSha256:haxe.crypto.Sha256.encode(sys.io.File.getContent(configPath)),
      waypoints:rows,knotsSeconds:knots,queries:proof == null ? 0 : proof.queries,
      failedSpan:proof == null ? -1 : proof.failedSpan,
      failure:proof == null ? null : proof.failure,error:error,
      clearanceMetres:0.0005,loweringToleranceRad:1e-6,
      scope:"Continuous stop-to-stop quintic; enclosing visual hull per part; standard articulation exclusions (adjacent bodies); no tool body, fixtures, flexible cables or exact CAD swept proof",
      hardwareQualified:false};
    sys.io.File.saveContent(directory+"/continuous-clearance.json",haxe.Json.stringify(report,null,"  "));
    Sys.println(haxe.Json.stringify(report));
    if (error != null) throw error;
  }
}
