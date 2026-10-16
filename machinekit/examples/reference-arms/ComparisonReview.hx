import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactData;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyDefinition;
import cadbridge.AssemblySimulationBridge;
import cadbridge.AssemblyPhysicalPartView;
import robotkit.manipulation.Manipulator;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;
import robotkit.spatial.Quat;
import motionkit.kinematics.Pose3;
import motionkit.kinematics.IkTolerance;
import motionkit.path.PoseMath;

/** Reproducible shared task, geometry/frame checks and portable scene exports. */
class ComparisonReview {
  static final IDS = ["zeroerr","ur5e","fairino5_v6"];
  static function check(value:Bool,message:String):Void {if (!value) throw message;}
  public static function fingerprint(scene:SceneArtifactData):String {
    var bytes = haxe.crypto.Sha256.make(SceneArtifact.encode(scene)), result = new StringBuf();
    for (i in 0...bytes.length) result.add(StringTools.hex(bytes.get(i),2).toLowerCase());
    return result.toString();
  }
  static function task(id:String):Void {
    var scene = ComparisonProject.source(id), sourceHash = fingerprint(scene);
    ComparisonProject.withTool(scene);
    var converted = AssemblySimulationBridge.toRobotModel(scene.assemblyDefinition,AssemblyPhysicalPartView.fromSceneArtifact(scene));
    var frames = [for (frame in converted.model.frames) if (StringTools.endsWith(frame.name," robot flange")) frame];
    check(frames.length == 1,"One flange per source arm");
    var windows:Array<String> = [];
    for (joint in converted.model.joints) if (joint.limits.lower < -1e8 || joint.limits.upper > 1e8) {
      joint.limits.lower = -Math.PI;joint.limits.upper = Math.PI;
      windows.push(joint.id);
    }
    var arm = new Manipulator(converted.model,converted.model.links[0].id,frames[0].id,
      new Transform3(new Vec3(0,0,0.15),Quat.identity()));
    check(arm.group.count()==6,"Six joints per comparison arm");
    var solver = new ReferenceKinematics(arm), tolerance = new IkTolerance(1e-6,1e-6,500,0.005);
    var target = new Pose3(0.45,0.15,0.45,1,0,0,0);
    var initial:Array<Float> = null;
    for (seed in [[0.2,-0.8,1.0,0.1,-0.6,0.2],[0.0,-1.0,1.3,-1.5,-1.0,0.0],
                  [1.0,1.0,-1.0,0.0,1.0,0.0],[-1.0,-1.0,1.0,0.0,-1.0,0.0]]) {
      initial = solver.solvePose(target,seed,tolerance);
      if (initial != null) break;
    }
    check(initial != null,id+" cannot reach comparison task");
    var samples:Array<Array<Float>> = [], maxPosition = 0.0, maxAngle = 0.0, maxJump = 0.0;
    for (i in 0...21) {
      var wanted = new Pose3(0.45+0.001*i,0.15,0.45,1,0,0,0);
      var q = solver.solvePose(wanted,i==0 ? initial : samples[i-1],tolerance);
      check(q != null,id+" lost Cartesian sample "+i);
      var actual = solver.forward(q);
      maxPosition = Math.max(maxPosition,PoseMath.distance(wanted,actual));
      maxAngle = Math.max(maxAngle,PoseMath.angle(wanted,actual));
      check(maxPosition<=1e-6 && maxAngle<=1e-6,id+" Cartesian residual");
      for (j in 0...6) {
        var limits = arm.group.limitsOf(j);
        check(q[j]>=limits.lower-1e-8 && q[j]<=limits.upper+1e-8,id+" joint limit");
        if (i>0) maxJump = Math.max(maxJump,Math.abs(q[j]-samples[i-1][j]));
      }
      samples.push(q);
    }
    check(maxJump<0.1,id+" Cartesian branch jump");
    var builder = new robotkit.collision.CollisionSceneBuilder(arm,samples[0],id,0.0005);
    var world = builder.clearance(() -> new collisionkit.native.NativeCollisionWorld());
    var violations:Array<Dynamic> = [];
    try {
      for (i in 0...samples.length) {
        var found = world.violation(samples[i]);
        if (found != null) violations.push({sample:i,a:found.a,b:found.b,distance:found.distance,required:found.required});
      }
    } catch (error:Dynamic) {world.world.dispose();throw error;}
    world.world.dispose();
    var output = {model:id,sourceFingerprint:sourceHash,samples:samples,planningWindows:windows,
      jointOrder:[for (j in arm.group.jointIds) Std.string(j)],maxPositionError:maxPosition,maxOrientationError:maxAngle,
      maxJointJump:maxJump,numericFallbacks:solver.numericFallbacks,collisionViolations:violations,
      tool:{lengthMetres:0.15,widthMetres:0.04,massKg:1.0,centerOfMassLocalMetres:[0.0,0.0,0.075]},
      task:"Common base-relative TCP (0.45 to 0.47, 0.15, 0.45)m, downward orientation",
      scope:"Sampled source collision hulls plus physical tool envelope; standard articulation exclusions; no fixture or hardware qualification"};
    sys.io.File.saveContent(ComparisonProject.ROOT+"/comparison/"+id+"-task.json",haxe.Json.stringify(output,null,"  "));
    Sys.println(id+" common-tool task: "+samples.length+" samples, "+violations.length+" sampled hull violations");
  }
  static function meshExport(scene:SceneArtifactData,stem:String):Void {
    var state = new cadkit.modeling.AssemblyState(scene.assemblyDefinition,scene.assemblyState);
    var entries:Array<Dynamic> = [];
    for (part in scene.parts) {
      sys.io.File.saveBytes(stem+"/"+part.id.split("/").join("_")+".vertices",part.vertices);
      sys.io.File.saveBytes(stem+"/"+part.id.split("/").join("_")+".indices",part.indices);
    }
    for (occurrence in (cast scene.assemblyDefinition:AssemblyDefinition).occurrences) {
      var part = [for (p in scene.parts) if (p.id == occurrence.definition) p][0];
      check(part != null,"Comparison occurrence lacks geometry");
      entries.push({id:occurrence.id,stem:part.id.split("/").join("_"),color:[part.red,part.green,part.blue],
        pose:state.worldPose(occurrence.id)});
    }
    sys.io.File.saveContent(stem+"/meshes.json",haxe.Json.stringify(entries));
  }
  static function validate(scene:SceneArtifactData,mounting:String):Void {
    // Validate the saved binary, rather than only the in-memory construction.
    var decoded = SceneArtifact.decode(SceneArtifact.encode(scene));
    var state = new cadkit.modeling.AssemblyState(decoded.assemblyDefinition,decoded.assemblyState);
    for (id in IDS) {
      var source = ComparisonProject.withTool(ComparisonProject.source(id));
      var original = new cadkit.modeling.AssemblyState(source.assemblyDefinition);
      var moving = [for (j in (cast source.assemblyDefinition:AssemblyDefinition).joints) if (j.type != "fixed") j];
      for (sample in [0,20]) {
      var q = ComparisonProject.taskSamples(id)[sample];
      for (i in 0...moving.length) original.setJoint(moving[i].id,q[i]);
      for (i in 0...moving.length) state.setJoint(id+"/"+moving[i].id,q[i]);
      var incoming:Map<String,Bool> = [];
      for (joint in (cast source.assemblyDefinition:AssemblyDefinition).joints) incoming.set(joint.child,true);
      var roots = [for (o in (cast source.assemblyDefinition:AssemblyDefinition).occurrences) if (!incoming.exists(o.id)) o];
      check(roots.length==1,"Comparison source must have one tree root");
      var root = roots[0];
      var rootPose = state.worldPose(id+"/"+root.id);
      var sourceRoot = original.worldPose(root.id);
      var scaledRoot = {x:sourceRoot.x*source.metresPerUnit,y:sourceRoot.y*source.metresPerUnit,z:sourceRoot.z*source.metresPerUnit,
        qx:sourceRoot.qx,qy:sourceRoot.qy,qz:sourceRoot.qz,qw:sourceRoot.qw};
      // Independent fixture specification: display stations along global X.
      var station = IDS.indexOf(id)-1;
      var mount = switch mounting {
        case "floor": {x:station*1.2,y:0.0,z:0.0,qx:0.0,qy:0.0,qz:0.0,qw:1.0};
        case "wall": {x:station*1.2,y:0.0,z:1.1,qx:Math.sqrt(0.5),qy:0.0,qz:0.0,qw:Math.sqrt(0.5)};
        case "ceiling": {x:station*1.2,y:0.0,z:1.5,qx:1.0,qy:0.0,qz:0.0,qw:0.0};
        default: throw "Unknown mounting";
      };
      var expectedRoot = AssemblyFrames.compose(mount,scaledRoot);
      check(Math.abs(rootPose.x-expectedRoot.x)<1e-9 && Math.abs(rootPose.y-expectedRoot.y)<1e-9 && Math.abs(rootPose.z-expectedRoot.z)<1e-9,"Root mounting translation");
      var tip = AssemblyFrames.transformPoint(state.worldPose(id+"/comparison-tool"),0,0,0.15);
      var wantedTip = AssemblyFrames.transformPoint(mount,0.45+0.001*sample,0.15,0.45);
      check(Math.abs(tip.x-wantedTip.x)<1e-6 && Math.abs(tip.y-wantedTip.y)<1e-6 && Math.abs(tip.z-wantedTip.z)<1e-6,"Saved common TCP placement "+id);
      for (o in (cast source.assemblyDefinition:AssemblyDefinition).occurrences) {
        var before = original.worldPose(o.id);
        var scaled = {x:before.x*source.metresPerUnit,y:before.y*source.metresPerUnit,z:before.z*source.metresPerUnit,
          qx:before.qx,qy:before.qy,qz:before.qz,qw:before.qw};
        var expected = AssemblyFrames.compose(mount,scaled), actual = state.worldPose(id+"/"+o.id);
        var a = AssemblyFrames.transformPoint(actual,0.017,-0.011,0.007),b = AssemblyFrames.transformPoint(expected,0.017,-0.011,0.007);
        check(Math.abs(a.x-b.x)<1e-8 && Math.abs(a.y-b.y)<1e-8 && Math.abs(a.z-b.z)<1e-8,mounting+" merged pose "+o.id);
        var ar = AssemblyFrames.toRotationMatrix(actual),br = AssemblyFrames.toRotationMatrix(expected);
        for (i in 0...9) check(Math.abs(ar[i]-br[i])<1e-8,"Merged orientation differs");
      }
      }
      for (part in source.parts) {
        var matches = [for (p in decoded.parts) if (p.id == id+"/"+part.id) p];
        check(matches.length==1,"Missing saved geometry "+id+"/"+part.id);
        var merged = matches[0];
        check(part.vertices.length==part.vertexCount*24 && merged.vertices.length==merged.vertexCount*24,
          "Unexpected mesh stream size for "+part.id);
        check(part.vertexCount>0,"Empty source geometry "+part.id);
        for (vertex in [0,Std.int(part.vertexCount/2),part.vertexCount-1]) for (axis in 0...3)
          check(Math.abs(merged.vertices.getDouble(vertex*24+axis*8)-part.vertices.getDouble(vertex*24+axis*8)*source.metresPerUnit)<1e-10,"Merged mesh scale");
        if (part.physicalMassKg != null) check(merged.physicalMassKg==part.physicalMassKg,"Merged physical mass "+part.id);
      }
    }
  }
  public static function run():Void {
    var directory = ComparisonProject.ROOT+"/comparison";
    sys.FileSystem.createDirectory(directory);
    for (id in IDS) task(id);
    for (mounting in ["floor","wall","ceiling"]) {
      var scene = ComparisonProject.scene(mounting);
      check(scene.parts.length==82 && (cast scene.assemblyDefinition:AssemblyDefinition).occurrences.length==137,"Unexpected combined geometry count");
      check([for (j in (cast scene.assemblyDefinition:AssemblyDefinition).joints) if (j.type != "fixed") j].length==18,"Unexpected combined joint count");
      validate(scene,mounting);
      sys.io.File.saveBytes(directory+"/"+mounting+".mtrg",SceneArtifact.encode(scene));
      var meshDirectory = directory+"/"+mounting;
      sys.FileSystem.createDirectory(meshDirectory);
      meshExport(scene,meshDirectory);
      Sys.println(mounting+": saved and checked "+scene.parts.length+" geometries, "+(cast scene.assemblyDefinition:AssemblyDefinition).occurrences.length+" occurrences");
    }
    sys.io.File.saveContent(directory+"/validation.json",haxe.Json.stringify({status:"passed",mountings:["floor","wall","ceiling"],
      models:IDS,geometries:82,occurrences:137,movingJoints:18,
      checks:["Saved binary reconstruction","All occurrence positions and rotations versus separate source scenes at both task endpoints",
        "Independent mounting transforms and common TCP placement","Mesh metre scaling and declared SI masses"],
      scope:"Static geometry and common-task kinematics; sampled hull checks include the tool envelope; no cell or dynamic qualification"},null,"  "));
  }
}
