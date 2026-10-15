import machinekit.robotics.ZeroErrReferenceArm;
import machinekit.assembly.AssemblyPreview;
import machinekit.assembly.PosedParts;
import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import cadkit.modeling.Part;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.SceneArtifact;

class ZeroErrArmPreview {
  public static final DIRECTORY = "machinekit/examples/zeroerr-arm";
  public static function make():ZeroErrReferenceArm
    return new ZeroErrReferenceArm(DIRECTORY + "/reference-layout.json", DIRECTORY + "/build/vendor");

  /** Independent world-axis product of exponentials, using Rodrigues rotations. */
  static function expected(arm:ZeroErrReferenceArm, id:String, q:Array<Float>):Array<Float> {
    var frame:AssemblyFrame = arm.referencePoses.get(id);
    // An off-axis local point also detects an incorrect wrist roll or orientation.
    var point = AssemblyFrames.transformPoint(frame,17,-11,7);
    var out = [point.x, point.y, point.z];
    // Only the tree ancestors of this member act on its reference point.
    var model = new AssemblyModel("mm"); arm.addTo(model, "");
    var joints = model.definition("reference-fk").joints;
    var active:Map<String, Bool> = [], cursor = id;
    while (cursor != "base") {
      var found = false;
      for (joint in joints) if (joint.child == cursor) {
        if (joint.type == "revolute") active.set(joint.id, true);
        cursor = joint.parent; found = true; break;
      }
      if (!found) throw "Reference arm has a disconnected tree";
    }
    for (reverse in 0...6) {
      var i = 5 - reverse, name = 'j${i + 1}';
      if (!active.exists(name)) continue;
      var f:AssemblyFrame = arm.jointFrames.get(name);
      var a = AssemblyFrames.transformVector(f, 0, 1, 0);
      var x = out[0] - f.x, y = out[1] - f.y, z = out[2] - f.z;
      var c = Math.cos(q[i]), s = Math.sin(q[i]), d = a.x*x + a.y*y + a.z*z;
      out = [f.x + x*c + (a.y*z-a.z*y)*s + a.x*d*(1-c),
        f.y + y*c + (a.z*x-a.x*z)*s + a.y*d*(1-c),
        f.z + z*c + (a.x*y-a.y*x)*s + a.z*d*(1-c)];
    }
    return out;
  }

  static function near(a:Float, b:Float, label:String, tolerance:Float = 1e-5):Void
    if (!Math.isFinite(a) || Math.abs(a-b) > tolerance) throw '$label: $a != $b';

  public static function check(arm:ZeroErrReferenceArm):Void {
    Sys.println("Checking reference assembly and supplier solids"); Sys.stdout().flush();
    arm.validate();
    var model = new AssemblyModel("mm"); arm.addTo(model, "");
    var definition = model.definition("zeroerr-5kg");
    var moving = [for (j in definition.joints) if (j.type == "revolute") j];
    if (moving.length != 6) throw "ZeroErr arm must have six revolute joints";
    for (j in moving) if (j.passive != true || j.driven == true)
      throw "Uncommissioned reference joints must be passive";
    var state = new AssemblyState(definition);
    for (entry in arm.components()) {
      var actual = state.worldPose(entry.id);
      var ref:AssemblyFrame = arm.referencePoses.get(entry.id);
      near(actual.x, ref.x, entry.id + " zero x"); near(actual.y, ref.y, entry.id + " zero y");
      near(actual.z, ref.z, entry.id + " zero z");
      var a = AssemblyFrames.toRotationMatrix(actual), b = AssemblyFrames.toRotationMatrix(ref);
      for (i in 0...9) near(a[i], b[i], entry.id + " zero orientation");
    }
    PosedParts.scope(parts -> {
      for (entry in arm.components()) {
        var part = parts.local(entry.component);
        if (!part.valid() || part.solidCount() != 1) throw 'Invalid supplier solid: ${entry.id}';
      }
      // Actual supplier geometry must reproduce the complete supplier assembly bounds.
      var whole = new Part(cadkit.Shape.importStep(DIRECTORY + "/build/vendor/assembly.STEP"));
      var supplied = PosedParts.boxOf(whole); whole.close();
      var low = [Math.POSITIVE_INFINITY,Math.POSITIVE_INFINITY,Math.POSITIVE_INFINITY];
      var high = [Math.NEGATIVE_INFINITY,Math.NEGATIVE_INFINITY,Math.NEGATIVE_INFINITY];
      for (entry in arm.components()) {
        var placed = parts.posed(entry.component, state.worldPose(entry.id));
        var box = PosedParts.boxOf(placed); placed.close();
        var l = [box.minX,box.minY,box.minZ], h = [box.maxX,box.maxY,box.maxZ];
        for (i in 0...3) { low[i] = Math.min(low[i],l[i]); high[i] = Math.max(high[i],h[i]); }
      }
      for (i in 0...3) {
        near(low[i],[supplied.minX,supplied.minY,supplied.minZ][i],"Supplier bounds minimum",0.02);
        near(high[i],[supplied.maxX,supplied.maxY,supplied.maxZ][i],"Supplier bounds maximum",0.02);
      }
    });
    var poses:Array<Array<Float>> = [[0,0,0,0,0,0], ready(), [0.2,-0.4,0.7,0.2,-0.3,0.4]];
    Sys.println("Checking independent six-axis kinematics"); Sys.stdout().flush();
    for (i in 0...6) { var q = [0.0,0,0,0,0,0]; q[i] = 0.3; poses.push(q); }
    for (q in poses) {
      for (i in 0...6) state.setJoint('j${i+1}',q[i]);
      for (id in ["toolFlange","joint1-stator","joint1-rotor","joint2-stator","joint2-rotor",
          "joint1-outputSpigot","joint2-outputSpigot","joint3-outputSpigot",
          "upperArm","elbowHousing","forearm","joint5-detail1"]) {
        var actual = AssemblyFrames.transformPoint(state.worldPose(id),17,-11,7), wanted = expected(arm,id,q);
        near(actual.x,wanted[0],id+" FK x"); near(actual.y,wanted[1],id+" FK y"); near(actual.z,wanted[2],id+" FK z");
      }
    }
    Sys.println('ZeroErr reference: ${arm.components().length} valid source-derived solids, exact zero placements and assembly bounds, six articulated joints, independent FK at ${poses.length} poses');
  }

  public static function ready():Array<Float> return [0.2,-0.8,1.0,0.1,-0.6,0.2];

  /** Exact overlap observations; positive volumes are reported, never silently exempted. */
  static function clearance(arm:ZeroErrReferenceArm, screen:Bool = false):Void {
    var model = new AssemblyModel("mm"); arm.addTo(model, "");
    var state = new AssemblyState(model.definition("zeroerr-clearance"));
    // Structural solids and output flanges; supplier fasteners are separately
    // retained in the model but are not a thread-fit validation by this report.
    var ids = ["base","joint1-stator","joint1-rotor","joint1-outputSpigot","joint2-stator","joint2-rotor","joint2-outputSpigot","upperArm",
      "joint3-stator","joint3-rotor","joint3-outputSpigot","elbowOutputAdapter","elbowHousingAdapter","elbowHousing","forearm",
      "joint4-stator","joint4-rotor","joint5-stator","joint5-rotor","joint6-stator","joint6-rotor","toolFlange"];
    var entries = [for (id in ids) [for (e in arm.components()) if (e.id == id) e][0]];
    var report:Array<Dynamic> = [];
    var samples:Array<Array<Float>> = [[0.0,0,0,0,0,0],ready()];
    if (screen) {
      // Synthetic +/-90 degree exploration, explicitly not commissioned limits.
      var primes = [2,3,5,7,11,13];
      for (n in 1...25) {
        var q:Array<Float> = [];
        for (b in primes) {
          var index = n, factor = 1.0, h = 0.0;
          while (index > 0) { factor /= b; h += factor*(index%b); index = Std.int(index/b); }
          q.push((2*h-1)*Math.PI/2);
        }
        samples.push(q);
      }
      for (j in 0...6) for (sign in [-1,1]) {
        var q = ready(); q[j] = sign*Math.PI/2; samples.push(q);
      }
    }
    // Cache fixed-body pair results; moving pairs remain checked at every pose.
    var roots:Map<String,String> = [];
    var joints = model.definition("clearance-roots").joints;
    for (id in ids) {
      var cursor = id, searching = true;
      while (searching) {
        searching = false;
        for (j in joints) if (j.child == cursor && j.type == "fixed") {
          cursor = j.parent; searching = true; break;
        }
      }
      roots.set(id,cursor);
    }
    var fixedVolumes:Map<String,Float> = [];
    PosedParts.scope(parts -> {
      for (q in samples) {
        Sys.println('Checking structural overlap at $q'); Sys.stdout().flush();
        for (i in 0...6) state.setJoint('j${i+1}',q[i]);
        var overlaps:Array<Dynamic> = [];
        var poses = [for (e in entries) state.worldPose(e.id)];
        var boxes = [for (i in 0...entries.length) parts.box(entries[i].component,poses[i])];
        var placed:Array<Null<Part>> = [for (_ in entries) null];
        try {
          for (i in 0...entries.length) for (j in i+1...entries.length) {
            var key = entries[i].id + "/" + entries[j].id;
            var fixedPair = roots.get(entries[i].id) == roots.get(entries[j].id);
            if (fixedPair && fixedVolumes.exists(key)) {
              var v:Float = cast fixedVolumes.get(key);
              if (v > 1e-3) overlaps.push({a:entries[i].id,b:entries[j].id,volumeMm3:v});
              continue;
            }
            if (PosedParts.apart(boxes[i],boxes[j])) {
              if (fixedPair) fixedVolumes.set(key,0);
              continue;
            }
            if (placed[i] == null) placed[i] = parts.posed(entries[i].component,poses[i]);
            if (placed[j] == null) placed[j] = parts.posed(entries[j].component,poses[j]);
            var first:Part = cast placed[i], second:Part = cast placed[j];
            // Face-only contact is legal; it need not construct a solid Part.
            var common = first.shape.common(second.shape);
            var v:Float;
            try {
              v = common.subshapeCount(CadKit.ShapeKind.Solid) == 0 ? 0 : common.volume(1e-8);
              common.close();
            } catch(error:Dynamic) { common.close(); throw error; }
            if (fixedPair) fixedVolumes.set(key,v);
            if (v > 1e-3) overlaps.push({a:entries[i].id,b:entries[j].id,volumeMm3:v});
          }
        } catch(error:Dynamic) {
          for (p in placed) if (p != null) p.close();
          throw error;
        }
        for (p in placed) if (p != null) p.close();
        report.push({joints:q,overlaps:overlaps});
        Sys.println('${overlaps.length} positive overlap observations'); Sys.stdout().flush();
      }
    });
    sys.io.File.saveContent(DIRECTORY+"/build/"+(screen ? "motion-screen.json" : "clearance-report.json"),haxe.Json.stringify(report,null,"  "));
  }

  /** Exterior properties for an explicitly estimated engineering mass model. */
  static function engineering(arm:ZeroErrReferenceArm):Void {
    var model = new AssemblyModel("mm"); arm.addTo(model, "");
    var definition = model.definition("engineering-input");
    var records:Array<Dynamic> = [];
    PosedParts.scope(parts -> {
      for (entry in arm.components()) {
        var p = parts.posed(entry.component,arm.referencePoses.get(entry.id));
        var props = p.massProperties(), c = props.centerOfMass, t = props.inertia;
        records.push({id:entry.id,volumeMm3:props.volume,centroidMm:[c.x,c.y,c.z],
          inertiaMm5:[[t.xx,t.xy,t.xz],[t.xy,t.yy,t.yz],[t.xz,t.yz,t.zz]]});
        p.close();
      }
    });
    var axes:Array<Dynamic> = [];
    for (i in 0...6) {
      var id = 'j${i+1}', f:AssemblyFrame = arm.jointFrames.get(id);
      var a = AssemblyFrames.transformVector(f,0,1,0);
      axes.push({id:id,originMm:[f.x,f.y,f.z],axis:[a.x,a.y,a.z]});
    }
    var tool:AssemblyFrame = arm.referencePoses.get("toolFlange");
    var normal = AssemblyFrames.transformVector(tool,0,-1,0);
    sys.io.File.saveContent(DIRECTORY+"/build/engineering-input.json",haxe.Json.stringify({
      layoutSha256:haxe.crypto.Sha256.encode(sys.io.File.getContent(DIRECTORY+"/reference-layout.json")),
      solids:records,joints:definition.joints,axes:axes,
      toolFaceMm:[tool.x,tool.y,tool.z],toolNormal:[normal.x,normal.y,normal.z]
    },null,"  "));
    Sys.println("Exported exterior engineering properties; supplier actuator masses must replace CAD mass");
  }

  public static function export(arm:ZeroErrReferenceArm):Void {
    Sys.println("Building review scene from supplier CAD"); Sys.stdout().flush();
    var scene:materia.project.SceneArtifact.SceneArtifactData = AssemblyPreview.scene(arm,"zeroerr-erobot5-reference");
    var definition:materia.assembly.AssemblyDefinition = scene.assemblyDefinition;
    var state = new AssemblyState(definition);
    for (i in 0...6) state.setJoint('j${i+1}',ready()[i]);
    scene.assemblyState = state.record();
    var stem = DIRECTORY + "/build/zeroerr5";
    sys.io.File.saveBytes(stem+".scene",SceneArtifact.encode(scene));
    var meshes:Array<Dynamic> = [];
    for (occurrence in definition.occurrences) {
      var pose = state.worldPose(occurrence.id);
      for (part in scene.parts) if (part.id == occurrence.definition) {
        var v:Array<Float> = [];
        for (i in 0...part.vertexCount) {
          var p = AssemblyFrames.transformPoint(pose,part.vertices.getDouble(i*24),
            part.vertices.getDouble(i*24+8),part.vertices.getDouble(i*24+16));
          v.push(p.x);v.push(p.y);v.push(p.z);
        }
        meshes.push({id:occurrence.id,material:part.materialId,vertices:v,
          indices:[for(i in 0...part.indexCount) part.indices.getInt32(i*4)]});
      }
    }
    sys.io.File.saveContent(stem+".mesh.json",haxe.Json.stringify(meshes));
    Sys.println('Exported $stem.scene and $stem.mesh.json');
  }

  public static function run():Void {
    Sys.println("Loading pinned ZeroErr reference CAD"); Sys.stdout().flush();
    var arm = make();
    try {
      check(arm);
      if (Sys.args().indexOf("--clearance") >= 0) clearance(arm);
      if (Sys.args().indexOf("--engineering") >= 0) engineering(arm);
      if (Sys.args().indexOf("--motion-screen") >= 0) clearance(arm,true);
      if (Sys.args().indexOf("--export") >= 0) export(arm);
      arm.close();
    } catch(error:Dynamic) { arm.close(); throw error; }
  }
}

function main():Void ZeroErrArmPreview.run();
