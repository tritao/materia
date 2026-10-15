package machinekit.robotics;

import cadkit.Shape;
import cadkit.modeling.Part;
import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

typedef ZeroErrAsset = {
  var id:String;
  var sha256:String;
  var solidCount:Int;
  var partitionOutputSpigot:Bool;
  var partitionSolid:Int;
  var shortenMm:Float;
}
typedef ZeroErrOccurrence = {
  var id:String;
  var asset:String;
  var matrix:Array<Float>;
  var ?parent:String;
  var ?statorSolid:Int;
  var ?rotorSolid:Int;
  var ?rotorDetails:Array<Int>;
}
typedef ZeroErrJoint = {var id:String; var parent:String; var child:String; var module:String;}
typedef ZeroErrLayout = {
  var schema:Int;
  var assets:Array<ZeroErrAsset>;
  var occurrences:Array<ZeroErrOccurrence>;
  var joints:Array<ZeroErrJoint>;
  var fixedModuleMounts:Array<{parent:String, child:String}>;
}

/** Mechanically articulated supplier reference, not a commissioned robot.
 * CAD, placements and exterior rotor groups are pinned by the example's layout.
 * No motor ratings, vendor command signs or hardware limits are invented.
 * The owner must call close() after producing a scene or finishing CAD checks.
 */
class ZeroErrReferenceArm extends MachineAssembly {
  public final layout:ZeroErrLayout;
  public final referencePoses:Map<String, AssemblyFrame> = [];
  public final modulePoses:Map<String, AssemblyFrame> = [];
  public final jointFrames:Map<String, AssemblyFrame> = [];
  final shapes:Map<String, Shape> = [];
  final sourceShapes:Map<String, Shape> = [];
  final assetCounts:Map<String, Int> = [];
  final solidComponents:Map<String, ZeroErrCadSolid> = [];
  var closed = false;

  public function new(layoutPath:String, vendorDirectory:String) {
    super();
    layout = decode(sys.io.File.getContent(layoutPath));
    if (layout.schema != 1) throw "Unsupported ZeroErr reference layout";
    try {
      for (asset in layout.assets) {
        var path = vendorDirectory + "/" + asset.id + ".STEP";
        if (!sys.FileSystem.exists(path)) throw "Missing ZeroErr CAD; run zeroerr-arm/prepare-reference.py";
        if (fileDigest(path) != asset.sha256)
          throw 'ZeroErr CAD changed: ${asset.id}';
        var shape = Shape.importStep(path);
        sourceShapes.set(asset.id,shape);
        if (shape.subshapeCount(CadKit.ShapeKind.Solid) != asset.solidCount)
          throw 'Supplier solid count changed: ${asset.id}';
        assetCounts.set(asset.id,asset.solidCount);
      }
      var preparedPath = vendorDirectory + "/repairs.json";
      if (!sys.FileSystem.exists(preparedPath)) throw "Prepare native CAD first: run prepare-reference.py --repair after building";
      {
        var records:Array<Dynamic> = haxe.Json.parse(sys.io.File.getContent(preparedPath));
        for (r in records) {
          var asset = Std.string(r.asset), index = Std.int(r.solid), key = asset + '-solid$index';
          var original = [for (a in layout.assets) if (a.id == asset) a];
          if (original.length != 1 || original[0].sha256 != Std.string(r.sourceSha256) ||
              index < 0 || index >= original[0].solidCount || shapes.exists(key) ||
              r.boundsDeltaMm > 0.02 || r.volumeChangeFraction > 0.001)
            throw "Unreviewed supplier CAD repair";
          var path = vendorDirectory + "/" + key + ".STEP";
          if (fileDigest(path) != Std.string(r.sha256)) throw "Derived supplier CAD changed";
          var shape:Shape;
          if (original[0].partitionOutputSpigot) {
            if (index != original[0].partitionSolid || r.partitionZ != 0 || r.partitionDifferenceSolids != 0)
              throw "Missing or changed output-spigot datum partition";
            var statorPath = vendorDirectory + "/" + key + "-stator.STEP";
            var rotorPath = vendorDirectory + "/" + key + "-rotor.STEP";
            if (fileDigest(statorPath) != Std.string(r.statorSha256) || fileDigest(rotorPath) != Std.string(r.rotorSha256))
              throw "Derived output-spigot partition changed";
            shape = Shape.importStep(statorPath);
            shapes.set(asset + "-spigot",Shape.importStep(rotorPath));
          } else shape = Shape.importStep(path);
          shapes.set(key,shape);
          if (shape.subshapeCount(CadKit.ShapeKind.Solid) != 1) throw "Prepared CAD must contain one solid";
        }
      }
      var derivedPath = vendorDirectory + "/derived-links.json";
      if (sys.FileSystem.exists(derivedPath)) {
        var records:Array<Dynamic> = haxe.Json.parse(sys.io.File.getContent(derivedPath));
        for (record in records) {
          var asset = Std.string(record.asset);
          var originals = [for (a in layout.assets) if (a.id == asset) a];
          if (originals.length != 1 || originals[0].shortenMm != 189 || originals[0].sha256 != record.sourceSha256 ||
              record.removedMm != 189 || record.nearCutY != -130 || record.farCutY != -359 ||
              record.validConnectedSolid != true || record.transitionLengthMm != 40 || record.method != "preserve-ends-and-hollow-loft")
            throw "Unreviewed derived link";
          var path = vendorDirectory + "/" + asset + "-shortened.STEP";
          if (fileDigest(path) != record.sha256) throw "Derived link changed";
          shapes.set(asset+"-solid0",Shape.importStep(path));
        }
      }
      for (asset in layout.assets) if (asset.shortenMm > 0 && !shapes.exists(asset.id+"-solid0"))
        throw "Missing derived hollow link; run prepare-revised.py";
      // Restrict source topology once per unique solid, then share it across
      // joint instances and scene/check passes. Re-exporting otherwise valid
      // STEP solids can introduce fresh round-trip tolerance problems.
      for (asset in layout.assets) for (index in 0...asset.solidCount) {
        var key = asset.id + '-solid$index';
        if (!shapes.exists(key)) {
          var source:Shape = sourceShapes.get(asset.id);
          shapes.set(key,source.subshape(CadKit.ShapeKind.Solid,index));
        }
      }
      for (asset in layout.assets) if (asset.partitionOutputSpigot && !shapes.exists(asset.id + "-spigot"))
        throw "Missing output spigot; rerun the corresponding CAD preparation";
      for (occurrence in layout.occurrences) {
        var pose = matrixFrame(occurrence.matrix);
        var count:Int = cast assetCounts.get(occurrence.asset);
        if (occurrence.statorSolid != null) {
          modulePoses.set(occurrence.id, pose);
          var statorIndex:Int = cast occurrence.statorSolid, rotorIndex:Int = cast occurrence.rotorSolid;
          addSolid(occurrence.id + "-stator", occurrence.asset, statorIndex, pose);
          addSolid(occurrence.id + "-rotor", occurrence.asset, rotorIndex, pose);
          if (shapes.exists(occurrence.asset + "-spigot"))
            addSourceSolid(occurrence.id + "-outputSpigot",occurrence.asset + "-spigot",pose);
          for (index in 0...count) if (index != occurrence.statorSolid && index != occurrence.rotorSolid) {
            var id = occurrence.id + '-detail$index';
            addSolid(id, occurrence.asset, index, pose);
          }
        } else for (index in 0...count)
          addSolid(index == 0 ? occurrence.id : occurrence.id + '-detail$index', occurrence.asset, index, pose);
      }
      // Hardware attached to each module is assigned to its physical side.
      for (occurrence in layout.occurrences) {
        var count:Int = cast assetCounts.get(occurrence.asset);
        if (occurrence.statorSolid != null) {
          if (shapes.exists(occurrence.asset + "-spigot")) fixed(occurrence.id + "-rotor",occurrence.id + "-outputSpigot");
          for (index in 0...count) if (index != occurrence.statorSolid && index != occurrence.rotorSolid)
            fixed(occurrence.id + (occurrence.rotorDetails.indexOf(index) >= 0 ? "-rotor" : "-stator"),
              occurrence.id + '-detail$index');
        } else {
          if (occurrence.parent != "") fixed(occurrence.parent, occurrence.id);
          for (index in 1...count) fixed(occurrence.id, occurrence.id + '-detail$index');
        }
      }
      for (mount in layout.fixedModuleMounts) fixed(mount.parent, mount.child);
      for (joint in layout.joints) {
        var pose:AssemblyFrame = modulePoses.get(joint.module);
        var direction = AssemblyFrames.transformVector(pose, 0, 0, 1);
        var world = AssemblyFrames.alongY(pose.x, pose.y, pose.z, direction.x, direction.y, direction.z);
        jointFrames.set(joint.id, world);
        connectAt(joint.id, "revolute", joint.parent, joint.child, world);
        setPassive(joint.id);
      }
      addMemberConnector("base", "floor", AssemblyFrames.identity());
      // Supplier terminal face is local y=0; its normal points along -Y.
      addMemberConnector("toolFlange", "toolFace", AssemblyFrames.alongY(0, 0, 0, 0, -1, 0));
      exposeConnector("floor", "base", "floor");
      exposeConnector("toolFace", "toolFlange", "toolFace");
    } catch (error:Dynamic) { close(); throw error; }
  }

  static function fileDigest(path:String):String {
    var digest = haxe.crypto.Sha256.make(sys.io.File.getBytes(path)), hex = new StringBuf();
    for (i in 0...digest.length) hex.add(StringTools.hex(digest.get(i),2).toLowerCase());
    return hex.toString();
  }

  static function decode(text:String):ZeroErrLayout {
    var value:Dynamic = haxe.Json.parse(text);
    var assets:Array<ZeroErrAsset> = [];
    var rawAssets:Array<Dynamic> = value.assets;
    for (a in rawAssets) assets.push({id:Std.string(a.id),sha256:Std.string(a.sha256),solidCount:Std.int(a.solidCount),
      shortenMm:a.shortenMm == null ? 0 : a.shortenMm,partitionSolid:a.partitionSolid == null ? 0 : Std.int(a.partitionSolid),partitionOutputSpigot:a.partitionOutputSpigot == true});
    var occurrences:Array<ZeroErrOccurrence> = [];
    var rawOccurrences:Array<Dynamic> = value.occurrences;
    for (o in rawOccurrences) {
      var matrix:Array<Float> = [], rawMatrix:Array<Dynamic> = o.matrix;
      for (v in rawMatrix) matrix.push(v);
      var row:ZeroErrOccurrence = {id:Std.string(o.id),asset:Std.string(o.asset),matrix:matrix};
      if (o.statorSolid != null) {
        row.statorSolid = Std.int(o.statorSolid); row.rotorSolid = Std.int(o.rotorSolid);
        var details:Array<Int> = [], rawDetails:Array<Dynamic> = o.rotorDetails;
        for (v in rawDetails) details.push(Std.int(v));
        row.rotorDetails = details;
      } else row.parent = Std.string(o.parent);
      occurrences.push(row);
    }
    var joints:Array<ZeroErrJoint> = [], rawJoints:Array<Dynamic> = value.joints;
    for (j in rawJoints) joints.push({id:Std.string(j.id),parent:Std.string(j.parent),child:Std.string(j.child),module:Std.string(j.module)});
    var mounts:Array<{parent:String,child:String}> = [], rawMounts:Array<Dynamic> = value.fixedModuleMounts;
    for (m in rawMounts) mounts.push({parent:Std.string(m.parent),child:Std.string(m.child)});
    return {schema:Std.int(value.schema),assets:assets,occurrences:occurrences,joints:joints,fixedModuleMounts:mounts};
  }

  function addSolid(id:String, asset:String, index:Int, pose:AssemblyFrame):Void {
    var key = asset + '-solid$index';
    addSourceSolid(id,key,pose);
  }

  function addSourceSolid(id:String, key:String, pose:AssemblyFrame):Void {
    var component:ZeroErrCadSolid = solidComponents.get(key);
    if (component == null) {
      component = new ZeroErrCadSolid(key,shapes.get(key)); solidComponents.set(key,component);
    }
    addComponent(id, component, pose);
    referencePoses.set(id, pose);
  }

  function fixed(parent:String, child:String):Void
    connectAt("mount-" + child, "fixed", parent, child, referencePoses.get(child));

  function connectAt(id:String, kind:materia.assembly.AssemblyDefinition.AssemblyJointType,
      parent:String, child:String, world:AssemblyFrame):Void {
    addMemberConnector(parent, id + "-parent", AssemblyFrames.compose(AssemblyFrames.inverse(referencePoses.get(parent)), world));
    addMemberConnector(child, id + "-child", AssemblyFrames.compose(AssemblyFrames.inverse(referencePoses.get(child)), world));
    // No hardware joint limits are known from these exterior reference files.
    addMateOnAxis(id, kind, parent, id + "-parent", child, id + "-child", {x:0,y:1,z:0}, 0);
  }

  public static function matrixFrame(m:Array<Float>):AssemblyFrame {
    if (m.length != 12) throw "Reference placement needs a 3x4 matrix";
    return AssemblyFrames.fromRotationMatrix(m[3], m[7], m[11],
      [m[0],m[1],m[2],m[4],m[5],m[6],m[8],m[9],m[10]]);
  }

  public function close():Void {
    if (closed) return;
    closed = true;
    for (shape in shapes) shape.close();
    for (shape in sourceShapes) shape.close();
  }
}

/** One unchanged supplier solid. Material-based mass is a visualization estimate
 * from exterior CAD; it does not account for the purchased module's internals.
 */
private class ZeroErrCadSolid extends MachineComponent {
  final source:Shape;
  public function new(id:String, source:Shape) {
    super("ZEROERR-REFERENCE-" + id, "Supplier reference CAD; exterior-only mass estimate", "aluminium 6061", true);
    this.source = source;
  }
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part
    return new Part(source.cloneShape());
}
