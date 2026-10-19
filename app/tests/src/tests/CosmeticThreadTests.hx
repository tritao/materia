package tests;

import app.CadGeometryData;
import app.CadSceneGeometry;
import app.EditorScene;
import app.PerspectiveCamera;
import app.SceneObjectData;
import app.editor.CosmeticThreadMesh;
import app.editor.CosmeticThreadTexture;
import haxe.io.Bytes;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.threads.ThreadSpec;
import cadkit.threads.Threads;
import cadkit.threads.ThreadedFace.ThreadStart;
import materia.project.ThreadSpecData.ThreadHand;
import materia.project.ThreadSpecData.ThreadProfile;
import nativekit.scene.GeometryData;
import nativekit.scene.GeometryData.GeometryStream;
import nativekit.scene.SceneView;
import nativekit.scene.Transform;

/** Thread intervals use the published mesh, and their runtime children follow the owning object. */
@:access(app.EditorScene)
@:access(nativekit.scene.GeometryData)
class CosmeticThreadTests {
  static var checks = 0;
  static function check(value:Bool, message:String):Void {
    checks++;
    if (!value) throw message;
  }
  static function near(a:Float, b:Float, message:String):Void check(Math.abs(a - b) < 5e-7, message + ': $a != $b');
  static function stream(data:GeometryData, semantic:Int):GeometryStream {
    var value = data.stream(semantic);
    if (value == null) throw 'No stream $semantic';
    return value;
  }
  static function pointKey(bytes:Bytes, index:Int):String
    return bytes.getInt32(index * 12) + ":" + bytes.getInt32(index * 12 + 4) + ":" + bytes.getInt32(index * 12 + 8);
  static function sameBytes(a:Bytes, b:Bytes):Bool {
    if (a.length != b.length) return false;
    for (index in 0...a.length) if (a.get(index) != b.get(index)) return false;
    return true;
  }
  static function cylinder(length:Float, hand:ThreadHand = RightHand, starts:Int = 1):CadGeometryData {
    var plain = Part.cylinderSpan(2.5, -30, 0);
    var threaded = new Part(Threads.attach(plain.shape, "cyl.side",
      new ThreadSpec("M6", 6, 1, starts, hand, true), AxisMaximum, length));
    plain.close();
    try {
      var result:CadGeometryData = cast CadSceneGeometry.fromMesh(threaded.shape.tessellate(), threaded.shape);
      threaded.close();
      return result;
    } catch (error:Dynamic) { threaded.close(); throw error; }
  }

  static function mesh():Void {
    var centimetres = new app.CadGeometryData.CadThreadGeometry(cylinder(20).threads[0].source, [0, 0, 0], 0.01);
    near(centimetres.pitch, 0.001, "nominal pitch keeps its millimetre unit independently of artifact coordinates");
    var full = cylinder(30), source = full.subelementGeometry(full.threads[0].source.faceIndex);
    if (source == null) throw "Missing source face";
    var overlay = CosmeticThreadMesh.build(full, full.threads[0]);
    check(overlay.triangleCount() == source.triangleCount(), "unclipped triangles retain the face's triangle count");
    var original = stream(source, 1).bytes, retained = stream(overlay, 1).bytes;
    var sourcePoints:Map<String, Bool> = [];
    for (index in 0...source.vertexCount()) sourcePoints.set(pointKey(original, index), true);
    for (index in 0...overlay.vertexCount())
      check(sourcePoints.exists(pointKey(retained, index)), "unclipped positions equal published Float32 vertices");
    var referenced:Map<Int, Bool> = [];
    for (index in 0...source.triangleCount() * 3) referenced.set(source.indexAt(index), true);
    var count = 0; for (_ in referenced.keys()) count++;
    check(overlay.vertexCount() >= count && overlay.vertexCount() < count + 10, "only the seam duplicates source vertices");
    var uv = stream(overlay, 4).bytes;
    for (triangle in 0...overlay.triangleCount()) {
      var values = [for (corner in 0...3) uv.getFloat(overlay.indexAt(triangle * 3 + corner) * 8)];
      check(Math.max(values[0], Math.max(values[1], values[2])) - Math.min(values[0], Math.min(values[1], values[2])) < 0.1,
        "seam triangles interpolate a short angular interval");
    }
    var right = cylinder(20, RightHand, 2), left = cylinder(20, LeftHand, 2);
    var rightMesh = CosmeticThreadMesh.build(right, right.threads[0]), leftMesh = CosmeticThreadMesh.build(left, left.threads[0]);
    check(rightMesh.triangleCount() == leftMesh.triangleCount(), "hand does not change clipped triangles");
    var rightUv = stream(rightMesh, 4).bytes, leftUv = stream(leftMesh, 4).bytes;
    var rightPoints = stream(rightMesh, 1).bytes;
    var end = 0.0;
    for (index in 0...rightMesh.vertexCount()) {
      near(rightUv.getFloat(index * 8) + leftUv.getFloat(index * 8), 2 * CosmeticThreadMesh.phase("cyl.side"), "left hand mirrors U");
      near(rightUv.getFloat(index * 8 + 4), leftUv.getFloat(index * 8 + 4), "hand preserves V");
      var local = right.threads[0].frame.toLocal(new Vector(rightPoints.getFloat(index * 12),
        rightPoints.getFloat(index * 12 + 4), rightPoints.getFloat(index * 12 + 8)));
      check(local.z >= -1e-8 && local.z <= 0.02000001, "20 mm thread clips inside a 30 mm face");
      end = Math.max(end, local.z);
    }
    near(end, 0.02, "clipped triangles reach the exact thread end");
    check(rightMesh.nativeValue().get_stroke_segment_count() > 0, "partial thread draws an end ring");
    for (triangle in 0...rightMesh.triangleCount()) {
      var low = 1e30, high = -1e30;
      for (corner in 0...3) { var u = rightUv.getFloat(rightMesh.indexAt(triangle * 3 + corner) * 8); low = Math.min(low, u); high = Math.max(high, u); }
      check(high - low < 0.2, "multi-start seams unwrap before multiplying by starts");
    }
    check(CosmeticThreadMesh.phase("bore1:cyl.side") != CosmeticThreadMesh.phase("bore2:cyl.side"), "phase belongs to the face name");
  }

  static function texture():Void {
    var spec = new ThreadSpec("M6", 6, 1).data();
    var normal = CosmeticThreadTexture.pixels(spec, true), color = CosmeticThreadTexture.pixels(spec, false);
    check(normal.length == 87380 && color.length == 87380, "all eight RGBA8 mip levels are supplied");
    var minimum = 255, maximum = 0, normalized = true;
    for (pixel in 0...Std.int(normal.length / 4)) {
      var offset = pixel * 4;
      var x = normal.get(offset) / 127.5 - 1, y = normal.get(offset + 1) / 127.5 - 1, z = normal.get(offset + 2) / 127.5 - 1;
      if (Math.abs(x * x + y * y + z * z - 1) >= 0.02) normalized = false;
      minimum = Std.int(Math.min(minimum, color.get(offset))); maximum = Std.int(Math.max(maximum, color.get(offset)));
    }
    check(normalized, "each mip normal stays normalized");
    check(minimum == 245 && maximum == 255, "roots are slightly darker than the part colour");
    check(!sameBytes(CosmeticThreadTexture.pixels(new ThreadSpec("TR6x1", 6, 1, 1, RightHand, false, null, Trapezoidal).data(), true), normal), "trapezoidal flanks differ from metric flanks");
  }

  static function ballRace():Void {
    for (length in [105.0,237.0,600.0]) {
      var screw = new machinekit.motion.BallScrew(length);
      var part = screw.geometry(), envelope = screw.geometry(Envelope);
      var outside = part.subtract(envelope);
      near(outside.volume(), 0, "detailed shaft remains inside nominal envelope"); outside.close();
      check(part.volume() < envelope.volume(), "turned end details remove material");
      check(Threads.ofShape(envelope.shape).length == 0, "collision envelope has no cosmetic thread");
      envelope.close();
      var threads = Threads.ofShape(part.shape);
      check(threads.length == 1 && threads[0].attribute.faceName == "race:cyl.side", "only central race is threaded");
      near(threads[0].frame.origin.z, 50, "race starts after input journal");
      near(threads[0].length, length - 75, "race ends before output journal");
      check(threads[0].attribute.spec.profile == Ball && !threads[0].attribute.spec.internal, "external rounded ball profile");
      near(threads[0].attribute.spec.pitch, 5, "ball race pitch");
      check(threads[0].attribute.spec.hand == RightHand, "ball race hand");
      var cad:CadGeometryData = cast CadSceneGeometry.fromMesh(part.shape.tessellate(), part.shape);
      var overlay = CosmeticThreadMesh.build(cad, cad.threads[0]);
      check(overlay.triangleCount() > 0, "ball race overlay exists");
      var positions = stream(overlay, 1).bytes;
      for (index in 0...overlay.vertexCount()) {
        var z = positions.getFloat(index * 12 + 8);
        check(z >= 0.05 - 1e-7 && z <= (length - 25) / 1000 + 1e-7, "overlay excludes both journals");
      }
      part.close();
      var exported = machinekit.assembly.AssemblyPreview.part("screw", "Ball screw", screw.geometry(), screw.materialId);
      var saved = materia.project.SceneArtifact.decode(materia.project.SceneArtifact.encode({lengthUnit:"mm",metresPerUnit:0.001,parts:[exported]}));
      var savedThreads = saved.parts[0].threads;
      if (savedThreads == null) throw "Missing saved ball race";
      check(savedThreads.length == 1 && savedThreads[0].spec.profile == Ball, "ball profile survives artifact round trip");
      var generated:Map<String,GeometryData> = []; generated.set("ball-race",cad);
      var scene = new EditorScene([{id:"ball",label:"Ball screw",type:"cad-preview",x:0.0,y:0.0,z:0.0,
        width:0.016,height:0.016,depth:length/1000,collisionEnabled:false,dynamicBody:false,mass:1.0,
        red:0.7,green:0.7,blue:0.7,visible:true,meshSnapshot:"ball-race"}],null,null,generated);
      try {
        scene.configureRenderView(new SceneView(),Transform.identity());
        check(scene.presentation.threads.nodesFor("ball").length == 1, "ball race creates runtime visual child");
      } catch (error:Dynamic) { scene.dispose(); throw error; }
      scene.dispose();
    }
    var spec = new ThreadSpec("BALL16x5",16,5,1,RightHand,false,null,Ball).data();
    var normal = CosmeticThreadTexture.pixels(spec,true);
    check(!sameBytes(normal,CosmeticThreadTexture.pixels(new ThreadSpec("M16x5",16,5).data(),true)), "rounded groove differs from metric flanks");
    for (pixel in 0...Std.int(normal.length/4)) {
      var offset = pixel*4, x = normal.get(offset)/127.5-1, y = normal.get(offset+1)/127.5-1, z = normal.get(offset+2)/127.5-1;
      check(Math.abs(x*x+y*y+z*z-1)<0.02,"ball race mip normals normalized");
    }
  }

  static function ballHardware():Void {
    var nut = new machinekit.motion.BallNut(), body = nut.geometry(), envelope = nut.geometry(Envelope);
    var outside = body.subtract(envelope);near(outside.volume(),0,"nut fittings included in envelope");outside.close();
    near(nut.connector("mountFace").frame.z,45,"nut mounting face retained");
    near(nut.connector("bore").frame.z,17.5,"nut bore datum retained");
    for (point in nut.boltPattern()) {
      var drill = Part.cylinderSpan(2.6,34.9,45.1,point.x,point.y), common = body.intersect(drill);
      near(common.volume(),0,"six flange holes retained");common.close();drill.close();
    }
    for (z in [0.1,44.9]) {
      var tool = Part.cylinderSpan(0.2,z-0.03,z+0.03,10.2,0), common = body.intersect(tool);
      near(common.volume(),0,"seal seat is recessed");common.close();tool.close();
    }
    var bore = Part.cylinderSpan(7.9,-0.1,45.1), common = body.intersect(bore);
    near(common.volume(),0,"nut bore stays clear");common.close();bore.close();body.close();envelope.close();
    var screw = new machinekit.motion.BallScrew(237), shaft = screw.geometry();
    var hardware = screw.retainer(), local = hardware.geometry(), placed = local.translated(new Vector(0,0,machinekit.motion.BallScrewRetainer.STATION));local.close();
    var overlap = shaft.intersect(placed);near(overlap.volume(),0,"retaining hardware clears shaft");overlap.close();
    var support = machinekit.motion.ScrewSupportUnit.bk12().geometry(), bearing = support.translated(new Vector(0,0,37.5));support.close();
    overlap = bearing.intersect(placed);near(overlap.volume(),0,"washer meets bearing without penetration");overlap.close();
    near(screw.connector("retainer").frame.z + hardware.connector("bearingFace").frame.z,25,"retainer bears at fixed support face");
    var pulley = new machinekit.transmission.TimingPulley(HTD5M,20,12,15).geometry();
    var mountedPulley = pulley.translated(new Vector(0,0,5));pulley.close();
    overlap = mountedPulley.intersect(placed);near(overlap.volume(),0,"retainer clears belt-drive screw pulley");overlap.close();mountedPulley.close();
    shaft.close();placed.close();bearing.close();
  }

  static function presentation():Void {
    var geometry = cylinder(20);
    var generated:Map<String, GeometryData> = []; generated.set("thread-mesh", geometry);
    var objects:Array<SceneObjectData> = [{id: "thread", label: "Thread", type: "cad-preview",
      x: 0.0, y: 0.0, z: 0.0, width: 0.005, height: 0.005, depth: 0.03,
      collisionEnabled: false, dynamicBody: false, mass: 1.0, red: 0.7, green: 0.7, blue: 0.7,
      visible: true, meshSnapshot: "thread-mesh"}];
    var scene = new EditorScene(objects, null, null, generated);
    try {
      var pose:{id:String, position:Array<Float>, rotation:Array<Float>} = {id: "thread", position: [2, 3, 4], rotation: [0, 0, 0, 1]};
      var camera = new PerspectiveCamera();
      camera.frame(2, 3, 3.985, 0.04, 0.04, 0.05, 4.0 / 3.0);
      var view = scene.configureRenderView(new SceneView(), camera.viewProjection(4.0 / 3.0), [pose]);
      check(scene.presentation.threads.nodesFor("thread").length == 1, "one thread creates one runtime child");
      check(view.poseOverrideCount() == 2, "the child receives its owner's runtime pose");
      var hit = scene.hoverHitRayWithView(view, 2.01, 3.001, 3.995, -1, 0, 0);
      check(hit.id == "thread" && hit.faceIndex == geometry.threads[0].source.faceIndex,
        "posed thread triangles pick the owning object and exact face: " + hit.id + "/" + hit.faceIndex + " expected " + geometry.threads[0].source.faceIndex);
      check(view.visibilityOverrideValues()[0].get_visible() != 0, "cosmetic threads are shown by default");
      var child = scene.presentation.threads.nodesFor("thread")[0];
      scene.select("thread");
      var selected = scene.configureRenderView(new SceneView(), Transform.identity());
      check(selected.selectionOverrideCount() == 1 && selected.materialOverrideValues().length == 0,
        "selection tint is inherited by the thread child without replacing its textured material");
      scene.select("scene");
      var hovered = scene.configureRenderView(new SceneView(), Transform.identity(), null, "thread");
      check(hovered.hoverOverrideCount() == 1 && hovered.materialOverrideValues().length == 0,
        "hover tint is inherited by the thread child without replacing its textured material");
      var diagnostic = scene.configureRenderView(new SceneView(), Transform.identity(), null, null, -1,
        [new app.SceneCollision.SceneCollisionPair("thread","environment",-0.001,[0,0,0],[0,0,0],true)]);
      check(diagnostic.diagnosticTintValues().length == 1 && diagnostic.materialOverrideValues().length == 0,
        "diagnostic tint reaches thread children through the parent and keeps their textures");
      var hidden = scene.configureRenderView(new SceneView(), Transform.identity(), null, null, -1, null, false);
      check(hidden.visibilityOverrideValues()[0].get_visible() == 0, "the view setting hides threads");
      scene.configureRenderView(new SceneView(), Transform.identity());
      check(scene.presentation.threads.nodesFor("thread")[0].equals(child), "unchanged mesh reuses its child");
      scene.object("thread").red = 0.3;
      scene.configureRenderView(new SceneView(), Transform.identity());
      check(!scene.presentation.threads.nodesFor("thread")[0].equals(child), "appearance edit rebuilds the child's material");
      scene.select("thread");
      scene.failureInjection = function(point) { if (point == "transaction.before-commit") throw "abort thread deletion"; };
      var rejected = false;
      try scene.deleteSelected() catch (_:Dynamic) rejected = true;
      scene.failureInjection = null;
      check(rejected && scene.object("thread") != null && scene.presentation.threads.nodesFor("thread").length == 1,
        "failed deletion retains the object and derived child");
      check(scene.deleteSelected(), "threaded object can be deleted");
      scene.configureRenderView(new SceneView(), Transform.identity());
      check(scene.presentation.threads.nodesFor("thread").length == 0, "deleted objects release thread children");
    } catch (error:Dynamic) { scene.dispose(); throw error; }
    scene.dispose();
  }

  public static function main():Int {
    mesh(); texture(); presentation(); ballRace(); ballHardware();
    trace('CosmeticThreadTests passed ($checks checks)');
    return 0;
  }
}
