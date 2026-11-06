import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyDefinition;
import materia.presentation.PresentationEvaluator;
import materia.presentation.PresentationGeometry;
import materia.presentation.Presentations;
import materia.project.SceneArtifact;
import materia.project.SceneArtifact.SceneArtifactPart;
import machinekit.tooling.CastingMoldView;

class CastingMoldViewTests {
  static var assertions = 0;
  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw message;
  }
  static function mesh(id:String):SceneArtifactPart {
    var vertices = haxe.io.Bytes.alloc(96), normals = haxe.io.Bytes.alloc(96);
    var indices = haxe.io.Bytes.alloc(48);
    vertices.setDouble(24, 1);
    vertices.setDouble(56, 1);
    vertices.setDouble(88, 1);
    var triangles = [0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3];
    for (i in 0...4) normals.setDouble(i * 24 + 16, 1);
    for (i in 0...triangles.length) indices.setInt32(i * 4, triangles[i]);
    return {
      id: id,
      name: id,
      red: 0.5,
      green: 0.5,
      blue: 0.5,
      vertexCount: 4,
      indexCount: 12,
      vertices: vertices,
      normals: normals,
      indices: indices,
      faceRanges: []
    };
  }
  static function main():Void {
    var ids = ["bed", "insert-seat", "anchor-bolt", "lifting-insert"];
    var definition:AssemblyDefinition = {schemaVersion: materia.assembly.AssemblyDefinitionCodec.VERSION,
      id: "mold", definitions: [for (id in ids) {
      id:id, connectors:[]
    }
    ], occurrences:[for (i in 0...ids.length) {
      id:ids[i], definition:ids[i], initialPose:F.translation(i * 2, i * 3, i * 5)
    }
    ], joints:[]};
    var scene:materia.project.SceneArtifact.SceneArtifactData = {
      metresPerUnit: 0.001,
      parts: [for (id in ids) mesh(id)],
      assemblyDefinition: definition
    };
    var original = scene.parts[0].vertices.sub(0, scene.parts[0].vertices.length);
    CastingMoldView.geometry(scene, mesh("casting"));
    var fingerprint = PresentationGeometry.fingerprint(scene);
    var presentation = Presentations.assembled(definition);
    scene.presentation = presentation;
    var service = F.fromRotationMatrix(10, 20, 30, [1, 0, 0, 0, -1, 0, 0, 0, -1]);
    CastingMoldView.presentation(scene, ["insert-seat", "anchor-bolt", "lifting-insert"], service, ["lifting-insert"]);
    check(presentation.views.length == 4, "Finished casting is an additional dropdown view");
    check(presentation.defaultView == "assembled", "Mold view remains the default");
    check(
      PresentationGeometry.fingerprint(scene) == fingerprint,
      "View generation preserves motion geometry fingerprints"
    );
    check(scene.parts[0].vertices.compare(original) == 0, "Existing tooling meshes remain intact");
    check(scene.parts[4].physicalMassKg == 0, "Reference casting adds no simulation mass");
    var hulls = definition.definitions[4].collisionHulls;
    check(hulls != null && hulls.length == 0, "Reference casting adds no collision hull");
    var solved = new Map<String, materia.assembly.AssemblyRecord.AssemblyFrame>();
    for (part in definition.occurrences) solved.set(part.id, part.initialPose);
    var cursor = PresentationEvaluator.reset(presentation);
    var frame = PresentationEvaluator.evaluate(definition, solved, presentation, cursor, fingerprint);
    check(frame.visibility.get(CastingMoldView.REFERENCE_ID) == Hidden, "Mold view hides the casting reference");
    check(frame.visibility.get("bed") == Shown, "Mold view still shows its tooling");
    cursor = new materia.presentation.PresentationEvaluator.PresentationCursor(CastingMoldView.REFERENCE_ID, cursor.explode, cursor.procedure, cursor.step, cursor.progress);
    frame = PresentationEvaluator.evaluate(definition, solved, presentation, cursor, fingerprint);
    check(frame.visibility.get("bed") == Hidden, "Finished view hides mold tooling");
    for (id in [CastingMoldView.REFERENCE_ID, "insert-seat", "anchor-bolt", "lifting-insert"]) check(
      frame.visibility.get(id) == Shown,
      "Finished view retains " + id
    );
    for (id in [CastingMoldView.REFERENCE_ID, "insert-seat", "anchor-bolt", "lifting-insert"]) {
      var pose = frame.poses.get(id);
      if (pose == null) throw "Missing installed pose";
      var index = ids.indexOf(id), n = index < 0 ? 0 : index;
      check(
        pose.x == 10 + 2 * n && pose.y == 20 - 3 * n && pose.z == 30 - 5 * n && pose.qx == 1,
        "Installed orientation moves every casting member together: " + id
      );
      var originalPose = solved.get(id);
      if (originalPose == null) throw "Missing original pose";
      check(
        originalPose.x == 2 * n && originalPose.y == 3 * n && originalPose.z == 5 * n && originalPose.qw == 1,
        "Engineering poses remain unchanged: " + id
      );
    }
    cursor = new materia.presentation.PresentationEvaluator.PresentationCursor(CastingMoldView.AS_POURED_ID, cursor.explode, cursor.procedure, cursor.step, cursor.progress);
    frame = PresentationEvaluator.evaluate(definition, solved, presentation, cursor, fingerprint);
    var poured = frame.poses.get(CastingMoldView.REFERENCE_ID);
    if (poured == null) throw "Missing as-poured pose";
    check(poured.z == 0 && poured.qw == 1, "As-poured view retains the mold orientation");
    cursor = new materia.presentation.PresentationEvaluator.PresentationCursor(CastingMoldView.LIFTING_ID, cursor.explode, cursor.procedure, cursor.step, cursor.progress);
    frame = PresentationEvaluator.evaluate(definition, solved, presentation, cursor, fingerprint);
    check(
      frame.visibility.get(CastingMoldView.REFERENCE_ID) == Transparent,
      "Lifting view exposes source inserts through transparent resin"
    );
    check(
      frame.visibility.get("lifting-insert") == Shown && frame.visibility.get("insert-seat") == Hidden,
      "Lifting view isolates the declared lifting inserts"
    );
    cursor = new materia.presentation.PresentationEvaluator.PresentationCursor(cursor.view, cursor.explode, "locate-casting-lifting-points", 0, cursor.progress);
    frame = PresentationEvaluator.evaluate(definition, solved, presentation, cursor, fingerprint);
    check(frame.highlight.indexOf("lifting-insert") >= 0, "Identification step highlights lifting inserts");
    check(frame.evidence == null, "Identification never claims lifting qualification");
    presentation.views[1].sections = [{occurrence: null, point: {x: 1, y: 2, z: 3}, normal: {x: 0, y: 0, z: 1}}];
    cursor = PresentationEvaluator.reset(presentation);
    cursor = new materia.presentation.PresentationEvaluator.PresentationCursor(CastingMoldView.REFERENCE_ID, cursor.explode, cursor.procedure, cursor.step, cursor.progress);
    frame = PresentationEvaluator.evaluate(definition, solved, presentation, cursor, fingerprint);
    check(
      frame.sections[0].point.z == 27 && frame.sections[0].normal.z == -1,
      "Section planes follow the display transform"
    );
    var restored = SceneArtifact.decode(SceneArtifact.encode(scene));
    var restoredPresentation = restored.presentation;
    if (restoredPresentation == null) throw "Missing restored presentation";
    check(
      restoredPresentation.views[1].label == "Finished casting — installed",
      "Finished view survives artifact encoding"
    );
    check(
      PresentationGeometry.fingerprint(restored) == fingerprint,
      "Artifact round trip retains the complete geometry fingerprint"
    );
    var restoredFrame = restoredPresentation.views[1].displayTransform;
    check(restoredFrame != null && restoredFrame.qx == 1, "Display orientation survives artifact encoding");
    presentation.views[1].displayTransform = {x: 0, y: 0, z: 0, qx: 0, qy: 0, qz: 0, qw: 2};
    var rejected = false;
    try materia.presentation.PresentationCodec.validate(definition, presentation) catch (_:Dynamic) rejected = true;
    check(rejected, "Invalid display rotations are rejected");
    Sys.println('CastingMoldViewTests: $assertions assertions passed');
  }
}
