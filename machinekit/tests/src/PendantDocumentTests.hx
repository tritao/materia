import cadkit.Shape;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.TimelineModel;
import machinekit.assembly.AssemblyPreview;
import machinekit.pendant.*;
import materia.project.SceneArtifact;
import materia.project.SceneSourceMap;

class PendantDocumentTests {
  static function check(ok:Bool, message:String):Void if (!ok) throw message;

  static function compare(name:String, direct:Shape, recorded:Document):Void {
    var result = recorded.result();
    var db = direct.bounds(), rb = result.bounds();
    var groups = new Map<String, Int>();
    var unlabelled = 0;
    for (i in 0...recorded.featureCount()) {
      var f = recorded.featureAt(i);
      groups.set(f.group, (groups.exists(f.group) ? groups.get(f.group) : 0) + 1);
      if (f.label == "") unlabelled++;
    }
    var summary = [for (g in groups.keys()) g + " " + groups.get(g)];
    Sys.println(name + ": " + recorded.featureCount() + " features (" + summary.join(", ") + "), volume " + direct.volume() + " / " + result.volume());
    check(Math.abs(direct.volume() - result.volume()) < 1e-3 * direct.volume(), name + ": volume differs");
    for (pair in [
      [db.get_min().get_x(), rb.get_min().get_x()], [db.get_min().get_y(), rb.get_min().get_y()],
      [db.get_min().get_z(), rb.get_min().get_z()], [db.get_max().get_x(), rb.get_max().get_x()],
      [db.get_max().get_y(), rb.get_max().get_y()], [db.get_max().get_z(), rb.get_max().get_z()]
    ]) check(Math.abs(pair[0] - pair[1]) < 1e-6, name + ": bounds differ");
    check(unlabelled == 0, name + ": " + unlabelled + " features have no label");

    // Every output face belongs to exactly one step, and a step's faces come from its own group.
    var rows = TimelineModel.of(recorded);
    check(rows.length == recorded.featureCount(), name + ": one timeline row per feature");
    var faceCount = result.subshapeCount(CadKit.ShapeKind.Face);
    var seen = new Map<Int, Bool>();
    var owned = 0;
    for (row in rows) for (face in row.faces) {
      check(!seen.exists(face), name + ": face " + face + " is owned twice");
      seen.set(face, true);
      owned++;
    }
    check(owned == faceCount, name + ": " + owned + " of " + faceCount + " faces are owned by a step");
    for (row in rows) check(row.parent < 0 || row.faces.length == 0, name + ": '" + row.label + "' is part of another step but owns faces");
    var perGroup = new Map<String, Int>();
    for (row in rows) {
      var count = 0;
      if (perGroup.exists(row.group)) count = perGroup.get(row.group);
      perGroup.set(row.group, count + row.faces.length);
    }
    Sys.println("  faces by group: " + [for (g in perGroup.keys()) g + " " + perGroup.get(g)].join(", "));

    // Labels, groups and the rounded profile survive a save and load.
    var reloaded = DocumentCodec.decode(DocumentCodec.encode(recorded));
    check(reloaded.featureCount() == recorded.featureCount(), name + ": reload feature count");
    for (i in 0...recorded.featureCount()) check(
      reloaded.featureAt(i).label == recorded.featureAt(i).label && reloaded.featureAt(i).group == recorded.featureAt(i).group,
      name + ": label and group survive a reload at " + i
    );
    reloaded.recompute();
    check(Math.abs(reloaded.result().volume() - result.volume()) < 1e-3, name + ": reloaded volume");
    reloaded.close();
  }

  public static function run():Void {
    var pendantCase = ReferencePendant.pendantCase();
    var shell = pendantCase.shell(), recordedShell = PendantCaseDocument.shell(pendantCase);
    compare("shell", shell.shape, recordedShell.document);
    var plate = pendantCase.plate(), recordedPlate = PendantCaseDocument.plate(pendantCase);
    compare("plate", plate.shape, recordedPlate.document);
    var board = pendantCase.board(), recordedBoard = PendantCaseDocument.board(pendantCase);
    compare("board", board.shape, recordedBoard.document);
    for (d in [recordedShell.document, recordedPlate.document, recordedBoard.document]) d.close();
    for (p in [shell, plate, board]) p.close();
    sceneSources();
    Sys.println("Pendant document parity ok");
  }

  /** The scene is built from the recorded steps, so the source map's face numbers are the scene's face numbers. */
  static function sceneSources():Void {
    var documents = new PendantDocuments(ReferencePendant.pendantCase());
    var scene = AssemblyPreview.scene(ReferencePendant.assembly(documents), "pendant");
    var map = documents.sources();
    scene.project = {dynamicParts: [], motions: [], sources: map, histories: documents.histories()};
    var decoded = SceneArtifact.decode(SceneArtifact.encode(scene));
    var expected = ["case/plate" => documents.plate, "case/shell" => documents.shell, "case/board" => documents.board];
    var project = decoded.project;
    if (project == null) throw "The project record survives the scene round trip";
    var sources = project.sources;
    if (sources == null) throw "The source map survives the scene round trip";
    SceneSourceMaps.validate(sources, decoded);
    var histories = project.histories;
    if (histories == null || histories.length != 3) throw "The artifact carries the three recorded histories";
    for (history in histories) {
      var part = expected.get(history.occurrence);
      var reloaded = DocumentCodec.decode(history.document);
      check(part != null && reloaded.featureCount() == part.document.featureCount(), history.occurrence + ": the history keeps every step");
      reloaded.close();
    }
    for (part in decoded.parts) if (expected.exists(part.id)) {
      var faces = expected.get(part.id).document.result().subshapeCount(CadKit.ShapeKind.Face);
      check(part.faceRanges.length == faces, part.id + ": scene has " + part.faceRanges.length + " faces, the recording " + faces);
    }
    var steps = 0, withFaces = 0;
    for (feature in sources.features) {
      steps++;
      if (feature.faces.length > 0) withFaces++;
    }
    Sys.println("scene sources: " + steps + " steps, " + withFaces + " with faces");
    check(steps > 0 && withFaces > 0, "The source map lists steps with faces");
    documents.close();
  }

  public static function main():Void run();
}
