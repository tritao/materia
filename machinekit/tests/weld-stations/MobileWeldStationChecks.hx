import materia.project.SceneArtifact;

class MobileWeldStationChecks {
  public static function main():Void {
    var cell = new MobileWelderCell();
    var scene = MobileWelderPreview.design(cell);
    var steps = new MobileWeldStations(cell, scene).mission();
    var names = new Map<String, Bool>();
    var stations = 0;
    for (step in steps) {
      if (step.kind == "goTo") stations++;
      var weld:materia.project.SceneArtifact.SceneArtifactWeld = cast step.weld;
      if (step.kind == "weld") for (segment in weld.path) {
        if (names.exists(segment.seam)) throw 'Mobile mission duplicates seam ${segment.seam}';
        names.set(segment.seam, true);
      }
    }
    var expected = cell.weldment().findIn(cell, cell.solvedPoses()).require();
    for (seam in expected) if (!names.exists(seam.name())) throw 'Mobile mission omits ${seam.name()}';
    if (stations == 0 || steps[0].kind != "goTo") throw "Mobile weld mission never parks";
    scene.mission = {steps: steps};
    var decoded = SceneArtifact.decode(SceneArtifact.encode(scene));
    var mission:materia.project.SceneArtifact.SceneArtifactMission = cast decoded.mission;
    if (mission.steps.length != steps.length) throw "Mobile mission failed current-schema round trip";
    Sys.println('Mobile station CAD checks pass: $stations stations, ${expected.length} seams, ${steps.length} steps');
  }
}
