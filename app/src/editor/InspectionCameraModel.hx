package app.editor;
import materia.project.SceneArtifact.SceneArtifactInspectionCamera;

/** Metric framing math shared by docked and overlay presentations. Offsets do not rotate with the part. */
class InspectionCameraModel {
  public static function target(preset:SceneArtifactInspectionCamera,position:Array<Float>):Array<Float> {
    if(position.length!=3||preset.offset.length!=3)throw "Inspection camera needs three coordinates";
    return [for(i in 0...3)position[i]+preset.offset[i]];
  }
}
