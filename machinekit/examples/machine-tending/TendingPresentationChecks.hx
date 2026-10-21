import materia.assembly.AssemblyDefinitionCodec;
import materia.presentation.PresentationCodec;
import materia.project.SceneArtifact;

/** Validate review presets against the real occurrence namespaces without running a mission. */
class TendingPresentationChecks {
  public static function main():Void {
    var tool=SceneArtifact.decode(TendingEndEffectorPreview.preview());
    if(tool.presentation==null||tool.presentation.views.length!=2)throw "End effector lost its service view";
    if(Sys.args().indexOf("--tool-only")>=0){Sys.println("Tending end-effector service view validated");return;}
    var definition=new TendingControllers().definition(),before=AssemblyDefinitionCodec.encode(definition);
    var presentation=TendingCell.presentation(definition);
    PresentationCodec.decode(definition,PresentationCodec.encode(definition,presentation));
    if(presentation.views.length!=4||presentation.procedures[0].steps.length!=3)throw "Tending review lost its views or steps";
    if(AssemblyDefinitionCodec.encode(definition)!=before)throw "Review changed the engineering cell";
    Sys.println("Tending presentation: real cell selectors, wire round trip, engineering isolation and end-effector service view passed");
  }
}
