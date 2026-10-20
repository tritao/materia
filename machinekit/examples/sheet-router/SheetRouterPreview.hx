import haxe.io.Bytes;
import machinekit.assembly.AssemblyPreview;
import materia.project.SceneArtifact;

class SheetRouterPreview {
    public static function router():Bytes return SceneArtifact.encode(AssemblyPreview.scene(new SheetRouter(),"sheet-router",256));
    public static function longRouter():Bytes return SceneArtifact.encode(AssemblyPreview.scene(new SheetRouter(new SheetRouterSpec(3150)),"sheet-router-3150",256));
}
