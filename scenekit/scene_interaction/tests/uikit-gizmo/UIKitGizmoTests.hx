import nativekit.scene.gizmo.*;
import nativekit.scene.gizmo.uikit.*;
import nativekit.scene.gizmo.uikit.GizmoTool;
import haxeon.ui.FontCollection;
import haxeon.ui.core.*;

/** Independent consumer: no Beartooth, World, asset, procedural or history imports. */
class UIKitGizmoTests {
  static function check(value:Bool,message:String):Void {if(!value) throw message;}
  static function rejects(action:Void->Void):Void {var failed=false;try action() catch(_:Dynamic) failed=true;check(failed,"Expected adapter rejection");}
  static function main():Void {
    var fonts=FontCollection.create(),overlay=new TransformGizmoOverlay(fonts);
    var camera=new GizmoCamera(new GizmoVec3(10,10,10),new GizmoVec3(0,0,0),new GizmoVec3(0,0,1),800,600);
    for(tool in [Move,Rotate,Scale]) {
      var key=Std.string(tool);overlay.layout(key,tool,camera,new GizmoVec3(0,0,0),GizmoBasis.world(),GizmoBasis.world());
      check(overlay.valid(key) && !overlay.valid("stale"),"Adapter admitted a stale generation");
      var shapes=overlay.shapes();check(shapes.length>0,"Missing adapter shapes");shapes.pop();check(overlay.shapes().length>shapes.length,"Adapter leaked shapes");
      overlay.hover="X";overlay.layout(key,tool,camera,new GizmoVec3(0,0,0),GizmoBasis.world(),GizmoBasis.world());check(overlay.hover=="X","Repaint discarded hover");
      overlay.layout(key+"new",tool,camera,new GizmoVec3(0,0,0),GizmoBasis.world(),GizmoBasis.world());check(overlay.hover==null,"New generation retained stale hover");
      overlay.invalidateSize(800.25,600.25);check(overlay.valid(key+"new"),"Subpixel layout noise invalidated adapter");
      overlay.invalidateSize(801,600);check(!overlay.valid(key+"new") && overlay.shapes().length==0,"Resize retained stale geometry");
    }
    var capture=new GizmoPointerCapture(),event=new UiEvent(UiEventKind.PointerDown,new WidgetId(1),0,0,0,0,0,0,0,null,null,0,7);
    capture.begin(event,"selected/revision/camera",800,600);check(capture.pointer==7 && event.pointerCaptureTarget!=null && event.pointerNativeCaptureRequested,"Capture did not use UIKit event routing");
    check(capture.matches("selected/revision/camera",800.25,600.25),"Capture rejected layout noise");
    check(!capture.matches("new-selection",800,600) && !capture.matches("selected/revision/camera",801,600) && !capture.matches("selected/revision/camera",Math.NaN,600),"Capture admitted stale/invalid context");
    rejects(()->capture.begin(event,"other",800,600));capture.clear();check(capture.pointer==null,"Capture did not clear");
    rejects(()->capture.begin(event,"other",0,600));capture.begin(event,"other",800,600);capture.clear();
    overlay.dispose();fonts.dispose();Sys.println("Independent UIKit adapter generation, immutable geometry, hover, resize and capture contract passed");
  }
}
