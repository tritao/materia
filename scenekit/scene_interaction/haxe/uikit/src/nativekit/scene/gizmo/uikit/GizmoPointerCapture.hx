package nativekit.scene.gizmo.uikit;
import haxeon.ui.core.UiEvent;
import haxeon.ui.core.UiEventKind;

/** Shared capture validity. The consumer supplies an opaque generation key and owns publication. */
class GizmoPointerCapture {
  public var pointer(default,null):Null<Int>;
  var generation:Null<String>;
  var width:Float=0;
  var height:Float=0;
  public function new() {}
  public function begin(event:UiEvent,key:String,width:Float,height:Float):Void {
    if(pointer!=null || event==null || event.kind!=UiEventKind.PointerDown || key==null || key.length==0 || !Math.isFinite(width) || !Math.isFinite(height) || width<=0 || height<=0) throw "Invalid or active gizmo capture";
    pointer=event.pointerId;generation=key;this.width=width;this.height=height;event.capturePointer();
  }
  public function matches(key:String,width:Float,height:Float):Bool
    return pointer!=null && generation==key && Math.isFinite(width) && Math.isFinite(height) && Math.abs(width-this.width)<=0.5 && Math.abs(height-this.height)<=0.5;
  public function clear():Void {pointer=null;generation=null;}
}
