package tests;

import app.EditorScene;
import app.EditorPerspectiveViewport;
import nativekit.scene.SceneView;
import haxeon.platform.NativeKitEvents;
import haxeon.ui.LayoutAxis;
import haxeon.ui.LayoutFrame;
import haxeon.ui.LayoutStyle;
import haxeon.ui.core.RenderNode;
import haxeon.ui.core.UiContext;
import haxeon.ui.host.UiHostContext;
import haxeon.ui.widgets.KeyedView;
import haxeon.ui.widgets.layout.Column;
import haxeon.ui.widgets.layout.Row;
import haxeon.ui.widgets.layout.Spacer;

/** Exercise real layout/event coordinates independently of the capped GPU render target. */
@:access(app.EditorPerspectiveViewport)
@:access(tests.SceneEditingTests)
class ViewportCoordinatesTests {
  static var assertions=0;
  static function check(ok:Bool,message:String):Void {assertions++;if(!ok)throw message;}
  static function near(actual:Float,expected:Float,message:String):Void
    check(Math.abs(actual-expected)<0.01,message+': $actual != $expected');
  static function viewportNode(root:RenderNode):Null<RenderNode> {
    if(root.semantics!=null && root.semantics.label=="Scene perspective GPU view")return root;
    for(child in root.children){var found=viewportNode(child);if(found!=null)return found;}
    return null;
  }
  static function fill():LayoutStyle {
    var style=new LayoutStyle();style.width=LayoutAxis.grow();style.height=LayoutAxis.grow();return style;
  }
  public static function main():Int {
    var scene=new EditorScene([{id:"target",label:"Target",type:"rectangle",x:0.3,y:-0.2,z:0.0,
      width:0.2,height:0.2,depth:0.2,collisionEnabled:false,dynamicBody:false,mass:1.0,
      red:0.5,green:0.5,blue:0.5,visible:true}]);
    var host=new UiHostContext(null,new NativeKitEvents(),function(){},function(){});
    var viewport=new EditorPerspectiveViewport("coordinate-test",scene,host,fill()), ui=new UiContext();
    var row=new Row("docked-row",[
      new KeyedView("left",new Spacer("hierarchy",LayoutAxis.fixed(300),LayoutAxis.grow())),
      new KeyedView("viewport",viewport),
      new KeyedView("right",new Spacer("inspector",LayoutAxis.fixed(200),LayoutAxis.grow()))],fill());
    var layout=new Column("docked-column",[
      new KeyedView("toolbar",new Spacer("toolbar",LayoutAxis.grow(),LayoutAxis.fixed(80))),
      new KeyedView("body",row)],fill());
    try {
      // Wide, tall, both capped, fractional layout, and resizing back below the cap.
      for(size in [[3840.0,2160.0],[1700.0,3000.0],[3600.0,2800.0],[3100.5,1700.25],[1400.0,900.0]]) {
        var node=viewportNode(ui.submit(layout,new LayoutFrame(size[0],size[1])));
        check(node!=null,"docked viewport resolves");
        var bounds=node.globalBounds(), width=bounds.width,height=bounds.height;
        near(bounds.x,300,"viewport accounts for the hierarchy dock");
        near(bounds.y,80,"viewport accounts for the toolbar");
        near(viewport.viewportWidth,width,"input dimensions update during layout before painting");
        near(viewport.viewportHeight,height,"input height updates during layout before painting");
        // Model the texture retained by the renderer; input must not use these dimensions.
        viewport.renderedWidth=Std.int(Math.min(2048,Math.ceil(width)));
        viewport.renderedHeight=Std.int(Math.min(2048,Math.ceil(height)));
        near(viewport.aspect(),width/height,"projection follows the displayed aspect ratio");
        scene.setPositionXY("target",0.3,-0.2);
        viewport.camera.frame(0,0,0,4,3,2,width/height);
        var point=viewport.camera.project(0.3,-0.2,0,width,height);
        if(point==null)throw "Target is behind the camera";
        check(viewport.pick(point.x,point.y)=="target","pick matches the displayed object after resize");
        if(width>2048 || height>2048)
          check(EditorPerspectiveViewport.pickScene(scene,viewport.camera,viewport.renderedWidth,
            viewport.renderedHeight,point.x,point.y)!="target","fixture reproduces the old texture-coordinate miss");
        var x=bounds.x+point.x,y=bounds.y+point.y;
        ui.events.pointerMove(x,y);
        check(viewport.hoveredId()=="target","hover uses viewport-local layout coordinates");
        ui.events.pointerDown(x,y,0);
        check(scene.selectedId=="target" && viewport.objectDrag!=null,"press selects and grabs the visible object");
        ui.events.pointerMove(x+36,y-18);
        ui.events.pointerUp(x+36,y-18,0);
        var target=scene.object("target");
        if(target==null)throw "Missing target";
        var moved=viewport.camera.project(target.x,target.y,target.z,width,height);
        if(moved==null)throw "Moved target is behind the camera";
        near(moved.x,point.x+36,"drag follows the cursor horizontally");
        near(moved.y,point.y-18,"drag follows the cursor vertically");
        var ray=viewport.camera.screenRay(moved.x,moved.y,width,height);
        var view=scene.configureRenderView(new SceneView(),viewport.camera.viewProjection(width/height));
        var hit=scene.pickHitRayWithView(view,ray.originX,ray.originY,ray.originZ,ray.directionX,ray.directionY,ray.directionZ);
        check(hit.id=="target","zoom has a visible surface anchor");
        var distance=viewport.camera.distance;
        ui.scroll(bounds.x+moved.x,bounds.y+moved.y,0,-120);
        check(viewport.camera.distance<distance,"wheel zoom reaches the docked viewport");
        var anchor=viewport.camera.project(hit.x,hit.y,hit.z,width,height);
        if(anchor==null)throw "Zoom anchor is behind the camera";
        near(anchor.x,moved.x,"zoom preserves the cursor's surface anchor horizontally");
        near(anchor.y,moved.y,"zoom preserves the cursor's surface anchor vertically");
      }
    } catch(error:Dynamic) {ui.dispose();viewport.dispose();scene.dispose();throw error;}
    ui.dispose();viewport.dispose();scene.dispose();
    SceneEditingTests.perspectiveHoverInput();
    SceneEditingTests.dockedNavigationInput();
    Sys.println('Viewport coordinates: $assertions assertions passed, plus hover and docked navigation regressions');
    return 0;
  }
}
