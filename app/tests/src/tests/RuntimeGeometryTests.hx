package tests;

import app.EditorScene;
import haxe.io.Bytes;
import nativekit.scene.GeometryData;
import nativekit.scene.SceneView;
import nativekit.scene.Transform;

/** Runtime geometry shown in parts, such as stock a simulated machine is cutting, and how it ends. */
@:access(app.EditorScene)
class RuntimeGeometryTests {
  static function check(value:Bool, message:String):Void if (!value) throw message;

  static function triangle(offset:Float):GeometryData {
    var positions = Bytes.alloc(36);
    for (vertex in 0...3) {
      positions.setFloat(vertex * 12, offset + (vertex == 1 ? 0.01 : 0.0));
      positions.setFloat(vertex * 12 + 4, vertex == 2 ? 0.01 : 0.0);
      positions.setFloat(vertex * 12 + 8, 0.0);
    }
    var indices = Bytes.alloc(12);
    for (index in 0...3) indices.setInt32(index * 4, index);
    var data = new GeometryData();
    data.addStream(1, 2, positions, 3, 12);
    data.setIndexBuffer(indices, 3);
    data.setBounds(offset, 0, 0, offset + 0.01, 0.01, 0);
    return data;
  }

  static function live(scene:EditorScene, node:nativekit.scene.NodeId):Bool
    return scene.renderSnapshot().findNode(node) != null;

  static function poseTriangle(x:Float):GeometryData {
    var positions=Bytes.alloc(36),normals=Bytes.alloc(36),indices=Bytes.alloc(12);
    var points=[x-0.1,-0.1,0.0,x+0.1,-0.1,0.0,x,0.1,0.0];
    for(i in 0...9)positions.setFloat(i*4,points[i]);
    for(i in 0...3){normals.setFloat(i*12+8,1);indices.setInt32(i*4,i);}
    var geometry=new GeometryData();geometry.addStream(1,2,positions,3,12);geometry.addStream(2,2,normals,3,12);
    geometry.setIndexBuffer(indices,3);geometry.setBounds(x-0.1,-0.1,0,x+0.1,0.1,0);
    return geometry;
  }
  static function hit(scene:EditorScene,view:SceneView,x:Float,y:Float):String
    return scene.hoverHitRayWithView(view,x,y,2,0,0,-1).id;

  static function posedChunksFollowOwner():Void {
    var scene=new EditorScene([{id:"stock",label:"Stock",type:"rectangle",x:0.0,y:0.0,z:0.0,
      width:1.0,height:1.0,depth:0.1,collisionEnabled:false,dynamicBody:false,mass:1.0,
      red:0.5,green:0.5,blue:0.5,visible:true}]);
    try {
      var changed:Map<Int,GeometryData> = [0=>poseTriangle(0.3)];
      scene.setRuntimeGeometryParts("stock",2,changed);
      var half=Math.PI/4;
      var pose:app.ApplicationSimulation.SimulationPoseVisual={id:"stock",position:[1,2,0],rotation:[0,0,Math.sin(half),Math.cos(half)]};
      var projection=Transform.identity().set(0,0.1).set(5,0.1).set(10,0.1);
      var rest=scene.configureRenderView(new SceneView(),projection,null,null,-1,null,false);
      check(hit(scene,rest,0.3,0)=="stock","chunk geometry is pickable at its authored position");
      var view=scene.configureRenderView(new SceneView(),projection,[pose],null,-1,null,false);
      check(hit(scene,view,1,2.3)=="stock","stock chunks follow owner translation and rotation: hit="+hit(scene,view,1,2.3)+", overrides="+view.poseOverrideCount());
      check(hit(scene,view,0.3,0)=="scene","stock chunks leave the saved position");
      scene.setRuntimeGeometryParts("stock",2,[0=>poseTriangle(0.5),1=>poseTriangle(-0.3)]);
      view=scene.configureRenderView(new SceneView(),projection,[pose],null,-1,null,false);
      check(hit(scene,view,1,2.5)=="stock"&&hit(scene,view,1,1.7)=="stock","replaced and newly created chunks share the live stock frame");
      check(hit(scene,view,1,2.3)=="scene","replaced chunks remove their prior surface");
      pose.position=[-2,1,0];pose.rotation=[0,0,0,1];
      view=scene.configureRenderView(new SceneView(),projection,[pose],null,-1,null,false);
      check(hit(scene,view,-1.5,1)=="stock"&&hit(scene,view,-2.3,1)=="stock","all chunks follow subsequent live poses");
      var authored=scene.configureRenderView(new SceneView(),projection,null,null,-1,null,false);
      check(hit(scene,authored,0.5,0)=="stock","unposed view retains authored coordinates");
      scene.clearRuntimeGeometry("stock");
      view=scene.configureRenderView(new SceneView(),projection,[pose],null,-1,null,false);
      check(view.poseOverrideCount()==1&&hit(scene,view,-2,1)=="stock","clearing chunks restores the posed original solid");
    }catch(error:Dynamic){scene.dispose();throw error;}
    scene.dispose();
  }

  public static function main():Int {
    var scene = new EditorScene([]);
    try {
      posedChunksFollowOwner();
      check(scene.createRectangle(), "runtime geometry test creates an object");
      var id = scene.selectedId;
      var authored = scene.runtimeFor(id).geometry;
      check(scene.createRectangle(), "batch runtime geometry test creates a second object");
      var second = scene.selectedId, secondAuthored = scene.runtimeFor(second).geometry;
      var revision = scene.revision, visualRevision = scene.visualRevision, environmentRevision = scene.environmentRevision;
      scene.setRuntimeGeometries([id => triangle(0.04), second => triangle(0.08)]);
      check(scene.revision == revision && scene.environmentRevision == environmentRevision &&
        scene.visualRevision == visualRevision + 1,
        "related runtime meshes publish one visual revision without invalidating document or physics caches");
      check(scene.runtimeFor(id).geometry != authored && scene.runtimeFor(second).geometry != secondAuthored,
        "a batch gives each object independent runtime geometry");
      var firstBounds = scene.info(id).bounds(), secondBounds = scene.info(second).bounds();
      check(firstBounds.valid && secondBounds.valid && Math.abs(firstBounds.maxX - firstBounds.minX - 0.01) < 1e-6 &&
        Math.abs(secondBounds.maxX - secondBounds.minX - 0.01) < 1e-6,
        "the batch snapshot contains both updated meshes");
      revision = scene.revision; visualRevision = scene.visualRevision;
      scene.setRuntimeGeometries([id => triangle(0.12), second => triangle(0.16)]);
      check(scene.revision == revision && scene.visualRevision == visualRevision + 1 &&
        scene.info(id).bounds().minX > firstBounds.maxX,
        "subsequent batched updates refresh the retained geometry and bounds");
      scene.clearRuntimeGeometry(id); scene.clearRuntimeGeometry(second);
      check(scene.runtimeFor(id).geometry == authored && scene.runtimeFor(second).geometry == secondAuthored,
        "both batched meshes restore their authored geometry");
      scene.select(id);
      scene.setRuntimeGeometryParts(id, 2, [0 => triangle(0), 1 => triangle(0.02)]);
      var parts = scene.runtimePartNodes(id);
      check(parts.length == 2 && live(scene, parts[0]) && live(scene, parts[1]),
        "each runtime part is a node of its own");
      check(scene.bridge.idForNode(parts[1]) == id, "a runtime part picks as its object");
      check(scene.runtimeFor(id).geometry != authored, "the object's own geometry is hidden behind its parts");
      scene.setRuntimeGeometryParts(id, 2, [1 => triangle(0.03)]);
      check(scene.runtimePartNodes(id).join(",") == parts.join(","), "an update replaces a part's geometry in place");

      var cached:app.EditorScene.RuntimeGeometryParts=scene.runtimeParts.get(id);
      var retained=cached.geometries.get(0);
      var transform=nativekit.scene.Transform.identity().translated(0.15,0,0.04);
      var beforeTransform=scene.visualRevision;
      scene.setRuntimeGeometryPartTransform(id,0,transform);
      var moved=scene.renderSnapshot().findNode(parts[0]).bounds();
      check(moved.valid&&Math.abs(moved.minX-0.15)<1e-6&&Math.abs(moved.minZ-0.04)<1e-6,
        "runtime part transform moves cached geometry and updates bounds");
      check(cached.geometries.get(0)==retained&&scene.visualRevision==beforeTransform+1,
        "runtime movement retains its mesh and publishes one visual revision");
      scene.setRuntimeGeometryPartTransform(id,0,transform);
      check(scene.visualRevision==beforeTransform+1,"unchanged transform does not republish");
      scene.setRuntimeGeometryParts(id,2,[0=>triangle(0.01)]);
      check(Math.abs(scene.renderSnapshot().findNode(parts[0]).bounds().minX-0.16)<1e-6,
        "material update retains runtime part transform");

      // Keep the unrelated rectangle from occluding the small posed triangle.
      scene.setRuntimeGeometry(second,new GeometryData());
      var posed=scene.configureRenderView(new nativekit.scene.SceneView(),nativekit.scene.Transform.identity(),
        [{id:id,position:[0.5,0,0],rotation:[0.0,0,0,1]}]);
      var hit=scene.pickHitRayWithView(posed,0.662,0.002,1,0,0,-1);
      check(hit.id==id,"live parent pose composes with cached child display transform for picking/rendering: "+haxe.Json.stringify(hit));
      scene.clearRuntimeGeometry(second);

      scene.clearRuntimeGeometry(id);
      check(scene.revision == revision && scene.environmentRevision == environmentRevision,
        "runtime part updates and restoration keep document and physics revisions stable");
      check(scene.runtimePartNodes(id).length == 0 && !live(scene, parts[0]) && !live(scene, parts[1]),
        "clearing removes the part nodes");
      check(scene.runtimeFor(id).geometry == authored && scene.bridge.idForNode(parts[0]) == null,
        "clearing gives the object its own geometry back");

      // Deleting the object takes its parts along; the scene does not destroy children with a parent.
      scene.setRuntimeGeometryParts(id, 1, [0 => triangle(0)]);
      var orphan = scene.runtimePartNodes(id)[0];
      check(scene.deleteSelected(), "the object deletes");
      check(!live(scene, orphan) && scene.runtimePartNodes(id).length == 0 &&
        !scene.ownedRuntimeGeometry.exists(id) && !scene.runtimeOriginalGeometry.exists(id),
        "deleting an object removes its runtime parts and forgets its runtime geometry");
      scene.dispose();
    } catch (error:Dynamic) {
      scene.dispose();
      Sys.println('Runtime geometry tests failed: $error');
      return 1;
    }
    Sys.println("Runtime geometry tests passed");
    return 0;
  }
}
