package tests;

import app.CadGeometryData;
import app.CadSceneGeometry;
import app.CadEdgeMode;
import app.EditorScene;
import cadkit.modeling.Part;
import haxe.io.Bytes;
import materia.project.SceneArtifact.SceneArtifactPart;

@:access(nativekit.scene.GeometryData)
@:access(app.EditorScene)
class CadFeatureEdgeTests {
  static var checks=0;
  static function check(ok:Bool,message:String):Void {checks++;if(!ok)throw message;}
  static function sample(crease:Bool=false,open:Bool=false):SceneArtifactPart {
    var vertices=Bytes.alloc(6*24),normals=Bytes.alloc(6*24),indices=Bytes.alloc((open ? 3 : 6)*4);
    var points=[[0.0,0,0],[1.0,0,0],[0.0,1,0],[1.0,0,0],[0.0,0,0],[1.0,-1,0]];
    for(i in 0...6) {
      for(axis in 0...3) {vertices.setDouble(i*24+axis*8,points[i][axis]);normals.setDouble(i*24+axis*8,axis==(crease && i>=3 ? 1 : 2) ? 1 : 0);}
      if(i<(open ? 3 : 6))indices.setInt32(i*4,i);
    }
    var edges=Bytes.alloc(48);edges.setDouble(24,1);
    var ids=Bytes.alloc(4);ids.setInt32(0,17);
    return {id:"sample",name:"sample",red:1,green:1,blue:1,vertexCount:6,indexCount:open ? 3 : 6,
      vertices:vertices,normals:normals,indices:indices,edgeSegments:edges,edgeIds:ids,faceRanges:[]};
  }
  static function classify(part:SceneArtifactPart):Array<Bool>
    return cadkit.MeshFeatureEdges.classify(part.vertices,part.normals,part.indices,part.vertexCount,part.indexCount,part.edgeSegments,part.edgeIds);
  public static function main():Void {
    check(!classify(sample())[0],"Coplanar face split is hidden");
    check(classify(sample(true))[0],"Sharp crease stays visible");
    check(classify(sample(false,true))[0],"Open boundary stays visible");
    var cylinder=Part.cylinder(10,30);
    var mesh=cylinder.shape.tessellate();
    var data:CadGeometryData=cast CadSceneGeometry.fromMesh(mesh,cylinder.shape);
    var features=data.strokeSegments.length;
    check(features>0,"Cylinder keeps real end rims");
    var featureIds=[for(stroke in data.strokeSegments)stroke.get_subelement()];
    var streams=data.vertexCount();
    data.setEdgeMode(All);var all=data.strokeSegments.length;
    check(all>features,"Cylinder periodic seam is hidden only in Feature mode");
    data.setEdgeMode(Shaded);check(data.strokeSegments.length==0,"Shaded omits CAD edge overlay");
    data.setEdgeMode(Feature);check(data.strokeSegments.length==features,"Feature mode restores exact stroke count");
    for(i in 0...features)check(data.strokeSegments[i].get_subelement()==featureIds[i],"Edge selection identity survives display changes");
    check(data.vertexCount()==streams,"Display modes do not alter surface geometry");
    var scene=new EditorScene([]);scene.createRectangle();
    var revision=scene.revision,environment=scene.environmentRevision;
    scene.setRuntimeGeometry(scene.selectedId,data);scene.setCadEdgeMode(All);
    check(data.strokeSegments.length==all,"Scene publishes All mode to live CAD geometry");
    scene.setCadEdgeMode(Shaded);check(data.strokeSegments.length==0,"Scene publishes Shaded mode");
    check(scene.revision==revision && scene.environmentRevision==environment,"Display changes leave document and physics revisions unchanged");
    scene.dispose();
    var generated:Map<String,nativekit.scene.GeometryData> = ["profile"=>data];
    var members:Array<app.SceneObjectData> = [for(i in 0...2) {
      id:'post$i',label:'Post $i',type:"cad-preview",x:i*0.1,y:0.0,z:0.0,
      width:0.04,height:0.8,depth:0.04,collisionEnabled:false,dynamicBody:false,
      mass:1.0,red:0.7,green:0.7,blue:0.7,visible:true,meshSnapshot:"profile"
    }];
    var loaded=new EditorScene(members,null,null,generated);
    check(data.strokeSegments.length==features,"Batch loading applies the initial Feature mode");
    loaded.setCadEdgeMode(Shaded);
    check(data.strokeSegments.length==0,"Shaded reaches shared batch-loaded assembly geometry");
    loaded.setCadEdgeMode(All);
    check(data.strokeSegments.length==all,"All restores batch-loaded assembly edges");
    loaded.setCadEdgeMode(Feature);
    check(data.strokeSegments.length==features,"Feature restores batch-loaded assembly edges");
    loaded.setCadEdgeMode(Shaded);
    var later:CadGeometryData=cast app.CadPreviewGeometry.fromArtifact(sample(true),[0,-1,0],[1,1,0],0.001);
    generated.set("later",later);
    var next:app.SceneObjectData=Reflect.copy(members[0]);
    next.id="later";next.meshSnapshot="later";
    loaded.addObjects([next]);
    check(later.strokeSegments.length==0,"A later batch inherits the active Shaded mode");
    loaded.setCadEdgeMode(All);
    check(later.strokeSegments.length==1,"Later batch geometry participates in subsequent mode changes");
    loaded.setCadEdgeMode(Shaded);
    var detached:CadGeometryData=cast app.CadPreviewGeometry.fromArtifact(sample(true),[0,-1,0],[1,1,0],0.001);
    loaded.setRuntimeGeometry("post0",detached);
    check(detached.strokeSegments.length==0,"Runtime CAD replacements inherit Shaded mode");
    loaded.setRuntimeGeometry("post0",new nativekit.scene.GeometryData());
    loaded.setCadEdgeMode(All);
    check(detached.strokeSegments.length==0,"Replaced CAD data no longer receives display updates");
    loaded.clearRuntimeGeometry("post0");
    loaded.setCadEdgeMode(Shaded);
    check(data.strokeSegments.length==0,"Restored shared assembly geometry remains registered");
    loaded.setCadEdgeMode(Feature);
    check(data.strokeSegments.length==features,"Restored assembly geometry retains its feature edges");
    loaded.dispose();cylinder.close();

    var growing=new CadGeometryData([], [0,0,0], 1);
    growing.setEdgeMode(Shaded);
    growing.addCadStroke([0,0,0,1,0,0],1,true);
    check(growing.strokeSegments.length==0,"Edges appended in Shaded mode stay hidden");
    growing.setEdgeMode(All);
    growing.addCadStroke([0,0,0,0,1,0],2,false);
    check(growing.strokeSegments.length==2,"All mode includes newly appended smooth edges");
    growing.setEdgeMode(Feature);
    check(growing.strokeSegments.length==1,"Feature mode consistently filters appended edges");
    Sys.println('CAD feature edge checks passed: $checks');
  }
}
