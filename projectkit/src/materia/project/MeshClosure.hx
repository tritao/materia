package materia.project;

import materia.project.SceneArtifact.SceneArtifactPart;

/** Producer-side proof for non-CAD meshes declared closed. Run once, never while clipping. */
class MeshClosure {
  public static function validate(part:SceneArtifactPart):Void {
    if(part.vertexCount<=0||part.indexCount<=0||part.indexCount%3!=0||part.vertices.length!=part.vertexCount*24||part.normals.length!=part.vertexCount*24||part.indices.length!=part.indexCount*4)throw "Closed mesh needs triangles";
    var scale=0.0;
    for(i in 0...part.vertexCount)for(a in 0...3) {
      var value=part.vertices.getDouble(i*24+a*8);
      if(!Math.isFinite(value)||!Math.isFinite(part.normals.getDouble(i*24+a*8)))throw "Closed mesh has non-finite vertices or normals";
      scale=Math.max(scale,Math.abs(value));
    }
    var tolerance=Math.max(1e-9,scale*1e-9),points:Array<String> = [];
    for(i in 0...part.vertexCount)points.push([for(a in 0...3)Std.string(Math.round(part.vertices.getDouble(i*24+a*8)/tolerance))].join(","));
    var edges=new Map<String,Int>(),uses=new Map<String,Int>(),triangles=0,signedVolume=0.0;
    function edge(a:String,b:String):Void {
      var forward=Reflect.compare(a,b)<0,key=forward?a+"|"+b:b+"|"+a;
      var n=edges.get(key),count=uses.get(key);
      edges.set(key,(n==null?0:n)+(forward?1:-1));uses.set(key,(count==null?0:count)+1);
    }
    for(t in 0...Std.int(part.indexCount/3)) {
      var ids=[for(a in 0...3)part.indices.getInt32(t*12+a*4)];
      for(id in ids)if(id<0||id>=part.vertexCount)throw "Closed mesh has an invalid index";
      var a=points[ids[0]],b=points[ids[1]],c=points[ids[2]];if(a==b||b==c||c==a)continue;
      var p=[for(id in ids)[for(axis in 0...3)part.vertices.getDouble(id*24+axis*8)]];
      var u=[for(axis in 0...3)p[1][axis]-p[0][axis]],v=[for(axis in 0...3)p[2][axis]-p[0][axis]];
      var cross=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]],agreement=0.0;
      for(id in ids)for(axis in 0...3)agreement+=cross[axis]*part.normals.getDouble(id*24+axis*8);
      for(axis in 0...3)signedVolume+=p[0][axis]*cross[axis];
      if(agreement<0)throw "Closed mesh winding disagrees with its shading normals";
      edge(a,b);edge(b,c);edge(c,a);triangles++;
    }
    if(triangles==0||signedVolume<=0)throw "Closed mesh needs outward winding and positive enclosed volume";
    for(key in edges.keys())if(edges.get(key)!=0||uses.get(key)!=2)throw "Mesh boundary is open or non-manifold";
  }
  public static function declare(part:SceneArtifactPart):Void {validate(part);part.closed=true;}
}
