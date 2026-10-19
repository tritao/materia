package app;

import materia.project.SceneArtifact.SceneArtifactPart;

private typedef EdgeSides = {var normals:Array<Array<Float>>;}

/** Conservative display classification for existing CAD artifacts, without changing their topology.
 * Only matching triangle-boundary segments with two agreeing analytic normal pairs are smooth.
 * Open, non-manifold, unmatched or degenerate segments remain visible.
 */
class CadFeatureEdges {
  public static function classify(part:SceneArtifactPart):Array<Bool> {
    var strokes=part.edgeSegments;
    if(strokes==null)return [];
    var count=Std.int(strokes.length/48),keys:Array<String> = [],requested:Map<String,EdgeSides> = [];
    function point(x:Float,y:Float,z:Float):String {
      // Source CAD units are mm; keep distinct features above OCCT's nominal modelling tolerance.
      return Std.string(Math.ffloor(x/1e-7+0.5))+":"+Std.string(Math.ffloor(y/1e-7+0.5))+":"+Std.string(Math.ffloor(z/1e-7+0.5));
    }
    function pair(a:String,b:String):String return a<b ? a+"/"+b : b+"/"+a;
    for(i in 0...count) {
      var o=i*48,key=pair(point(strokes.getDouble(o),strokes.getDouble(o+8),strokes.getDouble(o+16)),
        point(strokes.getDouble(o+24),strokes.getDouble(o+32),strokes.getDouble(o+40)));
      keys.push(key);requested.set(key,{normals:[]});
    }
    var vertices:Array<String> = [];
    for(i in 0...part.vertexCount)vertices.push(point(part.vertices.getDouble(i*24),part.vertices.getDouble(i*24+8),part.vertices.getDouble(i*24+16)));
    for(i in 0...Std.int(part.indexCount/3)) {
      var triangle=[part.indices.getInt32(i*12),part.indices.getInt32(i*12+4),part.indices.getInt32(i*12+8)];
      for(j in 0...3) {
        var a=triangle[j],b=triangle[(j+1)%3];
        if(a<0 || b<0 || a>=part.vertexCount || b>=part.vertexCount)throw "CAD feature edge index out of bounds";
        var sides=requested.get(pair(vertices[a],vertices[b]));
        if(sides==null)continue;
        // Order endpoint normals by position so opposite triangle winding cannot swap them.
        if(vertices[a]>vertices[b]){var swap=a;a=b;b=swap;}
        var normals:Array<Float> = [];
        for(v in [a,b])for(axis in 0...3)normals.push(part.normals.getDouble(v*24+axis*8));
        sides.normals.push(normals);
      }
    }
    var threshold=Math.cos(Math.PI/180),result:Array<Bool> = [],features:Map<Int,Bool> = [];
    for(i in 0...count) {
      var sides=requested.get(keys[i]),smooth=sides!=null && sides.normals.length==2;
      if(smooth && sides!=null)for(endpoint in 0...2) {
        var a=sides.normals[0],b=sides.normals[1],o=endpoint*3;
        var aa=a[o]*a[o]+a[o+1]*a[o+1]+a[o+2]*a[o+2];
        var bb=b[o]*b[o]+b[o+1]*b[o+1]+b[o+2]*b[o+2];
        var dot=a[o]*b[o]+a[o+1]*b[o+1]+a[o+2]*b[o+2];
        if(!Math.isFinite(dot+aa+bb) || aa<1e-20 || bb<1e-20 || dot/Math.sqrt(aa*bb)<threshold)smooth=false;
      }
      result.push(!smooth);
      if(!smooth && part.edgeIds!=null && part.edgeIds.length==count*4)features.set(part.edgeIds.getInt32(i*4),true);
    }
    // Never draw only part of one BRep edge when an endpoint/triangulation is uncertain.
    if(part.edgeIds!=null && part.edgeIds.length==count*4)for(i in 0...count)
      if(features.exists(part.edgeIds.getInt32(i*4)))result[i]=true;
    return result;
  }
}
