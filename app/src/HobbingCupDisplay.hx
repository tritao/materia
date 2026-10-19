package app;

import nativekit.scene.GeometryData;
import haxe.io.Bytes;

/** Analytical uncut cup wall/end, combined with the actual cut tooth-band mesh.
    Internal joining faces are omitted; handling offsets never mutate RingStock. */
class HobbingCupDisplay {
  public static function body(inner:Float,outer:Float,band:Float,height:Float,end:Float,center:Array<Float>):GeometryData {
    var geometry=CutterGeometry.sectionGeometry([{r:outer,z:band},{r:outer,z:height},{r:0.0,z:height},
      {r:0.0,z:height-end},{r:inner,z:height-end},{r:inner,z:band}],
      (x,y,z)->[x/1000-center[0],y/1000-center[1],z/1000-center[2]],(x,y,z)->[x,y,z],256);
    var colors=geometry.stream(6);if(colors!=null)for(i in 0...colors.bytes.length)colors.bytes.set(i,255);
    return geometry;
  }
  public static function combine(band:GeometryData,body:GeometryData,center:Array<Float>,root:Float,bandTop:Float):GeometryData {
    var a=band.vertexCount(),b=body.vertexCount(),positions=Bytes.alloc((a+b)*12),normals=Bytes.alloc((a+b)*12),colors=Bytes.alloc((a+b)*4);
    var lo=[1e30,1e30,1e30],hi=[-1e30,-1e30,-1e30],at=0;
    for(source in [band,body]) {
      var p=source.stream(1),n=source.stream(2),c=source.stream(6);
      if(p==null||n==null)throw "Cup mesh requires positions and normals";
      for(i in 0...source.vertexCount()) {
        var x=p.bytes.getFloat(i*12),y=p.bytes.getFloat(i*12+4),z=p.bytes.getFloat(i*12+8);
        // The upper band cap begins at the body's outside, not at its bore: remove the buried joining face.
        if(source==band&&n.bytes.getFloat(i*12+8)>0.9&&Math.abs((z+center[2])*1000-bandTop)<1e-4) {
          var rx=(x+center[0])*1000,ry=(y+center[1])*1000,r=Math.sqrt(rx*rx+ry*ry);
          if(r<root){x=rx*root/r/1000-center[0];y=ry*root/r/1000-center[1];}
        }
        var point=[x,y,z];
        for(axis in 0...3){positions.setFloat(at*12+axis*4,point[axis]);normals.setFloat(at*12+axis*4,n.bytes.getFloat(i*12+axis*4));lo[axis]=Math.min(lo[axis],point[axis]);hi[axis]=Math.max(hi[axis],point[axis]);}
        for(channel in 0...4)colors.set(at*4+channel,c==null?(channel==3?255:200):c.bytes.get(i*4+channel));
        at++;
      }
    }
    var corners:Array<Int> = [];
    for(source in [band,body])for(t in 0...source.triangleCount()) {
      var local=[for(k in 0...3)source.indexAt(t*3+k)],offset=source==body?a:0;
      // Omit the band/body joining cap, including whole radial cells beneath the body.
      var buried=source==band;
      for(i in local){var p=source.stream(1),n=source.stream(2);if(p==null||n==null)throw "Cup mesh streams absent";var rx=(p.bytes.getFloat(i*12)+center[0])*1000,ry=(p.bytes.getFloat(i*12+4)+center[1])*1000;
        if(n.bytes.getFloat(i*12+8)<0.9||Math.abs((p.bytes.getFloat(i*12+8)+center[2])*1000-bandTop)>1e-4||Math.sqrt(rx*rx+ry*ry)>root+1e-5)buried=false;
      }
      if(buried)continue;
      var ids=[for(i in local)i+offset],u=[for(k in 0...3)positions.getFloat(ids[1]*12+k*4)-positions.getFloat(ids[0]*12+k*4)],
        v=[for(k in 0...3)positions.getFloat(ids[2]*12+k*4)-positions.getFloat(ids[0]*12+k*4)];
      var cross=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]];
      if(cross[0]*cross[0]+cross[1]*cross[1]+cross[2]*cross[2]>1e-30)for(i in ids)corners.push(i);
    }
    var indices=Bytes.alloc(corners.length*4);for(i in 0...corners.length)indices.setInt32(i*4,corners[i]);
    return new GeometryData().addStream(1,2,positions,a+b,12).addStream(2,2,normals,a+b,12).addStream(6,4,colors,a+b,4)
      .setIndexBuffer(indices,corners.length).setBounds(lo[0],lo[1],lo[2],hi[0],hi[1],hi[2]);
  }
}
