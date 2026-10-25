package machinekit.motion;

import haxe.io.Bytes;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Hollow runs and 32-section bend matching CableChain.geometryAt. No CAD calls;
 * vertex/normal storage is retained and index storage changes only at topology boundaries. */
class PackedCableChainMesh {
 public var positions(default,null)=Bytes.alloc(1120*12);
 public var normals(default,null)=Bytes.alloc(1120*12);
 public var indices(default,null)=Bytes.alloc(0);
 public var vertexCount(default,null)=0;
 public var indexCount(default,null)=0;
 public final minimum=[0.0,0.0,0.0];
 public final maximum=[0.0,0.0,0.0];
 final rings:Array<Float>=[for(_ in 0...35*24)0.0];
 final transformed:{x:Float,y:Float,z:Float}={x:0.0,y:0.0,z:0.0};
 var count=0;
 public function new() {}
 public function update(chain:CableChain,position:Float,scale:Float,center:Array<Float>,?frame:AssemblyFrame):Void {
  chain.movingFrame(position);
  if(!Math.isFinite(scale)||scale<=0||center.length!=3)throw "Invalid chain display scale/centre";
  for(v in center)if(!Math.isFinite(v))throw "Invalid chain display centre";
  count=0;vertexCount=0;indexCount=0;
  for(i in 0...3){minimum[i]=Math.POSITIVE_INFINITY;maximum[i]=Math.NEGATIVE_INFINITY;}
  var delta=position-chain.n("travel")/2,lower=position/2,r=chain.n("bendRadius");
  section(chain,0,0,0,scale,center,frame);
  if(lower>1e-9)section(chain,lower,0,0,scale,center,frame);
  for(i in 1...33){var a=Math.PI*i/32;section(chain,lower+r*Math.sin(a),r*(1-Math.cos(a)),a,scale,center,frame);}
  if(lower-delta>1e-9)section(chain,delta,2*r,Math.PI,scale,center,frame);
  for(j in 0...count-1)for(k in 0...4){
   var a=j*8+k,b=j*8+(k+1)%4;
   quad(a,b,b+8,a+8);
   quad(a+4,a+12,b+12,b+4);
  }
  for(k in 0...4){
   quad(k,k+4,(k+1)%4+4,(k+1)%4);
   var end=(count-1)*8;quad(end+k,end+(k+1)%4,end+(k+1)%4+4,end+k+4);
  }
  indexCount=Std.int(vertexCount/4)*6;
  if(indices.length!=indexCount*4){
   indices=Bytes.alloc(indexCount*4);
   for(q in 0...Std.int(vertexCount/4))for(k in 0...6)
    indices.setInt32((q*6+k)*4,q*4+([0,1,2,0,2,3][k]));
  }
 }
 function section(chain:CableChain,y:Float,z:Float,a:Float,scale:Float,center:Array<Float>,frame:Null<AssemblyFrame>):Void {
  var w=chain.n("innerWidth"),h=chain.n("innerHeight"),wall=chain.n("wall");
  for(layer in 0...2)for(k in 0...4){
   var x=(k==0||k==3?-1:1)*(w+(layer==0?2*wall:0))/2;
   var sign=k<2?-1:1,half=(h+(layer==0?2*wall:0))/2;
   var yy=y-sign*half*Math.sin(a),zz=z+sign*half*Math.cos(a);
   if(frame!=null){F.transformPointInto(frame,x,yy,zz,transformed);x=transformed.x;yy=transformed.y;zz=transformed.z;}
   var offset=(count*8+layer*4+k)*3;
   rings[offset]=x*scale-center[0];rings[offset+1]=yy*scale-center[1];rings[offset+2]=zz*scale-center[2];
  }
  count++;
 }
 function quad(a:Int,b:Int,c:Int,d:Int):Void {
  var swap=b;b=d;d=swap;
  a*=3;b*=3;c*=3;d*=3;
  var ux=rings[b]-rings[a],uy=rings[b+1]-rings[a+1],uz=rings[b+2]-rings[a+2];
  var vx=rings[c]-rings[a],vy=rings[c+1]-rings[a+1],vz=rings[c+2]-rings[a+2];
  var nx=uy*vz-uz*vy,ny=uz*vx-ux*vz,nz=ux*vy-uy*vx,length=Math.sqrt(nx*nx+ny*ny+nz*nz);
  if(length<1e-20)return;
  for(k in 0...4){
   var source=switch k{case 0:a;case 1:b;case 2:c;default:d;};
   var offset=vertexCount*12;
   for(axis in 0...3){var v=rings[source+axis];positions.setFloat(offset+axis*4,v);minimum[axis]=Math.min(minimum[axis],v);maximum[axis]=Math.max(maximum[axis],v);}
   normals.setFloat(offset,nx/length);normals.setFloat(offset+4,ny/length);normals.setFloat(offset+8,nz/length);vertexCount++;
  }
 }
}
