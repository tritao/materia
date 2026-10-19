import gearkit.*;
import machinekit.hobbing.*;
import machinekit.component.ComponentDisplayMesh.ComponentDisplayMeshData;

class HobMeshTests {
  static var checks=0;
  static function check(ok:Bool,message:String):Void {checks++;if(!ok)throw message;}
  public static function validate(mesh:ComponentDisplayMeshData):Float {
    check(mesh.normals.length==mesh.positions.length&&mesh.positions.length%3==0&&mesh.indices.length%3==0,"invalid mesh stream lengths");
    // OCCT duplicates vertices at face/normal seams. Weld positions for topology
    // checks only, preserving the actual render normals and all triangles.
    var vertices:Map<String,Int> = [],ids:Array<Int> = [],next=0;
    for(i in 0...Std.int(mesh.positions.length/3)){
      var p=i*3,x=mesh.positions[p],y=mesh.positions[p+1],z=mesh.positions[p+2];
      check(Math.isFinite(x)&&Math.isFinite(y)&&Math.isFinite(z),"nonfinite mesh position");
      var nx=mesh.normals[p],ny=mesh.normals[p+1],nz=mesh.normals[p+2];
      check(Math.isFinite(nx)&&Math.isFinite(ny)&&Math.isFinite(nz)&&Math.abs(nx*nx+ny*ny+nz*nz-1)<1e-5,"invalid mesh normal");
      var key=Math.round(x*1e6)+":"+Math.round(y*1e6)+":"+Math.round(z*1e6);
      if(!vertices.exists(key))vertices.set(key,next++);ids.push(vertices.get(key));
    }
    var edges:Map<String,{count:Int,balance:Int}> = [],faces:Map<String,Bool> = [],volume=0.0;
    var neighbours:Map<Int,Array<Int>> = [];
    for(i in 0...Std.int(mesh.indices.length/3)){
      var a=mesh.indices[i*3],b=mesh.indices[i*3+1],c=mesh.indices[i*3+2];
      check(a>=0&&b>=0&&c>=0&&a<ids.length&&b<ids.length&&c<ids.length,"invalid triangle index");
      var u=ids[a],v=ids[b],w=ids[c];
      check(u!=v&&v!=w&&w!=u,"collapsed mesh triangle");
      var sorted=[u,v,w];sorted.sort((x,y)->x-y);var face=sorted.join(":");
      check(!faces.exists(face),"duplicate mesh face");faces.set(face,true);
      var ap=a*3,bp=b*3,cp=c*3;
      var ax=mesh.positions[ap],ay=mesh.positions[ap+1],az=mesh.positions[ap+2];
      var bx=mesh.positions[bp],by=mesh.positions[bp+1],bz=mesh.positions[bp+2];
      var cx=mesh.positions[cp],cy=mesh.positions[cp+1],cz=mesh.positions[cp+2];
      volume+=(ax*(by*cz-bz*cy)+ay*(bz*cx-bx*cz)+az*(bx*cy-by*cx))/6;
      for(pair in [[u,v],[v,w],[w,u]]){
        var lo=Std.int(Math.min(pair[0],pair[1])),hi=Std.int(Math.max(pair[0],pair[1])),key=lo+":"+hi;
        if(!edges.exists(key))edges.set(key,{count:0,balance:0});var edge=edges.get(key);
        edge.count++;edge.balance+=pair[0]==lo?1:-1;
        if(!neighbours.exists(pair[0]))neighbours.set(pair[0],[]);neighbours.get(pair[0]).push(pair[1]);
      }
    }
    for(key in edges.keys()){
      var edge=edges.get(key);check(edge.count==2,'nonmanifold or open mesh edge $key: ${edge.count} faces');
      check(edge.balance==0,'inconsistent mesh winding at $key');
    }
    var seen:Map<Int,Bool> = [],stack=[0];
    while(stack.length>0){var v=stack.pop();if(seen.exists(v))continue;seen.set(v,true);for(n in neighbours.get(v))if(!seen.exists(n))stack.push(n);}
    check(seen.keys().hasNext()&&[for(k in seen.keys())k].length==next,"disconnected mesh shells");
    check(next-[for(k in edges.keys())k].length+[for(k in faces.keys())k].length==0,"bored hob must have genus-one topology");
    check(volume>0,"mesh must be outward oriented");
    Sys.println('Hob mesh topology: vertices=$next triangles=${Std.int(mesh.indices.length/3)} signedVolumeMm3=$volume');
    return volume;
  }
  public static function run():Void {
    var scene=Sys.args().indexOf("scene");
    if(scene>=0){
      var data=materia.project.SceneArtifact.decode(sys.io.File.getBytes(Sys.args()[scene+1]));
      for(part in data.parts)validate({
        positions:[for(i in 0...part.vertexCount*3)part.vertices.getDouble(i*8)],
        normals:[for(i in 0...part.vertexCount*3)part.normals.getDouble(i*8)],
        indices:[for(i in 0...part.indexCount)part.indices.getInt32(i*4)]
      });
      Sys.println('Hob scene mesh: $checks checks passed');return;
    }
    var reference=Sys.args().indexOf("reference")>=0;
    var cases=reference?[{gashes:12,rake:0.0,starts:1}]:[{gashes:4,rake:0.0,starts:2},{gashes:4,rake:0.12,starts:2},{gashes:4,rake:-0.12,starts:2}];
    for(item in cases){
      var spec=new GearSpec(reference?0.3:1,reference?200:40,8,20*Math.PI/180);
      var design=new HobDesign(ConjugateRack.of(new ToothProfile(spec)),item.starts,32,item.gashes,item.rake);
      var tool=Hob.fromDesign(design,spec,4,16,false,true);
      var hob=new Hob(tool.values().copy().setNumber("keywayWidth",5));
      Sys.println('Hob mesh build: gashes=${item.gashes} rake=${item.rake}');Sys.stdout().flush();
      var shape=HobSolid.build(hob);
      try {
        check(shape.solidCount()==1,"mesh source must be a single fused solid");
        var mesh=HobMesh.fromShape(shape.shape,hob.parameterNumber("surfaceTolerance")),volume=validate(mesh);
        check(Math.abs(volume-shape.volume())/shape.volume()<0.005,"mesh volume differs from CAD by more than 0.5 percent");
        for(i in 0...Std.int(mesh.positions.length/3))check(mesh.positions[i*3+2]>=-1e-7&&mesh.positions[i*3+2]<=4+1e-7,"mesh crosses an end face");
      }catch(error:Dynamic){shape.close();throw error;}
      shape.close();
    }
    Sys.println('Hob mesh: $checks checks passed');
  }
}
