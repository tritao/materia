import cadkit.Shape;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.modeling.Plane;
import cadkit.modeling.Location;
import cadkit.modeling.AssemblyState;
import machinekit.assembly.MachineAssembly;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Narrow phase for coarse flags, using the same live states as the access study. */
class TendingExactCad {
  final local:Map<String,Shape> = [];
  final placed:Map<String,{key:String,shape:Shape}> = [];
  final factories:Map<String,Void->Shape> = [];
  final fs:AssemblyState;
  final armState:AssemblyState;
  final ts:AssemblyState;
  final placement:AssemblyFrame;
  final directory:String;
  public final sources:Array<{id:String,sha256:String}> = [];
  var wrist:AssemblyFrame;
  public final checks:Array<Dynamic> = [];
  public function new(fixtures:MachineAssembly,fs:AssemblyState,arm:MachineAssembly,armState:AssemblyState,
    ts:AssemblyState,placement:AssemblyFrame,offset:Array<Float>,topEntry:Bool,directory:String) {
    this.directory=directory;
    this.fs=fs;this.armState=armState;this.ts=ts;this.placement=placement;wrist=AssemblyFrames.identity();
    for(e in fixtures.components()) {var component=e.component;factories.set('cell/'+e.id,()->{var p=component.geometry();var shape=p.shape.cloneShape();p.close();return shape;});}
    for(e in arm.components()) {var component=e.component;factories.set('arm/'+e.id,()->{var p=component.geometry();var shape=p.shape.cloneShape();p.close();return shape;});}
    for(o in ts.definition.occurrences) {
      var id=o.id,file='/build/expert/'+id+'.step';
      if(id=='adapter')file='/build/dual/adapter.step';
      if(StringTools.startsWith(id,'a-') || StringTools.startsWith(id,'b-')) {
        var member=id.substr(2);
        if(member=='pgc-140-50')file='/build/pgc-140-50-local.step';
        if(member=='jaw-left' || member=='jaw-right')file='/build/'+member+'.step';
        if(member=='finger-left' || member=='finger-right')file='/build/cnc-aluminium/'+member+'.step';
        if(member=='blank-60') {
          if(topEntry) {
            factories.set('tool/'+id,()->{var p=TendingElectricAccess.topBlank();var moved=p.translated(new Vector(offset[0],offset[1],offset[2]));p.close();var shape=moved.shape.cloneShape();moved.close();return shape;});
            continue;
          }
          file='/build/blank-60-local.step';
        }
      }
      var source=DhGripperPreview.directory()+file;
      factories.set('tool/'+id,()->Shape.importStep(source));
    }
  }
  static function ensureDirectory(path:String):Void {
    if(sys.FileSystem.exists(path))return;
    var parent=haxe.io.Path.directory(path);if(parent!=path && parent!="")ensureDirectory(parent);
    sys.FileSystem.createDirectory(path);
  }
  static function digest(bytes:haxe.io.Bytes):String {
    var hash=haxe.crypto.Sha256.make(bytes),out=new StringBuf();
    for(i in 0...hash.length)out.add(StringTools.hex(hash.get(i),2).toLowerCase());return out.toString();
  }
  public function toolPose(value:AssemblyFrame):Void wrist=value;
  function pose(id:String):AssemblyFrame {
    return StringTools.startsWith(id,'cell/') ? fs.worldPose(id.substr(5))
      : StringTools.startsWith(id,'arm/') ? AssemblyFrames.compose(placement,armState.worldPose(id.substr(4)))
      : AssemblyFrames.compose(wrist,ts.worldPose(id.substr(5)));
  }
  function world(id:String):Shape {
    var raw=local.get(id);
    if(raw==null) {
      var factory=factories.get(id);if(factory==null)throw 'No exact CAD source for $id';
      raw=factory();local.set(id,raw);
      var file=directory+'/exact-cad/'+id+'.step';ensureDirectory(haxe.io.Path.directory(file));raw.exportStep(file);
      sources.push({id:'exact-cad/'+id+'.step',sha256:digest(sys.io.File.getBytes(file))});
    }
    var pose=pose(id);
    var key=haxe.Json.stringify(pose),prior=placed.get(id);if(prior!=null && prior.key==key)return prior.shape;
    if(prior!=null)prior.shape.close();
    var m=AssemblyFrames.toRotationMatrix(pose);
    var shape=new Location(new Plane(new Vector(pose.x,pose.y,pose.z),new Vector(m[0],m[3],m[6]),new Vector(m[2],m[5],m[8]))).apply(raw);
    placed.set(id,{key:key,shape:shape});return shape;
  }
  public function query(a:String,b:String,coarse:Float,station:String,side:String):{distanceMm:Float,overlapMm3:Float} {
    var x=world(a),y=world(b),gap=x.distance(y),volume=0.0;
    if(gap<0.001){var common=x.common(y);if(common.subshapeCount(CadKit.ShapeKind.Solid)>0)volume=common.volume();common.close();}
    checks.push({station:station,side:side,first:a,second:b,coarseDistanceMm:coarse,distanceMm:gap,overlapMm3:volume,wrist:wrist,openingA:ts.joint("a-jaw-opening"),openingB:ts.joint("b-jaw-opening"),
      firstPose:pose(a),secondPose:pose(b)});
    return {distanceMm:gap,overlapMm3:volume};
  }
  public function close():Void {for(p in placed)p.shape.close();for(s in local)s.close();}
}
