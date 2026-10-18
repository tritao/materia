import CadKit;
import cadkit.Shape;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.threads.ThreadSpec;
import cadkit.threads.Threads;
import cadkit.threads.ThreadedFace.ThreadStart;

class RefinementTests {
  static function check(value:Bool,label:String):Void {if(!value) throw label;}
  static function near(a:Float,b:Float,label:String):Void {check(Math.abs(a-b)<0.000001,label);}
  static function invariant(a:Part,b:Part):Void {
    check(b.valid() && b.solidCount()==a.solidCount(),"Validity and solid count");
    near(a.volume(),b.volume(),"Volume");near(a.shape.area(),b.shape.area(),"Area");
    var ab=a.shape.bounds(),bb=b.shape.bounds();
    var x=ab.get_min(),y=bb.get_min();near(x.get_x(),y.get_x(),"Min X");near(x.get_y(),y.get_y(),"Min Y");near(x.get_z(),y.get_z(),"Min Z");
    x=ab.get_max();y=bb.get_max();near(x.get_x(),y.get_x(),"Max X");near(x.get_y(),y.get_y(),"Max Y");near(x.get_z(),y.get_z(),"Max Z");
  }
  public static function main():Void {
    var a=Part.box(10,10,10),raw=Part.box(10,10,10),b=raw.translated(new Vector(5,0,0));raw.close();
    var joined=Part.fuseAll([a,b]);
    var labels=[for(i in 0...joined.shape.subshapeCount(Face)) 'mount-face-$i'];
    var named=new Part(joined.shape.withElementNames(Face,labels));
    var refined=named.refined();invariant(named,refined);
    check(refined.shape.subshapeCount(Face)==6,"Coplanar faces merged to six box faces");
    check(named.shape.subshapeCount(Face)>6,"Source remains unrefined");
    var names=refined.shape.elementNames(Face),aliases=refined.shape.elementAliases(Face);
    for(name in named.shape.elementNames(Face)) {
      var found=names.indexOf(name)>=0;
      for(list in aliases) if(list.indexOf(name)>=0) found=true;
      check(found,'Merged face reference lost: $name');
    }
    var again=refined.refined();invariant(refined,again);
    check(again.shape.subshapeCount(Edge)==refined.shape.subshapeCount(Edge),"Idempotent edge count");
    check(haxe.Json.stringify(again.shape.elementNames(Face))==haxe.Json.stringify(names),"Stable repeated face names");
    var path="/tmp/cadkit-refinement-roundtrip.step";refined.exportStep(path);
    var imported=new Part(Shape.importStep(path));invariant(refined,imported);sys.FileSystem.deleteFile(path);
    // Refine a primitive threaded cylinder through the operation's metadata path.
    var cylinder=Part.cylinder(3,10);
    var faceNames=cylinder.shape.elementNames(Face),wall="";
    for(i in 0...faceNames.length) {
      var face=cylinder.shape.subshape(Face,i);
      if(face.surfaceKind()==Cylinder) wall=faceNames[i];face.close();
    }
    var thread=Threads.attach(cylinder.shape,wall,new ThreadSpec("M6",6,1),AxisMinimum,8);
    var finished=thread.refined();
    check(finished.threadedFaces().length==1,"Thread interval survives refinement");
    var threadLength=finished.threadedFaces()[0].length;
    if(threadLength==null) throw "Thread interval lost its length";
    near(threadLength,8,"Thread length");
    // Separate bodies stay separate, even when their planes share a domain.
    var distant=a.translated(new Vector(30,0,0));
    var compound=new Part(Shape.compound([a.shape,distant.shape]));
    var separated=compound.refined();invariant(compound,separated);check(separated.solidCount()==2,"No cross-solid joining");
    separated.close();compound.close();distant.close();finished.close();thread.close();cylinder.close();
    imported.close();again.close();refined.close();named.close();joined.close();b.close();a.close();
    Sys.println("Refinement passed: geometry, naming aliases, idempotence, source ownership, STEP roundtrip, threads, separate solids");
  }
}
