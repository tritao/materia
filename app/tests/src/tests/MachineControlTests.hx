package tests;

import app.MachineControlModel;
import app.ProjectDocumentSession;
import app.SceneObjectData;
import cadkit.modeling.AssemblyModel;
import materia.assembly.AssemblyFrames;

class MachineControlTests {
  static var checks=0;
  static function check(ok:Bool,message:String):Void {checks++;if(!ok)throw message;}
  static function near(a:Null<Float>,b:Float,message:String):Void {if(a==null)throw message+": missing value";check(Math.abs(a-b)<1e-7,message+': $a != $b');}
  static function refused(fn:Void->Void,message:String):Void {var rejected=false;try fn() catch(_:Dynamic)rejected=true;check(rejected,message);}
  static function slidePosition(session:ProjectDocumentSession):Float {
    var object=session.scene.object("project:slide");if(object==null)throw "Missing slide";return object.x;
  }
  static function fixture(unit:String):ProjectDocumentSession {
    var model=new AssemblyModel(unit);
    for(id in ["base","slide","screw","head"]) {model.add(id);model.connector(id,"axis",AssemblyFrames.identity());}
    model.mateOnAxis("x","prismatic","base","axis","slide","axis",{x:1,y:0,z:0},0,
      {lower:unit=="mm" ? -100 : -0.1,upper:unit=="mm" ? 100 : 0.1,velocity:null,effort:null});
    model.mateOnAxis("screw-turn","continuous","base","axis","screw","axis",{x:0,y:0,z:1});
    model.mateOnAxis("tilt","revolute","slide","axis","head","axis",{x:0,y:0,z:1},0,
      {lower:-Math.PI/2,upper:Math.PI/2,velocity:null,effort:null});
    model.couple("drive","x","screw-turn",unit=="mm" ? Math.PI*2/5 : Math.PI*2/0.005);
    var data:Array<SceneObjectData> = [for(id in ["base","slide","screw","head"])
      {id:"project:"+id,label:id,type:"rectangle",x:0.0,y:0.0,z:0.0,width:0.01,height:0.01,depth:0.01,
        collisionEnabled:false,dynamicBody:false,mass:1.0,red:0.5,green:0.5,blue:0.5,visible:true}];
    var centers:Map<String,Array<Float>> = [];for(id in ["base","slide","screw","head"])centers.set(id,[0,0,0]);
    var session=new ProjectDocumentSession(null,false);
    session.openGeneratedScene(data,null,null,model.definition("machine-controls"),null,centers,unit=="mm" ? 0.001 : 1.0);
    return session;
  }
  public static function main():Int {
    var empty=new ProjectDocumentSession(null,false);check(MachineControlModel.axes(empty).length==0,"empty documents have no axes");empty.dispose();
    for(unit in ["mm","m"]) {
      var session=fixture(unit);
      try {
        var axes=MachineControlModel.axes(session);check(axes.length==3,"movable joints only");
        var x=axes.filter(a->a.id=="x")[0],turn=axes.filter(a->a.id=="screw-turn")[0],tilt=axes.filter(a->a.id=="tilt")[0];
        near(x.minimum,-100,"linear lower displayed in mm");near(x.maximum,100,"linear upper displayed in mm");
        near(tilt.maximum,90,"rotary limits displayed in degrees");check(turn.follows=="x","coupled targets are read-only");
        var history=session.document.history.undoCount;
        check(MachineControlModel.set(session,"x",75,true),"axis change applies");
        near(slidePosition(session),0.075,"axis moves geometry in metres");
        near(MachineControlModel.axes(session).filter(a->a.id=="screw-turn")[0].value,15*360,"screw follows axis");
        check(session.document.history.undoCount==history+1,"axis changes are undoable");
        check(session.document.undo(),"undo available");near(slidePosition(session),0,"undo restores pose");
        check(session.document.redo(),"redo available");near(slidePosition(session),0.075,"redo restores pose");
        refused(()->{MachineControlModel.set(session,"x",101,true);},"out-of-range rejected");
        refused(()->{MachineControlModel.set(session,"x",Math.NaN,true);},"NaN rejected");
        refused(()->{MachineControlModel.set(session,"x",0,false);},"runtime ownership lock respected");
        refused(()->{MachineControlModel.set(session,"screw-turn",0,true);},"coupling target cannot be driven");
        refused(()->{MachineControlModel.set(session,"missing",0,true);},"unknown joint rejected");
        near(MachineControlModel.axes(session).filter(a->a.id=="x")[0].value,75,"rejected edits leave coordinate unchanged");
        check(MachineControlModel.set(session,"x",-100,true),"lower stop reachable");
        check(MachineControlModel.set(session,"x",100,true),"upper stop reachable");
        check(MachineControlModel.set(session,"tilt",90,true),"rotary stop reachable");
        near(MachineControlModel.axes(session).filter(a->a.id=="tilt")[0].value,90,"rotary position uses degrees");
        session.dispose();
      } catch(error:Dynamic) {session.dispose();throw error;}
    }
    near(MachineControlModel.parsePosition(" -1.25e2 "),-125,"numeric entry accepts scientific notation");
    for(text in ["","-","1abc","NaN","Infinity"]) refused(()->{MachineControlModel.parsePosition(text);},"invalid numeric input rejected");
    check(EditorWorkspaceLayoutTests.main()==0,"machine tab layout regression");
    trace('MachineControlTests passed ($checks checks)');return 0;
  }
}
