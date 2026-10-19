import materia.project.SceneArtifact;
import cadkit.modeling.AssemblyState;
import materia.assembly.AssemblyFrames as F;
import cnckit.CncCompiler;
import cnckit.CncController;
import toolpathkit.tool.Tool;
import toolpathkit.tool.CutterProfile;
import toolpathkit.path.Point3;
import toolpathkit.path.ToolpathFrame;
import toolpathkit.path.GeometryTools;

/** Checks the authored G-code against the actual configured machine and tool datums. */
class MillMachiningChecks {
 static var checks=0;
 static function check(ok:Bool,message:String):Void {checks++;if(!ok)throw message;}
 static function rejects(action:Void->Void,message:String):Void {
  var rejected=false;try action()catch(_:Dynamic)rejected=true;check(rejected,message);
 }
 public static function main():Void {
  var root=Sys.getEnv("MATERIA_ROOT"),dir=root+"/machinekit/examples/bench-mill/build/mill";
  sys.FileSystem.createDirectory(dir);
  for(id in ["small","large"]) {
   var bytes=id=="small"?MillProject.small():MillProject.large();
   var data:materia.project.SceneArtifact.SceneArtifactData=SceneArtifact.decode(bytes);
   var job=data.machining;if(job==null)throw id+" has no machining job";
   check(job!=null&&job.mission!=null&&job.mission.id=="bearing-block"&&job.loop==false,id+" named nonlooping mission");
   var model=data.assemblyDefinition;if(model==null)throw "Missing model";
   var state=new AssemblyState(model);
   check(job.kinematicPreview==true,id+" explicit kinematic mode survives artifact roundtrip");
   job.loop=true;rejects(()->@:privateAccess SceneArtifact.validateHeader(data),"named missions cannot loop");job.loop=false;
   var mission=job.mission;if(mission==null)throw "Missing mission";
   var label=mission.label;mission.label="";
   rejects(()->@:privateAccess SceneArtifact.validateHeader(data),"mission needs a label");mission.label=label;
   var axis:materia.assembly.AssemblyDefinition.KinematicJoint=[for(j in model.joints)if(j.id==job.axes[0])j][0];
   var acceleration=axis.limits.acceleration;axis.limits.acceleration=null;
   rejects(()->@:privateAccess SceneArtifact.validateHeader(data),"kinematic previews need explicit acceleration bounds");axis.limits.acceleration=acceleration;
   check(data.missions==null,id+" CNC mission does not invent a robot program");
   var gauge=state.worldPose(job.spindle),stock=state.worldPose(job.stocks[0]),tip=state.worldPose(job.toolPart);
   var toolLength=job.tools[0].length;
   check(Math.abs(gauge.z-tip.z-toolLength*1000)<1e-6,id+" tool length includes BT30 holder extension");
   check(Math.abs((stock.z+20-gauge.z)/1000-job.workOffset[2])<1e-6,id+" G54 derives from real stock and spindle");
   // Check the same corner after simultaneous X/Y/Z displacement, including signs and nested joints.
   for(i in 0...3)state.setJoint(job.axes[i],i==2?-50:10);
   var moved=F.compose(F.inverse(state.worldPose(job.spindle)),state.worldPose(job.stocks[0]));
   var original=F.compose(F.inverse(gauge),stock);
   check(Math.abs(moved.x-original.x+10)<1e-6&&Math.abs(moved.y-original.y+10)<1e-6&&Math.abs(moved.z-original.z-50)<1e-6,id+" machine coordinates follow actual relative motion");
   var controller=new CncController();
   for(tool in job.tools)controller.toolLibrary.set(Tool.shaped(tool.number,tool.length,CutterProfile.decode(tool.profile)));
   controller.setWorkOffset(54,job.workOffset[0],job.workOffset[1],job.workOffset[2]);
   var compiled=CncCompiler.compileDetailed(job.program,controller,new Point3(0,0,0));
   check(compiled.diagnostics.length==0,id+" G-code compiles: "+compiled.diagnostics);
   var frame=ToolpathFrame.of(compiled.program),moves=0;
   for(op in compiled.program.ops) {
    frame.advance(op);
    switch(op) {
     case Move(_,geometry,_,_,_):
      var offset=frame.toMachine();
      for(sample in 0...11) {
       var p=GeometryTools.pointAt(geometry,GeometryTools.length(geometry)*sample/10),values=[p.x+offset[0],p.y+offset[1],p.z+offset[2]];
       for(i in 0...3) {
        var joint:materia.assembly.AssemblyDefinition.KinematicJoint=[for(j in model.joints)if(j.id==job.axes[i])j][0];
        var lower=joint.limits.lower,upper=joint.limits.upper;
        if(lower==null||upper==null)throw "Unbounded axis";
        check(values[i]>=lower/1000-1e-7&&values[i]<=upper/1000+1e-7,id+" program exceeds "+job.axes[i]+": "+values[i]);
       }
      }
      moves++;
     default:
    }
   }
   check(moves>10,id+" has actual cutting motion");
   sys.io.File.saveBytes(dir+"/"+id+".mtrg",bytes);
   Sys.println(id+" bearing-block mission: "+moves+" moves, correct datums and travel");
  }
  Sys.println("Mill machining missions: "+checks+" checks passed");
 }
}
