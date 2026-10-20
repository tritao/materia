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
   for(selection in [{part:"zAxis/motor",motor:"ELM2H-0750LA80E"},{part:"spindle/motor",motor:"ELM2L-1500LA100FT-H MS1AE"}]) {
    var selected=data.parts.filter(p->p.id==selection.part);
    check(selected.length==1&&StringTools.startsWith(selected[0].name,selection.motor),id+" shows the selected Leadshine model: "+selection.part);
   }
   check(job!=null&&job.mission!=null&&job.mission.id=="bearing-block"&&job.loop==false,id+" named nonlooping mission");
   var model=data.assemblyDefinition;if(model==null)throw "Missing model";
   var state=new AssemblyState(model,data.assemblyState);
   check(true,id+" authored coupled assembly state loads");
   var conditions=job.setupJoints;
   check(conditions!=null&&conditions.length==1&&conditions[0].joint=="workholding/half-opening"&&Math.abs(conditions[0].position-0.03)<1e-9,id+" vise condition survives artifact roundtrip in SI");
   var condition=conditions[0],tolerance=condition.tolerance;
   condition.tolerance=0;
   rejects(()->@:privateAccess SceneArtifact.validateHeader(data),"setup tolerance must be positive");condition.tolerance=tolerance;
   var setupJoint=condition.joint;condition.joint=job.axes[0];
   rejects(()->@:privateAccess SceneArtifact.validateHeader(data),"setup conditions cannot constrain CNC axes");condition.joint=setupJoint;
   var body=state.worldPose("workholding/body"),blank=state.worldPose("machining-stock");
   check(Math.abs(blank.z-body.z-(machinekit.milling.ZppWorkholding.ADAPTER+machinekit.milling.ZppWorkholding.PLATE+machinekit.milling.ZppWorkholding.PNEUMATIC_SEAT))<1e-6,id+" blank seats on actual vise jaw blocks");
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
   var zJoint=[for(j in model.joints)if(j.id==job.axes[2])j][0];
   var home=state.joint(job.axes[2]);
   var zLower=zJoint.limits.lower,zUpper=zJoint.limits.upper;
   if(zLower==null||zUpper==null)throw "Height audit needs bounded Z";
   var noseMin=gauge.z-body.z+zLower-home;
   var noseMax=gauge.z-body.z+zUpper-home;
   var tipMin=noseMin-toolLength*1000,tipMax=noseMax-toolLength*1000;
   check(tipMin<=176+1e-6,id+" tool reaches workpiece seat");
   check(tipMax>=196+5,id+" tool retracts at least 5 mm above blank");
   Sys.println(id+" height audit (mm): table datum="+body.z+", stock seat="+(blank.z-body.z)+", stock top="+(stock.z+20-body.z)+", gauge/table="+noseMin+".."+noseMax+", tool length="+(toolLength*1000)+", tip/table="+tipMin+".."+tipMax+", top clearance="+(tipMax-196));
   var machineLow=Math.POSITIVE_INFINITY,machineHigh=Math.NEGATIVE_INFINITY;
   var partBounds:Map<String,Array<Float>> = [];
   for(part in data.parts)if(part.id!="machining-target") {
    var pose=state.worldPose(part.id),low=Math.POSITIVE_INFINITY,high=Math.NEGATIVE_INFINITY;
    for(v in 0...part.vertexCount) {
     var p=F.transformPoint(pose,part.vertices.getDouble(v*24),part.vertices.getDouble(v*24+8),part.vertices.getDouble(v*24+16));
     low=Math.min(low,p.z);high=Math.max(high,p.z);
    }
    partBounds.set(part.id,[low,high]);machineLow=Math.min(machineLow,low);machineHigh=Math.max(machineHigh,high);
   }
   for(partId in ["workholding/body","workholding/plate","workholding/vise","workholding/positive-hard-jaw"])
    Sys.println(id+" "+partId+" mesh Z/table: "+[for(v in partBounds.get(partId))v-body.z]);
   Sys.println(id+" machine mesh Z="+machineLow+".."+machineHigh+", default-pose height="+(machineHigh-machineLow));
   var adapterBounds=partBounds.get("workholding/body"),plateBounds=partBounds.get("workholding/plate");
   check(Math.abs(adapterBounds[0]-body.z)<0.01&&Math.abs(adapterBounds[1]-body.z-20)<0.01,id+" adapter actual mesh is 20 mm and seats on table datum");
   check(Math.abs(plateBounds[0]-adapterBounds[1])<0.01&&Math.abs(plateBounds[1]-plateBounds[0]-30)<0.01,id+" plate actual mesh seats without gap and is 30 mm");
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
       var p=GeometryTools.pointAt(geometry,GeometryTools.length(geometry)*(sample/10)),values=[p.x+offset[0],p.y+offset[1],p.z+offset[2]];
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
