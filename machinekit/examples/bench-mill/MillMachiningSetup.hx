import cadkit.modeling.AssemblyState;
import materia.assembly.AssemblyFrames as F;
import materia.project.SceneArtifact;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.AssemblyPreview;
import machinekit.milling.*;
import toolpathkit.setup.Fixture;
import toolpathkit.tool.Tool;

/** Shared bearing-block setup, attached to a configuration's real table and spindle datums.
 * Motion limits are explicit kinematic preview assumptions, never motor ratings.
 */
class MillMachiningSetup {
 public static final MISSION:materia.project.SceneArtifact.SceneArtifactMachiningMission={id:"bearing-block",label:"Machine bearing block"};
 public final assembly:MachineAssembly;
 public final axes:Array<String>;
 public final block=new BearingBlock(BenchMill.STOCK_WIDTH,BenchMill.STOCK_DEPTH,BenchMill.STOCK_HEIGHT);
 final toolLength:Float;

 public function new(assembly:MachineAssembly,axes:Array<String>,table:String,tableTop:Float) {
  this.assembly=assembly;this.axes=axes;
  var poses:Map<String,materia.assembly.AssemblyRecord.AssemblyFrame> = assembly.solvedPoses();
  var gauge:materia.assembly.AssemblyRecord.AssemblyFrame=poses.get("spindle/spindle");
  var tip:materia.assembly.AssemblyRecord.AssemblyFrame=poses.get("spindle/tool");
  toolLength=gauge.z-tip.z;
  var model=new cadkit.modeling.AssemblyModel("mm");assembly.addTo(model,"");
  var definition=model.definition("setup-datums");
  var z:materia.assembly.AssemblyDefinition.KinematicJoint=[for(j in definition.joints)if(j.id==axes[2])j][0];
  var lower=z.limits.lower;if(lower==null)throw "Machining setup needs a bounded Z axis";
  // At the lower stop, the tip can reach the blank bottom. Raise the blank only when required.
  var riserHeight=Math.max(0,gauge.z+lower-toolLength-tableTop);
  var stockBottom=tableTop+riserHeight;
  function attach(id:String,c:machinekit.component.MachineComponent,pose:materia.assembly.AssemblyRecord.AssemblyFrame):Void {
   assembly.addComponent(id,c,pose);
   assembly.addMemberConnector(table,"to-"+id,F.compose(F.inverse(poses.get(table)),pose));
   assembly.addMemberConnector(id,"seat",F.identity());
   assembly.addMate("mount-"+id,"fixed",table,"to-"+id,id,"seat");
  }
  if(riserHeight>0.001)attach("machining-riser",new MillPanel(120,120,riserHeight,"cast iron"),F.translation(gauge.x,gauge.y,tableTop));
  attach("machining-stock",new MillPanel(block.width,block.depth,block.height,"aluminium 6061"),F.translation(gauge.x,gauge.y,stockBottom));
  for(side in [-1,1])attach(side<0?"machining-clampLeft":"machining-clampRight",new ToeClamp(block.height),
   F.fromRotationMatrix(gauge.x+side*(block.width/2-8),gauge.y+block.depth/2,stockBottom,[0,-1,0,1,0,0,0,0,1]));
 }

 public function scene(id:String,notes:Array<String>):haxe.io.Bytes {
  var data:materia.project.SceneArtifact.SceneArtifactData=SceneArtifact.decode(MillProject.scene(assembly,id,notes));
  var definition=data.assemblyDefinition;
  if(definition==null)throw "Machining setup needs an assembly definition";
  var state=new AssemblyState(definition);
  // No drive ratings are synthesized for this packaging model. Bound preview motion explicitly.
  for(joint in definition.joints)if(axes.indexOf(joint.id)>=0) {
   joint.limits.velocity=20; joint.limits.acceleration=100;
  }
  var gauge=state.worldPose("spindle/spindle"),stock=state.worldPose("machining-stock");
  var offset=[stock.x-block.width/2-gauge.x,stock.y-block.depth/2-gauge.y,stock.z+block.height-gauge.z];
  var fixtures:Array<Fixture> = [];
  var inverse=F.inverse(F.translation(stock.x-block.width/2,stock.y-block.depth/2,stock.z+block.height));
  for(id in ["machining-clampLeft","machining-clampRight"]) {
   var component:machinekit.component.MachineComponent=[for(row in assembly.components())if(row.id==id)row.component][0];
   var geometry=component.geometry(),bounds=geometry.shape.bounds();
   var lo=bounds.get_min(),hi=bounds.get_max(),pose=F.compose(inverse,state.worldPose(id));
   var low=[Math.POSITIVE_INFINITY,Math.POSITIVE_INFINITY,Math.POSITIVE_INFINITY],high=[Math.NEGATIVE_INFINITY,Math.NEGATIVE_INFINITY,Math.NEGATIVE_INFINITY];
   for(x in [lo.get_x(),hi.get_x()])for(y in [lo.get_y(),hi.get_y()])for(z in [lo.get_z(),hi.get_z()]) {
    var p=F.transformPoint(pose,x,y,z),v=[p.x,p.y,p.z];
    for(i in 0...3){low[i]=Math.min(low[i],v[i]);high[i]=Math.max(high[i],v[i]);}
   }
   geometry.close();fixtures.push(new Fixture(id,low[0]/1000,high[0]/1000,low[1]/1000,high[1]/1000,low[2]/1000,high[2]/1000));
  }
  var reference=new BenchMill(false,false);
  var holder:machinekit.component.MachineComponent=[for(row in assembly.components())if(row.id=="spindle/holder")row.component][0];
  var holderGeometry=holder.geometry(),holderBounds=holderGeometry.shape.bounds();
  var holderDiameter=2*Math.max(Math.abs(holderBounds.get_min().get_x()),Math.abs(holderBounds.get_max().get_x()));
  holderGeometry.close();
  var tool=Tool.shaped(1,toolLength/1000,reference.tool.cutter().withHolder(holderDiameter/1000,(toolLength-reference.tool.stickout)/1000));
  var target=block.geometry();
  var program=BearingBlockJob.generate({tools:[tool],fixtures:fixtures,workOffset:offset,start:[0,0,0]},block,target);
  data.parts.push(AssemblyPreview.part("machining-target","Finished bearing block",target,block.materialId,0.002));target.close();
  data.machining={kinematicPreview:true,mission:MISSION,program:program,axes:axes,spindle:"spindle/spindle",channelScope:"spindle",
   workOffset:[for(v in offset)v/1000],tools:[{number:1,length:tool.length,profile:tool.profile().encode()}],
   stocks:["machining-stock"],toolPart:"spindle/tool",loadedTool:1,target:"machining-target",loop:false};
  data.recipeDiagnostics.push("Bearing-block mission uses kinematic preview limits: 20 mm/s and 100 mm/s². Drive performance and cutting loads are not qualified.");
  return SceneArtifact.encode(data);
 }
}
