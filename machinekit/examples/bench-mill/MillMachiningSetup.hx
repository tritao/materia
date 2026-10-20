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
  var vise=new ZppWorkholding(block.depth,block.width);
  var minimumBottom=gauge.z+lower-toolLength;
  if(tableTop+vise.seat.z<minimumBottom-1e-6)
   throw "Vise stock seat is below the tool's reachable Z range; select a qualified mounting adapter";
  var tablePart:machinekit.component.MachineComponent=[for(row in assembly.components())if(row.id==table)row.component][0];
  if(tablePart.parameterNumber("width")<204 || tablePart.parameterNumber("depth")<194)
   throw "Table cannot support the 200 x 190 mm workholding adapter";
  var origin=F.translation(gauge.x,gauge.y,tableTop);
  assembly.include("workholding",vise,origin);
  assembly.addMemberConnector(table,"viseMount",F.compose(F.inverse(poses.get(table)),origin));
  assembly.addMemberConnector("workholding/body","tableSeat",F.identity());
  assembly.addMate("vise-on-table","fixed",table,"viseMount","workholding/body","tableSeat");
  var stockPose=F.translation(gauge.x,gauge.y,tableTop+vise.seat.z);
  assembly.addComponent("machining-stock",new MillPanel(block.width,block.depth,block.height,"aluminium 6061"),stockPose);
  assembly.addMemberConnector("workholding/body","blankSeat",F.compose(F.inverse(origin),stockPose));
  assembly.addMemberConnector("machining-stock","seat",F.identity());
  assembly.addMate("blank-on-parallels","fixed","workholding/body","blankSeat","machining-stock","seat");

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
  for(row in assembly.components())if(StringTools.startsWith(row.id,"workholding/")) {
   var id=row.id;
   var component=row.component;
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
  data.machining={setupJoints:[{joint:"workholding/half-opening",position:block.width/2000,tolerance:0.00005}],kinematicPreview:true,mission:MISSION,program:program,axes:axes,spindle:"spindle/spindle",channelScope:"spindle",
   workOffset:[for(v in offset)v/1000],tools:[{number:1,length:tool.length,profile:tool.profile().encode()}],
   stocks:["machining-stock"],toolPart:"spindle/tool",loadedTool:1,target:"machining-target",loop:false};
  data.recipeDiagnostics.push("Workholding uses ZPP AR155-I pneumatic-vise, manual TS96-155 plate and TS96 pull-stud STEP geometry. The hard-jaw setup is preset for a 60 mm blank. Air ports are exposed; jaw-release travel is a 2 mm-per-jaw packaging assumption. Valve, pressure, force and pressure-loss behavior are not simulated.");
  data.recipeDiagnostics.push("Bearing-block mission uses kinematic preview limits: 20 mm/s and 100 mm/s². Drive performance and cutting loads are not qualified.");
  return SceneArtifact.encode(data);
 }
}
