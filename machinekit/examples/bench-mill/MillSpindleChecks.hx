import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import machinekit.assembly.*;
import machinekit.component.MachineKitComponents;
import machinekit.milling.*;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Spindle packaging, release inspection motion and persistence regressions. */
class MillSpindleChecks {
	static var assertions=0;
	static function check(ok:Bool,label:String):Void { assertions++; if (!ok) throw label; }
	static function near(a:Float,b:Float,label:String):Void check(Math.abs(a-b)<1e-6,'$label ($a vs $b)');
	public static function main():Void {
		var reference=new BenchMill(false,false), f=MillFramePreview.layout(reference), l=MillFramePreview.zLayout(reference,f);
		var frame=new MillFrame(f,l,new MillXYStageLayout(f,l.spindleY,BenchMill.RAIL));
		var assembly=MillFramePreview.concept(reference,frame);
		assembly.validate();
		MillSpindleIntegrity.checkPayload(assembly,l).throwIfErrors();
		check(true,"actual payload and full XYZ stroke are clear");
		MillSpindleIntegrity.checkMotorRemoval(assembly).throwIfErrors();
		check(true,"motor withdraws upward 50 mm at nominal tension after drive disassembly");
		var zero:Map<String,AssemblyFrame> =assembly.solvedPoses();
		var motorPose:AssemblyFrame=zero.get("spindle/motor");
		near(motorPose.x,0,"spindle motor lies on machine centerline");
		near(motorPose.y-l.spindleY,MillSpindleMotorLayout.CENTRES,"motor sits behind release assembly");
		for (candidate in [assembly,MachineAssembly.decode(assembly.encode(),MachineKitComponents.standard())]) {
			var model=new AssemblyModel("mm"); candidate.addTo(model,"");
			var state=new AssemblyState(model.definition("bt30-package"));
			state.setJoint("spindle/rotor-turn",1.25); state.forwardKinematics();
			near(state.joint("spindle/motor-turn"),1.25,"motor and spindle follow 1:1 belt after recipe rebuild");
			state.setJoint("spindle/rotor-turn",0); state.forwardKinematics();
			var rod=state.worldPose("spindle/pistonRod").z, drawbar=state.worldPose("spindle/drawbar").z;
			state.setJoint("spindle/drawbar-release",4.5); state.setJoint("spindle/pistonRod-release",6.5); state.forwardKinematics();
			near(state.worldPose("spindle/drawbar").z,drawbar-4.5,"drawbar release inspection stroke");
			near(state.worldPose("spindle/pistonRod").z,rod-6.5,"rod takes up gap then releases");
			near(state.worldPose("spindle/drawbar").z+285,state.worldPose("spindle/pistonRod").z+287,"release rod contacts drawbar after gap take-up");
			near(state.worldPose("spindle/gripper").z,drawbar-4.5,"gripper follows drawbar release");
			var zeroTool=state.worldPose("spindle/tool").z;
			state.setJoint("zAxis/z",l.lower); state.forwardKinematics();
			near(state.worldPose("spindle/tool").z,zeroTool+l.lower,"complete spindle follows Z");
			near(state.worldPose("spindle/tool").z,(new MillXYStageLayout(f,l.spindleY,BenchMill.RAIL)).tableTop,"lower Z stop preserves original tool-tip/table datum");
		}
		for (adjustment in [-MillSpindleMotorLayout.ADJUSTMENT,MillSpindleMotorLayout.ADJUSTMENT]) {
			var packageStudy=MillFramePreview.concept(reference,frame,adjustment);
			MillSpindleIntegrity.checkPayload(packageStudy,l).throwIfErrors(); check(true,"belt adjustment endpoint clears its carrier and guard");
		}
		var cache=new PosedParts(), poses:Map<String,AssemblyFrame> =assembly.solvedPoses();
		for (pair in [["zAxis/head","spindle/motorCarrier"],["spindle/motorCarrier","spindle/motor"],
			["spindle/tensionScrew","spindle/motor"],
			["zAxis/head","spindle/spindle"],["spindle/spindle","spindle/reactionFrame"],["spindle/reactionFrame","spindle/cylinder"],["spindle/beltGuard","spindle/motorCarrier"]]) {
			var members=assembly.components(), a=members.filter(m->m.id==pair[0])[0],b=members.filter(m->m.id==pair[1])[0];
			var first=cache.posed(a.component,poses.get(a.id)),second=cache.posed(b.component,poses.get(b.id));
			near(first.distance(second),0,'physical support ${pair[0]}/${pair[1]}'); first.close(); second.close();
		}
		var carrierMember=assembly.components().filter(m->m.id=="spindle/motorCarrier")[0];
		var carrier=cache.posed(carrierMember.component,poses.get(carrierMember.id));
		for (x in [-MillSpindleMotorLayout.FOOT_BOLT_X,MillSpindleMotorLayout.FOOT_BOLT_X])
			for (y in [MillSpindleMotorLayout.FOOT_BOLT_FRONT,MillSpindleMotorLayout.FOOT_BOLT_REAR]) {
				var tool=cadkit.modeling.Part.cylinderSpan(3,l.headBottom+l.headHeight+13,l.headBottom+l.headHeight+110,x,l.spindleY+y);
				var common=carrier.intersect(tool); near(common.volume(),0,"carrier anchor has vertical Allen-key access"); common.close(); tool.close();
			}
		carrier.close(); cache.close();
		for (m in assembly.components()) if (StringTools.startsWith(m.id,"spindle/")) {
			var p=m.component.geometry(); check(p.valid(), 'valid spindle solid ${m.id}');
			check(p.solidCount()==(m.id=="spindle/springs"?16:1),'connected spindle part ${m.id}'); p.close();
		}
		var guard=new MillSpindleBeltGuard(new machinekit.component.ComponentValues().setBoolean("centered",true)).geometry(), faceted=false;
		for (i in 0...guard.shape.subshapeCount(CadKit.ShapeKind.Face)) {
			var face=guard.shape.subshape(CadKit.ShapeKind.Face,i);
			if (face.surfaceKind()==CadKit.SurfaceKind.Plane) {
				var n=face.faceNormal();
				if (Math.abs(n.get_z())<1e-6 && Math.abs(n.get_x())>1e-6 && Math.abs(n.get_y())>1e-6 &&
					Math.abs(150*n.get_x()+78*n.get_y())>1e-5) faceted=true;
			}
			face.close();
		}
		check(!faceted,"guard bends use cylindrical faces, not polygonal side facets"); guard.close();
		var probe=new MachineAssembly(),fixed=new MachineAssembly(),blocked=new MachineAssembly();
		probe.addComponent("probe",new MillPanel(20,20,20),materia.assembly.AssemblyFrames.translation(0,l.spindleY,l.headBottom+l.headHeight+150));
		fixed.addComponent("obstruction",new MillPanel(20,20,20),materia.assembly.AssemblyFrames.translation(0,l.spindleY,l.headBottom+l.headHeight-50));
		blocked.include("spindle",probe); blocked.include("frame",fixed);
		check(MillSpindleIntegrity.checkPayload(blocked,l).hasErrors(),"continuous spindle sweep detects obstruction away from zero pose");
		Sys.println('BT30 spindle package: $assertions assertions passed');
	}
}
