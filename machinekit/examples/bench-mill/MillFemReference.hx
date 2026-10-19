import cadkit.modeling.AssemblyState;
import cadkit.modeling.AssemblyModel;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.PosedParts;
import machinekit.milling.MillFrame;
import machinekit.milling.MillXYStageLayout;
import machinekit.milling.MillZAxisLayout;
import materia.assembly.AssemblyFrames as F;

/** Independent physical records in the concept report for checking the FEM export. */
class MillFemReference {
	public static function capture(assembly:MachineAssembly,frame:MillFrame):Dynamic {
		var xy:MillXYStageLayout=frame.xyAxis, z:MillZAxisLayout=frame.zAxis;
		var poses=[{x:0.0,y:0.0,z:0.0},{x:0.0,y:0.0,z:z.lower/2}];
		for (x in [-xy.xHalfTravel,xy.xHalfTravel]) for (y in [-xy.yHalfTravel,xy.yHalfTravel])
			for (height in [z.lower,0.0]) poses.push({x:x,y:y,z:height});
		var members=assembly.components();
		members.sort((a,b)->Reflect.compare(a.id,b.id));
		var records:Array<Dynamic> = [];
		// Physical preview records must build without importing the optional FEM exporter.
		var model=new AssemblyModel("mm"); assembly.addTo(model,"");
		var definition=model.definition("mill-physical-reference");
		PosedParts.scope(cache->{
			for (pose in poses) {
				var state=new AssemblyState(definition), parts:Array<Dynamic> = [];
				state.setJoint("xyStage/x",pose.x); state.setJoint("xyStage/y",pose.y); state.setJoint("zAxis/z",pose.z);
				state.forwardKinematics();
				for (member in members) {
					var world=state.worldPose(member.id), physical=member.component.massProperties(), c=physical.centreOfMass;
					var shape=cache.posed(member.component,world);
					try parts.push({id:member.id,volumeMm3:shape.volume(),massKg:physical.mass,
						centreOfMassMm:F.transformPoint(world,c.x,c.y,c.z),boundsMm:PosedParts.boxOf(shape)});
					catch(error:Dynamic) {shape.close();throw error;}
					shape.close();
				}
				var table=state.worldConnector("xyStage/table","top");
				var offset=F.transformVector(state.worldPose("xyStage/table"),0,0,50);
				records.push({jointsMm:pose,parts:parts,keyDatums:{toolTipMm:state.worldConnector("spindle/tool","tip"),
					tableTopMm:table,workpieceReferenceMm:F.translation(table.x+offset.x,table.y+offset.y,table.z+offset.z)}});
			}
		});
		return {schemaVersion:1,units:"mm kg",poses:records};
	}
}
