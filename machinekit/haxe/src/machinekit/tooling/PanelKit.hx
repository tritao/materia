package machinekit.tooling;

import machinekit.assembly.MachineAssembly;
import machinekit.assembly.PosedParts;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import materia.assembly.AssemblyFrames as F;
import materia.tooling.panel.PanelAssemblyPlan;
import materia.tooling.panel.PanelAssemblyPlan.PanelAssemblyRecord;
import materia.tooling.panel.PanelContours;
import materia.tooling.panel.PanelGeometry;
import machinekit.tooling.SweptRemoval.RemovalReport;
import machinekit.tooling.SweptRemoval.RemovalMember;

class PanelKit {
	public static function presentation(plan:PanelAssemblyRecord,definition:materia.assembly.AssemblyDefinition):materia.presentation.AssemblyPresentation {
		PanelAssemblyPlan.validate(plan);
		var low=new Vector(1e30,1e30,1e30),high=new Vector(-1e30,-1e30,-1e30);
		for(m in plan.members) {
			var b=PanelGeometry.bounds(PanelContours.outside(m.panel));
			for(x in [b.x,b.x+b.width])for(y in [b.y,b.y+b.height])for(z in [0.0,m.panel.stock.measuredThickness]) {
				var p=F.transformPoint(m.pose,x,y,z);
				low=new Vector(Math.min(low.x,p.x),Math.min(low.y,p.y),Math.min(low.z,p.z));
				high=new Vector(Math.max(high.x,p.x),Math.max(high.y,p.y),Math.max(high.z,p.z));
			}
		}
		var separation=high.subtract(low).length()+1;
		var offsets=new Map<String,materia.assembly.AssemblyDefinition.AssemblyVector>();
		for(i in 0...plan.members.length) {
			var m=plan.members[i],d=(i+1)*separation;
			offsets.set(m.panel.id,{x:m.withdrawal.x*d,y:m.withdrawal.y*d,z:m.withdrawal.z*d});
		}
		return ToolingExplode.presentation(definition,"panel-tooling",offsets);
	}

	public static function assembly(plan:PanelAssemblyRecord, material:String = "birch plywood"):MachineAssembly {
		PanelAssemblyPlan.validate(plan);
		var assembly = new MachineAssembly();
		for (m in plan.members)
			assembly.addComponent(m.panel.id, ToolingPanel.fromPlan(m.panel, material), m.pose);
		for (j in plan.joints) {
			assembly.addMemberConnector(j.fixedId, materia.tooling.ToolingIds.derive("", j.id, "-fixed"), j.fixedFrame);
			assembly.addMemberConnector(j.movingId, materia.tooling.ToolingIds.derive("", j.id, "-moving"), j.movingFrame);
			assembly.addMate(j.id, "fixed", j.fixedId, materia.tooling.ToolingIds.derive("", j.id, "-fixed"), j.movingId,
				materia.tooling.ToolingIds.derive("", j.id, "-moving"));
		}
		return assembly;
	}

	static function placedPanel(panel:materia.tooling.panel.PanelPlan, pose:materia.assembly.AssemblyRecord.AssemblyFrame):Part {
		var local = ToolingPanel.fromPlan(panel).geometry();
		try {
			var world = PosedParts.moved(local, pose);
			local.close();
			return world;
		} catch (e:Dynamic) {
			local.close();
			throw e;
		}
	}

	public static function check(plan:PanelAssemblyRecord):RemovalReport {
		PanelAssemblyPlan.validate(plan);
		var parts:Array<Part> = [], members:Array<RemovalMember> = [];
		try {
			for (m in plan.members) {
				var world = placedPanel(m.panel, m.pose);
				parts.push(world);
				members.push({
					id: m.panel.id,
					part: world,
					direction: new Vector(m.withdrawal.x, m.withdrawal.y, m.withdrawal.z)
				});
			}
			var report = SweptRemoval.analyze(members);
			for (p in parts)
				p.close();
			return report;
		} catch (e:Dynamic) {
			for (p in parts)
				p.close();
			throw e;
		}
	}

	/** Borrowed free-approach tool envelopes; contact/engagement belongs outside them. */
	public static function checkToolAccess(plan:PanelAssemblyRecord, accesses:Array<{
		id:String,
		clearance:Part
	}>, contactVolumeToleranceMm3:Float = 1e-5):Void {
		PanelAssemblyPlan.validate(plan);
		if (accesses == null || accesses.length > 128 || !Math.isFinite(contactVolumeToleranceMm3) || contactVolumeToleranceMm3 < 0)
			throw "Invalid tool access records";
		for (m in plan.members) {
			var world = placedPanel(m.panel, m.pose);
			try {
				for (access in accesses) {
					var volume = SweptRemoval.intersectionVolume(world, access.clearance);
					if (volume > contactVolumeToleranceMm3)
						throw 'Blocked tool access ${access.id}/${m.panel.id}';
				}
				world.close();
			} catch (e:Dynamic) {
				world.close();
				throw e;
			}
		}
	}
}
