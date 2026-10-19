package machinekit.milling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.*;
import machinekit.assembly.PosedParts.PartBox;
import materia.assembly.AssemblyRecord.AssemblyFrame;

private typedef SpindleAuditSolid = {id:String, part:Part, bounds:PartBox};

/** Actual payload geometry and continuous fixed-structure sweeps. Nominal
 * threads and intended belt/tooth contact are named exclusions, not global skips. */
class MillSpindleIntegrity {
	public static function checkPayload(assembly:MachineAssembly, layout:MillZAxisLayout):Diagnostics {
		var cache = new PosedParts(), owned:Array<Part> = [], result:Diagnostics;
		try result = run(assembly, layout, cache, owned) catch (error:Dynamic) {
			for (part in owned) if (!part.shape.isClosed()) part.close();
			cache.close(); throw error;
		}
		for (part in owned) part.close();
		cache.close(); return result;
	}

	/** Motor withdrawal after removing its belt, pulley, fasteners and guard. */
	public static function checkMotorRemoval(assembly:MachineAssembly):Diagnostics {
		var out=new Diagnostics(), cache=new PosedParts(), poses=assembly.solvedPoses();
		var motor=cast(assembly.components().filter(m->m.id=="spindle/motor")[0].component,machinekit.motion.ServoMotor);
		var local=motor.axialEnvelope(-50,0,true);
		var envelope=PosedParts.moved(local,poses.get("spindle/motor")); local.close();
		var sweep:SpindleAuditSolid={id:"motor removal",part:envelope,bounds:PosedParts.boxOf(envelope)};
		try {
			for (member in assembly.components()) {
				var id=member.id;
				if(StringTools.startsWith(id,"spindle/motorNut")) continue;
				if (id=="spindle/motor" || id=="spindle/motorShaft" || id=="spindle/motorPulley" || id=="spindle/belt" ||
					id=="spindle/beltGuard" || id=="spindle/tensionScrew" || StringTools.startsWith(id,"spindle/motorBolt") ||
					StringTools.startsWith(id,"spindle/guardBolt")) continue;
				var part=cache.posed(member.component,poses.get(id));
				try noOverlap(out,sweep,{id:id,part:part,bounds:PosedParts.boxOf(part)},'motor withdrawal/$id')
				catch(e:Dynamic) { part.close(); throw e; }
				part.close();
			}
		} catch(e:Dynamic) { envelope.close(); cache.close(); throw e; }
		envelope.close(); cache.close(); return out;
	}

	static function run(assembly:MachineAssembly, l:MillZAxisLayout, cache:PosedParts, owned:Array<Part>):Diagnostics {
		var out = new Diagnostics(), poses:Map<String,AssemblyFrame> = assembly.solvedPoses();
		var xyLayout = new MillXYStageLayout(l.frame,l.spindleY,l.profile);
		var payload:Array<SpindleAuditSolid> = [], fixed:Array<SpindleAuditSolid> = [],
			head:Array<SpindleAuditSolid> = [], xy:Array<SpindleAuditSolid> = [];
		function keep(id:String,part:Part):SpindleAuditSolid {
			owned.push(part); return {id:id,part:part,bounds:PosedParts.boxOf(part)};
		}
		// Build and pose each solid once. Most pairs separate at their cached boxes.
		for (member in assembly.components()) {
			var id = member.id;
			if (StringTools.startsWith(id,"spindle/")) payload.push(keep(id.substr(8),cache.posed(member.component,poses.get(id))));
			else if (StringTools.startsWith(id,"frame/")) fixed.push(keep(id,cache.posed(member.component,poses.get(id))));
			else if (StringTools.startsWith(id,"zAxis/")) {
				var row = keep(id,cache.posed(member.component,poses.get(id)));
				if (MillZAxis.isTranslating(id.substr(6))) head.push(row); else fixed.push(row);
			} else if (StringTools.startsWith(id,"xyStage/")) {
				var b = cache.box(member.component,poses.get(id)), localId = id.substr(8);
				var dx = MillXYStageIntegrity.tableMember(localId) ? 2*xyLayout.xHalfTravel : 0.0;
				var dy = MillXYStageIntegrity.yFixed(localId) ? 0.0 : 2*xyLayout.yHalfTravel;
				xy.push(keep(id,Part.box(b.maxX-b.minX+dx,b.maxY-b.minY+dy,b.maxZ-b.minZ)
					.translated(new Vector((b.minX+b.maxX)/2,(b.minY+b.maxY)/2,b.minZ))));
			}
		}
		for (i in 0...payload.length) for (j in i+1...payload.length) {
			var a = payload[i], b = payload[j];
			if (!engagement(a.id,b.id) && !engagement(b.id,a.id)) noOverlap(out,a,b,'payload ${a.id}/${b.id}');
		}
		for (member in payload) {
			for (other in head) {
				if ((StringTools.startsWith(member.id,"carrierBolt") || StringTools.startsWith(member.id,"collarBolt")) && other.id=="zAxis/head") continue;
				noOverlap(out,member,other,'head attachment ${member.id}/${other.id}');
			}
			// The extruded bounding box contains this part throughout its Z stroke.
			var b = member.bounds;
			var swept:Part;
			var actualMotor:Null<machinekit.component.MachineComponent> =null;
			if(member.id=="motor") for(m in assembly.components()) if(m.id=="spindle/motor") actualMotor=m.component;
			if (actualMotor!=null) {
				var motor=cast(actualMotor,machinekit.motion.ServoMotor);
				var local=motor.axialEnvelope(0,-l.lower);
				swept=PosedParts.moved(local,poses.get("spindle/motor")); local.close();
			} else swept=Part.box(b.maxX-b.minX,b.maxY-b.minY,b.maxZ-b.minZ-l.lower)
				.translated(new Vector((b.minX+b.maxX)/2,(b.minY+b.maxY)/2,b.minZ+l.lower));
			var sweep=keep(member.id,swept);
			for (other in fixed) noOverlap(out,sweep,other,'spindle sweep ${member.id}/${other.id}');
			for (other in xy) noOverlap(out,sweep,other,'XYZ payload sweep ${member.id}/${other.id}');
		}
		return out;
	}

	static function engagement(a:String,b:String):Bool return
		(a=="belt" && (b=="spindlePulley" || b=="motorPulley")) ||
		(StringTools.startsWith(a,"motorBolt") && b=="motor") ||
		(a=="tensionScrew" && b=="motorCarrier") ||
		(StringTools.startsWith(a,"guardBolt") && b=="motorCarrier") ||
		(StringTools.startsWith(a,"reactionBolt") && b=="spindle") ||
		(StringTools.startsWith(a,"cylinderBolt") && b=="cylinder");

	static function noOverlap(out:Diagnostics, a:SpindleAuditSolid, b:SpindleAuditSolid, label:String):Void {
		var x = a.bounds, y = b.bounds;
		if (x.maxX<=y.minX || y.maxX<=x.minX || x.maxY<=y.minY || y.maxY<=x.minY || x.maxZ<=y.minZ || y.maxZ<=x.minZ) return;
		var common = a.part.intersect(b.part), volume:Float;
		try volume = common.volume() catch (error:Dynamic) { common.close(); throw error; }
		common.close();
		if (volume>1e-5) out.error("mill-spindle.interference",label,'$label intersects ($volume mm3)');
	}
}
