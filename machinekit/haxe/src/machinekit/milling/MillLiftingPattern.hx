package machinekit.milling;

import machinekit.component.ComponentValues;
import machinekit.structural.CastInsert;
import machinekit.structural.MineralFoundation;
import machinekit.structural.MineralFoundation.CastInsertPlacement;
import machinekit.structural.InsertAnchorPattern;
import machinekit.structural.InsertHoles;
import materia.assembly.AssemblyFrames as F;
import materia.casting.CastingPatterns;

/** Four source connections enclose the foundation's nominal centre of mass.
 * The attachment footprint is a design declaration, never a capacity approval. */
class MillLiftingPattern {
	/** Native nominal mass in the pour state: transferred holes and finishing
	 * removal do not yet exist. Embedded anchor hardware remains real components. */
	public static function asCastMass(foundation:MineralFoundation):machinekit.assembly.MachineAssembly.MachineAssemblyMassProperties {
		var raw=new machinekit.assembly.MachineAssembly(),poses=foundation.solvedPoses();
		for (member in foundation.components()) {
			var part=member.component;
			if (Std.isOfType(part,CastInsert)) {
				var steel:CastInsert=cast part;
				part=new CastInsert(steel.values().copy().setToken("buildStage",materia.casting.InsertStages.InsertStage.BeforeCasting));
			}
			raw.addComponent(member.id,part,poses.get(member.id));
		}
		return raw.massProperties();
	}

	public static function placements(body:ComponentValues,existing:Array<CastInsertPlacement>,
		details:MillCastingDetails):Array<CastInsertPlacement> {
		var foundation=new MineralFoundation(body,existing),mass=asCastMass(foundation);
		if (mass.mass<=0 || !Math.isFinite(mass.mass) || mass.unaccounted.length>0)
			throw "Lifting layout needs the source foundation mass and centre of mass";
		var width=details.liftingPlateWidthMm,depth=details.liftingPlateDepthMm,height=details.liftingPlateThicknessMm;
		if (!Math.isFinite(width+depth+height) || width<=0 || depth<=0 || height<=0)
			throw "Invalid source lifting plate dimensions";
		var land=InsertAnchorPattern.minimumOuterLand(height,C45Bright,details.anchorRule);
		var x=body.number("width")/2-width/2-land;
		var y=body.number("depth")/2-depth/2-land;
		var z=body.number("height")-height;
		var com=mass.centreOfMass;
		if (x<=0 || y<=0 || z<0 || Math.abs(com.x)>=x || Math.abs(com.y)>=y)
			throw "Source lifting footprint cannot enclose the foundation centre of mass";
		var result:Array<CastInsertPlacement> = [];
		for (sx in [-1,1]) for (sy in [-1,1]) {
			var id="lift"+(sx<0 ? "Left" : "Right")+(sy<0 ? "Front" : "Rear");
			var part=new CastInsert(CastInsert.precastHoles(CastInsert.boxValues(width,depth,height),
				InsertHoles.tappedBlind(details.liftingThread,height,C45Bright))
				.setToken("role",materia.casting.InsertSpec.InsertRole.LiftingMount)
				.setToken("label","foundation lifting connection"));
			result.push({id:id,part:part,pose:F.translation(sx*x,sy*y,z),
				openings:[CastingPatterns.face("body.z-max")]});
		}
		return result;
	}
}
