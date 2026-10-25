package machinekit.structural;

import machinekit.component.ComponentValues;
import machinekit.structural.MineralFoundation.CastInsertPlacement;
import materia.casting.InsertSpec.InsertRole;

/** Structural sizing before source realization. The result contains declared
 * insert footprints, holes and anchors; no workshop capacity is assumed. */
class FoundationSizing {
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

	public static function anchors(rows:Array<CastInsertPlacement>,rule:materia.casting.InsertAnchorage.InsertAnchorPatternRule,
		castingValues:ComponentValues):Array<CastInsertPlacement> {
		var body=new machinekit.structural.MineralCasting(castingValues).geometry();
		try {
			var lifting=rows.filter(row -> row.part.parameterToken("role")==materia.casting.InsertSpec.InsertRole.LiftingMount);
			function share(placed:Array<CastInsertPlacement>):Float {
				if (lifting.length==0) return 0;
				var mass=asCastMass(new MineralFoundation(castingValues,placed));
				if (mass.unaccounted.length>0 || !Math.isFinite(mass.mass) || mass.mass<=0)
					throw "Anchor lifting demand requires the actual source casting mass";
				return mass.mass*rule.gravityMps2/Math.min(rule.liftingLoadPaths,lifting.length);
			}
			var liftingShare=share(rows);
			// Only lifting demand depends on the resolved casting mass. Other
			// patterns and their exact containment are unchanged across this loop.
			var fixed:Map<String,CastInsert> = [];
			for (row in rows) if (row.part.parameterToken("role")!=InsertRole.LiftingMount)
				fixed.set(row.id,machinekit.tooling.InsertAnchorGeometry.derive(row.id,row.part,rule,Casting(body,row.pose),0));
			// Hardware changes the casting mass. Resolve that known design coupling
			// without borrowing workshop strength values or dropping hardware mass.
			for (pass in 0...8) {
				var result:Array<CastInsertPlacement> = [for (row in rows) {id:row.id,pose:row.pose,openings:row.openings,
					part:fixed.exists(row.id) ? fixed.get(row.id)
						: machinekit.tooling.InsertAnchorGeometry.derive(row.id,row.part,rule,Casting(body,row.pose),liftingShare)}];
				var resolved=share(result);
				if (Math.abs(resolved-liftingShare)<1e-6) {body.close();return result;}
				liftingShare=resolved;
			}
			throw "Anchor/lifting mass sizing did not converge for "+[for (row in lifting) row.id].join(", ");
		} catch (error:Dynamic) {body.close();throw error;}
	}

}
