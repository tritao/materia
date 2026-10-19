package materia.tooling.casting;

import materia.assembly.AssemblyFrames as F;
import materia.tooling.ToolingDocument;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.MoldPlan.MoldBox;

/** Exact orthogonal resin moments and analytic through-bore steel moments, in mm/kg. */
class CastHandling {
	public static function plan(d:ToolingDocument, o:CastingMoldOptions, envelope:MoldBox):HandlingPlan {
		var resin = BoxVolumes.subtract([envelope], [for (f in d.input.features) CastingMoldPlanner.featureBox(d, f)]);
		var mass = 0.0, mx = 0.0, my = 0.0, mz = 0.0;
		for (b in resin) {
			var kg = b.width * b.depth * b.height * d.recipe.mixDensityKgM3 * 1e-9;
			mass += kg;
			mx += kg * (b.x + b.width / 2);
			my += kg * (b.y + b.depth / 2);
			mz += kg * (b.z + b.height / 2);
		}
		for (i in d.input.inserts) {
			var kg = (i.width * i.depth - Math.PI * i.boreDiameter * i.boreDiameter / 4) * i.height * d.recipe.steelDensityKgM3 * 1e-9;
			var center = F.transformPoint(F.compose(d.setup.sourceToMold, i.pose), 0, 0, i.height / 2);
			mass += kg;
			mx += kg * center.x;
			my += kg * center.y;
			mz += kg * center.z;
		}
		if (!Math.isFinite(mass) || !Math.isFinite(mx) || !Math.isFinite(my) || !Math.isFinite(mz) || mass <= 0)
			throw "Casting handling requires positive mass";
		var relative = F.compose(o.serviceOrientation, F.inverse(d.setup.sourceToMold));
		var rotation = relative.qx * relative.qx + relative.qy * relative.qy + relative.qz * relative.qz;
		return {
			requiresFlip: rotation > 1e-16,
			moldToService: relative,
			castingMassKg: mass,
			centerOfMassMoldMm: {x: mx / mass, y: my / mass, z: mz / mass}
		};
	}

	public static function qualified(d:ToolingDocument, o:CastingMoldOptions, p:HandlingPlan):Bool {
		for (q in o.liftingEvidence) {
			if (q.capacityKg < p.castingMassKg * d.recipe.handlingLoadFactor
				|| q.verifiedMassKg < p.castingMassKg
				|| q.attachmentIds.length == 0)
				continue;
			var c = p.centerOfMassMoldMm,
				dx = c.x - q.centerOfMass.x,
				dy = c.y - q.centerOfMass.y,
				dz = c.z - q.centerOfMass.z;
			if (Math.sqrt(dx * dx + dy * dy + dz * dz) > q.positionToleranceMm)
				continue;
			var valid = true, seen = new Map<String, Bool>();
			for (id in q.attachmentIds) {
				if (seen.exists(id))
					valid = false;
				seen.set(id, true);
				if (q.method == CastInPoints) {
					if (d.input.inserts.filter(i -> i.id == id
						&& i.role == materia.tooling.ToolingEnums.InsertRole.LiftingMount).length != 1)
						valid = false;
				} else if (d.input.envelope.filter(e -> e.id == id).length != 1)
					valid = false;
			}
			if (valid)
				return true;
		}
		return false;
	}
}
