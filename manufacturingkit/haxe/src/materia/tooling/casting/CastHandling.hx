package materia.tooling.casting;

import materia.casting.InsertSpec.InsertBodyCodec as InsertBodies;
import materia.casting.BoxAlgebra;

import materia.assembly.AssemblyFrames as F;
import materia.tooling.ToolingDocument;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.MoldPlan.MoldBox;

/** Exact orthogonal resin moments and analytic through-bore steel moments, in mm/kg. */
class CastHandling {
	public static function plan(d:ToolingDocument, o:CastingMoldOptions, envelope:MoldBox):HandlingPlan {
		var resin = BoxAlgebra.subtract([envelope], [for (f in d.input.features) CastingMoldPlanner.featureBox(d, f)]);
		var mass = 0.0, mx = 0.0, my = 0.0, mz = 0.0;
		for (b in resin) {
			var kg = b.width * b.depth * b.height * d.recipe.mixDensityKgM3 * 1e-9;
			mass += kg;
			mx += kg * (b.x + b.width / 2);
			my += kg * (b.y + b.depth / 2);
			mz += kg * (b.z + b.height / 2);
		}
		for (i in d.input.inserts) {
			var dimensions = InsertBodies.box(i.body, i.id);
			var steel = materia.casting.InsertHoleGeometry.massProperties(dimensions.width, dimensions.depth, dimensions.height, i.holes);
			var kg = steel.volume * d.recipe.steelDensityKgM3 * 1e-9;
			var center = F.transformPoint(F.compose(d.setup.sourceToMold, i.pose), steel.center.x, steel.center.y, steel.center.z);
			mass += kg;
			mx += kg * center.x;
			my += kg * center.y;
			mz += kg * center.z;
		}
		if (!Math.isFinite(mass) || !Math.isFinite(mx) || !Math.isFinite(my) || !Math.isFinite(mz) || mass <= 0)
			throw "Casting handling requires positive mass";
		var relative = F.compose(o.serviceOrientation, F.inverse(d.setup.sourceToMold));
		var vertical = F.transformVector(relative, 0, 0, 1);
		return {
			requiresFlip: vertical.z < 1 - 1e-8,
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
						&& i.role == materia.casting.InsertSpec.InsertRole.LiftingMount).length != 1)
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
