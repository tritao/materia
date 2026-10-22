package materia.tooling.casting;

import materia.casting.InsertSpec.InsertBodyCodec as InsertBodies;
import materia.casting.BoxAlgebra;

import materia.assembly.AssemblyFrames as F;
import materia.tooling.ToolingDocument;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.MoldPlan.MoldBox;

/** Exact box moments retain their algebraic path. Non-box source geometry
 * uses the application kernel, with results expressed in mm/kg. */
class CastHandling {
	/** Existing source mass calculation shared by handling and lifting proposals. */
	public static function moments(d:ToolingDocument,envelope:MoldBox):{
		massKg:Float,centerOfMassMoldMm:{x:Float,y:Float,z:Float}
	} {
		var unsupported=CastingBoxSupport.check(d.input,"tooling.handling.geometry",false)
			.filter(check -> check.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported);
		if (unsupported.length>0) {
			var kernel=CastingMoldPlanner.geometry;
			if (kernel==null)
				throw "Unsupported exact handling geometry: "+[for (check in unsupported) check.affectedIds.join(", ")].join("; ");
			var exact=kernel.castingMass(d),center=exact.centerOfMassMoldMm;
			for (value in [exact.massKg,center.x,center.y,center.z])
				if (!Math.isFinite(value)) throw "Nonfinite exact casting mass: "+d.input.id;
			if (exact.massKg<=0) throw "Casting handling requires positive mass";
			return exact;
		}
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
			var steel = materia.casting.InsertHoleGeometry.massProperties(dimensions.width, dimensions.depth, dimensions.height, InsertFixturePlanner.holes(i));
			var kg = steel.volume * d.recipe.steelDensityKgM3 * 1e-9;
			var center = F.transformPoint(F.compose(d.setup.sourceToMold, i.pose), steel.center.x, steel.center.y, steel.center.z);
			mass += kg;
			mx += kg * center.x;
			my += kg * center.y;
			mz += kg * center.z;
			var spec:materia.casting.InsertSpec={body:i.body,holes:i.holes,origin:i.origin,interfaces:i.interfaces,anchorage:i.anchorage};
			for (exposedOnly in [true,false]) {
				var hardware=materia.casting.EmbeddedAnchors.moments(spec,exposedOnly);
				var delta=hardware.volume*(exposedOnly ? -d.recipe.mixDensityKgM3 : d.recipe.steelDensityKgM3)*1e-9;
				var point=F.transformPoint(F.compose(d.setup.sourceToMold,i.pose),hardware.center.x,hardware.center.y,hardware.center.z);
				mass+=delta;mx+=delta*point.x;my+=delta*point.y;mz+=delta*point.z;
			}
		}
		if (!Math.isFinite(mass) || !Math.isFinite(mx) || !Math.isFinite(my) || !Math.isFinite(mz) || mass <= 0)
			throw "Casting handling requires positive mass";
		return {massKg:mass,centerOfMassMoldMm:{x:mx/mass,y:my/mass,z:mz/mass}};
	}

	public static function plan(d:ToolingDocument,o:CastingMoldOptions,envelope:MoldBox):HandlingPlan {
		var source=moments(d,envelope);
		var relative=F.compose(o.serviceOrientation,F.inverse(d.setup.sourceToMold));
		var vertical=F.transformVector(relative,0,0,1);
		return {requiresFlip:vertical.z<1-1e-8,moldToService:relative,
			castingMassKg:source.massKg,centerOfMassMoldMm:source.centerOfMassMoldMm};
	}

	/** Source attachment declarations are distinct from equipment qualification.
	 * Anchor holes are occupied and post-cast holes do not yet exist at the lift. */
	public static function liftingIds(d:ToolingDocument):Array<String> {
		var ids:Array<String> = [];
		for (insert in d.input.inserts) if (insert.role==materia.casting.InsertSpec.InsertRole.LiftingMount) {
			var occupied=materia.casting.InsertAnchorage.InsertAnchorageCodec.occupiedHoleIds(insert.anchorage);
			if (InsertFixturePlanner.holes(insert).filter(hole -> hole.thread!=null && occupied.indexOf(hole.id)<0).length>0)
				ids.push(insert.id);
		}
		ids.sort(Reflect.compare);
		return ids;
	}

	public static function sourceDeclared(d:ToolingDocument):Bool {
		var minimum=d.recipe.designReview.minimumLiftingPoints;
		return liftingIds(d).length>=(minimum==null || minimum<1 ? 1 : minimum);
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
					if (liftingIds(d).indexOf(id)<0)
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
