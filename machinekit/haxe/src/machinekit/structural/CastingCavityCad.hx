package machinekit.structural;

import cadkit.modeling.Curve;
import cadkit.modeling.Model;
import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Scope;
import cadkit.modeling.Sketch;
import cadkit.modeling.Vector;
import machinekit.assembly.PosedParts;
import materia.casting.CastingCavities;
import materia.casting.CastingCavities.FinishedCastingCavity;
import materia.casting.CastingFeature;
import materia.casting.CastingPrism;

/** Native realization of the authoritative derived region, including exact
 * cut/body expressions. This is the shared CAD backend, not local cut clipping.
 */
class CastingCavityCad {
	/** Numerical contact thresholds in mm²/mm³, not fabrication allowances. */
	static inline var CONTACT_AREA = 1e-8;
	static inline var CONTACT_VOLUME = 1e-8;

	public static function prism(prism:CastingPrism):Part {
		return Scope.run(scope -> {
			var local = scope.own(Part.prism([for (point in prism.profile) new Vector(point.x, point.y)], 0, prism.height));
			return scope.own(PosedParts.moved(local, prism.pose));
		});
	}

	static function cut(feature:CastingFeature):Part {
		if (feature.draft.degrees != 0 || feature.cornerRadius != 0)
			throw "Unsupported casting cavity geometry: " + feature.id;
		return Scope.run(scope -> {
			var local = scope.own(switch feature.geometry.solid {case Cut(solid): CastingSolids.part(solid); case Insert(body): InsertBodies.part(body);});
			return scope.own(PosedParts.moved(local, feature.geometry.pose));
		});
	}

	static function union(prisms:Array<CastingPrism>):Part {
		return Scope.run(scope -> {
			var parts = [for (prism in prisms) scope.own(CastingCavityCad.prism(prism))];
			return scope.own(Part.fuseAll(parts));
		});
	}

	public static function part(cavity:FinishedCastingCavity):Part {
		return switch cavity.geometry {
			case Prisms(pieces): union(pieces);
			case UnsupportedGeometry(reason): throw "Unsupported casting cavity geometry: " + cavity.featureId + ": " + reason;
			case SolidIntersection(body, feature, features):
				Scope.run(scope -> {
					var envelope = scope.own(union(body)), tool = scope.own(cut(feature));
					var common = scope.own(new Model(envelope.shape.common(tool.shape)));
					if (common.shape.volume() <= CONTACT_VOLUME)
						throw "Casting feature does not reach body: " + feature.id;
					var finished = scope.own(new Part(common.shape.cloneShape()));
					validateOpenings(scope, body, feature, features, envelope, tool, finished);
					return finished;
				});
		};
	}

	static function validateOpenings(scope:Scope, body:Array<CastingPrism>, feature:CastingFeature,
			features:Array<CastingFeature>, envelope:Part, tool:Part, finished:Part):Void {
		var reached = new Map<String, Bool>(), declared = new Map<String, Bool>();
		var throughFeatures:Array<Model> = [];
		for (opening in feature.openings) {
			if (opening.kind == BodyFace) {
				declared.set(opening.target, true);
				continue;
			}
			var candidates = features.filter(other -> other.id == opening.target || other.derivedFrom == opening.target);
			var meets = false;
			for (other in candidates) {
				var otherCut = scope.own(cut(other));
				var otherRegion = scope.own(new Model(envelope.shape.common(otherCut.shape)));
				throughFeatures.push(otherRegion);
				var common = scope.own(new Model(finished.shape.common(otherRegion.shape)));
				if (common.shape.area() > CONTACT_AREA) meets = true;
			}
			if (!meets)
				throw "Unreached declared feature opening: " + feature.id + " " + opening.target;
		}
		for (face in CastingCavities.boundary(body)) {
			var points = [for (point in face.points) new Vector(point.x, point.y, point.z)];
			var normal = CastingCavities.bodyFaceNormal(body, face.id);
			var plane = new Plane(points[0], points[1].subtract(points[0]), new Vector(normal.x, normal.y, normal.z));
			var wire = scope.own(Curve.polyline(points, true));
			var surface = scope.own(Sketch.face(wire, [], plane));
			var opening = scope.own(new Model(surface.shape.common(tool.shape)));
			if (opening.shape.area() <= CONTACT_AREA) continue;
			reached.set(face.id, true);
			if (declared.exists(face.id)) continue;
			var remaining:Model = opening;
			for (other in throughFeatures)
				remaining = scope.own(new Model(remaining.shape.cut(other.shape)));
			if (remaining.shape.area() > CONTACT_AREA)
				throw "Undeclared casting breakout: " + feature.id + " " + face.id;
		}
		for (id in declared.keys())
			if (!reached.exists(id))
				throw "Unreached declared opening: " + feature.id + " " + id;
	}
}
