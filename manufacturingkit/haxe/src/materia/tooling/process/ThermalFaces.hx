package materia.tooling.process;

import materia.tooling.ToolingDocument;
import materia.tooling.ToolingCodec;
import materia.tooling.process.CastingProcessSpec;
import materia.tooling.process.JunctionThickness;
import materia.tooling.process.OrthogonalSections;

/** Geometric screening contacts, not a heat-transfer coefficient assumption.
 * Mold-contact faces can remove heat; EPS and an undeclared fill face cannot.
 * Explicit per-face conditions override mold contacts, never the EPS constraint.
 */
class ThermalFaces {
	public static function id(face:ThicknessFace):String
		return "thermal-face-" + haxe.crypto.Sha256.encode(ToolingCodec.canonical(face)).substr(0, 20);

	public static function kind(face:ThicknessFace, document:Null<ToolingDocument>, spec:CastingProcessSpec, fill:Float):ThermalBoundaryKind {
		if (document != null
			&& face.sourceIds.filter(owner -> materia.tooling.casting.CastingStrategies.resolve(document).filter(v -> v.featureId == owner
				&& v.strategy == materia.tooling.casting.CastingSetup.CoreStrategy.SacrificialEPS)
				.length > 0)
				.length > 0)
			return Insulated;
		var declarations = spec.faceContacts.filter(v -> v.faceId == id(face));
		if (declarations.length > 0)
			return declarations[0].kind;
		if (face.axis == Z && Math.abs(face.planeMm - fill) < 1e-7)
			return spec.openFaceContact == null ? Insulated : spec.openFaceContact.kind;
		return Convection;
	}

	public static function interval(document:ToolingDocument, sections:OrthogonalSectionReport, spec:CastingProcessSpec):Null<JunctionThicknessReport> {
		var faces = sections.junction.faces.filter(face -> kind(face, document, spec, document.setup.fillHeightMm) != Insulated);
		if (faces.length == 0)
			return null;
		var region = [
			for (span in sections.spans) if (span.axis == X) for (strip in span.strips) strip
		];
		return JunctionThickness.analyze(document.input.id, sections.sourceFingerprint, region, [], sections.junction.settings, faces);
	}

	public static function matches(span:PlanarSectionSpan, lower:Bool, boundary:ThermalBoundary, sections:OrthogonalSectionReport, spec:CastingProcessSpec,
			document:Null<ToolingDocument>):Bool {
		var fill = document == null ? Math.NEGATIVE_INFINITY : document.setup.fillHeightMm;
		var faces = sections.junction.faces.filter(f -> f.axis == span.axis
			&& Math.abs(f.planeMm - (lower ? span.startMm : span.endMm)) < 1e-7);
		var found = false;
		for (face in faces)
			for (strip in span.strips) {
				var overlaps = switch span.axis {
					case X: Math.min(strip.y + strip.depth,
							face.region.y + face.region.depth) > Math.max(strip.y,
							face.region.y) + 1e-7 && Math.min(strip.z + strip.height,
							face.region.z + face.region.height) > Math.max(strip.z, face.region.z) + 1e-7;
					case Y: Math.min(strip.x + strip.width,
							face.region.x + face.region.width) > Math.max(strip.x,
							face.region.x) + 1e-7 && Math.min(strip.z + strip.height,
							face.region.z + face.region.height) > Math.max(strip.z, face.region.z) + 1e-7;
					case Z: Math.min(strip.x + strip.width,
							face.region.x + face.region.width) > Math.max(strip.x,
							face.region.x) + 1e-7 && Math.min(strip.y + strip.depth, face.region.y + face.region.depth) > Math.max(strip.y, face.region.y) + 1e-7;
					default: throw "Invalid section axis";
				};
				if (overlaps) {
					found = true;
					if (kind(face, document, spec, fill) != boundary.kind)
						return false;
				}
			}
		return found;
	}
}
