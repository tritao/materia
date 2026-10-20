package materia.tooling.process;

import materia.tooling.ToolingValidation;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.casting.BoxVolumes;
import materia.tooling.casting.MoldPlan.MoldBox;
import materia.tooling.process.CastingProcessSpec;
import materia.tooling.process.OrthogonalSections;

typedef ThermalSectionBinding = {
	var sectionId:String;
	var spanIds:Array<String>;
	var status:ToolingCheckStatus;
	var message:String;
}

typedef ThermalCoverageReport = {
	var version:Int;
	var sourceFingerprint:String;
	var processFingerprint:String;
	var totalVolumeMm3:Float;
	var coveredVolumeMm3:Float;
	var uncoveredVolumeMm3:Float;
	var uncoveredRegions:Array<MoldBox>;
	var bindings:Array<ThermalSectionBinding>;
	var validation:Array<ToolingValidation>;
}

/** Exact geometric accounting for declared slabs, separate from solver validity,
 * thermal boundary evidence, junction heat flow and physical qualification.
 */
class ThermalCoverage {
	public static function analyze(sourceId:String, sections:OrthogonalSectionReport, raw:CastingProcessSpec, processFingerprint:String,
			?document:materia.tooling.ToolingDocument):ThermalCoverageReport {
		materia.tooling.ToolingIds.validate(sourceId);
		var spec = ProcessSpecCodec.snapshot(raw),
			covered:Array<MoldBox> = [],
			bindings:Array<ThermalSectionBinding> = [],
			checks:Array<ToolingValidation> = [];
		var assigned = new Map<String, Array<String>>();
		for (section in spec.sections)
			for (id in section.spanIds) {
				if (!assigned.exists(id))
					assigned.set(id, []);
				assigned.get(id).push(section.id);
			}
		for (section in spec.sections) {
			var status:ToolingCheckStatus = Pass,
				message = "Exact extracted slab spans and source faces match the declared thickness.";
			if (section.spanIds.length > 0 && section.geometryFingerprint != sections.geometryFingerprint) {
				status = Fail;
				message = "Stale cure section geometry fingerprint.";
			}
			var matched:Array<PlanarSectionSpan> = [],
				ids:Array<String> = [],
				axis:Null<SectionAxis> = null;
			if (section.spanIds.length == 0) {
				status = Pending;
				message = "Authored solver section has no geometric span binding.";
			}
			for (id in section.spanIds) {
				var spans = sections.spans.filter(s -> s.id == id);
				if (spans.length != 1) {
					status = Fail;
					message = "Unknown extracted span: " + id;
					continue;
				}
				var span = spans[0];
				matched.push(span);
				var assignments:Array<String> = cast assigned.get(id);
				if (assignments.length > 1) {
					status = Fail;
					message = "Span assigned to multiple sections " + assignments.join(", ") + ": " + id;
				}
				if (Math.abs(span.thicknessMm - section.thicknessMm) > 1e-7) {
					status = Fail;
					message = "Declared thickness differs from extracted span: " + id;
				}
				if (axis != null && axis != span.axis) {
					status = Fail;
					message = "One slab section cannot bind different axes.";
				}
				axis = span.axis;
				if (document != null
					&& (!ThermalFaces.matches(span, true, section.left, sections, spec, document)
						|| !ThermalFaces.matches(span, false, section.right, sections, spec, document))) {
					status = Fail;
					message = "Declared slab boundary kind differs from owning thermal faces.";
				}
				for (owner in span.lowerSourceIds.concat(span.upperSourceIds))
					if (ids.indexOf(owner) < 0)
						ids.push(owner);
			}
			ids.sort(Reflect.compare);
			if (matched.length > 0
				&& materia.tooling.ToolingCodec.canonical(ids) != materia.tooling.ToolingCodec.canonical(section.sourceIds)) {
				status = Fail;
				message = "Section source IDs differ from bound opposing faces.";
			}
			if (status == Pass)
				for (span in matched)
					for (strip in span.strips)
						if (covered.filter(previous -> BoxVolumes.intersection(previous, strip) != null).length > 0) {
							status = Fail;
							message = "Slab volume is covered twice, including assignments across different axes.";
						}
			if (status == Pass)
				for (span in matched)
					for (strip in span.strips)
						covered.push(strip);
			bindings.push({
				sectionId: section.id,
				spanIds: section.spanIds.copy(),
				status: status,
				message: message
			});
			checks.push(FillAnalysis.check("process.cure.binding." + section.id, section.sourceIds, status, message,
				[sections.sourceFingerprint, processFingerprint]));
		}
		checks.push(FillAnalysis.check("process.cure.boundary-ownership", [sourceId],
			document == null ? Pending : bindings.filter(b -> b.status == Fail).length > 0 ? Fail : Pass,
			"Slab contact kinds are checked against source-owned EPS, fill-plane and mold-contact faces.", []));
		var region = [
			for (span in sections.spans)
				if (span.axis == X) for (strip in span.strips) strip
		];
		var missing = BoxVolumes.subtract(region, covered),
			uncovered = BoxVolumes.volume(missing);
		var failed = bindings.filter(b -> b.status == Fail).length > 0;
		var complete = uncovered <= Math.max(1e-6, sections.mixVolumeMm3 * 1e-10) && bindings.filter(b -> b.status != Pass).length == 0;
		checks.push(FillAnalysis.check("process.cure.geometric-coverage", [sourceId], failed ? Fail : complete ? Pass : Pending,
			"Bound slab geometry covers "
			+ (sections.mixVolumeMm3 - uncovered)
			+ " of "
			+ sections.mixVolumeMm3
			+ " mm3. Geometric coverage does not establish 1D validity at junctions.",
			[sections.sourceFingerprint, processFingerprint]));
		checks.push(FillAnalysis.check("process.cure.junction-model", [sourceId], sections.maximumInscribedThicknessMm == null ? Pending : Pass,
			sections.maximumInscribedThicknessMm == null ? "Nonrectangular casting needs a qualified junction thermal model; an inscribed sphere is not a slab or a cooling boundary." : "Rectangular solid has no nonrectangular junction; declared slab boundaries and numerical/physical qualification remain separate.",
			[sections.sourceFingerprint]));
		return {
			version: 1,
			sourceFingerprint: sections.sourceFingerprint,
			processFingerprint: processFingerprint,
			totalVolumeMm3: sections.mixVolumeMm3,
			coveredVolumeMm3: sections.mixVolumeMm3 - uncovered,
			uncoveredVolumeMm3: uncovered,
			uncoveredRegions: missing,
			bindings: bindings,
			validation: checks
		};
	}
}
