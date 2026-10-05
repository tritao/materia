package machinekit.pressbrake.planning;

import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetDiagnosis;

/** Analytic refinement prevents rounded bending contact from becoming a false prism collision. */
class PressContactQuery {
	final sections:PressSectionQuery;
	public function new() { sections = new PressSectionQuery(); }
	/** Null requests the general error-aware query; false proves no positive material overlap for this pair. */
	public function overlap(sheet:Sheet, pose:PressSheetPose, body:PressCollisionBody, tool:PressToolContact):Null<Bool> {
		switch body.owner {
			case SheetWall(_):
				if (body.error != 0) return null;
				for (prism in body.pieces()) for (contact in tool.prisms()) {
					var section = PressPrismSection.of(prism, contact.start, contact.end);
					if (section != null && sections.overlap(section, contact.section)) return true;
				}
				return false;
			case SheetBend(id):
				for (contact in tool.prisms()) {
					var material = PressBendSection.of(sheet.bend(id), pose.evaluation.bend(id), pose.placement, contact.start, contact.end);
					if (material == null) return null;
					for (section in material) if (sections.overlap(section, contact.section)) return true;
				}
				return false;
			case _: throw new SheetDiagnosis("press.contact-body", "Tool contact refinement requires evaluated sheet material");
		}
	}
}
