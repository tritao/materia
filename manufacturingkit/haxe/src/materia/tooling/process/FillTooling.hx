package materia.tooling.process;

import materia.casting.InsertSpec.InsertBodyCodec as InsertBodies;

import materia.tooling.casting.CastingMoldPlan;
import materia.casting.BoxAlgebra;
import materia.tooling.casting.BoxMoldPlanner;
import materia.tooling.casting.MoldPlan.MoldBox;
import materia.tooling.panel.PanelContours;
import materia.casting.CastingProfileGeometry;
import materia.casting.CastingProfile.CastingSegment;
import materia.assembly.AssemblyFrames as F;

/** Orthogonal material projection. Curved openings are never credited without
 * an exact model: affected footprints stay blocked and explicitly Unsupported.
 */
class FillTooling {
	public static function project(p:CastingMoldPlan):{blockers:Array<MoldBox>, unsupportedIds:Array<String>} {
		var blockers:Array<MoldBox> = [], unsupportedIds:Array<String> = [];
		var fill = p.document.setup.fillHeightMm;
		function add(box:MoldBox):Bool {
			if (box.z + box.height < fill - 1e-7)
				return false;
			var footprint:MoldBox = {
				x: box.x,
				y: box.y,
				z: p.envelope.z,
				width: box.width,
				depth: box.depth,
				height: p.envelope.height
			};
			var clip = BoxAlgebra.intersection(footprint, p.envelope);
			if (clip == null)
				return false;
			clip.z = 0;
			clip.height = 1;
			blockers.push(clip);
			return true;
		}
		function prism(contour:Array<CastingSegment>, depth:Float, pose:materia.assembly.AssemblyRecord.AssemblyFrame):Array<MoldBox> {
			var xs = [for (edge in contour) edge.start.x],
				ys = [for (edge in contour) edge.start.y];
			xs.sort(Reflect.compare);
			ys.sort(Reflect.compare);
			var boxes:Array<MoldBox> = [];
			for (i in 0...xs.length - 1)
				for (j in 0...ys.length - 1)
					if (xs[i + 1] - xs[i] > 1e-7
						&& ys[j + 1] - ys[j] > 1e-7
						&& CastingProfileGeometry.inside(contour, (xs[i] + xs[i + 1]) / 2, (ys[j] + ys[j + 1]) / 2))
						boxes.push(BoxMoldPlanner.box(xs[i + 1] - xs[i], ys[j + 1] - ys[j], depth,
							F.compose(pose, F.translation((xs[i] + xs[i + 1]) / 2, (ys[j] + ys[j + 1]) / 2, 0))));
			return boxes;
		}
		function orthogonal(contour:Array<CastingSegment>):Bool
			return contour.filter(edge -> edge.center != null
				|| (Math.abs(edge.end.x - edge.start.x) > 1e-7 && Math.abs(edge.end.y - edge.start.y) > 1e-7))
				.length == 0;
		for (member in p.panels.members.concat(p.supportPanels)) {
			var outline = PanelContours.outside(member.panel),
				thickness = member.panel.stock.measuredThickness;
			var bounds = CastingProfileGeometry.bounds(outline);
			var broad = BoxMoldPlanner.box(bounds.width, bounds.height, thickness,
				F.compose(member.pose, F.translation(bounds.x + bounds.width / 2, bounds.y + bounds.height / 2, 0)));
			var old = blockers.length;
			if (!add(broad))
				continue;
			blockers.splice(old, 1);
			if (!orthogonal(outline)) {
				add(broad);
				unsupportedIds.push(member.panel.id);
				continue;
			}
			var material = prism(outline, thickness, member.pose);
			for (op in member.panel.operations)
				if (op.kind != OutsideProfile) {
					var pose = op.side == materia.tooling.ToolingEnums.MachiningSide.PositiveZ ? F.compose(member.pose,
						F.translation(0, 0, thickness - op.depth)) : member.pose;
					if (orthogonal(op.contour))
						material = BoxAlgebra.subtract(material, prism(op.contour, op.depth, pose));
					else {
						var cutBounds = CastingProfileGeometry.bounds(op.contour);
						var cut = BoxMoldPlanner.box(cutBounds.width, cutBounds.height, op.depth,
							F.compose(pose, F.translation(cutBounds.x + cutBounds.width / 2, cutBounds.y + cutBounds.height / 2, 0)));
						var before = blockers.length;
						if (add(cut)) {
							blockers.splice(before, 1);
							unsupportedIds.push(member.panel.id);
						}
					}
				}
			for (box in material)
				add(box);
		}
		for (volume in [for (f in p.formers) f.volume].concat([for (f in p.fixtures) f.datum])) {
			var material = volume.pieces.copy();
			for (cut in volume.cuts) {
				if (orthogonal(cut.operation.contour))
					material = BoxAlgebra.subtract(material, prism(cut.operation.contour, cut.operation.depth, cut.pose));
				else {
					var bounds = CastingProfileGeometry.bounds(cut.operation.contour);
					var before = blockers.length;
					if (add(BoxMoldPlanner.box(bounds.width, bounds.height, cut.operation.depth,
						F.compose(cut.pose, F.translation(bounds.x + bounds.width / 2, bounds.y + bounds.height / 2, 0))))) {
						blockers.splice(before, 1);
						unsupportedIds.push(volume.id);
					}
				}
			}
			for (box in material)
				add(box);
			for (profile in volume.profiles) {
				if (orthogonal(profile.contour))
					for (box in BoxAlgebra.subtract(prism(profile.contour, profile.depth, profile.pose), profile.subtract))
						add(box);
				else {
					var bounds = CastingProfileGeometry.bounds(profile.contour);
					if (add(BoxMoldPlanner.box(bounds.width, bounds.height, profile.depth,
						F.compose(profile.pose, F.translation(bounds.x + bounds.width / 2, bounds.y + bounds.height / 2, 0)))))
						unsupportedIds.push(volume.id);
				}
			}
		}
		for (core in p.cores)
			add(core.cavity);
		for (insert in p.document.input.inserts) {
			var dimensions = InsertBodies.box(insert.body, insert.id);
			add(BoxMoldPlanner.box(dimensions.width, dimensions.depth, dimensions.height, F.compose(p.document.setup.sourceToMold, insert.pose)));
		}
		var unique:Array<String> = [];
		for (id in unsupportedIds)
			if (unique.indexOf(id) < 0)
				unique.push(id);
		unique.sort(Reflect.compare);
		return {blockers: BoxAlgebra.subtract(blockers, []), unsupportedIds: unique};
	}
}
