import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.PosedParts;
import machinekit.component.Solids;
import ReferenceHousing.HousingFeature;

/** DT0 checks for the reference housing: supplier envelope and interfaces, walls, tool access from each setup,
 * and the index angles each setup needs. Prints the setup sheet that DT6, DT7 and DT9 size against.
 */
class ReferenceHousingChecks {
	static var assertions = 0;
	/** Lateral wall around a cut, mm: bores and pockets, and the ligament around a tapped hole. */
	static inline var MIN_WALL:Float = 3;
	static inline var MIN_LIGAMENT:Float = 1.5;

	static function check(ok:Bool, label:String):Void {
		assertions++;
		if (!ok) throw label;
	}

	static function near(a:Float, b:Float, tolerance:Float, label:String):Void check(Math.abs(a - b) <= tolerance, '$label ($a vs $b)');

	static function fmt(value:Float):String return Std.string(Math.round(value * 100) / 100);

	/** The feature's cutter beyond `depth` below its entry, so walls are measured without the entry face. */
	static function clipped(feature:HousingFeature, depth:Float):Part {
		var cutter = feature.cutter();
		var keep = Part.cylinderAlong(200, feature.entry.subtract(feature.approach.scale(depth)), feature.approach.scale(-1), 400);
		var result = cutter.intersect(keep);
		cutter.close();
		keep.close();
		return result;
	}

	/** 0 when the cut has no wall requirement (faces, contours, supplier features). */
	static function minimumWall(feature:HousingFeature):Float {
		if (feature.section.length == 0) return 0;
		if (StringTools.startsWith(feature.kind, "form-tap")) return MIN_LIGAMENT;
		return switch (feature.kind) {
			case "precision-bore" | "bore" | "pocket" | "hole": MIN_WALL;
			case _: 0;
		}
	}

	public static function main():Void {
		var housing = new ReferenceHousing();
		var part = housing.geometry();
		check(part.valid() && part.solidCount() == 1, "finished housing is one valid solid");

		// Supplier exterior, eRob70H××T.STEP solid 8: X ±36.5, Y -48.5..36.5, Z -45..40.
		var box = PosedParts.boxOf(part);
		for (pair in [[box.minX, -36.5], [box.maxX, 36.5], [box.minY, -48.5], [box.maxY, 36.5], [box.minZ, -45], [box.maxZ, 40]])
			near(pair[0], pair[1], 0.01, "envelope matches supplier housing bounds");
		near(ReferenceHousing.FRONT - ReferenceHousing.REAR, 85, 1e-9, "drum length between output and rear cover faces");
		near(-ReferenceHousing.SOCKET_FLOOR, 39, 1e-9, "socket floor 39 ± 0.05 from the drum axis (drawing)");
		near(2 * ReferenceHousing.SOCKET_RADIUS, 58, 1e-9, "link socket Ø58 G7 (drawing)");
		var cbores = [for (f in housing.features) if (StringTools.startsWith(f.id, "link-cbore")) f];
		var csks = [for (f in housing.features) if (StringTools.startsWith(f.id, "link-csk")) f];
		check(cbores.length == 11 && csks.length == 3, "ZeroErr link pattern: 11 counterbored + 3 countersunk M3");

		var volume = part.volume();
		Sys.println('Housing: ${fmt(volume / 1000)} cm³, ${fmt(volume * ReferenceHousing.DENSITY)} g in 6061');

		// Every feature is reachable by indexing its setup's rotary axis, with nothing of the part above it.
		var angles:Array<Float> = [];
		for (feature in housing.features) {
			var setup = housing.setup(feature.setup);
			var index = setup.index(feature.approach);
			check(index != null, '${feature.id}: approach lies in the ${setup.id} rotary plane');
			var angle:Float = index == null ? -1 : index;
			angles.push(angle);
			if (feature.section.length > 0) {
				var path = Part.cylinderAlong(feature.diameter() / 2 * 0.999, feature.entry.add(feature.approach.scale(0.01)),
					feature.approach, 150);
				var hit = path.intersect(part);
				check(hit.volume() < 1e-3, '${feature.id}: the tool path from the spindle side is clear of the part (${hit.volume()} mm³)');
				hit.close();
				path.close();
			}
		}

		// Walls to the exterior: the supplier envelope is the outside, cuts may not come closer than their minimum.
		var envelope = ReferenceHousing.envelope();
		var air = Solids.cut(Part.box(300, 300, 300, Center, Center, Center), [envelope]);
		var exempt = ["output-recess", "link-socket"];
		var thinnest = Math.POSITIVE_INFINITY, thinnestId = "";
		for (feature in housing.features) {
			var limit = minimumWall(feature);
			if (limit <= 0 || exempt.indexOf(feature.id) >= 0) continue;
			// A cut no deeper than its wall limit has no lateral wall beyond the entry; its neighbours' checks cover it.
			if (feature.depth() <= limit + 1e-9) continue;
			var inner = clipped(feature, limit);
			var wall = inner.distance(air);
			inner.close();
			check(wall >= limit - 1e-6, '${feature.id}: wall to the exterior ${fmt(wall)} mm below ${limit} mm');
			// The clipped cut reads at most `limit` (its entry plane); only a thinner lateral wall would show here.
			if (wall < limit - 1e-6 && wall < thinnest) {
				thinnest = wall;
				thinnestId = feature.id;
			}
		}
		air.close();
		// Walls between cuts that must stay separate.
		var pairs:Array<Array<String>> = [
			["stem-pocket", "stator-bore"], ["cable-hole", "stator-bore"], ["cable-hole", "stem-pocket"], ["rear-cavity", "rotor-bearing-seat"],
			["stem-pocket", "rear-cavity"]
		];
		for (i in 0...12) pairs.push(['gear-screw-$i', "stator-bore"]);
		for (i in 0...6) pairs.push(['cover-screw-$i', "rear-cavity"]);
		for (pair in pairs) {
			var a = housing.feature(pair[0]), b = housing.feature(pair[1]);
			var limit = Math.min(minimumWall(a), minimumWall(b));
			var ca = a.cutter(), cb = b.cutter();
			var wall = ca.distance(cb);
			ca.close();
			cb.close();
			check(wall >= limit - 1e-6, '${pair[0]} / ${pair[1]}: wall ${fmt(wall)} mm below ${limit} mm');
			if (wall < thinnest) {
				thinnest = wall;
				thinnestId = pair.join(" / ");
			}
		}
		Sys.println('Thinnest checked wall: ${fmt(thinnest)} mm at $thinnestId');

		// Stock, removal and tool reach from the setup's outer stock surface.
		var stock = ReferenceHousing.stock(), tabs = ReferenceHousing.tabs();
		var stockVolume = stock.volume(), tabVolume = tabs.volume();
		Sys.println('Stock: ${ReferenceHousing.BAR} mm square bar × ${ReferenceHousing.BAR_LENGTH} mm, ${fmt(stockVolume / 1000)} cm³; '
			+ 'removed ${fmt((stockVolume - volume) / 1000)} cm³ (${fmt(100 * (1 - volume / stockVolume))} %), tabs ${fmt(tabVolume / 1000)} cm³');
		stock.close();
		tabs.close();
		var indexCounts:Array<Int> = [];
		for (setup in housing.setups) {
			Sys.println('\n${setup.label}');
			var keys:Array<Float> = [];
			for (i in 0...housing.features.length)
				if (housing.features[i].setup == setup.id && keys.indexOf(angles[i]) < 0) keys.push(angles[i]);
			keys.sort((a, b) -> Reflect.compare(a, b));
			indexCounts.push(keys.length);
			for (key in keys) {
				Sys.println('  A ${fmt(key)}°:');
				for (i in 0...housing.features.length) {
					var feature = housing.features[i], id = feature.id;
					if (feature.setup != setup.id || angles[i] != key) continue;
					var outer = setup.id == "op1" ? ReferenceHousing.BAR / 2 + setup.centre.dot(feature.approach) : ReferenceHousing.RADIUS;
					var reach = outer - feature.entry.dot(feature.approach) + feature.depth();
					var size = feature.section.length > 0 ? 'Ø${fmt(feature.diameter())} × ${fmt(feature.depth())}, ' : "";
					Sys.println('    ${id}: ${feature.kind}, ${size}reach ${fmt(reach)} mm, ${feature.tolerance} [${feature.source}]');
				}
			}
			Sys.println('  ${keys.length} index positions');
		}
		check(indexCounts[0] == 3 && indexCounts[1] == 14, "Op 1 cuts front, socket and rear at 3 indices; Op 2 needs 14");
		part.close();
		envelope.close();
		Sys.println('\nReferenceHousingChecks: $assertions assertions passed');
	}
}
