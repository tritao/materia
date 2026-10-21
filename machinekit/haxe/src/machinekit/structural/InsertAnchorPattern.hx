package machinekit.structural;

import materia.assembly.AssemblyFrames as F;
import materia.casting.InsertSpec;
import materia.casting.InsertAnchorage;
import machinekit.standard.SocketHeadCapScrew;
import machinekit.standard.FlatWasher;
import cadkit.modeling.Part;

enum InsertAnchorContext {
	FootprintOnly;
	Casting(body:Part,pose:materia.assembly.AssemblyRecord.AssemblyFrame);
}

/** One source design rule, shared by every insert in a machine recipe. All
 * lengths are derived from the chosen standard thread or the back footprint. */
typedef InsertAnchorPatternRule = {
	var minimumCount:Int;
	var maximumPitchInShortWidths:Float;
	var embedmentDiameters:Float;
	var coverDiameters:Float;
	var spacingDiameters:Float;
	var screwLengthsMm:Array<Float>;
}

class InsertAnchorPattern {
	/** Minimum pad depth around a central access opening. Both anchor bands use
	 * the same smallest stock-compatible thread and edge/cover rule as derive. */
	public static function minimumBandDepth(openingMm:Float,thicknessMm:Float,material:materia.casting.InsertMaterial,
		rule:InsertAnchorPatternRule):Float {
		validateRule(rule);
		if (!Math.isFinite(openingMm) || openingMm<0) throw "Invalid anchor-band opening";
		for (size in availableSizes()) {
			var hole:materia.casting.InsertHole;
			try hole=holeForStock(size,thicknessMm,material) catch (_:Dynamic) continue;
			var diameter=SocketHeadCapScrew.catalog().get(size).diameter;
			var radius=FlatWasher.catalog().get(size).outerDiameter/2;
			var edge=Math.max(hole.limits.minWall+diameter/2,radius);
			return openingMm+2*(edge+radius+rule.coverDiameters*diameter);
		}
		throw "No standard blind anchor thread fits the source stock thickness";
	}

	/** Extra resin land beyond a steel plate edge for its outermost anchor. */
	public static function minimumOuterLand(thicknessMm:Float,material:materia.casting.InsertMaterial,
		rule:InsertAnchorPatternRule):Float {
		validateRule(rule);
		for (size in availableSizes()) {
			var hole:materia.casting.InsertHole;
			try hole=holeForStock(size,thicknessMm,material) catch (_:Dynamic) continue;
			var diameter=SocketHeadCapScrew.catalog().get(size).diameter;
			var radius=FlatWasher.catalog().get(size).outerDiameter/2;
			var edge=Math.max(hole.limits.minWall+diameter/2,radius);
			return Math.max(0,radius+rule.coverDiameters*diameter-edge);
		}
		throw "No standard blind anchor thread fits the lifting plate stock";
	}

	static function availableSizes():Array<String> {
		var catalog=SocketHeadCapScrew.catalog();
		var sizes=catalog.designations().filter(size -> FlatWasher.catalog().exists(size));
		sizes.sort((a,b) -> Reflect.compare(catalog.get(a).diameter,catalog.get(b).diameter));
		return sizes;
	}

	static function holeForStock(size:String,thickness:Float,material:materia.casting.InsertMaterial):materia.casting.InsertHole {
		var nominal=SocketHeadCapScrew.catalog().get(size);
		var thread=machinekit.standard.FastenerThreads.coarse(nominal.diameter).mating();
		return InsertHoles.records(InsertHoles.tappedBlind(InsertHoles.metricThread(thread),thickness,material))[0];
	}

	static function validateRule(rule:InsertAnchorPatternRule):Void {
		if (rule==null || rule.minimumCount<2 || rule.maximumPitchInShortWidths<=0
			|| rule.embedmentDiameters<=0 || rule.coverDiameters<=0 || rule.spacingDiameters<=0
			|| !Math.isFinite(rule.maximumPitchInShortWidths+rule.embedmentDiameters+rule.coverDiameters+rule.spacingDiameters)
			|| rule.screwLengthsMm==null || rule.screwLengthsMm.length==0)
			throw "Invalid anchor pattern rule";
		for (length in rule.screwLengthsMm)
			if (!Math.isFinite(length) || length<=0) throw "Invalid declared standard anchor screw length";
	}

	public static function derive(id:String, source:CastInsert, rule:InsertAnchorPatternRule,context:InsertAnchorContext):CastInsert {
		try return build(source, rule,context)
		catch (error:Dynamic) throw "Insert " + id + ": anchor pattern: " + Std.string(error);
	}

	static function build(source:CastInsert, rule:InsertAnchorPatternRule,context:InsertAnchorContext):CastInsert {
		validateRule(rule);
		var spec = source.insertSpec();
		var steel = switch materia.casting.InsertFinishing.assess(spec.body, spec.interfaces) {
			case Resolved(result): result;
			case Invalid(reason), Unsupported(reason): throw reason;
		};
		var stock = materia.casting.InsertFinishing.box(steel);
		if (stock == null) throw "Back-face anchor rule requires rectangular steel";
		var catalog = SocketHeadCapScrew.catalog();
		var sizes=availableSizes();
		var failures:Array<String> = [];
		for (size in sizes) {
			var nominal = catalog.get(size);
			var thread = machinekit.standard.FastenerThreads.coarse(nominal.diameter).mating();
			var hole:materia.casting.InsertHole;
			try hole=holeForStock(size,stock.height,source.insertMaterial().grade)
			catch (error:Dynamic) { failures.push(size + ": " + Std.string(error)); continue; }
			var limits = materia.casting.InsertHole.InsertHoleCodec.threadLimits(hole);
			var washer = FlatWasher.metric(size), diameter = nominal.diameter;
			var cover = rule.coverDiameters * diameter;
			var spacing = rule.spacingDiameters * diameter;
			// Steel edge wall and surrounding resin cover are distinct dimensions.
			// The latter is checked against the actual casting, not added to steel.
			var edge = Math.max(hole.limits.minWall + diameter / 2, washer.outerDiameter / 2);
			if (stock.width <= 2 * edge || stock.depth <= 2 * edge) {
				throw "Insufficient back-face footprint for the smallest stock-compatible thread " + size + " and its policy edge wall";
			}
			var insertion = limits.minEngagement;
			var lengths = rule.screwLengthsMm.filter(length -> length >= insertion
				+ rule.embedmentDiameters * diameter + washer.thickness);
			lengths.sort(Reflect.compare);
			if (lengths.length == 0) throw "No declared standard screw length satisfies embedment";
			var points:Array<InsertAnchorHardware.InsertBackAnchorChoice> = [];
			var rules:InsertAnchorRules = {minimumEmbedmentMm:rule.embedmentDiameters * diameter,
				minimumCoverMm:cover, minimumSpacingMm:spacing,
				minimumBearingAreaMm2:Math.PI * (washer.outerDiameter * washer.outerDiameter
					- washer.innerDiameter * washer.innerDiameter) / 4};
			var pitch = rule.maximumPitchInShortWidths * Math.min(stock.width, stock.depth);
			var nx = Std.int(Math.ceil((stock.width - 2 * edge) / pitch));
			var ny = Std.int(Math.ceil((stock.depth - 2 * edge) / pitch));
			for (ix in 0...nx + 1) for (iy in 0...ny + 1) {
				var x = stock.x + edge + ix * (stock.width - 2 * edge) / nx;
				var y = stock.y + edge + iy * (stock.depth - 2 * edge) / ny;
				var clear = true;
				for (other in spec.holes) {
					var axis = F.transformVector(other.frame,0,0,1);
					if (Math.abs(axis.x) + Math.abs(axis.y) > 1e-8)
						throw "Mounting-hole direction needs an explicit back-pattern clearance construction: " + other.id;
					var radius = (other.thread == null ? other.diameter : other.thread.majorDiameter) / 2;
					if (other.counterbore != null) radius = Math.max(radius,other.counterbore.diameter / 2);
					if (other.relief != null) radius = Math.max(radius,other.relief.diameter / 2);
					var dx = x - other.frame.x, dy = y - other.frame.y;
					if (Math.sqrt(dx*dx + dy*dy) < radius + diameter / 2
						+ Math.max(hole.limits.minClearance,other.limits.minClearance)) clear = false;
				}
				for (other in points) {
					var dx = x - other.xMm, dy = y - other.yMm;
					if (Math.sqrt(dx*dx + dy*dy) < washer.outerDiameter + spacing) clear = false;
				}
				var point:InsertAnchorHardware.InsertBackAnchorChoice={id:"back-anchor-" + ix + "-" + iy, xMm:x, yMm:y,
					lengthMm:lengths[0], insertionMm:insertion,
					locking:InsertAnchorLock.RetainingCompound, lockingReference:null};
				if (clear && contained(InsertAnchorHardware.backPattern(source,thread,rules,[point]),context,cover)) points.push(point);
			}
			if (points.length < rule.minimumCount) {
				throw "Smallest stock-compatible thread " + size + " leaves " + points.length
					+ " compliant anchor sites; requires " + rule.minimumCount + " at the declared clearances";
			}
			var result = InsertAnchorHardware.backPattern(source,thread,rules,points);
			// This final native check includes the axial spans of counterbores and
			// reliefs, rather than crediting only their projected clearance.
			InsertAnchorHardware.parts(result.insertSpec());
			return result;
		}
		throw "No standard thread/pattern satisfies the source policy: " + failures.join("; ");
	}

	/** Candidate rejection uses exact native occupancy, not a sampled pose or a
	 * box stand-in. No rule is relaxed when the casting leaves too few sites. */
	static function contained(source:CastInsert,context:InsertAnchorContext,cover:Float):Bool {
		return switch context {
			case FootprintOnly:true;
			case Casting(body,pose):
				var owned:Array<Part> = [],accepted=true;
				function keep(part:Part):Part {owned.push(part);return part;}
				try {
					var spec=source.insertSpec();
					var exterior=keep(machinekit.assembly.PosedParts.moved(keep(InsertBodies.part(spec.body)),pose));
					var hardware:Array<Part> = [];
					for (component in InsertAnchorHardware.parts(spec)) {
						var shape=keep(machinekit.assembly.PosedParts.moved(keep(component.part.geometry()),F.compose(pose,component.pose)));
						shape.shape.setThreadedFaces([]);hardware.push(shape);
					}
					var exposed=keep(keep(Part.fuseAll(hardware)).subtract(exterior));
					if (keep(exposed.subtract(body)).volume()>1e-6) accepted=false;
					if (accepted) for (index in 0...body.shape.subshapeCount(CadKit.ShapeKind.Face)) {
						var face=body.shape.subshape(CadKit.ShapeKind.Face,index);
						try {if (exposed.shape.distance(face)+1e-7<cover) accepted=false;}
						catch (error:Dynamic) {face.close();throw error;}
						face.close();if (!accepted) break;
					}
				} catch (error:Dynamic) {for (part in owned) part.close();throw error;}
				for (part in owned) part.close();
				accepted;
		};
	}
}
