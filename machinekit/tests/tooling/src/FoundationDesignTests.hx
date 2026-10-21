import machinekit.structural.MineralCasting;
import machinekit.component.ComponentValues;
import machinekit.component.Solids;
import machinekit.assembly.PosedParts;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import materia.casting.CastingCavities;
import materia.casting.CastingPatterns;
import materia.assembly.AssemblyFrames as F;

/** Independent frozen pre-migration CAD plus exact neutral/CAD intersection checks. */
class FoundationDesignTests {
	static var assertions = 0;

	static function same(a:Part, b:Part, id:String):Void {
		var added = a.subtract(b),
			removed = b.subtract(a),
			difference = added.volume() + removed.volume();
		added.close();
		removed.close();
		assertions++;
		if (difference > 1e-3)
			throw id + " symmetric difference: " + difference;
		Sys.println(id + " symmetric difference: " + difference);
	}

	/** The accepted socket cut-over is an explicit design correction to the frozen box seats.
	 * Keep the pre-socket STEP as an independent control: restore its twelve 20 mm
	 * seat voids, then remove only the declared source socket cavities.
	 */
	static function socketSeats(foundation:machinekit.structural.MineralFoundation):Array<materia.casting.Box3> {
		var sockets = foundation.insertLocations().filter(row -> switch row.part.insertSpec().origin {
			case Catalogue(product): product.reference == machinekit.structural.FixingSockets.PFEIFER_REFERENCE;
			default:false;
		});
		assertions++;
		if (sockets.length != 12) throw "Hobber tray must use twelve catalogue sockets, with no duplicate box seats";
		return [for (row in sockets) {
			var hole = row.part.insertSpec().holes[0], mouth = F.compose(row.pose,hole.frame);
			{x:mouth.x-10,y:mouth.y-10,z:mouth.z-20,width:20.0,depth:20.0,height:20.0};
		}];
	}

	static function socketDesignChange(previous:Part,foundation:machinekit.structural.MineralFoundation):Part {
		var oldSeats = socketSeats(foundation);
		var restored = Solids.building([new Part(previous.shape.cloneShape())], parts -> {
			for (box in oldSeats) {
				var local = Part.box(box.width,box.depth,box.height);
				try parts.push(PosedParts.moved(local,F.translation(box.x+box.width/2,box.y+box.depth/2,box.z)))
				catch (error:Dynamic) {local.close(); throw error;}
				local.close();
			}
			return Solids.union(parts);
		});
		var source = foundation.body;
		var ids = source.castingFeatures().filter(feature -> !materia.casting.CastingFeature.CastingGeometrySolids.isBox(feature.geometry.solid));
		assertions++;
		if (ids.length != oldSeats.length || ids.filter(feature -> feature.kind != InsertRecess).length != 0)
			throw "Socket correction must change only source-owned insert recesses";
		var names = [for (feature in ids) feature.id];
		return Solids.building([restored], parts -> {
			for (cavity in source.finishedCavities())
				if (names.indexOf(cavity.featureId) >= 0)
					parts.push(machinekit.structural.CastingCavityCad.part(cavity));
			return Solids.cut(parts[0],parts.slice(1));
		});
	}

	static function prism(p:materia.casting.CastingPrism):Part {
		var local = Part.prism([for (v in p.profile) new Vector(v.x, v.y)], 0, p.height);
		try {
			var out = PosedParts.moved(local, p.pose);
			local.close();
			return out;
		} catch (e:Dynamic) {
			local.close();
			throw e;
		}
	}

	static function slopedFoot():Void {
		var values = new ComponentValues().setNumber("width", 100)
			.setNumber("depth", 80)
			.setNumber("height", 120)
			.setNumber("footWidth", 160)
			.setNumber("footHeight", 60)
			.setNumber("footShoulder", 20);
		var source = new MineralCasting(values), body = source.geometry();
		assertions++;
		if (source.collisionHullQualification.status != machinekit.structural.CastingCollisionQualification.CastingCollisionStatus.Unsupported
			|| !source.collisionHullQualification.diagnostics("foot-fixture").hasErrors())
			throw "Non-box casting collision hull must report Unsupported explicitly";
		var feature = CastingPatterns.box("foot-crossing", materia.casting.CastingFeature.CastingFeatureKind.ServiceOpening,
			[CastingPatterns.face("foot.side-2")], 60, 30, 30, 65, 0, 25);
		var cavity = CastingCavities.derive(source.castingEnvelope(), [feature])[0];
		var input:materia.tooling.casting.CastingToolingInput = {
			id: "foot-fixture", revision: 1, lengthUnit: materia.tooling.ToolingEnums.ToolingLengthUnit.Millimetres,
			datumScheme:null, interfaces:[], interfaceContext:null,
			envelope: source.castingEnvelope(), features: [feature], inserts: []
		};
		var frame = MillMoldSetup.frame();
		var document = MillMoldSetup.document(frame), options = MillMoldSetup.options(frame);
		document.input = input;
		document.setup.sourceToMold = F.identity();
		document.setup.fillHeightMm = 120;
		document.setup.features = [];
		document.setup.sourceFingerprint = materia.tooling.ToolingCodec.inputFingerprint(input);
		options.precision = []; options.anchorage = []; options.groutedJoints = [];
		var settings:materia.tooling.process.FillAnalysis.FillSourceSettings = {
			sourceFingerprint: materia.tooling.ToolingCodec.documentFingerprint(document), processFingerprint: "source-screen",
			cores: [], densityKgM3: 2200, gravityMps2: 9.81, pourRateMm3PerSecond: null,
			maximumAirEscapeWidthMm: null, extraLevels: [], fillObstructions: [], panels: []
		};
		var fill = materia.tooling.process.FillAnalysis.assessSource(document, settings);
		var sections = materia.tooling.process.OrthogonalSections.assessSource(document, settings.sourceFingerprint);
		var planned = materia.tooling.casting.CastingMoldPlanner.assess(document, options);
		for (checks in [fill.validation, sections.validation, planned.validation]) {
			assertions++;
			if (checks.filter(check -> check.status == materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported
				&& check.affectedIds.indexOf(feature.id) >= 0).length != 1)
				throw "Actual box-only consumer must report the exact non-box source feature";
		}
		assertions++;
		if (fill.report != null || sections.report != null || planned.plan != null || planned.fabrication.ready
			|| planned.changeRequests.length != 1 || planned.changeRequests[0].blockingChecks[0] != planned.validation[0].rule)
			throw "Unsupported source must have no approximate analysis or mold and must retain a source-linked request";
		var foundation = new machinekit.structural.MineralFoundation(values.copy()
			.setToken("features", CastingPatterns.encode([feature])), []);
		var diagnostics = machinekit.structural.FoundationIntegrity.castingReview(foundation, document);
		assertions++;
		if (diagnostics.diagnostics.items.filter(item -> StringTools.startsWith(item.code, "casting.collision-hull.unsupported")).length != 1
			|| diagnostics.unsupported == 0)
			throw "Collision-hull Unsupported must reach the real foundation design diagnostics";
		var boxDocument = materia.tooling.ToolingCodec.snapshot(document);
		boxDocument.input.id = "box-control";
		boxDocument.input.envelope = materia.casting.CastingEnvelope.build(100, 80, 120);
		boxDocument.input.features = [];
		boxDocument.setup.sourceFingerprint = materia.tooling.ToolingCodec.inputFingerprint(boxDocument.input);
		settings.sourceFingerprint = materia.tooling.ToolingCodec.documentFingerprint(boxDocument);
		var boxFill = materia.tooling.process.FillAnalysis.assessSource(boxDocument, settings);
		var boxSections = materia.tooling.process.OrthogonalSections.assessSource(boxDocument, settings.sourceFingerprint);
		var fillReport:materia.tooling.process.FillAnalysis.FillReport = cast boxFill.report;
		var sectionReport:materia.tooling.process.OrthogonalSections.OrthogonalSectionReport = cast boxSections.report;
		assertions++;
		if (boxFill.report == null || boxSections.report == null || fillReport.geometricResinVolumeMm3 != 960000
			|| sectionReport.mixVolumeMm3 != 960000)
			throw "Supported box source entry points must run the actual fill and section algorithms";
		var localTool = Part.box(60, 30, 30),
			tool = localTool.translated(new Vector(65, 0, 25));
		localTool.close();
		var reference = tool.intersect(body);
		var actual = Solids.union([for (p in materia.casting.CastingCavities.prisms(cavity)) prism(p)]);
		same(reference, actual, "foot-crossing cut/body intersection");
		reference.close();
		actual.close();
		tool.close();
		body.close();
		var message = "";
		try
			CastingCavities.boxes(cavity)
		catch (e:Dynamic)
			message = Std.string(e);
		assertions++;
		if (message.indexOf("Unsupported non-box finished cavity: foot-crossing") < 0)
			throw "Box consumer lost feature ID";
	}

	static function unrealizedSource():Void {
		for (rounded in [false, true]) {
			var feature = CastingPatterns.box("unrealized", materia.casting.CastingFeature.CastingFeatureKind.Lightening,
				[CastingPatterns.face("body.z-min")], 20, 20, 20);
			if (rounded) feature.cornerRadius = 3; else feature.draft.degrees = 3;
			var source = new MineralCasting(new ComponentValues().setNumber("width", 100).setNumber("depth", 80).setNumber("height", 120)
				.setToken("features", CastingPatterns.encode([feature])));
			assertions++;
			if (source.collisionHullQualification.status != machinekit.structural.CastingCollisionQualification.CastingCollisionStatus.Unsupported)
				throw "Unrealized source intent must not publish an approximate collision hull";
			var failed = false;
			try { var body = source.geometry(); body.close(); }
			catch (error:Dynamic) failed = Std.string(error).indexOf("Unsupported casting cavity geometry: unrealized") >= 0;
			assertions++;
			if (!failed) throw "CAD must not silently omit the unrealized design feature";
		}
	}

	static function buriedThreadExit():Void {
		var choice = machinekit.structural.InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(12).mating());
		var dimensions = new ComponentValues().setNumber("width", 100).setNumber("depth", 100).setNumber("height", 60);
		var insertValues = machinekit.structural.CastInsert.boxValues(40, 40, 25).setToken("role", "threaded-anchor");
		var failed = false;
		try new machinekit.structural.MineralFoundation(dimensions, [{
			id: "anchor", pose: F.translation(0, 0, 35),
			openings: [materia.casting.CastingPatterns.face("body.z-max")],
			part: new machinekit.structural.CastInsert(insertValues
				.setToken("holes", machinekit.structural.InsertHoles.tappedThrough(choice, 25)))
		}]) catch (error:Dynamic)
			failed = Std.string(error).indexOf("Through insert thread exits into casting material: anchor/fixing") >= 0;
		assertions++;
		if (!failed) throw "A buried tapped through-hole must be rejected by the source design";
		var foundation = new machinekit.structural.MineralFoundation(dimensions, [{
			id: "anchor", pose: F.translation(0, 0, 35),
			openings: [materia.casting.CastingPatterns.face("body.z-max")],
			part: new machinekit.structural.CastInsert(insertValues
				.setToken("holes", machinekit.structural.InsertHoles.tappedBlind(choice, 25)))
		}]);
		var hole = machinekit.structural.InsertHoles.records(foundation.insertLocations()[0].part.parameterToken("holes"))[0];
		assertions++;
		if (hole.extent != Blind || hole.depth != 22 || hole.clearanceDiameter != 13.5)
			throw "Blind C45 anchor must preserve its policy back wall and ISO 273 medium clearance";
		for (width in [40.0, 4.0]) {
			var opening = machinekit.structural.CastingFeaturePatterns.box("service", ServiceOpening,
				[materia.casting.CastingPatterns.face("body.z-min")],
				width, 40, 35, 0, 0, 0);
			var accepted = true;
			try new machinekit.structural.MineralFoundation(dimensions.setToken("features", materia.casting.CastingPatterns.encode([opening])), [{
				id: "anchor", pose: F.translation(0, 0, 35),
				openings: [materia.casting.CastingPatterns.face("body.z-max"), materia.casting.CastingPatterns.feature("service")],
				part: new machinekit.structural.CastInsert(insertValues
					.setToken("holes", machinekit.structural.InsertHoles.tappedThrough(choice, 25)))
			}]) catch (error:Dynamic) {
				if (Std.string(error).indexOf("Through insert thread exits into casting material: anchor/fixing") < 0) throw error;
				accepted = false;
			}
			assertions++;
			if (accepted != (width == 40))
				throw "The whole thread exit disk must open into the declared feature, not just its centre";
		}

	}

	public static function main():Void {
		unrealizedSource();
		buriedThreadExit();
		var flat = new machinekit.assembly.Diagnostics();
		machinekit.design.DesignChecks.blindFastener(flat, "flat-bottom anchor", 40, 20, 23.5, 27, 0, 30, 15.6, 1.75, 3);
		flat.throwIfErrors();
		assertions++;
		var negativePoint = false;
		try machinekit.design.DesignChecks.blindFastener(flat, "invalid point", 40, 20, 23.5, 27, -1, 30, 15.6, 1.75, 3)
		catch (error:Dynamic) negativePoint = Std.string(error).indexOf("Invalid blind drill point") >= 0;
		assertions++;
		if (!negativePoint) throw "A flat bottom must not admit an invalid negative drill point";


		var spec = machinekit.standard.FastenerThreads.coarse(12).mating();
		var limits = machinekit.design.ThreadedHolePolicy.resolve(spec, C45Bright);
		assertions++;
		if (Math.abs(limits.minEngagement - 15.6) > 1e-9 || limits.minTipGap != spec.pitch || limits.minBack != 3
			|| limits.minWall != 12 || Math.abs(limits.minClearance - 1.2) > 1e-9)
			throw "Steel hole policy must resolve dimensional design minima";
		var custom = machinekit.design.ThreadedHolePolicy.resolve(spec, C45Bright, {minEngagement: 18});
		assertions++;
		if (custom.minEngagement != 18 || custom.overrides.minEngagement != 18
			|| custom.minWall != limits.minWall || limits.overrides.minEngagement != null)
			throw "Hole limit overrides must be recorded without changing policy defaults";
		var rejected = false;
		try machinekit.design.ThreadedHolePolicy.resolve(spec, C45Bright, {minWall: 0})
		catch (e:Dynamic) rejected = Std.string(e).indexOf("finite and positive: minWall") >= 0;
		assertions++;
		if (!rejected)
			throw "Hole policy accepted a placeholder wall limit";
		var pattern = machinekit.structural.InsertHoles.records(machinekit.structural.InsertHoles.through(6, 20));
		pattern[0].id = "left";
		pattern[0].frame.x = -10;
		var right = machinekit.structural.InsertHoles.records(machinekit.structural.InsertHoles.through(6, 20))[0];
		right.id = "right";
		right.frame.x = 10;
		pattern.push(right);
		var encoded = machinekit.structural.InsertHoles.fromNeutral(pattern);
		var restored = machinekit.structural.InsertHoles.records(encoded);
		var interfaces = machinekit.structural.InsertHoles.facet(restored, 50, 40, 20);
		var copied = machinekit.structural.InsertHoles.copy(interfaces);
		assertions++;
		if (copied.length != 2 || copied[0].frame.x != -10 || copied[1].frame.x != 10
			|| machinekit.structural.InsertHoles.fromNeutral(copied) != encoded)
			throw "Insert hole pattern must retain frames and resolved policy through the facet";
		copied[1].counterbore = {diameter: 12, depth: 3};
		var analytic = materia.casting.InsertHoleGeometry.massProperties(50, 40, 20, copied);
		var insert = new machinekit.structural.CastInsert(machinekit.structural.CastInsert.boxValues(50, 40, 20)
			.setToken("role", "generic-mount")
			.setToken("holes", machinekit.structural.InsertHoles.fromNeutral(copied)));
		var solid = insert.geometry(), physical = solid.massProperties();
		solid.close();
		assertions++;
		if (Math.abs(analytic.volume - physical.volume) > 1e-6
			|| Math.abs(analytic.center.x - physical.centerOfMass.x) > 1e-7
			|| Math.abs(analytic.center.z - physical.centerOfMass.z) > 1e-7)
			throw "Multi-hole insert mass and centroid must agree with actual drilled steel CAD";
		var blind = machinekit.structural.InsertHoles.records(machinekit.structural.InsertHoles.through(6, 20))[0];
		blind.extent = materia.casting.InsertHole.HoleExtent.Blind;
		blind.depth = 8;
		blind.counterbore = {diameter: 12, depth: 3};
		var blindInsert = new machinekit.structural.CastInsert(machinekit.structural.CastInsert.boxValues(50, 40, 20)
			.setToken("role", "generic-mount")
			.setToken("holes", machinekit.structural.InsertHoles.fromNeutral([blind])));
		var drilled = blindInsert.geometry(), actualBlind = drilled.massProperties();
		var pin = machinekit.tooling.ToolingPin.boreMask(blind).geometry(), mask = PosedParts.moved(pin, blind.frame);
		pin.close();
		var common = mask.intersect(drilled), overlapVolume = common.volume();
		common.close();
		mask.close();
		drilled.close();
		var expectedBlind = materia.casting.InsertHoleGeometry.massProperties(50, 40, 20, [blind]);
		assertions++;
		if (overlapVolume > 1e-7 || Math.abs(expectedBlind.volume - actualBlind.volume) > 1e-6)
			throw "Blind counterbore depth and stepped protection must agree with actual CAD";
		var tapped = machinekit.structural.InsertHoles.records(machinekit.structural.InsertHoles.tappedThrough(
			machinekit.structural.InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(12).mating()), 20));
		tapped[0].limits = machinekit.design.ThreadedHolePolicy.neutral(custom);
		var overridden = machinekit.structural.InsertHoles.fromNeutral(tapped);
		var recovered = machinekit.structural.InsertHoles.copy(machinekit.structural.InsertHoles.facet(
			machinekit.structural.InsertHoles.records(overridden), 50, 40, 20));
		assertions++;
		if (recovered[0].limits.policyId != custom.policyId || recovered[0].limits.overrides[0].value != 18
			|| machinekit.structural.InsertHoles.fromNeutral(recovered) != overridden)
			throw "Insert facet lost resolved limits or their recorded overrides";
		var target = new machinekit.structural.CastInsert(machinekit.structural.CastInsert.boxValues(50, 40, 20)
			.setToken("role", "generic-mount")
			.setToken("holes", machinekit.structural.InsertHoles.tappedThrough(machinekit.structural.InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(12).mating()), 20)));
		var clamp = new machinekit.structural.CastInsert(machinekit.structural.CastInsert.boxValues(50, 50, 5)
			.setToken("role", "generic-mount")
			.setToken("holes", machinekit.structural.InsertHoles.through(14, 5)));
		var screw = machinekit.standard.SocketHeadCapScrew.metric("M12", 30, machinekit.standard.PropertyClass.Steel88);
		var head = materia.assembly.AssemblyFrames.fromRotationMatrix(0, 0, 25, [1, 0, 0, 0, -1, 0, 0, 0, -1]);
		var joint = new machinekit.assembly.MachineAssembly();
		joint.addComponent("target", target);
		joint.addComponent("clamp", clamp, materia.assembly.AssemblyFrames.translation(0, 0, 20));
		joint.addComponent("screw", screw, materia.assembly.AssemblyFrames.compose(head,
			materia.assembly.AssemblyFrames.inverse(screw.bearingFrame())));
		joint.addDesignRequirement(machinekit.design.DesignRequirement.FastenerJoint("through-fixing", "screw",
			{member: "target", feature: "fixing"}, [{member: "clamp", feature: "fixing"}]));
		var throughAudit = machinekit.design.AssemblyDesignChecks.check(joint);
		throughAudit.throwIfErrors();
		assertions++;
		joint.replaceComponent("screw", machinekit.standard.SocketHeadCapScrew.metric("M12", 10, machinekit.standard.PropertyClass.Steel88));
		var shortAudit = machinekit.design.AssemblyDesignChecks.check(joint);
		assertions++;
		if (shortAudit.items.filter(item -> item.subject == "through-fixing thread engagement").length != 1)
			throw "Through-hole fasteners must still enforce numeric engagement limits";
		rejected = false;
		try machinekit.structural.InsertHoles.records('{"version":2,"holes":[]}')
		catch (e:Dynamic) rejected = Std.string(e).indexOf("Unsupported insert hole recipe") >= 0;
		assertions++;
		if (!rejected)
			throw "Legacy insert hole encoding must be rejected";
		var cwd = Sys.getCwd(),
			root = (sys.FileSystem.exists(cwd + "/machinekit") ? cwd + "/machinekit/tests/tooling" : cwd) + "/fixtures/foundation-design";
		var frame = MillMoldSetup.frame();
		var columnBolts = frame.components().filter(row -> StringTools.startsWith(row.id, "columnBolt"));
		assertions++;
		if (columnBolts.length != 4) throw "Mill needs its four column service screws";
		for (row in columnBolts) {
			var bolt:machinekit.standard.SocketHeadCapScrew = cast row.component;
			assertions++;
			if (bolt.length != 40 || bolt.propertyClass != machinekit.standard.PropertyClass.Steel88)
				throw "Mill column service screws must derive the 40 mm stack length and class 8.8";
		}
		var mold = materia.tooling.casting.CastingMoldPlanner.plan(MillMoldSetup.document(frame), MillMoldSetup.options(frame));
		assertions++;
		if (mold.coreUnits.length != 2 || mold.coreUnits.filter(unit -> unit.coreIds.length == 3 && unit.featureIds.length == 3).length != 2)
			throw "Retained mill rail beams must preserve two authored core units with three foam fragments each";
		var reference:Dynamic = haxe.Json.parse(sys.io.File.getContent(root + "/reference.json"));
		if (Sys.args().indexOf("--audit-openings") >= 0)
			for (source in [frame.base.body, frame.column.body])
				for (feature in source.castingFeatures())
					Sys.println(feature.id + " " + haxe.Json.stringify(feature.openings));
		var foundations:Array<{id:String, part:machinekit.structural.MineralFoundation}> = [
			{id: "mill-base", part: frame.base}, {id: "mill-column", part: frame.column},
			{id: "hobber-bed", part: new machinekit.hobbing.MineralCastBed()},
			{id: "hobber-column", part: new machinekit.hobbing.MineralCastColumn()}
		];
		var baseline:Dynamic = haxe.Json.parse(sys.io.File.getContent(root + "/../../../../examples/flexspline-hobber/BASELINE_FOUNDATIONS.json"));
		if (baseline.format != "machinekit.foundation-baseline" || baseline.version != 3)
			throw "Unsupported foundation migration baseline";
		var frozenRows:Array<Dynamic> = cast baseline.currentBeforeMigration;
		var currentRows:Array<Dynamic> = [];
		for (row in foundations) {
			var process = sys.io.Process.run("gzip", ["-dc", root + "/" + row.id + ".step.gz"]);
			var text = process.readStdout(), status = process.exitCode();
			process.close();
			if (status != 0)
				throw "Cannot decode frozen migration CAD fixture";
			var records:Array<Dynamic> = cast reference.files;
			var matches = records.filter(record -> record.id == row.id);
			assertions++;
			if (matches.length != 1 || haxe.crypto.Sha256.encode(text) != matches[0].stepSha256
				|| haxe.io.Bytes.ofString(text).length != matches[0].stepBytes)
				throw "Frozen migration CAD fixture fingerprint mismatch: " + row.id;
			var previous = new Part(cadkit.Shape.importStepText(text)),
				actual = row.part.body.geometry();
			if (row.id == "hobber-bed") {
				var corrected = socketDesignChange(previous,row.part);
				try {
					same(corrected,actual,"hobber-bed explicit socket design correction");
					var added = actual.subtract(previous), removed = previous.subtract(actual);
					try Sys.println("hobber-bed socket correction: added="+added.volume()+" removed="+removed.volume()+" mm3")
					catch (error:Dynamic) {added.close(); removed.close(); throw error;}
					added.close(); removed.close();
				} catch (error:Dynamic) {corrected.close(); previous.close(); actual.close(); throw error;}
				corrected.close();
			} else same(previous,actual,row.id);
			var sourceInput = machinekit.tooling.FoundationToolingAdapter.snapshot(row.part,row.id,1);
			var unsupported = row.part.body.finishedCavities().filter(cavity -> !CastingCavities.supportsBoxes(cavity));
			var boxSupported = row.part.body.collisionHullQualification.status == machinekit.structural.CastingCollisionQualification.CastingCollisionStatus.Supported;
			var boxes = boxSupported ? row.part.body.materialBoxes() : null;
			var half = row.id == "hobber-bed" ? machinekit.hobbing.HobberXLayout.front().footSpan/2 : row.part.body.parameterNumber("width")/2;
			currentRows.push({id:row.id,sourceFingerprint:materia.tooling.ToolingCodec.inputFingerprint(sourceInput),
				sourceCodecVersion:materia.tooling.ToolingCodec.VERSION,volumeMm3:actual.volume(),massKg:row.part.body.massProperties().mass,
				boxStatus:boxSupported ? "pass" : "unsupported",boxReason:row.part.body.collisionHullQualification.reason,unsupportedFeatures:[for (cavity in unsupported) cavity.featureId],
				minimumIyMm4:boxes == null ? null : materia.casting.BoxAlgebra.minimumIy(boxes,-half,half)});
			previous.close();
			actual.close();
			if (StringTools.startsWith(row.id, "hobber-")) {
				var kind = row.id.substr(7), frozen = frozenRows.filter(before -> before.id == kind)[0];
				var boxes:Array<Dynamic> = cast frozen.cells;
				var previousCells:Array<materia.casting.Box3> = [for (box in boxes) {
					x: materia.casting.CastingRecordCodec.number(box, "x"), y: materia.casting.CastingRecordCodec.number(box, "y"),
					z: materia.casting.CastingRecordCodec.number(box, "z"), width: materia.casting.CastingRecordCodec.number(box, "width"),
					depth: materia.casting.CastingRecordCodec.number(box, "depth"), height: materia.casting.CastingRecordCodec.number(box, "height")
				}];
				var half:Float = frozen.halfSpan;
				var historicalIy = materia.casting.BoxAlgebra.minimumIy(previousCells,-half,half), frozenIy:Float = frozen.minimumIy;
				assertions++;
				if (Math.abs(historicalIy-frozenIy) > frozenIy*1e-10) throw "Shared algebra changes historical frozen section Iy: " + row.id;
				var boxFeatures = row.part.body.castingFeatures().filter(feature -> materia.casting.CastingFeature.CastingGeometrySolids.isBox(feature.geometry.solid));
				var cells = materia.casting.BoxAlgebra.material(row.part.body.castingEnvelope(),boxFeatures);
				if (kind == "bed") previousCells = materia.casting.BoxAlgebra.coalesce(previousCells.concat(socketSeats(row.part)));
				var difference = materia.casting.BoxAlgebra.volume(materia.casting.BoxAlgebra.subtract(cells, previousCells))
					+ materia.casting.BoxAlgebra.volume(materia.casting.BoxAlgebra.subtract(previousCells, cells));
				assertions++;
				if (difference > 1e-6)
					throw "Shared algebra changes frozen hobber material: " + row.id;
				assertions++;
				if (Math.abs(materia.casting.BoxAlgebra.volume(cells) - materia.casting.BoxAlgebra.volume(previousCells)) > 1e-6)
					throw "Shared algebra changes frozen hobber volume: " + row.id;
				var expected = materia.casting.BoxAlgebra.minimumIy(previousCells,-half,half);
				var iy = materia.casting.BoxAlgebra.minimumIy(cells,-half,half);
				if (kind == "bed") {
					var message = "";
					try row.part.body.minimumSectionIy(-half,half) catch (error:Dynamic) message = Std.string(error);
					assertions++;
					if (message.indexOf("Unsupported non-box finished cavity: insert-trayClampSeat") < 0)
						throw "Final casting box-only stiffness must name its unsupported socket feature";
				}
				assertions++;
				if (Math.abs(iy - expected) > expected * 1e-10)
					throw "Shared algebra changes frozen hobber minimum section Iy: " + row.id + ": " + iy + " vs " + expected;
				var features = row.part.body.castingFeatures();
				assertions++;
				if (features.filter(feature -> feature.derivedFrom != null).length == 0)
					throw "Hobber retained supports must preserve authored parent IDs";
				if (kind == "bed") {
					assertions++;
					if (features.filter(feature -> feature.kind == InterfaceRecess && feature.owner != null
						&& StringTools.startsWith(feature.owner, "footSeat")).length != 3)
						throw "Removable foot plate registers must name their owners";
				}
				assertions++;
				if (row.part.insertLocations().filter(insert -> insert.id == "driveSeat")[0].part.parameterToken("role") != "drive-mount")
					throw "Hobber drive seats require their explicit source role";
			}
		}
		if (Sys.args().indexOf("--write-baseline-current") >= 0) {
			Reflect.setField(baseline,"currentDesign",currentRows);
			Reflect.setField(baseline,"currentDesignBasis","Exact native volumes; box-only section results are Unsupported for socket recesses, never approximated.");
			sys.io.File.saveContent(root+"/../../../../examples/flexspline-hobber/BASELINE_FOUNDATIONS.json",haxe.Json.stringify(baseline,null,"  ")+"\n");
		}
		slopedFoot();
		Sys.println("FoundationDesignTests: " + assertions + " assertions passed");
	}
}
