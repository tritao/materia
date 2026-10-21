import materia.casting.InsertSpec.InsertBodyCodec as InsertBodies;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.PosedParts;
import machinekit.component.ComponentValues;
import machinekit.component.Solids;
import machinekit.structural.CastInsert;
import materia.casting.CastingPatterns;
import machinekit.structural.MineralFoundation;
import machinekit.structural.MineralFoundation.CastInsertPlacement;
import machinekit.tooling.FoundationToolingAdapter;
import materia.assembly.AssemblyFrames as F;
import materia.tooling.ToolingCodec;
import materia.tooling.ToolingEnums.ToolingLengthUnit;
import materia.tooling.ToolingEnums.CoatedFace;
import materia.tooling.ToolingEnums.StockDirection;
import materia.casting.InsertSpec.InsertRole;
import materia.casting.InsertSpec.InsertFinish;
import materia.tooling.ToolingDocument;
import materia.tooling.casting.CastingToolingInput;
import materia.tooling.casting.CastingSetup.CoreStrategy;

/** Rebuild source CAD from neutral snapshots, without teaching ManufacturingKit about CAD. */
class FoundationToolingTests {
	static var assertions = 0;

	static function check(ok:Bool, label:String):Void {
		assertions++;
		if (!ok)
			throw label;
	}

	static function near(a:Float, b:Float, label:String):Void
		check(Math.abs(a - b) <= 1e-5, '$label: $a vs $b');

	static function rejectsCavity(fn:Void->Void, expected:String):Void {
		var message = "";
		try fn() catch (error:Dynamic) message = Std.string(error);
		check(message.indexOf(expected) >= 0, 'Expected "$expected", got "$message"');
	}

	static function curvedCastingCavities():Void {
		var feature = CastingPatterns.box("round-port", materia.casting.CastingFeature.CastingFeatureKind.ServiceOpening,
			[CastingPatterns.face("body.z-max")], 1, 1, 1, 0, 0, 80);
		feature.geometry.solid = materia.casting.CastingFeature.CastingGeometrySolid.Cut(materia.casting.CastingSolid.Cylinder(8, 40));
		var values = new ComponentValues().setNumber("width", 100).setNumber("depth", 80).setNumber("height", 100);
		var companions:Array<materia.casting.CastingFeature> = [];
		function source():machinekit.structural.MineralCasting
			return new machinekit.structural.MineralCasting(values.copy().setToken("features", CastingPatterns.encode(companions.concat([feature]))));
		var casting = source(), shape = casting.geometry();
		try near(shape.volume(), 100 * 80 * 100 - Math.PI * 4 * 4 * 20, "cylindrical cut is intersected with the body exactly")
		catch (error:Dynamic) {shape.close(); throw error;}
		shape.close();
		check(casting.collisionHullQualification.status == machinekit.structural.CastingCollisionQualification.CastingCollisionStatus.Unsupported,
			"round cavity never receives a box collision hull");
		feature.openings = [CastingPatterns.face("body.z-max"), CastingPatterns.face("body.z-min")];
		rejectsCavity(() -> {var p = source().geometry(); p.close();}, "Unreached declared opening: round-port body.z-min");
		feature.openings = [];
		rejectsCavity(() -> {var p = source().geometry(); p.close();}, "Undeclared casting breakout: round-port body.z-max");
		feature.openings = [CastingPatterns.face("body.z-max")];
		feature.geometry.pose.z = 110;
		rejectsCavity(() -> {var p = source().geometry(); p.close();}, "Casting feature does not reach body: round-port");
		feature.geometry.pose.z = 80;
		feature.openings = [CastingPatterns.feature("passage")];
		companions = [CastingPatterns.box("passage", materia.casting.CastingFeature.CastingFeatureKind.ServiceOpening,
			[CastingPatterns.face("body.z-max")], 20, 20, 60, 0, 0, 40)];
		var viaPassage = source().geometry();
		try near(viaPassage.volume(), 100 * 80 * 100 - 20 * 20 * 60, "round port may open through a declared existing cavity")
		catch (error:Dynamic) {viaPassage.close(); throw error;}
		viaPassage.close();
		companions[0].geometry.pose.x = 30;
		rejectsCavity(() -> {var p = source().geometry(); p.close();}, "Unreached declared feature opening: round-port passage");
		companions = [];

		feature.geometry.solid = materia.casting.CastingFeature.CastingGeometrySolid.Cut(materia.casting.CastingSolid.Cylinder(16, 30));
		feature.geometry.pose = F.translation(65, 0, 25);
		feature.openings = [CastingPatterns.face("foot.side-2")];
		values.setNumber("height", 120).setNumber("footWidth", 160).setNumber("footHeight", 60).setNumber("footShoulder", 20);
		var foot = source();
		var cavity = machinekit.structural.CastingCavityCad.part(foot.finishedCavities()[0]);
		// The plane x + 0.75z = 95 bisects this cylinder under (x,z)->(130-x,80-z).
		try near(cavity.volume(), Math.PI * 8 * 8 * 15, "sloped-foot cylinder agrees with analytic half-volume")
		catch (error:Dynamic) {cavity.close(); throw error;}
		cavity.close();
	}

	static function roundFoundation():Void {
		var insert = new CastInsert(new ComponentValues().setToken("role", InsertRole.ThreadedAnchor)
			.setToken("body", materia.casting.InsertSpec.InsertBodyCodec.encode(materia.casting.InsertSpec.InsertBody.Primitive(materia.casting.CastingSolid.Cylinder(8, 40)))));
		var foundation = new MineralFoundation(new ComponentValues().setNumber("width", 100).setNumber("depth", 80).setNumber("height", 100),
			[{id: "round", part: insert, pose: F.translation(0, 0, 60), openings: [CastingPatterns.face("body.z-max")]}]);
		var input = FoundationToolingAdapter.snapshot(foundation, "round-foundation", 1);
		check(switch materia.casting.InsertSpec.InsertBodyCodec.mouth(input.inserts[0].body) {case Cylinder(diameter, height): diameter == 8 && height == 40; default: false;},
			"adapter preserves a source round insert without rectangular dimensions");
		check(!machinekit.structural.FoundationIntegrity.check(foundation).hasErrors(), "round insert participates in foundation plane checks");
		var shape = foundation.body.geometry();
		try near(shape.volume(), 100 * 80 * 100 - Math.PI * 4 * 4 * 40, "foundation recess uses the exact source cylinder")
		catch (error:Dynamic) {shape.close(); throw error;}
		shape.close();
		var support = materia.tooling.casting.CastingBoxSupport.check(input, "round-box-support");
		check(support.length == 1 && support[0].status == materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported
			&& support[0].affectedIds[0] == "insert-round", "box consumers report the actual non-box source feature");
	}

	static function sourcedSocket():Void {
		var socket = machinekit.structural.FixingSockets.pfeiferStudy({outsideDiameter: 8, reliefDiameter: 6.2,
			tailWidth: 11, tailThickness: 2.2, tailWave: 1.4, tailBlend: 0.5}, 0.5);
		var type = socket.componentType();
		check(type != null && type.id == CastInsert.recipeType().id, "purchased socket uses the shared insert component family");
		check(socket.materialId == "stainless-sheet" && socket.insertMaterial().grade == materia.casting.InsertMaterial.Stainless14571,
			"socket preserves its declared stainless grade rather than becoming C45");
		var spec = socket.insertSpec(), hole = spec.holes[0];
		check(hole.extent == Blind && hole.depth == 25 && hole.threadDepth == 6 && hole.relief != null
			&& hole.relief.diameter == 6.2, "catalogue thread and study clearance bore remain distinct");
		check(hole.limits.policyId == machinekit.structural.FixingSockets.POLICY && hole.limits.threaded != null
			&& hole.limits.threaded.minEngagement == 6,
			"catalogue engagement uses its explicit product policy");
		check(switch spec.origin {case Catalogue(product): product.geometryBasis == materia.casting.InsertOrigin.InsertGeometryBasis.NominalStudy; default:false;},
			"unverified product geometry is declared in source provenance");
		var solid = socket.geometry();
		try {
			check(solid.valid() && solid.solidCount() == 1, "nominal purchased socket remains one valid steel solid");
			var cap = Part.cylinderSpan(6.2 / 2, 40 - 25 - hole.limits.minBack, 40 - 25);
			try {
				var missing = cap.subtractAll([solid]);
				try near(missing.volume(), 0, "closed socket end satisfies its declared geometric cap budget")
				catch (error:Dynamic) {missing.close(); throw error;}
				missing.close();
			} catch (error:Dynamic) {cap.close(); throw error;}
			cap.close();
		} catch (error:Dynamic) {solid.close(); throw error;}
		solid.close();
		var rebuilt:CastInsert = cast CastInsert.recipeType().create(socket.values());
		check(CastInsert.recipeType().key(rebuilt.values()) == CastInsert.recipeType().key(socket.values()),
			"socket roundtrips through the shared component recipe");
		var foundation = new MineralFoundation(new ComponentValues().setNumber("width", 100).setNumber("depth", 80).setNumber("height", 100),
			[{id: "socket", part: socket, pose: F.translation(0, 0, 60), openings: [CastingPatterns.face("body.z-max")]}]);
		var input = FoundationToolingAdapter.snapshot(foundation, "socket-source", 1);
		var qualification = materia.tooling.casting.InsertQualifications.check(input.inserts);
		check(materia.tooling.ToolingCodec.canonical(input.inserts[0].origin)
			== materia.tooling.ToolingCodec.canonical(spec.origin), "adapter preserves catalogue geometry provenance exactly");
		var geometryChecks = qualification.filter(record -> record.rule == "tooling.insert.geometry.socket");
		check(geometryChecks.length == 1 && geometryChecks[0].status == materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pending,
			"nominal product geometry survives the adapter and blocks readiness");
		var capacityChecks = qualification.filter(record -> record.rule == "tooling.insert.anchorage.socket.capacity");
		check(capacityChecks.length == 1 && capacityChecks[0].status == materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pending,
			"catalogue retention capacity remains independently pending");
		var sourceToMold = F.fromRotationMatrix(0, 0, 100, [1, 0, 0, 0, -1, 0, 0, 0, -1]);
		var selected = materia.tooling.casting.InsertFixturePlanner.select(input.inserts[0].holes[0],
			F.compose(sourceToMold, input.inserts[0].pose), F.identity(), SinglePocketSetup.document().recipe.hardware);
		check(selected != null && selected.engagement == 6 && selected.clearanceDiameter == 6.6 && selected.plug == null,
			"shared source-hole selection fixes the socket through a clearance hole without a resin-side plug");
		var document = SinglePocketSetup.document();
		document.input = input;
		document.setup.sourceToMold = sourceToMold;
		document.setup.features = [];
		document.setup.sourceFingerprint = materia.tooling.ToolingCodec.inputFingerprint(input);
		// Explicit nominal mouth land study: this is not physical seal evidence.
		document.recipe.hardware.sealMargin = 0.5;
		var plate = materia.tooling.casting.InsertFixturePlanner.plate(document, {
			id:"socket-carrier",insertIds:["socket"],contactZ:0,edgeMargin:50,locatorEndInset:40,locatorSideInset:20,
			screwStock:[{thread:cast hole.thread,lengths:[25,30,35],
				threadedLength:machinekit.standard.SocketHeadCapScrew.catalog().get(
					machinekit.structural.InsertHoles.threadSpec(cast hole.thread).designation).threadLength,washerThickness:1.6,driverDiameter:14,driverToolDiameter:6}]
		});
		check(plate.sites.length == 1 && plate.sites[0].screwLength == 30 && plate.masks.length == 0,
			"shared plate uses the catalogue source hole and no mask on its fixing screw");
		check(plate.validation.filter(row -> row.rule == "tooling.insert.geometry.socket").length == 1,
			"shared plate retains nominal supplier exterior qualification");
		var carrier = machinekit.tooling.InsertFixtureCad.assembly(plate,true);
		check(carrier.validate().length == 0,"shared insert carrier has a complete assembly reference frame");
		var copiedCarrier = machinekit.assembly.MachineAssembly.decode(carrier.encode());
		check(copiedCarrier.components().length == carrier.components().length,"shared carrier source and hardware recipes roundtrip");
		var fixtureReview = machinekit.tooling.InsertFixtureCad.check(plate);
		check(fixtureReview.removal.removalOrder.length == carrier.components().length-2
			&& fixtureReview.releaseBeforeTranslation.indexOf(plate.sites[0].id) >= 0,
			"shared carrier checks its complete tooling graph against stationary socket and casting after thread release");
		var detached = machinekit.tooling.ToolingSourceCasting.fromInput(input).geometry();
		try {
			var sourceBody = foundation.body.geometry();
			try {
				var missing = sourceBody.subtract(detached), added = detached.subtract(sourceBody);
				try near(missing.volume()+added.volume(),0,"detached neutral casting exactly reconstructs source socket cavities")
				catch (error:Dynamic) {missing.close(); added.close(); throw error;}
				missing.close(); added.close();
			} catch (error:Dynamic) {sourceBody.close(); throw error;}
			sourceBody.close();
		} catch (error:Dynamic) {detached.close(); throw error;}
		detached.close();
		var removal = machinekit.tooling.PanelKit.check(plate.assembly);
		check(removal.removalOrder.length == 2,
			"carrier plate removal order derives from exact native blocking checks");
		rejectsCavity(() -> machinekit.design.ThreadedHolePolicy.resolve(machinekit.standard.FastenerThreads.coarse(6).mating(),
			materia.casting.InsertMaterial.Stainless14571), "C45 threaded hole policy requires C45 stock");
	}

	static function insertBodyGeometry():Void {
		var box = machinekit.structural.CastingSolids.part(materia.casting.CastingSolid.Box(20, 30, 40));
		var cylinder = machinekit.structural.CastingSolids.part(materia.casting.CastingSolid.Cylinder(8, 40));
		try {
			near(box.volume(), 20 * 30 * 40, "neutral box body CAD volume");
			near(cylinder.volume(), Math.PI * 4 * 4 * 40, "neutral cylinder body CAD volume");
			var blank = Part.box(1, 1, 10);
			var corner:Part;
			try corner = blank.translated(new Vector(3.5, 3.5, 10))
			catch (error:Dynamic) {blank.close(); throw error;}
			blank.close();
			try {
				var common = new Part(cylinder.shape.common(corner.shape));
				try near(common.volume(), 0, "cylindrical insert does not fill bounding-box corners")
				catch (error:Dynamic) {common.close(); throw error;}
				common.close();
			} catch (error:Dynamic) {corner.close(); throw error;}
			corner.close();
		} catch (error:Dynamic) {box.close(); cylinder.close(); throw error;}
		box.close();
		cylinder.close();
	}

	static function relievedHoleGeometry():Void {
		var pattern = machinekit.structural.InsertHoles.tappedBlind(
			machinekit.structural.InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(6).mating()), 40);
		var holes = machinekit.structural.InsertHoles.records(pattern), hole = holes[0];
		hole.depth = 25;
		hole.threadDepth = 6;
		hole.limits.threaded.minEngagement = 6;
		hole.limits.overrides.push({name: "minEngagement", value: 6});
		hole.relief = {diameter: 6.2, start: 6};
		var insert = new CastInsert(CastInsert.boxValues(30, 30, 40).setToken("role", InsertRole.GenericMount)
			.setToken("holes", machinekit.structural.InsertHoles.fromNeutral(holes)));
		var part = insert.geometry();
		try {
			near(part.volume(), 30 * 30 * 40 - Math.PI * (2.5 * 2.5 * 6 + 3.1 * 3.1 * 19),
				"source hole CAD distinguishes thread bore from enlarged relief");
			var analytic = materia.casting.InsertHoleGeometry.massProperties(30, 30, 40, holes);
			near(analytic.volume, part.volume(), "relieved insert mass geometry agrees with CAD");
			near(analytic.center.z, part.massProperties().centerOfMass.z, "relieved insert centroid agrees with CAD");
		}
		catch (error:Dynamic) {part.close(); throw error;}
		part.close();
		var copied = insert.insertSpec().holes[0];
		check(copied.threadDepth == 6 && copied.relief != null && copied.relief.start == 6 && copied.relief.diameter == 6.2,
			"authoritative design interfaces preserve the relieved thread geometry");
		var diagnostics = new machinekit.assembly.Diagnostics();
		machinekit.design.DesignChecks.blindFastener(diagnostics, "relieved test thread", 30, 20, 6, 25, 0, 40, 6, 1, 1.5, 6);
		check(!diagnostics.hasErrors(), "screw may extend beyond short thread into a declared relief");
		var tooLittleThread = new machinekit.assembly.Diagnostics();
		machinekit.design.DesignChecks.blindFastener(tooLittleThread, "relieved test thread", 30, 20, 6, 25, 0, 40, 7, 1, 1.5, 6);
		check(tooLittleThread.hasErrors(), "clearance bore is never counted as usable thread");
		var bottomed = new machinekit.assembly.Diagnostics();
		machinekit.design.DesignChecks.blindFastener(bottomed, "relieved test thread", 45, 20, 6, 25, 0, 40, 6, 1, 1.5, 6);
		check(bottomed.hasErrors(), "relieved thread still rejects a screw at the blind bottom");
	}

	static function formedSocketGeometry():Void {
		var neutral = materia.casting.InsertSpec.InsertBody.CrimpedSocket(materia.casting.CastingSolid.Cylinder(8, 25),
			{length: 15, width: 11, thickness: 2.2, wave: 1.4, blend: 0.5});
		var exterior = machinekit.structural.InsertBodies.part(neutral);
		var cwd = Sys.getCwd();
		var folder = (sys.FileSystem.exists(cwd+"/machinekit") ? cwd+"/machinekit/tests/tooling" : cwd)+"/fixtures/socket";
		var reference:Dynamic = haxe.Json.parse(sys.io.File.getContent(folder+"/reference.json"));
		var process = sys.io.Process.run("gzip",["-dc",folder+"/nominal-crimped-exterior.step.gz"]);
		var text = process.readStdout(), status = process.exitCode();
		process.close();
		check(status == 0 && reference.version == 1 && haxe.crypto.Sha256.encode(text) == reference.stepSha256
			&& haxe.io.Bytes.ofString(text).length == reference.stepBytes,"frozen nominal exterior fingerprint");
		var moved:Part;
		try moved = new Part(cadkit.Shape.importStepText(text))
		catch (error:Dynamic) {exterior.close(); throw error;}

		try {
			var missing = moved.subtractAll([exterior]);
			try near(missing.volume(), 0, "neutral socket exterior has no missing crimp geometry")
			catch (error:Dynamic) {missing.close(); throw error;}
			missing.close();
			var excess = exterior.subtractAll([moved]);
			try near(excess.volume(), 0, "neutral socket exterior has no added bounding cylinder")
			catch (error:Dynamic) {excess.close(); throw error;}
			excess.close();
		} catch (error:Dynamic) {moved.close(); exterior.close(); throw error;}
		moved.close();
		exterior.close();
	}

	static function rebuild(input:CastingToolingInput):Part {
		var envelope:Array<Part> = [];
		for (p in input.envelope) {
			var local = Part.prism([for (v in p.profile) new Vector(v.x, v.y)], 0, p.height);
			try
				envelope.push(PosedParts.moved(local, p.pose))
			catch (e:Dynamic) {
				local.close();
				throw e;
			}
			local.close();
		}
		var body = Solids.union(envelope);
		return Solids.building([body], parts -> {
			for (cavity in materia.casting.CastingCavities.derive(input.envelope, input.features))
				parts.push(machinekit.structural.CastingCavityCad.part(cavity));
			return Solids.cut(parts[0], parts.slice(1));
		});
	}

	static function equivalent(foundation:MineralFoundation, label:String):CastingToolingInput {
		var input = FoundationToolingAdapter.snapshot(foundation, label, 1);
		// Exercise the complete public JSON boundary, including source/setup binding.
		var document:ToolingDocument = {
			input: input,
			setup: {
				id: "test-setup",
				revision: 1,
				sourceFingerprint: ToolingCodec.inputFingerprint(input),
				fillHeightMm: 100,
				freeboardMm: 0,
				sourceToMold: F.identity(),
				features: [
					for (f in input.features)
						if (f.kind != "insert-recess") {
							featureId: f.id,
							strategy: SacrificialEPS,
							priority: 0,
							withdrawal: null
						}
				]
			},
			recipe: {
				featureRules: materia.tooling.casting.CastingStrategies.standardRules(),
				id: "test-only-recipe",
				revision: 1,
				panel: {
					materialSpec: "test board",
					nominalThickness: 18,
					measuredThickness: 18,
					thicknessVariation: 0,
					width: 2440,
					height: 1220,
					coatedFace: CoatedFace.Both,
					allowedRotations: [0, 90],
					directionConstraint: StockDirection.Free
				},
				core: {
					materialSpec: "test EPS",
					width: 1000,
					depth: 500,
					height: 100,
					densityKgM3: null,
					compressionStrengthPa: null,
					compressionStrainFraction: null,
					maximumTemperatureC: null,
					evidence: []
				},
				jointLiningAllowance: 0.0,
				jointMachiningAllowance: 0.05,
				coreWireAxis: materia.tooling.ToolingRecipe.CoreWireAxis.Y,
				adhesiveLineThickness: 0.2,
				mixDensityKgM3: 2400,
				steelDensityKgM3: 7850,
				handlingLoadFactor: 1.5,
				hardware: {
					plugDepth: 5,
					plugMaterial: "steel",
					sealMargin: 3,
					sealMaterial: "pending compatible sealing compound",
					minimumBridgeWidth: 80,
					locatingPinDiameter: 6,
					supportScrewDiameter: 6,
					supportClearanceDiameter: 7,
					secondaryPinTravel: 2,
					padDepth: 54,
					labelOffset: 8,
					datumThickness: 8,
					datumPostWidth: 20,
					formerFlangeThickness: 12,
					formerFlangeMargin: 10,
					fixturePinDiameter: 3,
					fixturePinEmbed: 3,
					fixturePinSpacing: 8,
					fixtureSlotToolDiameter: 3
				},
				jointClearance: 0.3,
				linerThickness: 0.1,
				routerDiameter: 6,
				minimumLigament: 15,
				resinSpec: "unselected",
				releaseSpec: "unselected",
				adhesiveSpec: "unselected",
				processEvidence: [],
				designReview: {
					minimumCornerRadiusMm: null,
					minimumInsertCoverMm: null,
					minimumInsertEdgeMm: null,
					minimumFixingAccessMm: null,
					minimumLiftingPoints: null,
					removalClearanceMm: null
				}
			}
		};
		input = ToolingCodec.decode(ToolingCodec.encode(document)).input;
		var original = foundation.body.geometry(),
			reconstructed = rebuild(input);
		try {
			check(reconstructed.valid(), '$label rebuilt shape is valid');
			near(original.volume(), reconstructed.volume(), '$label volume');
			var missing = original.subtractAll([reconstructed]);
			try
				near(missing.volume(), 0, '$label no missing material')
			catch (e:Dynamic) {
				missing.close();
				throw e;
			}
			missing.close();
			var excess = reconstructed.subtractAll([original]);
			try
				near(excess.volume(), 0, '$label no excess material')
			catch (e:Dynamic) {
				excess.close();
				throw e;
			}
			excess.close();
		} catch (e:Dynamic) {
			original.close();
			reconstructed.close();
			throw e;
		}
		original.close();
		reconstructed.close();
		check(input.inserts.length == foundation.insertLocations().length, '$label insert count');
		check(input.features.filter(f -> f.kind == "insert-recess").length == input.inserts.length, '$label insert recesses classified separately');
		return input;
	}

	static function declareFixtureOpenings(values:ComponentValues, declarations:Map<String, Array<String>>):ComponentValues {
		var features = CastingPatterns.decode(values.token("features"));
		for (feature in features) {
			if (!declarations.exists(feature.id)) throw "Fixture needs explicit opening declarations: " + feature.id;
			feature.openings = [for (id in declarations.get(feature.id)) CastingPatterns.face(id)];
		}
		return values.copy().setToken("features", CastingPatterns.encode(features));
	}
	public static function main():Void {
		FoundationRelationshipTests.run();
		FoundationContextTests.run();
		InsertStageTests.run();
		ToolingFastenerFitTests.run();
		ReplicationContactTests.run();
		FoundationJointDesignTests.run();
		InsertAnchorTests.run();
		if (Sys.args().indexOf("socket-carrier") >= 0) {
			sourcedSocket();
			Sys.println('SocketCarrierChecks: $assertions assertions passed');
			return;
		}
		curvedCastingCavities();
		roundFoundation();
		sourcedSocket();
		insertBodyGeometry();
		relievedHoleGeometry();
		formedSocketGeometry();
		var values = new ComponentValues().setNumber("width", 240)
			.setNumber("depth", 200)
			.setNumber("height", 180)
			.setToken("features", CastingPatterns.encode([
				CastingPatterns.box("underside",materia.casting.CastingFeature.CastingFeatureKind.Lightening,[CastingPatterns.face("body.z-min")], 100, 80, 61, 0, 0, -1),
				CastingPatterns.box("side-opening",materia.casting.CastingFeature.CastingFeatureKind.ServiceOpening,[CastingPatterns.face("body.x-min")], 30, 60, 40, -110, 0, 90)
			]));
		var seat = new CastInsert(CastInsert.boxValues(40, 30, 20).setToken("role", "generic-mount")
			.setToken("holes",machinekit.structural.InsertHoles.through(8,20)));
		var specification = seat.insertSpec();
		check(switch materia.casting.InsertSpec.InsertBodyCodec.mouth(specification.body) {case Box(w, d, h): w == 40 && d == 30 && h == 20; default: false;},
			"source insert publishes its exact neutral exterior");
		check(specification.holes.length == 1 && specification.holes[0].diameter == 8,
			"source insert publishes the authoritative interface holes");
		var placements:Array<CastInsertPlacement> = [
			{id: "topSeat", part: seat, openings: [CastingPatterns.face("body.z-max")], pose: F.translation(80, 0, 170)},
			{
				id: "sideSeat",
				openings: [CastingPatterns.face("body.y-min")],
				part: seat,
				pose: F.fromRotationMatrix(0, -90, 80, [1, 0, 0, 0, 0, -1, 0, 1, 0])
			}
		];
		var foundation = new MineralFoundation(values, placements);
		var input = equivalent(foundation, "box-foundation");
		check(input.features.length == 4, "ordinary voids and insert recesses all retained");
		check(input.features.filter(f -> f.id == "underside")[0].geometry.pose.z == -1, "external cut extension retained");
		var side = input.inserts.filter(i -> i.id == "sideSeat")[0];
		check(side.holes[0].diameter == 8 && side.positionTolerance == 0.01, "bore and precision tolerance retained");
		var normal = F.transformVector(side.pose, 0, 0, 1);
		near(normal.y, -1, "side insert mounting normal retained");
		var before = ToolingCodec.inputFingerprint(input);
		input.envelope[0].profile[0].x = -999;
		input.features[0].geometry.pose.x = -999;
		input.inserts[0].pose.z = -999;
		input.inserts[0].body = materia.casting.InsertSpec.InsertBody.Primitive(materia.casting.CastingSolid.Box(999, 30, 20));
		check(ToolingCodec.inputFingerprint(FoundationToolingAdapter.snapshot(foundation, "box-foundation", 1)) == before,
			"mutating exported nested data cannot mutate source components");
		var second = FoundationToolingAdapter.snapshot(foundation, "box-foundation", 2);
		check(ToolingCodec.inputFingerprint(second) != before, "source revision participates in fingerprint");
		equivalent(new MineralFoundation(values, placements, "sideSeat"), "side-root-foundation");

		var chamfered = values.copy();
		chamfered.setNumber("cornerChamfer", 12);
		var chamferedInput = equivalent(new MineralFoundation(declareFixtureOpenings(chamfered, ["underside" => ["body.z-min"], "side-opening" => ["body.x-min"]]), placements), "chamfered-foundation");
		check(chamferedInput.envelope[0].profile.length == 8, "chamfer represented by actual profile");
		var flared = values.copy();
		flared.setNumber("footWidth", 320).setNumber("footHeight", 100).setNumber("footShoulder", 50);
		var flaredPlacements:Array<CastInsertPlacement> = [placements[0], {id: placements[1].id, part: placements[1].part, pose: placements[1].pose, openings: [CastingPatterns.face("foot.y-min")]}];
		var flaredInput = equivalent(new MineralFoundation(declareFixtureOpenings(flared, ["underside" => ["foot.z-min"], "side-opening" => ["body.x-min", "foot.side-4"]]), flaredPlacements), "flared-foundation");
		check(flaredInput.envelope.length == 2 && flaredInput.envelope[1].id == "foot", "flared foot is an additive oriented prism");
		equivalent(new MineralFoundation(declareFixtureOpenings(flared.setNumber("cornerChamfer", 12), ["underside" => ["foot.z-min"], "side-opening" => ["body.x-min", "foot.side-4"]]), flaredPlacements), "flared-chamfered-foundation");

		// Existing independent machine family exercises the same adapter unchanged.
		var hobber = FoundationToolingAdapter.snapshot(new machinekit.hobbing.MineralCastBed(), "typed-hobber", 1);
		check(hobber.inserts.filter(i -> i.id == "tablePlate")[0].role == materia.casting.InsertSpec.InsertRole.TableMount,
			"hobber table plate role explicit");
		check(hobber.inserts.filter(i -> i.id == "columnPlate").length == 0,
			"moving-column hobber has no static bed column plate");
		for (id in ["railLeft", "railRight"]) {
			var seats = hobber.inserts.filter(i -> i.id == id);
			check(seats.length == 1, "hobber bed declares column carriage guide " + id);
			check(seats[0].role == materia.casting.InsertSpec.InsertRole.GuideSeat, "column carriage guide role explicit");
		}
		var column = FoundationToolingAdapter.snapshot(new machinekit.hobbing.MineralCastColumn(), "typed-hobber-column", 1);
		var sole = column.inserts.filter(i -> i.id == "basePlate");
		check(sole.length == 1, "hobber moving column declares its carriage sole");
		check(sole[0].role == materia.casting.InsertSpec.InsertRole.ColumnMount, "hobber carriage sole role explicit");
		var missingRole = false;
		try
			new CastInsert()
		catch (error:Dynamic)
			missingRole = Std.string(error).indexOf("Missing component parameter") >= 0;
		check(missingRole, "CastInsert requires an explicit role with no guide-seat fallback");
		var requiredSchema = CastInsert.parameters().filter(v -> v.name == "role")[0];
		check(requiredSchema.defaultValue == machinekit.component.ComponentValue.Unset, "role choice has no default value");
		var rejectedRole = false;
		try
			new CastInsert(new ComponentValues().setToken("role", "unknown role"))
		catch (e:Dynamic)
			rejectedRole = true;
		check(rejectedRole, "unknown source roles fail at the typed source parameter");
		equivalent(new machinekit.hobbing.MineralCastBed(), "hobber-bed");
		equivalent(new machinekit.hobbing.MineralCastColumn(), "hobber-column");
		var finished = new CastInsert(new ComponentValues().setToken("role", "generic-mount").setToken("finish", "replicated-after-casting"));
		var finishFoundation = new MineralFoundation(values, [{id: "finished", part: finished, openings: [CastingPatterns.face("body.z-min"), CastingPatterns.face("body.y-min"), CastingPatterns.face("body.y-max")], pose: F.identity()}]);
		check(FoundationToolingAdapter.snapshot(finishFoundation, "finish-contract", 1)
			.inserts[0].finish == materia.casting.InsertSpec.InsertFinish.ReplicatedAfterCasting,
			"typed source finishing route survives adapter");
		Sys.println('FoundationToolingTests: $assertions assertions passed');
		PanelNativeTests.run();
		BoxMoldTests.run();
		ProcessNativeTests.run();
	}
}
