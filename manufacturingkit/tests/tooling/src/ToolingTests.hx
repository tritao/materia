import materia.casting.CastingFeature.CastingDimension;
import haxe.Json;
import materia.assembly.AssemblyFrames as F;
import materia.tooling.ToolingCodec;
import materia.tooling.ToolingEnums.ToolingLengthUnit;
import materia.tooling.ToolingEnums.CoatedFace;
import materia.tooling.ToolingEnums.StockDirection;
import materia.casting.InsertSpec.InsertRole;
import materia.casting.InsertSpec.InsertFinish;
import materia.tooling.ToolingDocument;
import materia.tooling.ToolingRecipe;
import materia.tooling.ToolingReview;
import materia.tooling.casting.CastingToolingInput;
import materia.casting.CastingFeature.CastingFeatureKind;
import materia.tooling.casting.CastingSetup.CoreStrategy;

/** Pure records/codec tests: this target has no CAD or MachineKit dependency. */
class ToolingTests {
	static var assertions = 0;

	static function items(value:Dynamic):Array<Dynamic>
		return cast value;

	static function check(ok:Bool, label:String):Void {
		assertions++;
		if (!ok)
			throw label;
	}

	static function rejects(fn:Void->Void, expected:String):Void {
		var message:Null<String> = null;
		try
			fn()
		catch (e:Dynamic)
			message = Std.string(e);
		check(message != null && message.indexOf(expected) >= 0, 'Expected "$expected", got "$message"');
	}

	static function fixture():ToolingDocument {
		var input:CastingToolingInput = {
			id: "sample-foundation",
			revision: 1,
			lengthUnit: ToolingLengthUnit.Millimetres,
			envelope: [
				{
					id: "body",
					profile: [
						{
							x: -100,
							y: -90
						},
						{
							x: 100,
							y: -90
						},
						{
							x: 100,
							y: 90
						},
						{
							x: -100,
							y: 90
						}
					],
					height: 100,
					pose: F.identity()
				}
			],
			features: [
				{id:"pocket",kind:materia.casting.CastingFeature.CastingFeatureKind.Lightening,owner:null,geometry:{kind:materia.casting.CastingFeature.CastingGeometryKind.Box,width:100,depth:80,height:61,pose:F.translation(0, 0, -1)},openings:[materia.casting.CastingPatterns.face("body.z-min")],draft:{degrees:0,axis:{x:0,y:0,z:1}},cornerRadius:0,adjustable:[CastingDimension.Width, CastingDimension.Depth, CastingDimension.Height],derivedFrom:null},
				{id:"insert-seat",kind:InsertRecess,owner:"seat",geometry:{kind:materia.casting.CastingFeature.CastingGeometryKind.Box,width:40,depth:30,height:20,pose:F.translation(80, 0, 90)},openings:[materia.casting.CastingPatterns.face("body.z-max"),materia.casting.CastingPatterns.face("body.x-max")],draft:{degrees:0,axis:{x:0,y:0,z:1}},cornerRadius:0,adjustable:[CastingDimension.Width, CastingDimension.Depth, CastingDimension.Height],derivedFrom:null}
			],
			inserts: [
				{
					id: "seat",
					width: 40,
					depth: 30,
					height: 20,
					holes: TestInsertHoles.through(8,20),
					pose: F.translation(80, 0, 90),
					role: InsertRole.GuideSeat,
					label: "rail seat",
					positionTolerance: 0.01,
					material: materia.casting.InsertMaterial.InsertMaterials.specification(C45Bright),
					finish: InsertFinish.GroundAfterCasting
				}
			]
		};
		var recipe:ToolingRecipe = {
			featureRules: materia.tooling.casting.CastingStrategies.standardRules(),
			id: "eps-panel-study",
			revision: 1,
			panel: {
				materialSpec: "melamine-faced particleboard, study",
				nominalThickness: 18,
				measuredThickness: 17.8,
				thicknessVariation: 0.2,
				width: 2440,
				height: 1220,
				coatedFace: CoatedFace.Both,
				allowedRotations: [0, 90],
				directionConstraint: StockDirection.Free
			},
			core: {
				materialSpec: "EPS grade unselected",
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
				minimumBridgeWidth: 40,
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
			resinSpec: "binder unselected",
			releaseSpec: "covering unselected",
			adhesiveSpec: "adhesive unselected",
			processEvidence: [],
			designReview: {
				minimumCornerRadiusMm: null,
				minimumInsertCoverMm: null,
				minimumInsertEdgeMm: null,
				minimumFixingAccessMm: null,
				minimumLiftingPoints: null,
				removalClearanceMm: null
			}
		};
		return {
			input: input,
			recipe: recipe,
			setup: {
				id: "upside-down",
				revision: 1,
				sourceFingerprint: ToolingCodec.inputFingerprint(input),
				fillHeightMm: 100,
				freeboardMm: 0,
				sourceToMold: F.fromRotationMatrix(0, 0, 100, [1, 0, 0, 0, -1, 0, 0, 0, -1]),
				features: [
					{
						featureId: "pocket",
						strategy: SacrificialEPS,
						priority: 0,
						withdrawal: null
					}
				]
			}
		};
	}

	static function withdrawalTie():Void {
		var d = fixture();
		d.input.features = d.input.features.filter(f -> f.kind == materia.casting.CastingFeature.CastingFeatureKind.InsertRecess);
		d.input.features.push(materia.casting.CastingPatterns.box("through-opening", ServiceOpening,
			[materia.casting.CastingPatterns.face("body.z-min"), materia.casting.CastingPatterns.face("body.z-max")], 20, 20, 100));
		d.setup.features = [];
		d.setup.sourceFingerprint = ToolingCodec.inputFingerprint(d.input);
		rejects(() -> ToolingCodec.snapshot(d), "Ambiguous former withdrawal");
		d.setup.features = [{featureId: "through-opening", strategy: ReusableFormer, priority: 5, withdrawal: {x: 0, y: 0, z: 1}}];
		var chosen = materia.tooling.casting.CastingStrategies.resolve(ToolingCodec.snapshot(d));
		var direction:{x:Float, y:Float, z:Float} = cast chosen[0].withdrawal;
		check(chosen.length == 1 && direction.z == 1, "an explicit source feature override resolves withdrawal ambiguity");
		check(materia.tooling.casting.CastingStrategies.standardRules().filter(rule -> rule.priority != 0).length == 0,
			"generic strategy rules contain no machine-specific ownership priorities");
	}

	public static function main():Void {
		withdrawalTie();
		var original = fixture(),
			encoded = ToolingCodec.encode(original),
			decoded = ToolingCodec.decode(encoded);
		check(ToolingCodec.encode(decoded) == encoded, "canonical roundtrip");
		var stock = fixture();
		stock.input.inserts[0].material = materia.casting.InsertMaterial.InsertMaterials.specification(C45Ground);
		check(ToolingCodec.inputFingerprint(stock.input) != ToolingCodec.inputFingerprint(original.input),
			"insert stock condition enters the source fingerprint");
		var boreRules = decoded.input.inserts[0].holes[0].limits;
		check(boreRules.threaded == null && Reflect.fields(boreRules).indexOf("minEngagement") < 0,
			"plain bores carry no placeholder engagement or tip-gap rules");
		check(decoded.input.features[0].id == "insert-seat", "features canonicalized by ID");
		check(original.input.features[0].id == "pocket", "canonicalization does not reorder caller arrays");
		check(decoded.input.inserts[0].holes[0].diameter == 8 && decoded.input.inserts[0].positionTolerance == 0.01,
			"insert manufacturing interfaces survive roundtrip");
		var top = F.transformPoint(decoded.setup.sourceToMold, 0, 0, 100);
		check(Math.abs(top.z) < 1e-9, "setup maps rail-side source top to bed");
		check(decoded.input.features[1].geometry.pose.z == -1 && decoded.input.features[1].geometry.height == 61,
			"design-cut extensions preserved, not misrepresented as clipped cavity sizes");
		decoded.input.envelope[0].profile[0].x = -999;
		decoded.input.inserts[0].pose.x = -999;
		decoded.recipe.core.evidence.push("changed");
		check(original.input.envelope[0].profile[0].x == -100
			&& original.input.inserts[0].pose.x == 80
			&& original.recipe.core.evidence.length == 0,
			"nested snapshots are detached");
		var ordered = fixture(),
			fingerprint = ToolingCodec.documentFingerprint(ordered);
		ordered.input.features.reverse();
		check(ToolingCodec.documentFingerprint(ordered) == fingerprint, "named collection order does not change fingerprint");
		var before = ToolingCodec.recipeFingerprint(ordered.recipe);
		var defaults = fixture();
		defaults.setup.features = [];
		var rules = materia.tooling.casting.CastingStrategies.resolve(ToolingCodec.snapshot(defaults));
		check(rules.length == 1 && rules[0].strategy == SacrificialEPS,
			"recipe rules select lightening cores without feature-ID tables");
		defaults.input.features.filter(feature -> feature.id == "pocket")[0].kind = ServiceOpening;
		defaults.setup.sourceFingerprint = ToolingCodec.inputFingerprint(defaults.input);
		rules = materia.tooling.casting.CastingStrategies.resolve(ToolingCodec.snapshot(defaults));
		check(rules[0].strategy == ReusableFormer && rules[0].withdrawal != null && rules[0].withdrawal.z == 1,
			"body opening and mold orientation derive a default former withdrawal");
		ordered.recipe.panel.measuredThickness = 17.6;
		check(ToolingCodec.recipeFingerprint(ordered.recipe) != before, "measured stock changes recipe fingerprint");
		ordered.input.features[1].geometry.height += 1;
		rejects(() -> ToolingCodec.encode(ordered), "Stale tooling setup");
		ordered.setup.sourceFingerprint = ToolingCodec.inputFingerprint(ordered.input);
		check(ToolingCodec.encode(ordered).length > 0, "explicitly rebind setup to updated source");
		var antipodal = fixture();
		antipodal.input.features[1].geometry.pose.qw = -1;
		antipodal.setup.sourceFingerprint = ToolingCodec.inputFingerprint(antipodal.input);
		check(ToolingCodec.encode(antipodal).length > 0, "antipodal quaternions describe the same insert pose");

		function corrupt(edit:Dynamic->Void, expected:String):Void {
			var raw:Dynamic = Json.parse(encoded);
			edit(raw);
			rejects(() -> ToolingCodec.decode(Json.stringify(raw)), expected);
		}
		corrupt(r -> {
			r.version = 7;
		}, "Unsupported tooling format");
		corrupt(r -> {
			r.version = 1.5;
		}, "Expected tooling integer");
		corrupt(r -> {
			r.document.input.lengthUnit = "inch";
		}, "explicit mm");
		corrupt(r -> {
			r.document.input.revision = 0;
		}, "revision must be positive");
		corrupt(r -> {
			items(r.document.input.envelope)[0].height = "100";
		}, "finite tooling number");
		corrupt(r -> {
			items(r.document.input.envelope)[0].height = true;
		}, "finite tooling number");
		corrupt(r -> {
			items(r.document.input.envelope)[0].height = 0;
		}, "positive tooling dimension");
		corrupt(r -> {
			items(r.document.input.envelope)[0].pose.qw = 2;
		}, "frame");
		corrupt(r -> {
			items(items(r.document.input.envelope)[0].profile).reverse();
		}, "convex and CCW");
		corrupt(r -> {
			items(items(r.document.input.envelope)[0].profile)[1] = {x: 0, y: 0};
		}, "convex and CCW");
		corrupt(r -> {
			items(items(r.document.input.envelope)[0].profile)[1] = items(items(r.document.input.envelope)[0].profile)[0];
		}, "convex and CCW");
		corrupt(r -> {
			items(r.document.input.features).push(items(r.document.input.features)[0]);
		}, "Duplicate casting feature");
		corrupt(r -> {
			items(r.document.input.features)[0].owner = "missing";
		}, "Invalid insert reference");
		corrupt(r -> {
			items(r.document.input.features)[0].geometry.pose.x += 1;
		}, "does not match insert");
		corrupt(r -> {
			items(r.document.input.features)[0].geometry.pose.qx = Math.sin(0.00005);
			items(r.document.input.features)[0].geometry.pose.qw = Math.cos(0.00005);
		}, "does not match insert");
		corrupt(r -> {
			items(r.document.input.features)[1].kind = "hole";
		}, "Unknown casting feature kind");
		corrupt(r -> {
			items(r.document.input.features)[1].owner = 17;
		}, "Invalid casting text");
		corrupt(r -> {
			items(r.document.input.features).splice(0, 1);
		}, "Missing recess");
		corrupt(r -> {
			items(items(r.document.input.inserts)[0].holes)[0].diameter = 40;
			items(items(r.document.input.inserts)[0].holes)[0].clearanceDiameter = 40;
		}, "bore consumes");
		corrupt(r -> {
			items(r.document.input.inserts)[0].role = "unrecognized";
		}, "Invalid insert role");
		corrupt(r -> {
			items(r.document.input.inserts)[0].finish = "ground after casting";
		}, "Invalid insert finish");
		corrupt(r -> {
			r.document.input.envelope = new Array<Dynamic>();
		}, "needs an envelope");
		corrupt(r -> {
			r.document.input.envelope = [for (_ in 0...513) items(r.document.input.envelope)[0]];
		}, "Too many tooling records");
		corrupt(r -> {
			r.document.setup.features = new Array<Dynamic>();
			r.document.recipe.featureRules = new Array<Dynamic>();
		}, "Missing or duplicate recipe strategy rule");
		corrupt(r -> {
			items(r.document.setup.features)[0].featureId = "insert-seat";
		}, "insert-recess strategy");
		corrupt(r -> {
			items(r.document.setup.features).push(items(r.document.setup.features)[0]);
		}, "duplicate or insert-recess strategy");
		corrupt(r -> {
			items(r.document.setup.features)[0].strategy = "reusable-former";
		}, "needs a withdrawal");
		corrupt(r -> {
			items(r.document.setup.features)[0].withdrawal = {x: 0, y: 0, z: 2};
		}, "unit length");
		corrupt(r -> {
			items(r.document.setup.features)[0].strategy = "guess";
		}, "Unsupported core strategy token");
		corrupt(r -> {
			r.document.recipe.panel.thicknessVariation = 18;
		}, "variation consumes");
		corrupt(r -> {
			r.document.recipe.panel.coatedFace = "up";
		}, "Invalid coated face");
		corrupt(r -> {
			r.document.recipe.panel.allowedRotations = [0, 180];
		}, "Invalid or duplicate panel stock rotation");
		corrupt(r -> {
			r.document.recipe.panel.allowedRotations = [0, 0];
		}, "Invalid or duplicate panel stock rotation");
		corrupt(r -> {
			r.document.recipe.panel.directionConstraint = "width-aligned";
		}, "Invalid panel direction constraint");
		corrupt(r -> {
			r.document.recipe.core.compressionStrengthPa = 100000;
		}, "requires a strain fraction");
		corrupt(r -> {
			r.document.recipe.core.densityKgM3 = -1;
		}, "positive tooling dimension");
		corrupt(r -> {
			r.document.recipe.routerDiameter = 0;
		}, "positive tooling dimension");
		corrupt(r -> {
			r.document.recipe.processEvidence = ["trial", "trial"];
		}, "Duplicate evidence");
		corrupt(r -> {
			Reflect.deleteField(r.document.recipe, "linerThickness");
		}, "Missing recipe field");
		corrupt(r -> {
			r.document.recipe.linerThicknes = 1;
		}, "Unknown recipe field");
		rejects(() -> ToolingCodec.decode("[]"), "Expected tooling object");
		rejects(() -> ToolingCodec.decode("{"), "");
		rejects(() -> ToolingCodec.decode(StringTools.lpad("", "x", ToolingCodec.MAX_TEXT_BYTES + 1)), "exceeds size limit");
		rejects(() -> ToolingCodec.decode(StringTools.lpad("", "é", Std.int(ToolingCodec.MAX_TEXT_BYTES / 2) + 1)), "exceeds size limit");
		var invalid = fixture();
		invalid.input.envelope[0].height = Math.NaN;
		rejects(() -> ToolingCodec.encode(invalid), "finite tooling number");
		invalid.input.envelope[0].height = Math.POSITIVE_INFINITY;
		rejects(() -> ToolingCodec.inputFingerprint(invalid.input), "finite tooling number");

		var review = ToolingReview.inspect(fixture());
		check(review.filter(d -> d.status == "pending").length == 4, "unknown process, geometry, loads and stock remain pending");
		var unsupported = fixture();
		unsupported.setup.features[0].strategy = Unsupported;
		check(ToolingReview.inspect(unsupported).filter(d -> d.status == "unsupported").length == 1,
			"explicit unsupported strategy survives serialization and review");
		var reusable = fixture();
		reusable.setup.features[0].strategy = ReusableFormer;
		reusable.setup.features[0].withdrawal = {x: 0, y: 0, z: 1};
		var restoredDirection = ToolingCodec.decode(ToolingCodec.encode(reusable)).setup.features[0].withdrawal;
		check(restoredDirection != null && restoredDirection.z == 1, "explicit reusable former direction");
		reusable.recipe.core.compressionStrengthPa = 100000;
		reusable.recipe.core.compressionStrainFraction = 0.1;
		reusable.recipe.core.densityKgM3 = 25;
		reusable.recipe.core.maximumTemperatureC = 60;
		reusable.recipe.core.evidence = ["test-only supplier reference"];
		reusable.recipe.processEvidence = ["test-only trial reference"];
		check(ToolingReview.inspect(reusable).filter(d -> d.ruleId == "tooling.process" && d.status == "pending").length == 1,
			"evidence references alone never qualify process");
		Sys.println('ToolingTests: $assertions assertions passed');
		PanelTests.run();
		BoxPlannerTests.run(fixture());
		CastingMoldTests.run(fixture());
	}
}
