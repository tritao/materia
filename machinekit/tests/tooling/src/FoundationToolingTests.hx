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
			for (c in materia.casting.CastingCavities.derive(input.envelope,input.features))for(p in c.pieces) {
				var local=Part.prism([for(v in p.profile)new Vector(v.x,v.y)],0,p.height);
				try parts.push(PosedParts.moved(local,p.pose)) catch(e:Dynamic){local.close();throw e;}
				local.close();
			}
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

    static function declareFixtureOpenings(values:ComponentValues):ComponentValues {
        values=machinekit.component.ComponentRecipeSupport.resolve(machinekit.structural.MineralCasting.parameters(),values);
        var envelope=materia.casting.CastingEnvelope.build(values.number("width"),values.number("depth"),values.number("height"),values.number("cornerChamfer"),values.number("footWidth"),values.number("footHeight"),values.number("footShoulder"));
        var features=CastingPatterns.decode(values.token("features"));
        for(feature in features)feature.openings=materia.casting.CastingCavities.reachedOpenings(envelope,feature.geometry);
        return values.copy().setToken("features",CastingPatterns.encode(features));
    }
	public static function main():Void {
		var values = new ComponentValues().setNumber("width", 240)
			.setNumber("depth", 200)
			.setNumber("height", 180)
			.setToken("features", CastingPatterns.encode([
				CastingPatterns.box("underside",materia.casting.CastingFeature.CastingFeatureKind.Lightening,[CastingPatterns.face("body.z-min")], 100, 80, 61, 0, 0, -1),
				CastingPatterns.box("side-opening",materia.casting.CastingFeature.CastingFeatureKind.ServiceOpening,[CastingPatterns.face("body.x-min")], 30, 60, 40, -110, 0, 90)
			]));
		var seat = new CastInsert(new ComponentValues().setToken("role", "generic-mount")
			.setNumber("width", 40)
			.setNumber("depth", 30)
			.setNumber("height", 20)
			.setToken("holes",machinekit.structural.InsertHoles.through(8,20)));
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
		input.inserts[0].width = 999;
		check(ToolingCodec.inputFingerprint(FoundationToolingAdapter.snapshot(foundation, "box-foundation", 1)) == before,
			"mutating exported nested data cannot mutate source components");
		var second = FoundationToolingAdapter.snapshot(foundation, "box-foundation", 2);
		check(ToolingCodec.inputFingerprint(second) != before, "source revision participates in fingerprint");
		equivalent(new MineralFoundation(values, placements, "sideSeat"), "side-root-foundation");

		var chamfered = values.copy();
		chamfered.setNumber("cornerChamfer", 12);
		var chamferedInput = equivalent(new MineralFoundation(declareFixtureOpenings(chamfered), placements), "chamfered-foundation");
		check(chamferedInput.envelope[0].profile.length == 8, "chamfer represented by actual profile");
		var flared = values.copy();
		flared.setNumber("footWidth", 320).setNumber("footHeight", 100).setNumber("footShoulder", 50);
		var flaredPlacements:Array<CastInsertPlacement> = [placements[0], {id: placements[1].id, part: placements[1].part, pose: placements[1].pose, openings: [CastingPatterns.face("foot.y-min")]}];
		var flaredInput = equivalent(new MineralFoundation(declareFixtureOpenings(flared), flaredPlacements), "flared-foundation");
		check(flaredInput.envelope.length == 2 && flaredInput.envelope[1].id == "foot", "flared foot is an additive oriented prism");
		equivalent(new MineralFoundation(declareFixtureOpenings(flared.setNumber("cornerChamfer", 12)), flaredPlacements), "flared-chamfered-foundation");

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
