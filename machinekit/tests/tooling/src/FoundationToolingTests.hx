import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.PosedParts;
import machinekit.component.ComponentValues;
import machinekit.component.Solids;
import machinekit.structural.CastInsert;
import machinekit.structural.CastingFeatures;
import machinekit.structural.MineralFoundation;
import machinekit.structural.MineralFoundation.CastInsertPlacement;
import machinekit.tooling.FoundationToolingAdapter;
import materia.assembly.AssemblyFrames as F;
import materia.tooling.ToolingCodec;
import materia.tooling.ToolingEnums.ToolingLengthUnit;
import materia.tooling.ToolingEnums.CoatedFace;
import materia.tooling.ToolingEnums.StockDirection;
import materia.tooling.ToolingEnums.InsertRole;
import materia.tooling.ToolingEnums.InsertFinish;
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
			for (f in input.features) {
				var local = Part.box(f.width, f.depth, f.height);
				try
					parts.push(PosedParts.moved(local, f.pose))
				catch (e:Dynamic) {
					local.close();
					throw e;
				}
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
						if (f.kind == "void") {
							featureId: f.id,
							strategy: SacrificialEPS,
							draftDegrees: 0,
							priority: 0,
							withdrawal: null
						}
				]
			},
			recipe: {
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
					insertBoreDiameter: 5,
					insertClearanceDiameter: 7,
					insertScrewDiameter: 6,
					insertThreadSpec: "M6",
					insertEngagement: 8,
					plugDepth: 2,
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
				processEvidence: []
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

	public static function main():Void {
		var values = new ComponentValues().setNumber("width", 240)
			.setNumber("depth", 200)
			.setNumber("height", 180)
			.setToken("voids", CastingFeatures.encode([
				CastingFeatures.box("underside", 100, 80, 61, 0, 0, -1),
				CastingFeatures.box("side-opening", 30, 60, 40, -110, 0, 90)
			]));
		var seat = new CastInsert(new ComponentValues().setNumber("width", 40).setNumber("depth", 30).setNumber("height", 20).setNumber("boreDiameter", 8));
		var placements:Array<CastInsertPlacement> = [
			{id: "topSeat", part: seat, pose: F.translation(80, 0, 170)},
			{
				id: "sideSeat",
				part: seat,
				pose: F.fromRotationMatrix(0, -90, 80, [1, 0, 0, 0, 0, -1, 0, 1, 0])
			}
		];
		var foundation = new MineralFoundation(values, placements);
		var input = equivalent(foundation, "box-foundation");
		check(input.features.length == 4, "ordinary voids and insert recesses all retained");
		check(input.features.filter(f -> f.id == "underside")[0].pose.z == -1, "external cut extension retained");
		var side = input.inserts.filter(i -> i.id == "sideSeat")[0];
		check(side.boreDiameter == 8 && side.positionTolerance == 0.01, "bore and precision tolerance retained");
		var normal = F.transformVector(side.pose, 0, 0, 1);
		near(normal.y, -1, "side insert mounting normal retained");
		var before = ToolingCodec.inputFingerprint(input);
		input.envelope[0].profile[0].x = -999;
		input.features[0].pose.x = -999;
		input.inserts[0].pose.z = -999;
		input.inserts[0].width = 999;
		check(ToolingCodec.inputFingerprint(FoundationToolingAdapter.snapshot(foundation, "box-foundation", 1)) == before,
			"mutating exported nested data cannot mutate source components");
		var second = FoundationToolingAdapter.snapshot(foundation, "box-foundation", 2);
		check(ToolingCodec.inputFingerprint(second) != before, "source revision participates in fingerprint");
		equivalent(new MineralFoundation(values, placements, "sideSeat"), "side-root-foundation");

		var chamfered = values.copy();
		chamfered.setNumber("cornerChamfer", 12);
		var chamferedInput = equivalent(new MineralFoundation(chamfered, placements), "chamfered-foundation");
		check(chamferedInput.envelope[0].profile.length == 8, "chamfer represented by actual profile");
		var flared = values.copy();
		flared.setNumber("footWidth", 320).setNumber("footHeight", 100).setNumber("footShoulder", 50);
		var flaredInput = equivalent(new MineralFoundation(flared, placements), "flared-foundation");
		check(flaredInput.envelope.length == 2 && flaredInput.envelope[1].id == "foot", "flared foot is an additive oriented prism");
		equivalent(new MineralFoundation(flared.setNumber("cornerChamfer", 12), placements), "flared-chamfered-foundation");

		// Existing independent machine family exercises the same adapter unchanged.
		var hobber = FoundationToolingAdapter.snapshot(new machinekit.hobbing.MineralCastBed(), "typed-hobber", 1);
		check(hobber.inserts.filter(i -> i.id == "tablePlate")[0].role == materia.tooling.ToolingEnums.InsertRole.TableMount,
			"hobber table plate role explicit");
		check(hobber.inserts.filter(i -> i.id == "columnPlate")[0].role == materia.tooling.ToolingEnums.InsertRole.ColumnMount,
			"hobber column plate role explicit");
		var rejectedRole = false;
		try
			new CastInsert(new ComponentValues().setToken("role", "unknown role"))
		catch (e:Dynamic)
			rejectedRole = true;
		check(rejectedRole, "unknown source roles fail at the typed source parameter");
		equivalent(new machinekit.hobbing.MineralCastBed(), "hobber-bed");
		equivalent(new machinekit.hobbing.MineralCastColumn(), "hobber-column");
		var finished = new CastInsert(new ComponentValues().setToken("finish", "replicated after casting"));
		var finishFoundation = new MineralFoundation(values, [{id: "finished", part: finished, pose: F.identity()}]);
		check(FoundationToolingAdapter.snapshot(finishFoundation, "finish-contract", 1)
			.inserts[0].finish == materia.tooling.ToolingEnums.InsertFinish.ReplicatedAfterCasting,
			"typed source finishing route survives adapter");
		Sys.println('FoundationToolingTests: $assertions assertions passed');
		PanelNativeTests.run();
		BoxMoldTests.run();
		ProcessNativeTests.run();
	}
}
