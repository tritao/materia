import materia.tooling.ToolingRecipe;
import materia.tooling.panel.PanelJoints;
import materia.tooling.export.FabricationSpec;
import camkit.CamToolingPanels.PanelCamSettings;
import toolpathkit.path.Point3;
import toolpathkit.setup.TravelEnvelope;
import toolpathkit.tool.Tool;
import materia.tooling.panel.PanelPlan;

/** Shared workshop declaration. Values remain study inputs until qualified.
 * Casting-specific geometry, allocation priorities and orientations belong to setups.
 * Every call returns fresh records, so an explicit setup override cannot mutate another casting.
 */
class CastingShopRecipe {
 public static inline var BOARD_THICKNESS_MM = 18;
 public static function featureRules():Array<materia.tooling.casting.CastingStrategies.CastingStrategyRule> {
  var rules = materia.tooling.casting.CastingStrategies.standardRules();
  for (rule in rules) rule.useEPSAboveBoardThickness = true;
  return rules;
 }
 public static function tooling(id:String, thickness:Float = BOARD_THICKNESS_MM):ToolingRecipe {
		var stock = PanelJoints.stock(thickness);
		stock.materialSpec = "coated particleboard";
		var formerStocks=[stock];
		if (stock.measuredThickness<25) {
			var thicker=materia.tooling.ToolingCodec.stockSnapshot(stock);
			thicker.nominalThickness=25;thicker.measuredThickness=25;
			formerStocks.push(thicker);
		}
		return {
				former:{pocketScrew:{diameterMm:4,lengthMm:30,jigAngleDegrees:15,edgeDistanceMm:20,maximumPitchMm:100},stocks:formerStocks,finishingStockMm:0.5,toleranceMm:0.1,cutters:[
					{id:"former-straight-6",tipDiameterMm:6,taperDegrees:0,fluteLengthMm:30,stickOutMm:35},
					{id:"former-taper-3deg-6",tipDiameterMm:6,taperDegrees:3,fluteLengthMm:30,stickOutMm:35}
				]},
				featureRules: featureRules(),
				id: id,
				revision: 1,
				panel: stock,
				core: {
					materialSpec: "EPS supplier grade pending",
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
					fixtureReceiver:materia.tooling.ToolingRecipe.FixtureReceiver.IndependentDatums,
					fixturePinSpacing: 8,
					fixtureSlotToolDiameter: 3
				},
				jointClearance: 0.2,
				linerThickness: 0.1,
				routerDiameter: 6,
				minimumLigament: 2,
				resinSpec: "pending binder",
				releaseSystem: materia.tooling.ReleaseSystem.ReleaseSystems.unresolved(2,5,1,materia.tooling.ReleaseSystem.EPSCoveringKind.Tape),
				adhesiveSpec: "pending EPS compatible adhesive",
				processEvidence: [],
				designReview: {
					minimumCornerRadiusMm: null,
					minimumInsertCoverMm: null,
					minimumInsertEdgeMm: null,
					minimumFixingAccessMm: null,
					minimumLiftingPoints: 4,
					removalClearanceMm: null
				}
			};
 }
 public static function compaction():materia.tooling.casting.CastingMoldPlan.CompactionSpec
  return {
				method: Undeclared,
				bedInterfaceReference: null,
				frequencyHz: null,
				amplitudeMm: null,
				durationSeconds: null,
				evidence: []
			};
	public static function fabrication():FabricationSpec
		return {
			revision: 1,
			sheetMaterialId: "tooling-coated-board",
			solvent: Unknown,
			adhesiveContainsEPS: null,
			resinReference: "pending binder",
			compatibilityEvidence: [],
			gravityMps2: 9.80665,
			loadFactor: 1.5,
			mixBearingStressMPa:null,mixTensileStrengthMPa:null,
			compactionPressurePa: null,
			compactionEvidence: [],
			plates: [],
			braces: [],
			fasteners: [],
			coreRestraints: [],
			rigidRestraints: [],
			blankMarginMm: 12,
			blankKerfMm: 3.2,
			couponClearancesMm: [0.1, 0.2, 0.3]
		};

	/** Production compound and workshop equipment are unresolved inputs. */
	public static function process(recipe:materia.tooling.ToolingRecipe, id:String):materia.tooling.process.CastingProcessSpec return {
		id:id,revision:1,recipeFingerprint:materia.tooling.ToolingCodec.recipeFingerprint(recipe),
		pourRateMm3PerSecond:null,maximumAirEscapeWidthMm:null,mixComposition:null,mixer:null,
		castability:{maximumAggregateSizeMm:null,minimumPassageAggregateRatio:null,evidence:[]},
		compound:{reference:materia.casting.CastingInput.CastingTextInput.Unresolved("Production compound and datasheet"),
			thickness:materia.tooling.process.CastingProcessSpec.CompoundThicknessLimit.Unresolved("Compound section or layer thickness limit"),
			maximumMassPerLiftKg:materia.casting.CastingInput.Unresolved("Compound maximum mass per lift")}
	};

	public static function cam(panels:Array<PanelPlan>):PanelCamSettings {
		if (panels.length == 0)
			throw "Panel machining requires panels";
		var diameters:Array<Float> = [];
		for (p in panels)
			for (op in p.operations)
				if (diameters.indexOf(op.toolDiameter) < 0)
					diameters.push(op.toolDiameter);
		diameters.sort((a, b) -> Reflect.compare(a, b));
		return {
			chordTolerance: 0.00002,
			arcTolerance: 0.00002,
			blendTolerance: 0.00002,
			cutDirection: Climb,
			spindleSpinUpSeconds: 2,
			diameterMatchTolerance: 0.00005,
			toolMeasurementReference: null,
			singleToolPrograms: true,
			spoilboardPlane: -0.018,
			breakthroughAllowance: 0.001,
			vacuumHold: null,
			onionSkinDepth: 0.0005,
			approachClearance: 0.0005,
			drillPeckRatio: 3,
			drillPeckDepth: 0.003,
			travelSpace: MachineCoordinates,
			workOriginReference: null,
			nominalDiameters: [
				for (i in 0...diameters.length)
					{
						number: i + 1,
						diameter: diameters[i] * 0.001
					}
			],
			safeZ: 0.012,
			spindleRpm: 12000,
			feed: 0.01,
			plungeFeed: 0.002,
			stepDown: 0.003,
			stepOverFraction: 0.4,
			blankMargin: 0.012,
			tools: [
				for (i in 0...diameters.length) new Tool(i + 1, 0, diameters[i] * 0.001, null, 0.025, 0.03, diameters[i] * 0.001)
			],
			fixtures: [],
			travel: new TravelEnvelope(new Point3(-0.1, -0.1, -0.1), new Point3(3, 3, 0.3)),
			workOrigin: new Point3(0, 0, 0),
			workholdingReference: null
		};
	}
}
