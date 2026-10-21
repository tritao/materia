import machinekit.component.ComponentValues;
import machinekit.hobbing.FoundationLayout;
import machinekit.structural.MineralFoundation;
import machinekit.structural.MineralFoundation.CastInsertPlacement;
import machinekit.tooling.FoundationToolingAdapter;
import materia.assembly.AssemblyFrames as F;
import materia.casting.CastingPatterns;
import materia.tooling.ToolingDocument;
import materia.tooling.ToolingCodec;
import materia.tooling.ToolingRecipe;
import materia.tooling.casting.InsertFixturePlanner;
import materia.tooling.casting.InsertFixturePlanner.InsertPlatePlan;
import materia.tooling.casting.InsertFixturePlanner.InsertPlateOptions;

/** Declared coupon/setup choices. The library derives the carrier from this source. */
class HobberSocketFixtureSetup {
	public static function document():ToolingDocument {
		var placements:Array<CastInsertPlacement> = [for (i in 0...2) {
			id:"socket"+i,part:FoundationLayout.traySocket(),pose:F.translation(i == 0 ? -80 : 80,0,30),
			openings:[CastingPatterns.face("body.z-max")]
		}];
		var foundation = new MineralFoundation(new ComponentValues().setNumber("width",400).setNumber("depth",240).setNumber("height",70),placements);
		var input = FoundationToolingAdapter.snapshot(foundation,"socket-coupon",1);
		var stock = materia.tooling.panel.PanelJoints.stock(18);
		stock.materialSpec = "melamine-faced particleboard; supplier and sealing qualification pending";
		stock.thicknessVariation = 0.1;
		return {
			input:input,
			setup:{id:"mouths-on-carrier",revision:1,sourceFingerprint:ToolingCodec.inputFingerprint(input),
				fillHeightMm:106,freeboardMm:0,sourceToMold:F.fromRotationMatrix(250,170,106,[1,0,0,0,-1,0,0,0,-1]),features:[]},
			recipe:{
				id:"socket-coupon-study",revision:1,featureRules:materia.tooling.casting.CastingStrategies.standardRules(),panel:stock,
				core:{materialSpec:"no EPS in this coupon",width:1000,depth:500,height:100,densityKgM3:null,
					compressionStrengthPa:null,compressionStrainFraction:null,maximumTemperatureC:null,evidence:[]},
				jointClearance:0.3,jointLiningAllowance:0,jointMachiningAllowance:0.05,
				hardware:{plugDepth:5,plugMaterial:"steel",sealMargin:0.5,sealMaterial:"mouth seal/release selection pending",
					minimumBridgeWidth:80,locatingPinDiameter:6,supportScrewDiameter:6,supportClearanceDiameter:7,
					secondaryPinTravel:2,padDepth:54,labelOffset:8,datumThickness:8,datumPostWidth:20,
					formerFlangeThickness:12,formerFlangeMargin:10,fixturePinDiameter:6,fixturePinEmbed:3,
					fixturePinSpacing:80,fixtureSlotToolDiameter:6},
				coreWireAxis:Y,adhesiveLineThickness:0.2,mixDensityKgM3:2400,steelDensityKgM3:7850,handlingLoadFactor:1.5,
				linerThickness:0,routerDiameter:6,minimumLigament:6,resinSpec:"binder selection pending",
				releaseSpec:"mouth seal and mold release pending",adhesiveSpec:"no EPS adhesive in coupon",processEvidence:[],
				designReview:{minimumCornerRadiusMm:null,minimumInsertCoverMm:null,minimumInsertEdgeMm:null,
					minimumFixingAccessMm:null,minimumLiftingPoints:null,removalClearanceMm:null}
			}
		};
	}

	public static function options(document:ToolingDocument, lengths:Array<Float>):InsertPlateOptions {
		var hole = document.input.inserts[0].holes[0];
		if (hole.thread == null) throw "Socket coupon requires its declared source thread";
		var thread = hole.thread;
		return {id:"socket-fixture",insertIds:[for (insert in document.input.inserts) insert.id],contactZ:36,
			edgeMargin:50,locatorEndInset:80,locatorSideInset:20,
			screwStock:[{thread:thread,lengths:lengths,
				threadedLength:machinekit.standard.SocketHeadCapScrew.catalog().get(thread.designation).threadLength,washerThickness:machinekit.standard.FlatWasher.metric(thread.designation).thickness,
				driverDiameter:14,driverToolDiameter:6}]};
	}

	public static function plan():InsertPlatePlan {
		var source = document();
		return InsertFixturePlanner.plate(source,options(source,[25,30,35]));
	}
}
