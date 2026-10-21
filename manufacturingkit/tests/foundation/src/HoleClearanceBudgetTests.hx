import materia.casting.HoleClearanceBudget;
import materia.casting.HoleClearanceBudget.HoleClearanceBudgets;
import materia.casting.InsertManufacturing.InsertHoleRoute;
import materia.casting.InsertMountHoleRequirement.InsertMountHoleRequirements;
import materia.casting.InsertHole;
import materia.casting.InsertHole.HoleExtent;
import materia.casting.InsertHole.InsertThreadHand;
import materia.casting.InsertHole.InsertThreadProfile;
import materia.assembly.AssemblyFrames as F;

class HoleClearanceBudgetTests {
	static var assertions = 0;
	static function check(ok:Bool, message:String):Void {
		assertions++;
		if (!ok) throw message;
	}

	static function budget():HoleClearanceBudget
		return {boltDiameterMm: 4, clearanceDiameterMm: 4.5, distanceFromDatumMm: 200,
			placementOffsetMm: 0.02, placementAngleRad: 0, assemblyAdjustmentMm: 0.02,
			cureBowMm: 0.03, machiningErrorMm: 0.03, pitchAccumulationMm: 0.01, requiredMarginMm: 0.02};

	public static function run():Void {
		var both = [InsertHoleRoute.BeforeCasting, InsertHoleRoute.TransferAfterCasting];
		var source = budget();
		check(switch HoleClearanceBudgets.choose(source, both) {
			case UsePreCast(margin): Math.abs(margin - 0.12) < 1e-12;
			default: false;
		}, "closed worst-case budget selects pre-cast drilling");
		source.placementAngleRad = 0.001;
		check(switch HoleClearanceBudgets.choose(source, both) {
			case UseTransfer(margin): Math.abs(margin - 0.2) < 1e-12;
			default: false;
		}, "rotation over the real lever arm requires transfer drilling");
		check(switch HoleClearanceBudgets.choose(source, [InsertHoleRoute.BeforeCasting]) {
			case ChangeDesign(_): true;
			default: false;
		}, "a forbidden transfer route cannot silently rescue pre-cast drilling");
		source = budget();
		source.placementOffsetMm = null;
		check(switch HoleClearanceBudgets.choose(source, both) {
			case UseTransfer(_): true;
			default: false;
		}, "transfer eliminates unresolved insert placement error");
		check(switch HoleClearanceBudgets.choose(source, [InsertHoleRoute.BeforeCasting]) {
			case AwaitInputs(missing): missing.indexOf("placementOffsetMm") >= 0;
			default: false;
		}, "unknown pre-cast error stays pending when transfer is forbidden");
		source.machiningErrorMm = null;
		check(switch HoleClearanceBudgets.choose(source, both) {
			case ProposeTransfer(missing): missing.join(",") == "machiningErrorMm";
			default: false;
		}, "proposed transfer still requires its guide and machining budget");
		source.machiningErrorMm = 0.3;
		check(switch HoleClearanceBudgets.choose(source, both) {
			case ChangeDesign(_): true;
			default: false;
		}, "inadequate machining accuracy fails both routes even with unknown placement");
		check(switch HoleClearanceBudgets.choose(budget(), []) {
			case ChangeDesign(_): true;
			default: false;
		}, "no route permission requires a source design change");
		var error = "";
		source = budget(); source.cureBowMm = -0.1;
		try HoleClearanceBudgets.assess(source) catch (e:Dynamic) error = Std.string(e);
		check(error.indexOf("cureBowMm") >= 0, "negative error bounds are rejected by field name");
		error = "";
		try HoleClearanceBudgets.choose(budget(), [InsertHoleRoute.BeforeCasting, InsertHoleRoute.BeforeCasting])
		catch (e:Dynamic) error = Std.string(e);
		check(error.indexOf("Duplicate") >= 0, "route permissions are unique");
		var requirement:materia.casting.InsertMountHoleRequirement = {
			holeId: "rail-mount-1", componentPath: "xyStage/railYLeft",
			componentFingerprint: haxe.crypto.Sha256.encode("test rail recipe"), connectorId: "mount1",
			connectorFrame: F.alongY(0, 200, 35, 0, 0, 1), clearanceReference: "test reference",
			permittedRoutes: both, budget: budget()
		};
		var copied = InsertMountHoleRequirements.snapshot([requirement]);
		check(copied[0].componentPath == requirement.componentPath && copied[0].connectorFrame.z == 35,
			"placed component and connector lineage survive the strict codec");
		var hole:InsertHole = {
			id: "rail-mount-1", frame: F.fromRotationMatrix(0, 200, 20, [1, 0, 0, 0, -1, 0, 0, 0, -1]),
			diameter: 3.3, depth: 17, extent: HoleExtent.Blind,
			thread: {designation: "M4", majorDiameter: 4, pitch: 0.7, starts: 1, hand: InsertThreadHand.Right,
				internal: true, toleranceClass: "6H", profile: InsertThreadProfile.Metric},
			tapDrill: 3.3, threadDepth: 16, clearanceDiameter: 4.5, counterbore: null, relief: null,
			limits: {policyId: "test-thread-policy", threaded: {minEngagement: 5.2, minTipGap: 1},
				minBack: 3, minWall: 4, minClearance: 1, overrides: []}
		};
		var holes = materia.casting.InsertHole.InsertHoleCodec.snapshot([hole]);
		InsertMountHoleRequirements.references(copied, holes);
		check(holes[0].frame.z == 20, "validation preserves the one source entry frame below the rail connector");
		holes[0].frame.x = 1; holes[0].frame.y = Math.sqrt(200*200-1); error = "";
		try InsertMountHoleRequirements.references(copied, holes) catch (e:Dynamic) error = Std.string(e);
		check(error.indexOf("not coaxial") >= 0, "a shifted hole breaks only the connector line, preserving its datum distance");
		holes[0].frame.x = 0; holes[0].frame.y = 200;
		holes[0].frame = F.alongY(0, 200, 20, 0, 0, 1); error = "";
		try InsertMountHoleRequirements.references(copied, holes) catch (e:Dynamic) error = Std.string(e);
		check(error.indexOf("axis differs") >= 0, "a rotated hole breaks connector lineage independently");
		error = "";
		try InsertMountHoleRequirements.snapshot([requirement, requirement]) catch (e:Dynamic) error = Std.string(e);
		check(error.indexOf("Duplicate mounting-hole") >= 0, "one requirement per source hole");
		var changed:Dynamic = haxe.Json.parse(haxe.Json.stringify(requirement));
		Reflect.setField(changed, "future", true); error = "";
		try InsertMountHoleRequirements.snapshot([changed]) catch (e:Dynamic) error = Std.string(e);
		check(error.length > 0, "unknown mounting requirement fields are rejected");
		holes = materia.casting.InsertHole.InsertHoleCodec.snapshot([hole]);
		var material=materia.casting.InsertMaterial.InsertMaterials.specification(materia.casting.InsertMaterial.C45Bright);
		var manufacturing=materia.casting.InsertManufacturing.InsertManufacturingCodec.draft(material);
		manufacturing.mountingRequirements=copied;
		manufacturing.holeRoutes=[{holeId:hole.id,route:InsertHoleRoute.BeforeCasting}];
		materia.casting.InsertManufacturing.InsertManufacturingCodec.references(manufacturing,holes,[]);
		var insert:materia.tooling.casting.CastingToolingInput.CastingToolingInsert={id:"rail-seat",
			body:Primitive(materia.casting.CastingSolid.Box(40,500,20)),holes:holes,
			origin:Fabricated(manufacturing),anchorage:Undeclared,interfaces:[],material:material,pose:F.identity(),
			role:materia.casting.InsertSpec.InsertRole.GuideSeat,label:"test guide seat",positionTolerance:0.01,
			finish:materia.casting.InsertSpec.InsertFinish.AsSupplied};
		var job=materia.tooling.casting.InsertManufacturingPlan.build(insert);
		check(job.holeBudgets.length==1 && job.holeBudgets[0].status=="pass"
			&& job.holeBudgets[0].route==InsertHoleRoute.BeforeCasting,"source budget drives the manufacturing report");
		manufacturing.mountingRequirements[0].budget.machiningErrorMm=null;
		manufacturing.mountingRequirements[0].budget.requiredMarginMm=null;
		error="";
		try materia.casting.InsertManufacturing.InsertManufacturingCodec.references(manufacturing,holes,[])
		catch (e:Dynamic) error=Std.string(e);
		check(error.indexOf("differs from clearance budget")>=0,"stale pre-cast route cannot survive missing budget inputs");
		manufacturing.holeRoutes=[{holeId:hole.id,route:InsertHoleRoute.TransferAfterCasting}];
		job=materia.tooling.casting.InsertManufacturingPlan.build(insert);
		check(job.holeBudgets[0].status=="pending" && job.holeBudgets[0].missingInputs.indexOf("machiningErrorMm")>=0,
			"post-cast proposal retains unresolved guide accuracy");
		var transfer=job.operations.filter(operation -> operation.kind==materia.tooling.casting.InsertManufacturingPlan.InsertOperationKind.TransferDrill);
		check(transfer.length==1 && materia.tooling.casting.InsertManufacturingPlan.instructions(transfer[0]).join(" ").indexOf("guide bushing")>=0,
			"transfer operation names the bushing and swarf protection");
		var before=materia.casting.InsertStages.holes({body:insert.body,holes:holes,origin:insert.origin,anchorage:insert.anchorage,interfaces:[]},
			materia.casting.InsertStages.InsertStage.BeforeCasting);
		check(before.length==0,"proposed transfer holes never enter pre-cast geometry");
		var raw:Dynamic=materia.casting.InsertManufacturing.InsertManufacturingCodec.record(manufacturing);
		Reflect.deleteField(raw,"mountingRequirements");error="";
		try materia.casting.InsertManufacturing.InsertManufacturingCodec.snapshot(raw) catch (e:Dynamic) error=Std.string(e);
		check(error.length>0,"old manufacturing encoding has no compatibility default");
		Sys.println('HoleClearanceBudgetTests: $assertions assertions passed');
	}

	public static function main():Void run();
}
