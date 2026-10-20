package materia.tooling.casting;

import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.casting.InsertHole;
import materia.casting.InsertHole.InsertHoleCodec;
import materia.tooling.ToolingRecipe.ToolingHardware;
import materia.tooling.panel.PanelPlan.PanelOperationKind;

typedef FixtureScrewStock = {
	var thread:InsertThread;
	var lengths:Array<Float>;
	var washerThickness:Float;
	var driverDiameter:Float;
	var driverToolDiameter:Float;
}

typedef InsertPlateOptions = {
	var id:String;
	var insertIds:Array<String>;
	var contactZ:Float;
	var edgeMargin:Float;
	var locatorEndInset:Float;
	var locatorSideInset:Float;
	var screwStock:Array<FixtureScrewStock>;
}

typedef InsertPlateSite = {
	var id:String;
	var insertId:String;
	var sourceHoleId:String;
	var entry:AssemblyFrame;
	var thread:InsertThread;
	var screwLength:Float;
	var insertion:Float;
	var engagement:Float;
	var clearanceDiameter:Float;
	var washerThickness:Float;
	var driverDiameter:Float;
	var plug:Null<{diameter:Float, length:Float, materialSpec:String}>;
}

typedef InsertPlatePlan = {
	var document:materia.tooling.ToolingDocument;
	var options:InsertPlateOptions;
	var assembly:materia.tooling.panel.PanelAssemblyPlan.PanelAssemblyRecord;
	var sites:Array<InsertPlateSite>;
	var penetrations:Array<MoldPlan.PenetrationPlan>;
	var primary:{x:Float,y:Float};
	var secondary:{x:Float,y:Float};
	var clampPoints:Array<{x:Float,y:Float}>;
	var validation:Array<materia.tooling.ToolingValidation>;
	var changeRequests:Array<materia.tooling.casting.CastingMoldPlan.DesignChangeRequest>;
	var masks:Array<{insertId:String,protection:materia.tooling.casting.CastingMoldPlan.BoreMaskProtection}>;
}

/** Both mold planners use the same source-thread access and axial depth budget. */
class InsertFixturePlanner {
	/** Reusable two-plate datum carrier. Only source holes facing the plate are used.
	 * Screw lengths are selected from declared stock after the complete depth budget.
	 * This constructs no thread in tooling and makes no fixture-accuracy claim.
	 */
	public static function plate(document:materia.tooling.ToolingDocument, options:InsertPlateOptions):InsertPlatePlan {
		var d = materia.tooling.ToolingCodec.snapshot(document);
		options = copyOptions(options);
		materia.tooling.ToolingIds.validate(options.id);
		for (value in [options.edgeMargin, options.locatorEndInset, options.locatorSideInset])
			if (!Math.isFinite(value) || value <= 0) throw "Fixture plate layout dimensions must be positive";
		if (!Math.isFinite(options.contactZ) || options.insertIds == null || options.insertIds.length == 0 || options.screwStock == null)
			throw "Fixture plate needs a contact plane and selected inserts";
		var ids = options.insertIds.copy();
		ids.sort(Reflect.compare);
		options.insertIds = ids.copy();
		for (choice in options.screwStock) {
			if (choice.lengths == null || choice.lengths.length == 0) throw "Fixture screw stock needs declared lengths";
			choice.lengths.sort(Reflect.compare);
			for (i in 0...choice.lengths.length)
				if (!Math.isFinite(choice.lengths[i]) || choice.lengths[i] <= 0 || i > 0 && choice.lengths[i] == choice.lengths[i-1])
					throw "Fixture screw stock lengths must be positive and unique";
		}
		options.screwStock.sort((a,b) -> Reflect.compare(materia.tooling.ToolingCodec.canonical(a.thread),materia.tooling.ToolingCodec.canonical(b.thread)));
		for (i in 0...ids.length)
			if (i > 0 && ids[i] == ids[i-1]) throw "Duplicate fixture insert: " + ids[i];
		var xs:Array<Float> = [], ys:Array<Float> = [], zs:Array<Float> = [];
		for (prism in d.input.envelope) {
			var pose = F.compose(d.setup.sourceToMold, prism.pose);
			for (point in prism.profile) for (z in [0.0, prism.height]) {
				var at = F.transformPoint(pose, point.x, point.y, z);
				xs.push(at.x); ys.push(at.y); zs.push(at.z);
			}
		}
		xs.sort(Reflect.compare); ys.sort(Reflect.compare); zs.sort(Reflect.compare);
		var minX = xs[0] - options.edgeMargin, minY = ys[0] - options.edgeMargin;
		var width = xs[xs.length-1] - xs[0] + 2*options.edgeMargin;
		var depth = ys[ys.length-1] - ys[0] + 2*options.edgeMargin;
		if (2*options.locatorEndInset >= width || 2*options.locatorSideInset >= depth
			|| options.locatorSideInset >= options.edgeMargin)
			throw "Fixture locators and clamps must stay outside the casting footprint";
		var primary = {x:options.locatorEndInset, y:options.locatorSideInset};
		var secondary = {x:width-options.locatorEndInset, y:options.locatorSideInset};
		var clamps = [{x:primary.x,y:depth-options.locatorSideInset}, {x:secondary.x,y:depth-options.locatorSideInset}];
		var recipe = d.recipe, hardware = recipe.hardware, thickness = recipe.panel.measuredThickness;
		var pair = materia.tooling.panel.PanelJoints.pinBolt(options.id, recipe.panel, hardware.fixturePinDiameter,
			recipe.jointClearance, hardware.secondaryPinTravel, hardware.supportClearanceDiameter,
			hardware.fixtureSlotToolDiameter, recipe.minimumLigament,
			{width:width,height:depth,primary:primary,secondary:secondary,bolt:clamps[0]});
		pair.joint.liningAllowance = recipe.jointLiningAllowance;
		pair.joint.machiningAllowance = recipe.jointMachiningAllowance;
		for (panel in [pair.fixed,pair.moving]) {
			var closure = materia.tooling.panel.PanelJoints.operation("second-closure", Drill,
				materia.tooling.panel.PanelContours.circle(clamps[1].x,clamps[1].y,hardware.supportClearanceDiameter/2),
				thickness,hardware.supportClearanceDiameter);
			closure.through = true;
			panel.operations.push(closure);
		}
		var sites:Array<InsertPlateSite> = [];
		var changes:Array<materia.tooling.casting.CastingMoldPlan.DesignChangeRequest> = [];
		var masks:Array<{insertId:String,protection:materia.tooling.casting.CastingMoldPlan.BoreMaskProtection}> = [];
		var validation = InsertQualifications.check(d.input.inserts.filter(row -> ids.indexOf(row.id) >= 0));
		var onFormFace = Math.abs(options.contactZ-zs[0]) <= 1e-7;
		if (!onFormFace)
			validation.push(materia.tooling.ToolingValidation.ToolingReadiness.pending("tooling.fixture.integration."+options.id,ids,
				"Carrier is not on the casting envelope's lower form face; integrate the separate bed, support and sealing paths"));
		var outsideScope = d.input.inserts.filter(row -> ids.indexOf(row.id) < 0);
		if (outsideScope.length > 0)
			validation.push(materia.tooling.ToolingValidation.ToolingReadiness.pending("tooling.fixture.coverage."+options.id,
				[for (row in outsideScope) row.id],"Inserts outside this carrier's declared scope require their own fixture and hole protection"));
		for (id in ids) {
			var matches = d.input.inserts.filter(row -> row.id == id);
			if (matches.length != 1) throw "Unknown fixture insert: " + id;
			var insert = matches[0], count = sites.length;
			validation = validation.concat(protectionValidation(insert));
			var pose = F.compose(d.setup.sourceToMold, insert.pose);
			for (hole in insert.holes) {
				var fixing = select(hole, pose, F.translation(0,0,options.contactZ), hardware);
				if (fixing == null) continue;
				var thread:InsertThread = cast hole.thread;
				var sealRadius = switch materia.casting.InsertSpec.InsertBodyCodec.mouth(insert.body) {
					case Box(width,depth,_): Math.min(width/2-Math.abs(hole.frame.x),depth/2-Math.abs(hole.frame.y));
					case Cylinder(diameter,_): diameter/2-Math.sqrt(hole.frame.x*hole.frame.x+hole.frame.y*hole.frame.y);
				};
				if (sealRadius-fixing.clearanceDiameter/2 < hardware.sealMargin-1e-7)
					throw "Fixture insert sealing margin exceeded: " + id + "/" + hole.id;
				var stock = options.screwStock.filter(row -> materia.tooling.ToolingCodec.canonical(row.thread) == materia.tooling.ToolingCodec.canonical(thread));
				if (stock.length != 1) throw "Fixture screw stock must identify one source thread: " + id + "/" + hole.id;
				var choice = stock[0];
				for (value in [choice.washerThickness,choice.driverDiameter,choice.driverToolDiameter])
					if (!Math.isFinite(value) || value <= 0) throw "Fixture screw hardware dimensions must be positive";
				if (choice.driverDiameter <= fixing.clearanceDiameter) throw "Fixture driver access must pass the screw head";
				var lengths = choice.lengths.copy(); lengths.sort(Reflect.compare);
				var length:Null<Float> = null;
				var limits = InsertHoleCodec.threadLimits(hole);
				var tipDepth = hole.relief == null ? (cast hole.threadDepth:Float) : hole.depth;
				if (fixing.plug != null) tipDepth = Math.min(tipDepth,hole.depth-fixing.plug.length);
				for (candidate in lengths) {
					if (!Math.isFinite(candidate) || candidate <= 0) throw "Fixture screw stock lengths must be positive";
					var insertion = candidate-thickness-choice.washerThickness;
					if (length == null && Math.min(insertion,cast hole.threadDepth) >= limits.minEngagement-1e-7
						&& insertion+limits.minTipGap <= tipDepth+1e-7) length = candidate;
				}
				if (length == null) throw "No stock fixture screw satisfies source thread engagement and tip gap: " + id + "/" + hole.id;
				var x = fixing.entry.x-minX, y = fixing.entry.y-minY;
				var siteId = materia.tooling.ToolingIds.derive("fixture-",id,"-"+haxe.crypto.Sha256.encode(hole.id).substr(0,12));
				var clearance = materia.tooling.panel.PanelJoints.operation(siteId,Drill,
					materia.tooling.panel.PanelContours.circle(x,y,fixing.clearanceDiameter/2),thickness,fixing.clearanceDiameter);
				clearance.through = true;
				pair.moving.operations.push(clearance); pair.moving.contactOpenings.push(siteId);
				var access = materia.tooling.panel.PanelJoints.operation(materia.tooling.ToolingIds.derive("",siteId,"-access"),InsideProfile,
					materia.tooling.panel.PanelContours.circle(x,y,choice.driverDiameter/2),thickness,choice.driverToolDiameter);
				access.through = true; pair.fixed.operations.push(access);
				sites.push({id:siteId,insertId:id,sourceHoleId:hole.id,entry:fixing.entry,thread:thread,
					screwLength:length,insertion:length-thickness-choice.washerThickness,engagement:fixing.engagement,
					clearanceDiameter:fixing.clearanceDiameter,washerThickness:choice.washerThickness,
					driverDiameter:choice.driverDiameter,plug:fixing.plug});
			}
			for (hole in insert.holes) {
				if (sites.filter(site -> site.insertId == id && site.sourceHoleId == hole.id).length > 0) continue;
				var maskId = materia.tooling.ToolingIds.derive("mask-",id,"-"+haxe.crypto.Sha256.encode(hole.id).substr(0,12));
				var entry = F.compose(pose,hole.frame), axis = F.transformVector(entry,0,0,-1);
				masks.push({insertId:id,protection:mask(hole,maskId,hardware.plugDepth,axis)});
			}
			if (sites.length == count) {
				changes.push({id:materia.tooling.ToolingIds.derive("fixture-hole-",id),
					kind:materia.tooling.casting.CastingMoldPlan.DesignChangeKind.InsertFixtureHole,sourceIds:[id],
					reason:"Declare an accessible steel thread facing the fixture plate; tooling creates no insert hole.",
					blockingChecks:["tooling.fixture.source-hole."+id]});
				validation.push(materia.tooling.ToolingValidation.ToolingReadiness.pending("tooling.fixture.source-hole."+id,[id],
					"Design must declare a usable source thread facing the fixture; tooling creates no insert hole"));
			}
		}
		var penetrations:Array<MoldPlan.PenetrationPlan> = [];
		for (panel in [pair.fixed,pair.moving]) for (operation in panel.operations) {
			if (!operation.through || operation.kind == OutsideProfile) continue;
			var matches = sites.filter(site -> panel.id == pair.moving.id && operation.id == site.id);
			var site:Null<InsertPlateSite> = onFormFace && matches.length == 1 ? matches[0] : null;
			penetrations.push({panelId:panel.id,operationId:operation.id,
				kind:site == null ? OutsideCasting : InsertSealed,insertId:site == null ? null : site.insertId,
				sourceHoleId:site == null ? null : site.sourceHoleId,coreId:null,
				sealMargin:hardware.sealMargin,sealMaterial:hardware.sealMaterial,
				sealingFace:site == null ? null : site.entry,plug:site == null ? null : site.plug});
		}
		var assembly = materia.tooling.panel.PanelAssemblyPlan.pair(pair,F.translation(minX,minY,options.contactZ-2*thickness));
		for (member in assembly.members) member.withdrawal = {x:0,y:0,z:-1};
		materia.tooling.panel.PanelAssemblyPlan.validate(assembly);
		validation.push(materia.tooling.ToolingValidation.ToolingReadiness.pending("tooling.fixture.accuracy."+options.id,ids,
			"Locator fit and six-DOF fixture accuracy require an allocated tolerance budget and physical evidence"));
		validation.push(materia.tooling.ToolingValidation.ToolingReadiness.pending("tooling.fixture.seal."+options.id,ids,
			"Source thread fixing has no bore mask; qualify the mouth-face seal and release before casting"));
		return {document:d,options:options,assembly:assembly,sites:sites,penetrations:penetrations,
			primary:primary,secondary:secondary,clampPoints:clamps,validation:validation,changeRequests:changes,masks:masks};
	}

	static function copyOptions(options:InsertPlateOptions):InsertPlateOptions {
		if (options == null || options.insertIds == null || options.screwStock == null)
			throw "Fixture plate needs selected inserts and declared screw stock";
		materia.casting.CastingFeatureCodec.fields(options,["id","insertIds","contactZ","edgeMargin","locatorEndInset","locatorSideInset","screwStock"]);
		var stock:Array<FixtureScrewStock> = [];
		for (row in options.screwStock) {
			materia.casting.CastingFeatureCodec.fields(row,["thread","lengths","washerThickness","driverDiameter","driverToolDiameter"]);
			if (row.thread == null || row.lengths == null) throw "Fixture screw stock needs a source thread and lengths";
			var t = row.thread;
			stock.push({thread:{designation:t.designation,majorDiameter:t.majorDiameter,pitch:t.pitch,starts:t.starts,
				hand:t.hand,internal:t.internal,toleranceClass:t.toleranceClass,profile:t.profile},lengths:row.lengths.copy(),
				washerThickness:row.washerThickness,driverDiameter:row.driverDiameter,driverToolDiameter:row.driverToolDiameter});
		}
		return {id:options.id,insertIds:options.insertIds.copy(),contactZ:options.contactZ,edgeMargin:options.edgeMargin,
			locatorEndInset:options.locatorEndInset,locatorSideInset:options.locatorSideInset,screwStock:stock};
	}

	public static function validatePlate(plan:InsertPlatePlan):Void {
		var rebuilt = plate(plan.document,plan.options);
		if (materia.tooling.ToolingCodec.canonical(rebuilt) != materia.tooling.ToolingCodec.canonical(plan))
			throw "Insert fixture plan differs from source holes and declared setup";
	}

	public static function protectionValidation(insert:materia.tooling.casting.CastingToolingInput.CastingToolingInsert):Array<materia.tooling.ToolingValidation> {
		var result:Array<materia.tooling.ToolingValidation> = [];
		for (hole in insert.holes) {
			var thread = hole.thread;
			if (thread == null)
				continue;
			var reason = hole.counterbore != null ? "A separate counterbore sealing cap must be specified."
				: thread.profile != Metric || thread.hand != Right || thread.starts != 1
					? "Select protection hardware for this non-standard set-screw thread." : null;
			if (reason != null)
				result.push({rule: "tooling.fixture.hole-protection." + insert.id + "." + hole.id,
					affectedIds: [insert.id, hole.id], status: materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported,
					severity: materia.tooling.ToolingDiagnostic.ToolingSeverity.Error, message: reason, evidence: []});
		}
		return result;
	}

	public static function mask(hole:InsertHole, id:String, plugLength:Float,
			withdrawal:{x:Float, y:Float, z:Float}):materia.tooling.casting.CastingMoldPlan.BoreMaskProtection {
		var threaded = hole.thread != null;
		if (threaded && plugLength > (cast hole.threadDepth:Float))
			throw "Threaded plug exceeds usable source thread: " + hole.id;
		if (threaded && plugLength * (hole.extent == Through ? 2 : 1) + InsertHoleCodec.threadLimits(hole).minTipGap > hole.depth + 1e-7)
			throw "Thread protection exceeds source hole depth budget: " + hole.id;
		return {sourceHoleId: hole.id, hardwareId: id, plugLength: threaded ? plugLength : null,
			farEndId: threaded && hole.extent == Through ? materia.tooling.ToolingIds.derive("far-end-", id) : null,
			withdrawal: withdrawal};
	}

	public static function select(hole:InsertHole, insertToMold:AssemblyFrame, supportFace:AssemblyFrame,
			hardware:ToolingHardware):Null<{
		entry:AssemblyFrame,
		engagement:Float,
		clearanceDiameter:Float,
		plug:Null<{diameter:Float, length:Float, materialSpec:String}>
	}> {
		var entry = F.compose(insertToMold, hole.frame), axis = F.transformVector(entry, 0, 0, 1);
		if (hole.thread == null || axis.z < 1 - 1e-8 || Math.abs(entry.z - supportFace.z) > 1e-7)
			return null;
		var thread:InsertThread = cast hole.thread;
		if (hole.extent == Through && hardware.plugMaterial != "steel")
			throw "Unsupported threaded fixing plug material: " + hardware.plugMaterial;
		var plug = hole.extent == Through ? {
			diameter: thread.majorDiameter, length: hardware.plugDepth, materialSpec: hardware.plugMaterial
		} : null;
		var limits = InsertHoleCodec.threadLimits(hole);
		var occupied = limits.minEngagement + limits.minTipGap + (plug == null ? 0 : plug.length);
		var tipDepth = hole.relief == null ? (cast hole.threadDepth:Float) : hole.depth;
		if (occupied > hole.depth + 1e-7
			|| limits.minEngagement > (cast hole.threadDepth:Float) + 1e-7
			|| limits.minEngagement + limits.minTipGap > tipDepth + 1e-7)
			return null;
		return {entry: entry, engagement: limits.minEngagement,
			clearanceDiameter: hole.clearanceDiameter, plug: plug};
	}
}
