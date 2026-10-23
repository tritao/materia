package materia.tooling.casting;

import materia.tooling.casting.CastingMoldPlan.MoldFixtures;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.tooling.ToolingEnums.ContactFace;
import materia.tooling.panel.PanelJoints.PanelJointKind;
import materia.tooling.panel.PanelJoints.PanelClosure;
import materia.tooling.panel.PanelAssemblyPlan.PanelPlacement;
import materia.tooling.panel.PanelAssemblyPlan.PanelAssemblyRecord;
import materia.tooling.casting.CastingMoldPlan.PenetrationPlan;
import materia.tooling.casting.CoreSupport.CoreSupportKind;

import materia.casting.InsertSpec.InsertBodyCodec as InsertBodies;
import materia.casting.BoxAlgebra;
import materia.tooling.casting.MoldRegions.MoldVolumeRecords;

import materia.assembly.AssemblyFrames as F;
import materia.tooling.ToolingDocument;
import materia.tooling.ToolingCodec;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.CastingMoldPlan.MoldBox;
import materia.tooling.casting.MoldPanels.RaisedWalls;
import materia.tooling.casting.CastingSetup.CoreStrategy;
import materia.casting.CastingFeature.CastingFeatureKind;
import materia.tooling.panel.PanelJoints;
import materia.tooling.panel.PanelCodec;
import materia.tooling.panel.PanelContours;
import materia.casting.CastingProfileGeometry;
import materia.tooling.ToolingEnums.MachiningSide;

/** Source assessment can decline geometry without manufacturing a box stand-in. */
typedef CastingMoldAssessment = {
	var sourceFingerprint:String;
	var plan:Null<CastingMoldPlan>;
	var validation:Array<materia.tooling.ToolingValidation>;
	var changeRequests:Array<DesignChangeRequest>;
	var fabrication:materia.tooling.ToolingValidation.FabricationReadiness;
}

/** Orthogonal review template for multiple explicit EPS/former strategies.
 * All casting dimensions come from the source document. It never adds design cuts.
 */
class CastingMoldPlanner {
	/** Construction identity includes the source and its declared or recipe-derived choices. */
	public static function fingerprint(plan:CastingMoldPlan):String {
		var recipe=plan.document.recipe;
		var choices:Dynamic=switch plan.construction {
			case SharedBridges:MoldOptions.record(plan.options);
			case SinglePocket:{mode:"auto",embedDepth:recipe.panel.measuredThickness/3,
				fitClearance:recipe.jointClearance,adhesiveThickness:recipe.adhesiveLineThickness};
		};
		return haxe.crypto.Sha256.encode(ToolingCodec.canonical({document:ToolingCodec.documentRecord(plan.document),options:choices}));
	}

	/** Application-owned exact kernel. A pure box-only process leaves it unset. */
	public static var geometry(default,null):Null<CastingMoldKernel>;

	public static function setGeometry(kernel:Null<CastingMoldKernel>):Void
		geometry=kernel;

	public static function sourceSupport(input:CastingToolingInput):Array<materia.tooling.ToolingValidation>
		return CastingBoxSupport.check(input, "tooling.feature.box-geometry", false).concat(InsertQualifications.check(input.inserts));
	public static function envelope(d:ToolingDocument):MoldBox {
		var rows = d.input.envelope;
		if (rows.length != 1 || rows[0].profile.length != 4)
			throw "Multi-core box template needs one rectangular envelope";
		var e = rows[0], xs = [for (p in e.profile) p.x], ys = [for (p in e.profile) p.y];
		xs.sort(Reflect.compare);
		ys.sort(Reflect.compare);
		for (p in e.profile)
			if ((p.x != xs[0] && p.x != xs[3]) || (p.y != ys[0] && p.y != ys[3]))
				throw "Nonrectangular envelope unsupported by box template";
		return CastingMoldPlanner.box(xs[3] - xs[0], ys[3] - ys[0], e.height,
			F.compose(d.setup.sourceToMold, F.compose(e.pose, F.translation((xs[0] + xs[3]) / 2, (ys[0] + ys[3]) / 2, 0))));
	}

	/** Shell stock bounds only. Material queries use MoldRegions.envelope. */
	public static function shellBounds(document:ToolingDocument):MoldBox {
		var region=MoldRegions.envelope(document),pieces=MoldRegions.boxes(region);
		if (pieces!=null && pieces.length==1 && document.input.envelope.length==1
			&& document.input.envelope[0].profile.length==4) return envelope(document);
		return MoldRegions.exactBounds(region,document.input.id);
	}

	public static function insertBox(document:ToolingDocument,id:String):MoldBox {
		var matches=document.input.inserts.filter(insert -> insert.id==id);
		if (matches.length!=1) throw "Expected one source insert: "+id;
		var insert=matches[0],dimensions=InsertBodies.box(insert.body,id);
		return box(dimensions.width,dimensions.depth,dimensions.height,
			F.compose(document.setup.sourceToMold,insert.pose));
	}

	public static function featureBox(d:ToolingDocument, f:materia.casting.CastingFeature):MoldBox {
		var boxes=MoldRegions.boxes(MoldRegions.feature(d,f.id));
		if (boxes==null || boxes.length!=1) throw "Unsupported non-box finished cavity: "+f.id;
		return boxes[0];
	}

	public static function assess(document:ToolingDocument, authored:CastingMoldOptions):CastingMoldAssessment {
		var d = ToolingCodec.copy(document), options = MoldOptions.copy(authored);
		var checks = sourceSupport(d.input);
		// Hardware qualifications affect readiness, not whether the source cavity
		// can be panelled. Exact hardware is realized and checked by the CAD bridge.
		var unsupported = checks.filter(check -> StringTools.startsWith(check.rule,"tooling.feature.box-geometry")
			&& check.status == materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported);
		var regionTooling=unsupported.length>0 && canPlanRegions(d);
		if (regionTooling) {
			checks=checks.filter(check -> unsupported.indexOf(check)<0);
			checks.push(materia.tooling.process.FillAnalysis.check("tooling.feature.box-geometry",
				[d.input.id].concat([for (feature in d.input.features) feature.id]),Pass,
				"Panel shell with exact source envelope/cavity tooling and registered CAD geometry.",[]));
			unsupported=[];
		}
		if (unsupported.length > 0) {
			var changes:Array<DesignChangeRequest> = [for (check in unsupported) if (d.input.features.filter(feature -> check.affectedIds.indexOf(feature.id)>=0).length>0) {
				id: materia.tooling.ToolingIds.derive("geometry-", check.affectedIds.join("-")),
				kind: DesignChangeKind.ReleaseAlternative,
				sourceIds: check.affectedIds.copy(),
				reason: check.message + ". Select supported source geometry or implement profile-capable tooling before fabrication.",
				blockingChecks: [check.rule]
			}];
			checks=checks.concat(InterfaceQualification.source(d.input)).concat(InsertQualifications.manufacturing(d.input.inserts))
				.concat(ReplicationJigs.analyze(d.input,options.replicationJigs,options.precision))
				.concat(FoundationJointQualification.analyze(d.input,options.foundationJointProcesses));
			return {sourceFingerprint: ToolingCodec.documentFingerprint(d), plan: null, validation: checks,
				changeRequests: changes, fabrication: materia.tooling.ToolingValidation.ToolingReadiness.evaluate(checks)};
		}
		var result = build(d, options, checks);
		return {sourceFingerprint: result.sourceFingerprint, plan: result, validation: result.validation.copy(),
			changeRequests: result.changeRequests.copy(), fabrication: result.fabrication};
	}

	public static function plan(document:ToolingDocument, authored:CastingMoldOptions):CastingMoldPlan {
		var assessment = assess(document, authored);
		if (assessment.plan == null)
			throw [for (check in assessment.validation) check.message].join("\n");
		return cast assessment.plan;
	}

	static function canPlanRegions(d:ToolingDocument):Bool {
		if (geometry==null) return false;
		var strategies=CastingStrategies.resolve(d);
		shellBounds(d);
		for (insert in d.input.inserts) switch insert.body {
			case Primitive(materia.casting.CastingSolid.Box(_,_,_)):
			default:return false;
		}
		for (feature in d.input.features) {
			if (feature.draft.degrees>0 && materia.casting.CastingDraftSections.unsupportedReason(d.input.envelope,feature)!=null)
				return false;
			var region=MoldRegions.feature(d,feature.id);
			if (MoldRegions.boxes(region)==null) {
				var selected=strategies.filter(strategy -> strategy.featureId==feature.id);
				if (selected.length!=1) return false;
				if (selected[0].strategy==CoreStrategy.SacrificialEPS) {
					if (!MoldCoreLayers.supported(region,feature.id)) return false;
				} else if (selected[0].strategy!=CoreStrategy.ReusableFormer
					|| selected[0].withdrawal==null || !MoldFormerAccess.supported(d,feature,selected[0].withdrawal)) return false;
			}
		}
		return true;
	}

	static function requiresRegions(d:ToolingDocument):Bool {
		var body=MoldRegions.boxes(MoldRegions.envelope(d));
		return body==null || body.length!=1 || materia.casting.CastingCavities.derive(d.input.envelope,d.input.features)
			.filter(cavity -> !materia.casting.CastingCavities.supportsBoxes(cavity)).length>0;
	}

	/** Interior allocation uses authoritative cavities; panel sealing profiles
	 * extend outward from the source boundary and have no source interior.
	 */
	static function formerRegion(former:MoldFormer):materia.casting.CastingRegion
		return materia.casting.CastingRegion.Union([for (piece in former.volume.pieces) MoldRegions.box(piece)].concat(former.volume.regions));

	static function seed(d:ToolingDocument,o:CastingMoldOptions,e:MoldBox,b:MoldBox,
			geometryChecks:Array<materia.tooling.ToolingValidation>,walls:RaisedWalls=RaisedWalls.EastWest):CastingMoldPlan {
		var r=d.recipe;
		var shell = MoldPanels.box(b, r, walls),
			panels = shell.panels,
			bed = panels.members[0];
		var openings:Array<MoldOpening> = [
			for (p in shell.penetrations)
				{
					panelId: p.panelId,
					operationId: p.operationId,
					kind: ExteriorOpening,
					sealingIds: [],
					protection:null, sealSpec: p.sealMaterial
				}
		];
		return {
			construction:SharedBridges,
			handling: CastHandling.plan(d, o),
			id: materia.tooling.ToolingIds.derive("", d.input.id, "-mold"),
			document: d,
			sourceFingerprint: ToolingCodec.documentFingerprint(d),
			options: o,
			envelope: e,
			cavity: b,
			panels: panels,
			supportPanels: [],
			cores: [],
			coreUnits: [],
			formers: [],
			fixtures: [],
			features: [],
			openings: openings,
			hardware: [],
			changeRequests: [],
			validation: geometryChecks,
			fabrication: {
				ready: false,
				blockingRules: []
			}
		};
	}

	static function finish(result:CastingMoldPlan,
			strategies:Array<materia.tooling.casting.CastingSetup.CastingFeatureStrategy>):CastingMoldPlan {
		var d=result.document,o=result.options;
		for (strategy in strategies) if (strategy.strategy==CoreStrategy.ReusableFormer) {
			var feature=d.input.features.filter(feature -> feature.id==strategy.featureId)[0];
			var axis=F.transformVector(d.setup.sourceToMold,feature.draft.axis.x,feature.draft.axis.y,feature.draft.axis.z);
			var direction=strategy.withdrawal;
			var aligned=direction!=null && axis.x*direction.x+axis.y*direction.y+axis.z*direction.z>1-1e-8;
			if (feature.draft.degrees==0 || !aligned)
				request(result,"draft-"+feature.id,ReleaseAlternative,[feature.id],
					"Rigid former needs positive source draft aligned with its withdrawal, or a declared sacrificial alternative.",
					"tooling.former.draft."+feature.id);
			else result.validation.push(materia.tooling.process.FillAnalysis.check("tooling.former.draft."+feature.id,
				[feature.id],Pass,"Positive source draft is aligned with the declared withdrawal; CAD verifies the complete removal sweep.",[]));
		}
		for (former in result.formers)
			if (former.featureIds.length == 0 && former.draftDegrees == 0)
				request(result, "draft-" + former.volume.id, ReleaseAlternative, [former.volume.id],
					(StringTools.startsWith(former.volume.id,"envelope-former-")
						? "Envelope-shaping zero-draft rigid former needs release qualification or a supported source/setup alternative."
						: "Bed-backing zero-draft rigid former needs a source/setup or supported sacrificial-tooling change; draft on a different feature does not qualify this part."),
					"tooling.former.draft." + former.volume.id);
		var handlingQualified=CastHandling.qualified(d,o,result.handling);
		if (result.handling.requiresFlip && !CastHandling.sourceDeclared(d) && !handlingQualified)
			request(result,"handling-source-points",LiftingPoints,[d.input.id],
				"Declare lifting inserts with unoccupied pre-cast threads, or a qualified external cradle.",
				"tooling.handling.lifting-points");
		else result.validation.push(materia.tooling.process.FillAnalysis.check("tooling.handling.lifting-points",
			[d.input.id].concat(CastHandling.liftingIds(d)),Pass,
			"Source lifting connections are declared, a cradle is qualified, or this setup needs no flip.",[]));
		result.validation.push(materia.tooling.process.FillAnalysis.check("tooling.handling.flip",[d.input.id],
			!result.handling.requiresFlip || handlingQualified ? Pass : Pending,
			"Equipment capacity, measured mass/CoM and the flip procedure require separate handling evidence.",[]));
		for (seat in o.precision) {
			var source = d.input.inserts.filter(i -> i.id == seat.insertId)[0];
			var stock=InterfaceQualification.finishingStock(d.input,source);
			if (stock.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass) result.validation.push(stock);
			else requestDecision(result,"allowance-"+seat.insertId,InterfaceAllowance,stock);
			var replication=InterfaceQualification.replicationRequirement(source,seat);
			if (replication!=null) result.validation.push(replication);
		}
		for (insert in d.input.inserts) switch insert.origin {
			case Fabricated(manufacturing):
				for (report in InsertManufacturingPlan.mountingHoleReports(manufacturing))
					if (report.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Fail) {
						var rule=InsertManufacturingPlan.mountingRule(insert.id,report.holeId);
						var finding=result.validation.filter(check -> check.rule==rule)[0];
						requestDecision(result,"hole-route-"+insert.id+"-"+report.holeId,DesignChangeKind.MountingHoleRoute,finding);
					}
			case Catalogue(_):
		}
		var missingJoint=FoundationJointQualification.missingSource(d.input);
		if (missingJoint!=null) requestDecision(result,"foundation-joint-source",DesignChangeKind.FoundationJointDefinition,missingJoint);

		result.validation = result.validation.concat([
			materia.tooling.ToolingValidation.ToolingReadiness.pending("closure-loads", [d.input.id],
				"Wall/pad clamps, braces and uplift capacity require qualification"),
			materia.tooling.ToolingValidation.ToolingReadiness.pending("materials-process", [d.recipe.id],
				"Adhesive, covering, release and cure compatibility require qualification"),
			materia.tooling.ToolingValidation.ToolingReadiness.pending("precision-interfaces", [d.input.id],
				"Replication and grout interfaces require evidence"),
			materia.tooling.ToolingValidation.ToolingReadiness.pending("anchorage-compaction", [d.input.id], "Anchorage and compaction loads require evidence")
		]);
		if (d.input.inserts.length>0)
			result.validation.push(materia.tooling.ToolingValidation.ToolingReadiness.pending("insert-fixtures",
				[for (i in d.input.inserts) i.id],"Attachment, datum accuracy and thread protection require evidence"));
		result.fabrication = materia.tooling.ToolingValidation.ToolingReadiness.evaluate(result.validation);
		validate(result);
		return result;
	}

	static function build(d:ToolingDocument, o:CastingMoldOptions, geometryChecks:Array<materia.tooling.ToolingValidation>):CastingMoldPlan {
		var e = shellBounds(d), r = d.recipe, t = r.panel.measuredThickness, l = r.minimumLigament;
		var exactRegions=requiresRegions(d);
		if (o.precision.length != d.input.inserts.length)
			throw "Every insert needs a precision process record";
		var strategies = CastingStrategies.resolve(d);
		var reusable = strategies.filter(v -> v.strategy == CoreStrategy.ReusableFormer);
		for (index in 0...reusable.length)
			for (other in index + 1...reusable.length) {
				var a = reusable[index], b = reusable[other];
				if (a.priority == b.priority
					&& MoldRegions.overlaps(MoldRegions.feature(d,a.featureId),MoldRegions.feature(d,b.featureId),1e-7,a.featureId))
					throw "Overlapping reusable formers require distinct priorities: " + a.featureId + ", " + b.featureId;
			}
		var sideEPS=new Map<String,Bool>(),epsVolumes=new Map<String,String>();
		var bodies = new Map<String, MoldBox>(), low = e.z,raisedWalls=RaisedWalls.EastWest;
		for (i in d.input.inserts) {
			var dimensions = InsertBodies.box(i.body, i.id);
			var pose=F.compose(d.setup.sourceToMold,i.pose);
			var b = CastingMoldPlanner.box(dimensions.width, dimensions.depth, dimensions.height,pose);
			var face=F.compose(pose,F.translation(0,0,dimensions.height)),normal=F.transformVector(face,0,0,1);
			// Dry cross-carriers span X. Keep their supporting side walls at
			// the fill rim when source steel sits below the raised-wall lap.
			if (normal.z>1-1e-8 && face.z>=e.z+e.height+l-1e-7 && face.z<e.z+e.height+t/2)
				raisedWalls=RaisedWalls.NorthSouth;
			bodies.set(i.id, b);
			low = Math.min(low, b.z);
			if (o.precision.filter(p -> p.insertId == i.id).length != 1)
				throw "Missing or duplicate insert process record";
		}
		var b:MoldBox = {
			x: e.x,
			y: e.y,
			z: low,
			width: e.width,
			depth: e.depth,
			height: e.z + e.height - low
		};
		var result=seed(d,o,e,b,geometryChecks.concat(InterfaceQualification.source(d.input)).concat(InsertQualifications.manufacturing(d.input.inserts))
				.concat(ReplicationJigs.analyze(d.input,o.replicationJigs,o.precision))
				.concat(FoundationJointQualification.analyze(d.input,o.foundationJointProcesses)),raisedWalls),panels=result.panels,bed=panels.members[0];
		var epsBoxes:Array<MoldBox> = [], insertBoxes:Array<MoldBox> = [];
		var epsRegions:Array<materia.casting.CastingRegion> = [];
		for (i in d.input.inserts) {
			var ib = bodies.get(i.id);
			if (ib == null)
				throw "Missing insert body";
			insertBoxes.push(ib);
		}
		for (f in d.input.features)
			if (f.kind != CastingFeatureKind.InsertRecess) {
				var s = strategies.filter(s -> s.featureId == f.id)[0];
				if (s.strategy == CoreStrategy.SacrificialEPS) {
					var cavity=MoldRegions.feature(d,f.id),boxes=MoldRegions.boxes(cavity);
					var fillWithdrawal=s.withdrawal!=null && s.withdrawal.z>1-1e-8;
					if (boxes==null || boxes.length!=1) {
						var bounds=MoldCores.stockBounds(cavity,f.id);
						var internal=bounds.x>e.x && bounds.x+bounds.width<e.x+e.width
							&& bounds.y>e.y && bounds.y+bounds.depth<e.y+e.depth;
						if (Math.abs(bounds.z+bounds.height-b.z-b.height)>1e-7 || !internal && !fillWithdrawal) {
							if (s.withdrawal==null)
								throw "Side EPS requires an explicit body-opening withdrawal: "+f.id;
							// Exact access allocation below is shared with rigid formers;
							// it retains the full cavity, neck and registration profile.
							sideEPS.set(f.id,true);
							continue;
						}
						for (reserved in [for (box in insertBoxes) MoldRegions.box(box)].concat(epsRegions))
							if (MoldRegions.overlaps(cavity,reserved,1e-7,f.id))
								throw "Intersecting EPS/insert cavities need an explicit allocation: "+f.id;
						var cores=MoldCoreLayers.build(cavity,f.id,r);
						for (core in cores) result.cores.push(core);
						result.features.push({featureId:f.id,kind:EPSFeature,memberIds:[for (core in cores) core.id]});
						epsRegions.push(cavity);
						continue;
					}
					var cb=featureBox(d,f);
					var fillOpen=Math.abs(cb.z+cb.height-b.z-b.height)<=1e-7
						&& (fillWithdrawal || cb.x>e.x && cb.x+cb.width<e.x+e.width
						&& cb.y>e.y && cb.y+cb.depth<e.y+e.depth);
					if (!fillOpen) {
						if (s.withdrawal==null) throw "Side EPS requires an explicit body-opening withdrawal: "+f.id;
						sideEPS.set(f.id,true);
						continue;
					}
					if (BoxAlgebra.volume(BoxAlgebra.subtract([cb], insertBoxes.concat(epsBoxes))) < CastingMoldPlanner.volume(cb) - 1e-5)
						throw "Intersecting EPS/insert cavities need an explicit allocation";
					var core=MoldCoreLayers.box(cb,f.id,r,CoreSupportKind.Bridge),id=core.id;
					result.cores.push(core);
					result.features.push({featureId: f.id, kind: EPSFeature, memberIds: [id]});
					epsBoxes.push(cb);
					epsRegions.push(cavity);
				} else if (s.strategy == CoreStrategy.Unsupported) {
					result.features.push({featureId: f.id, kind: UnsupportedFeature, memberIds: []});
					request(result, "unsupported-" + f.id, ReleaseAlternative, [f.id],
						"No supported tooling route is declared for this feature. Select an available exact source-region former or supported ruled EPS, or revise the source.",
						"tooling.feature.unsupported." + f.id, true);
				}
			}
		result.coreUnits = coreUnits(d.input, result.cores);
		CoreSupports.sharedBridges(result.cavity, r, result.cores, result.supportPanels, result.hardware,raisedWalls);
		// Priority is explicit: inserts, EPS and datum supports are reserved first;
		// rigid formers then resolve by descending authored priority and sorted IDs.
		var ordered = ownershipOrder(d.input.features, strategies);
		// Datum fixtures occupy negative space only; neutral validation checks their resin clearance.
		for (i in d.input.inserts) {
			var dimensions = InsertBodies.box(i.body, i.id);
			var pose = F.compose(d.setup.sourceToMold, i.pose),
				face = F.compose(pose, F.translation(0, 0, dimensions.height)),
				normal = F.transformVector(face, 0, 0, 1),
				body = bodies.get(i.id);
			if (body == null)
				throw "Missing insert body";
			if (normal.z > -1 + 1e-8) {
				var receiver=DatumFixtures.panelOnFace(result.panels.members.concat(result.supportPanels),face);
				if (receiver==null && normal.z>1-1e-8)
					receiver=CoreSupports.insertCarrier(result,i.id,face,body);
				if (receiver==null)
					throw "Insert mounting face needs an accessible receiver panel: "+i.id;
				DatumFixtures.addPanel(result,i,face,receiver);
				result.features.push({
					featureId:d.input.features.filter(f -> f.kind==CastingFeatureKind.InsertRecess && f.owner==i.id)[0].id,
					kind:InsertFeature,memberIds:[materia.tooling.ToolingIds.derive("insert-",i.id)]
				});
				continue;
			}
			var dt = r.hardware.datumThickness, pw = r.hardware.datumPostWidth;
			var pocketId:Null<String> = null;
			var pad:MoldBox = {
				x: body.x,
				y: body.y,
				z: face.z - dt,
				width: body.width,
				depth: body.depth,
				height: dt
			}, pieces = [pad], withdrawal = {
				x: 0.0,
				y: 0.0,
				z: -1.0
			};
			var access = ordered.filter(f -> f.kind != CastingFeatureKind.InsertRecess
				&& strategies.filter(s -> s.featureId == f.id
					&& (s.strategy == CoreStrategy.ReusableFormer || sideEPS.exists(s.featureId))
					&& s.withdrawal != null
					&& Math.abs(s.withdrawal.x) > 0.9)
					.length == 1 && MoldRegions.overlaps(MoldRegions.feature(d,f.id),MoldRegions.box(pad),1e-7,f.id));
			if (access.length > 0) {
				var dir = strategies.filter(s -> s.featureId == access[0].id)[0].withdrawal;
				if (dir == null)
					throw "Missing side fixture direction";
				withdrawal = {x: dir.x, y: dir.y, z: dir.z};
				pad = dir.x > 0 ? {
					x: body.x,
					y: body.y,
					z: face.z - dt,
					width: b.x + b.width + 3 * t - body.x,
					depth: body.depth,
					height: dt
				} : {
					x: b.x - 3 * t,
					y: body.y,
					z: face.z - dt,
					width: body.x + body.width - b.x + 3 * t,
					depth: body.depth,
					height: dt
					};
				pieces = [pad];
			} else {
				var padEntersBed=pad.z<b.z-1e-7;
				if (padEntersBed) {
					var inset = r.routerDiameter / 2 + r.minimumLigament;
					pad = {
						x: body.x + inset,
						y: body.y + inset,
						z: face.z - dt,
						width: body.width - 2 * inset,
						depth: body.depth - 2 * inset,
						height: dt
					};
					if (pad.width <= r.routerDiameter || pad.depth <= r.routerDiameter)
						throw "Insert too small for sealed relieved datum socket: " + i.id;
				}
				pieces=[pad];
				var baseZ=Math.min(pad.z,b.z-dt),height=pad.z-baseZ;
				if (height>1e-8) pieces.push({
					x:padEntersBed ? pad.x : face.x-pw/2,
					y:padEntersBed ? pad.y : face.y-pw/2,
					z:baseZ,width:padEntersBed ? pad.width : pw,
					depth:padEntersBed ? pad.depth : pw,height:height
				});
				// A pad less than one datum thickness above the bed crosses its
				// surface. Its full section, including locator support, needs a socket.
				var socket:MoldBox = padEntersBed ? {
					x: pad.x,
					y: pad.y,
					z: b.z - dt,
					width: pad.width,
					depth: pad.depth,
					height: dt
				} : {
					x: face.x - pw / 2,
					y: face.y - pw / 2,
					z: b.z - dt,
					width: pw,
					depth: pw,
					height: dt
					};
				var op = PanelJoints.operation(materia.tooling.ToolingIds.derive("datum-", i.id), Pocket,
					PanelContours.relievedRectangle(socket.x - bed.pose.x + socket.width / 2, socket.y - bed.pose.y + socket.depth / 2, socket.width,
						socket.depth, r.routerDiameter / 2),
					dt, r.routerDiameter, MachiningSide.PositiveZ, false, r.routerDiameter);
				pocketId = op.id;
				bed.panel.operations.push(op);
				bed.panel.contactOpenings.push(op.id);
				result.openings.push({
					panelId: "bed",
					operationId: op.id,
					kind: ToolingOpening,
					sealingIds: [materia.tooling.ToolingIds.derive("datum-", i.id)],
					protection:null, sealSpec: r.hardware.sealMaterial
				});
			}
			var datum:MoldVolume = {
				id: materia.tooling.ToolingIds.derive("datum-", i.id),
				material: SteelStock,
				cuts: [],
				regions: [],
				profiles: [],
				pieces: BoxAlgebra.subtract(pieces, [])
			};
			DatumFixtures.add(result, i, face, datum, withdrawal, bed, pocketId);
			result.features.push({
				featureId: d.input.features.filter(f -> f.kind == CastingFeatureKind.InsertRecess && f.owner == i.id)[0].id,
				kind: InsertFeature,
				memberIds: [materia.tooling.ToolingIds.derive("insert-", i.id), datum.id]
			});
			result.validation.push(materia.tooling.ToolingValidation.ToolingReadiness.pending("tooling.fixture.attachment." + i.id, [i.id],
				"Bed pins register the nominal support only. Qualify insert XY location/clamping, datum accuracy and release. Screws use declared source threads only."));
		}
		var fixtures:Array<MoldBox> = [];
		for (volume in MoldFixtures.allVolumes(result))
			for (p in volume.pieces)
				fixtures.push(p);
		var occupied = insertBoxes.concat(epsBoxes).concat(fixtures),
			previous:Array<MoldBox> = [];
		var occupiedRegions=[for (box in insertBoxes.concat(fixtures)) MoldRegions.box(box)].concat(epsRegions);
		if (!requiresRegions(d)) {
			var backing = BoxAlgebra.subtract([b], [e].concat(occupied));
			for (group in BoxAlgebra.groups(backing)) {
				var id = "bed-backing-" + result.formers.length;
				result.formers.push({
					volume: {
						id: id,
						material: FormerStock,
						cuts: [],
						regions: [],
						profiles: [],
						pieces: group
					},
					draftDegrees: 0,
					featureIds: [],
					withdrawal: {
						x: 0,
						y: 0,
						z: -1
					}
				});
			}
		}

		var previousRegions:Array<materia.casting.CastingRegion> =[];
		var claims:Array<{id:String,priority:Int,region:materia.casting.CastingRegion}> =[];
		function claim(id:String,priority:Int,region:materia.casting.CastingRegion):Void {
			if (exactRegions) for (earlier in claims)
				if (earlier.priority==priority && MoldRegions.overlaps(earlier.region,region,1e-7,id))
					throw "Overlapping reusable formers require distinct priorities: "+earlier.id+", "+id;
			claims.push({id:id,priority:priority,region:region});
		}

		for (f in ordered)
			if (f.kind != CastingFeatureKind.InsertRecess) {
				var s = strategies.filter(s -> s.featureId == f.id)[0];
				if (s.strategy != CoreStrategy.ReusableFormer && !sideEPS.exists(f.id)) continue;
				var sourceRegion=MoldRegions.feature(d,f.id);
				var sourceBoxes=MoldRegions.boxes(sourceRegion);
				if (sourceBoxes==null || sourceBoxes.length!=1
					|| sideEPS.exists(f.id) && s.withdrawal!=null && Math.abs(s.withdrawal.x)<1-1e-7) {
					var pull=s.withdrawal;
					if (pull==null || !MoldFormerAccess.supported(d,f,pull))
						throw "Non-box former needs a supported withdrawal: "+f.id;
					var allocated=materia.casting.CastingRegion.Difference(sourceRegion,
						occupiedRegions.concat(previousRegions));
					var ids:Array<String> =[],blank=sourceRegion;
					if (MoldRegions.volume(allocated,f.id)>1e-7) {
						var id=materia.tooling.ToolingIds.derive("former-",f.id,"-0");
						var volume:MoldVolume={id:id,material:FormerStock,pieces:[],profiles:[],cuts:[],regions:[allocated]};
						var exclusions=occupiedRegions.concat(previousRegions);
						var extension=sideEPS.exists(f.id) && pull.z< -1+1e-7
							? MoldFormerAccess.registeredBedAccess(result,f,volume,exclusions)
							: MoldFormerAccess.add(result,f,pull,volume,exclusions);
						blank=materia.casting.CastingRegion.Union([sourceRegion].concat(extension));
						claim(f.id,s.priority,blank);
						result.formers.push({volume:volume,featureIds:[f.id],withdrawal:pull,draftDegrees:f.draft.degrees});
						if (sideEPS.exists(f.id)) epsVolumes.set(id,f.id);
						ids.push(id);
					}
					result.features.push({featureId:f.id,kind:FormerFeature,memberIds:ids});
					previousRegions.push(blank);
					continue;
				}
				var cb = featureBox(d, f);
				if (cb == null)
					throw "Missing former intersection";
				var direction = s.withdrawal;
				if (direction == null)
					throw "Former needs a withdrawal";
				var add = [cb];
				var profiles:Array<materia.tooling.casting.CastingMoldPlan.MoldVolumeProfile> = [];
				if (Math.abs(direction.x) > 0.9) {
					var side = direction.x > 0 ? e.x + e.width : e.x,
						wall = panels.members.filter(m -> m.panel.id == (direction.x > 0 ? "east" : "west"))[0];
					var extended = BoxAlgebra.copy(cb);
					if (direction.x > 0)
						extended.width = b.x + b.width + t - cb.x;
					else {
						extended.x = b.x - t;
						extended.width = cb.x + cb.width - extended.x;
					}
					add = [extended];
					var flange:MoldBox = {
						x: direction.x > 0 ? side + t : side - t - r.hardware.formerFlangeThickness,
						y: cb.y - r.hardware.formerFlangeMargin,
						z: cb.z - r.hardware.formerFlangeMargin,
						width: r.hardware.formerFlangeThickness,
						depth: cb.depth + 2 * r.hardware.formerFlangeMargin,
						height: cb.height + 2 * r.hardware.formerFlangeMargin
					};
					add.push(flange);
					var inverse = F.inverse(wall.pose),
						a = F.transformPoint(inverse, side, cb.y, cb.z),
						c = F.transformPoint(inverse, side, cb.y + cb.depth, cb.z + cb.height);
					var x = Math.min(a.x, c.x), y = Math.min(a.y, c.y);
					var op = PanelJoints.operation(materia.tooling.ToolingIds.derive("access-", f.id), InsideProfile,
						PanelContours.relievedRectangle((a.x + c.x) / 2, (a.y + c.y) / 2, Math.abs(c.x - a.x), Math.abs(c.y - a.y), r.routerDiameter / 2), t,
						r.routerDiameter, MachiningSide.PositiveZ, true, r.routerDiameter);
					profiles.push({
						panelId: wall.panel.id,
						operationId: op.id,
						contour: op.contour,
						pose: wall.pose,
						depth: t,
						subtract: occupiedRegions.concat([for (piece in previous) MoldRegions.box(piece)])
					});
					wall.panel.operations.push(op);
					wall.panel.contactOpenings = [for (id in wall.panel.contactOpenings) id].concat([op.id]);
					result.openings.push({
						panelId: wall.panel.id,
						operationId: op.id,
						kind: ToolingOpening,
						sealingIds: [],
						protection:null, sealSpec: r.hardware.sealMaterial
					});
				}
				var blank=materia.casting.CastingRegion.Union([for (piece in add) MoldRegions.box(piece)]
					.concat([for (profile in profiles) MoldRegions.profile(profile)]));
				claim(f.id,s.priority,blank);
				var pieces = BoxAlgebra.subtract(add, occupied.concat(previous)),
					ids:Array<String> = [],
					groups = BoxAlgebra.groups(pieces);
				if (profiles.length > 0 && groups.length != 1)
					throw "Access former needs an explicit split for its sealing profile: " + f.id;
				for (index in 0...groups.length) {
					var remaining=exactRegions ? materia.casting.CastingRegion.Difference(
						materia.casting.CastingRegion.Union([for (piece in groups[index]) MoldRegions.box(piece)]),epsRegions.concat(previousRegions)) : null;
					if (remaining!=null && MoldRegions.volume(remaining,f.id)<=1e-7) continue;
					var id = materia.tooling.ToolingIds.derive("former-", f.id, "-" + index);
					if (sideEPS.exists(f.id)) epsVolumes.set(id,f.id);
					ids.push(id);
					result.formers.push({
						volume: {
							id: id,
							material: FormerStock,
							cuts: [],
							regions: remaining==null ? [] : [remaining],
							profiles: profiles,
							pieces: remaining==null ? groups[index] : []
						},
						draftDegrees: f.draft.degrees,
						featureIds: [f.id],
						withdrawal: {
							x: direction.x,
							y: direction.y,
							z: direction.z
						}
					});
				}
				if (ids.length == 0) {
					// A feature wholly consumed by earlier tooling still keeps explicit ownership.
					ids = [
						for (former in result.formers)
							if (former.volume.pieces.filter(p -> BoxAlgebra.intersection(p, cb) != null).length > 0) former.volume.id
					];
					// Final ownership below also includes reserved EPS, inserts and supports.
				}
				result.features.push({featureId: f.id, kind: FormerFeature, memberIds: ids});
				for (p in pieces)
					previous.push(p);
				previousRegions.push(blank);
				for (open in result.openings)
					if (open.operationId == materia.tooling.ToolingIds.derive("access-", f.id)) {
						open.sealingIds = ids.copy();
						for (volume in MoldFixtures.allVolumes(result))
							if (volume.pieces.filter(p -> BoxAlgebra.intersection(p, cb) != null).length > 0)
								open.sealingIds.push(volume.id);
					}
			}
		for (m in panels.members)
			m.panel = PanelCodec.copy(m.panel);
		for (feature in result.features)
			if (feature.kind == FormerFeature) {
				var source = d.input.features.filter(f -> f.id == feature.featureId)[0],
					cavity=MoldRegions.feature(d,source.id);
				feature.memberIds = [];
				for (former in result.formers)
					if (MoldRegions.overlaps(formerRegion(former),cavity,1e-7,source.id)) {
						feature.memberIds.push(former.volume.id);
						if (former.featureIds.indexOf(source.id) < 0)
							former.featureIds.push(source.id);
					}
				for (core in result.cores)
					if (MoldRegions.overlaps(core.cavity,cavity,1e-7,source.id))
						feature.memberIds.push(core.id);
				for (volume in MoldFixtures.allVolumes(result))
					if (volume.pieces.filter(v -> MoldRegions.overlaps(MoldRegions.box(v),cavity,1e-7,source.id)).length>0)
						feature.memberIds.push(volume.id);
				for (insert in d.input.inserts)
					if (MoldRegions.overlaps(MoldRegions.box(bodies.get(insert.id)),cavity,1e-7,source.id))
						feature.memberIds.push(materia.tooling.ToolingIds.derive("insert-", insert.id));
				feature.memberIds.sort(Reflect.compare);
				if (feature.memberIds.length == 0) {
					feature.kind = UnsupportedFeature;
					request(result, "unallocated-" + source.id, ReleaseAlternative, [source.id],
						"Feature has no supported physical tooling owner; choose an explicit alternative.", "tooling.feature.unsupported." + source.id, true);
				}
			}
		var converted=new Map<String,String>();
		for (former in result.formers) {
			// Shared cavity attribution is not construction ownership. A rigid
			// higher-priority tool may also bound a neighbouring EPS feature.
			var owner=epsVolumes.get(former.volume.id);
			if (owner==null) continue;
			var wall=result.openings.filter(open -> open.sealingIds.indexOf(former.volume.id)>=0);
			if (wall.length!=1) throw "Side EPS needs one registered access panel: "+owner
				+" ("+wall.length+": "+[for (opening in wall) opening.panelId+"/"+opening.operationId].join(", ")+")";
			var id=materia.tooling.ToolingIds.derive("core-",former.volume.id);
			var direction=former.withdrawal;
			// The profile frame is a neutral datum. Whole-band extrusion proofs
			// qualify its wire sections; it is not a sampled motion check.
			var normal={x:-direction.x,y:-direction.y,z:-direction.z};
			var axis=materia.casting.CastingBounds.axis(normal);
			if (axis==null) throw "EPS access former needs an axial stock frame: "+owner;
			var rotation=axis==0 ? (normal.x>0 ? [0.0,0,1,1,0,0,0,1,0] : [0.0,0,-1,-1,0,0,0,1,0])
				: axis==1 ? (normal.y>0 ? [1.0,0,0,0,0,1,0,-1,0] : [1.0,0,0,0,0,-1,0,1,0])
				: normal.z>0 ? [1.0,0,0,0,1,0,0,0,1] : [1.0,0,0,0,-1,0,0,0,-1];
			var stockFrame=F.fromRotationMatrix(0,0,0,rotation);
			var core=MoldCoreLayers.registeredFlange(MoldRegions.tooling(former.volume),owner,id,r,stockFrame,wall[0].panelId);
			core.support.registrationProfiles=former.volume.profiles;
			result.cores.push(core);
			converted.set(former.volume.id,id);
		}
		for (feature in result.features) {
			feature.memberIds=[for (id in feature.memberIds) converted.exists(id) ? converted.get(id) : id];
			if (sideEPS.exists(feature.featureId)) feature.kind=EPSFeature;
		}
		for (open in result.openings)
			open.sealingIds=[for (id in open.sealingIds) converted.exists(id) ? converted.get(id) : id];
		result.formers=result.formers.filter(former -> !converted.exists(former.volume.id));
		if (requiresRegions(d)) {
			// Allocate source access necks before envelope fillers, so an exact
			// bed/side access is never occupied twice by backing and a core.
			var reserved=occupiedRegions.concat([for (core in result.cores) core.cavity])
				.concat([for (former in result.formers) formerRegion(former)]);
			addEnvelopeFormers(result,materia.casting.CastingRegion.Difference(MoldRegions.box(b),
				[MoldRegions.envelope(d)].concat(reserved)));
		}
		splitBoardContacts(result);
		result.coreUnits=coreUnits(d.input,result.cores);
		return finish(result,strategies);
	}

	/** Square source steel and datum contacts must remain square. A ring cut
	 * around them is made from separate rectangular boards, rather than routing
	 * an impossible inside corner or changing the mating source geometry.
	 * BoxAlgebra supplies exact disjoint pieces; their joints require sealing.
	 */
	static function splitBoardContacts(plan:CastingMoldPlan):Void {
		var replacements=new Map<String,Array<String>>(),formers:Array<MoldFormer>=[];
		for (former in plan.formers) {
			var volume=former.volume;
			if (former.draftDegrees!=0 || volume.pieces.length<=1 || volume.regions.length>0
				|| volume.profiles.length>0 || volume.cuts.length>0) {
				formers.push(former);
				continue;
			}
			var bounds=MoldCores.stockBounds(MoldRegions.tooling(volume),volume.id);
			var direction=former.withdrawal;
			var height=Math.abs(direction.x)*bounds.width+Math.abs(direction.y)*bounds.depth+Math.abs(direction.z)*bounds.height;
			var thickest=0.0;
			for (stock in plan.document.recipe.former.stocks) thickest=Math.max(thickest,stock.measuredThickness);
			if (height>thickest+1e-7) {
				// Tall formers retain their shared screwed panel-box construction.
				formers.push(former);
				continue;
			}
			var ids:Array<String>=[];
			for (index in 0...volume.pieces.length) {
				var id=materia.tooling.ToolingIds.derive("",volume.id,"-board-"+index);
				ids.push(id);
				formers.push({volume:{id:id,material:volume.material,pieces:[volume.pieces[index]],
					regions:[],profiles:[],cuts:[]},featureIds:former.featureIds.copy(),
					withdrawal:former.withdrawal,draftDegrees:former.draftDegrees});
			}
			replacements.set(volume.id,ids);
		}
		function expand(ids:Array<String>):Array<String> {
			var result:Array<String>=[];
			for (id in ids) result=result.concat(replacements.exists(id) ? replacements.get(id) : [id]);
			result.sort(Reflect.compare);
			return result;
		}
		for (feature in plan.features) feature.memberIds=expand(feature.memberIds);
		for (opening in plan.openings) opening.sealingIds=expand(opening.sealingIds);
		plan.formers=formers;
	}

	/** Complement of the actual source body, never its bounding-box stand-in.
	 * Separate section islands become separate removable physical fillers.
	 */
	static function addEnvelopeFormers(plan:CastingMoldPlan,outside:materia.casting.CastingRegion):Void {
		if (MoldRegions.volume(outside,plan.document.input.id)<=1e-7) return;
		var boxes=MoldRegions.boxes(outside);
		var regions:Array<materia.casting.CastingRegion>=[];
		if (boxes!=null) {
			for (group in BoxAlgebra.groups(boxes))
				regions.push(materia.casting.CastingRegion.Union([for (piece in group) MoldRegions.box(piece)]));
		} else {
			if (geometry==null) throw "Exact envelope tooling kernel required: "+plan.document.input.id;
			var section:Null<materia.casting.CastingSection.CastingPrismSection>=null;
			for (axis in [{x:1.0,y:0.0,z:0.0},{x:0.0,y:1.0,z:0.0},{x:0.0,y:0.0,z:1.0}]) {
				section=geometry.prismaticSection(outside,axis,1e-5,plan.document.input.id);
				if (section!=null) break;
			}
			var sections:Array<materia.casting.CastingSection.CastingPrismSection>=[];
			if (section!=null) sections.push(section);
			else {
				var bounds=MoldRegions.exactBounds(outside,plan.document.input.id);
				var origin=F.translation(bounds.x,bounds.y,bounds.z);
				var bands=geometry.prismaticBands(MoldRegions.moved(outside,F.inverse(origin)),
					bounds.width,bounds.depth,bounds.height,plan.document.input.id);
				for (band in bands) sections.push({contours:band.contours,depth:band.topMm-band.bottomMm,
					pose:F.compose(origin,F.translation(0,0,band.bottomMm))});
			}
			for (slice in sections) for (outer in slice.contours) if (outer.depth%2==0) {
				var contours=[outer];
				for (hole in slice.contours) if (hole.depth==outer.depth+1
					&& CastingProfileGeometry.inside(outer.contour,hole.contour[0].start.x,hole.contour[0].start.y))
					contours.push(hole);
				regions.push(MoldRegions.section(contours,slice.pose,slice.depth));
			}
		}
		regions.sort((a,b) -> Reflect.compare(ToolingCodec.canonical(a),ToolingCodec.canonical(b)));
		for (index in 0...regions.length) {
			var region=regions[index],bounds=MoldRegions.exactBounds(region,plan.document.input.id);
			var shell=plan.cavity,body=plan.envelope;
			var direction={x:0.0,y:0.0,z:0.0};
			if (bounds.z+bounds.height<=body.z+1e-7) direction.z=-1;
			else if (Math.abs(bounds.x-shell.x)+Math.abs(bounds.width-shell.width)
				+Math.abs(bounds.y-shell.y)+Math.abs(bounds.depth-shell.depth)<1e-7
				&& Math.abs(bounds.z+bounds.height-shell.z-shell.height)<1e-7) direction.z=1;
			else if (Math.abs(bounds.x-shell.x)<1e-7) direction.x=-1;
			else if (Math.abs(bounds.x+bounds.width-shell.x-shell.width)<1e-7) direction.x=1;
			else if (Math.abs(bounds.y-shell.y)<1e-7) direction.y=-1;
			else if (Math.abs(bounds.y+bounds.depth-shell.y-shell.depth)<1e-7) direction.y=1;
			else throw "Envelope filler has no exterior withdrawal: "+plan.document.input.id;
			var id=materia.tooling.ToolingIds.derive("envelope-former-",plan.document.input.id,"-"+index);
			var pieces=MoldRegions.boxes(region);
			plan.formers.push({volume:{id:id,material:FormerStock,pieces:pieces==null ? [] : pieces,
				regions:pieces==null ? [region] : [],profiles:[],cuts:[]},
				featureIds:[],draftDegrees:0,withdrawal:direction});
		}
	}

	/** Ownership ordering accepts raw caller order, before codec canonicalisation. */
	public static function ownershipOrder(features:Array<materia.casting.CastingFeature>,
			strategies:Array<materia.tooling.casting.CastingSetup.CastingFeatureStrategy>):Array<materia.casting.CastingFeature> {
		var ordered = features.copy();
		ordered.sort((a, b) -> {
			var aa = strategies.filter(v -> v.featureId == a.id),
				bb = strategies.filter(v -> v.featureId == b.id);
			var ap = aa.length == 0 ? 0 : aa[0].priority,
				bp = bb.length == 0 ? 0 : bb[0].priority;
			return ap == bp ? Reflect.compare(a.id, b.id) : Reflect.compare(bp, ap);
		});
		return ordered;
	}

	public static function coreUnits(input:CastingToolingInput, cores:Array<CastingMoldCore>):Array<CastingCoreUnit> {
		var groups = new Map<String, CastingCoreUnit>();
		for (core in cores) {
			var sources = input.features.filter(feature -> feature.id == core.featureId);
			if (sources.length != 1)
				throw "Core has no unique source feature: " + core.id;
			var source = sources[0], parent = source.derivedFrom == null ? source.id : source.derivedFrom;
			var group = groups.get(parent);
			if (group == null) {
				group = {id: materia.tooling.ToolingIds.derive("core-unit-", parent), parentFeatureId: parent, featureIds: [], coreIds: []};
				groups.set(parent, group);
			}
			group.featureIds.push(source.id);
			group.coreIds.push(core.id);
		}
		var result = [for (group in groups) group];
		for (group in result) {
			group.featureIds.sort(Reflect.compare);
			group.coreIds.sort(Reflect.compare);
		}
		result.sort((a, b) -> Reflect.compare(a.id, b.id));
		return result;
	}

	static function request(p:CastingMoldPlan, id:String, kind:DesignChangeKind, ids:Array<String>, message:String, rule:String,
			unsupported:Bool = false):Void {
		var check = materia.tooling.ToolingValidation.ToolingReadiness.pending(rule, ids, message);
		if (unsupported) {
			check.status = materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported;
			check.severity = materia.tooling.ToolingDiagnostic.ToolingSeverity.Error;
		}
		requestDecision(p,id,kind,check);
	}

	static function requestDecision(p:CastingMoldPlan,id:String,kind:DesignChangeKind,check:materia.tooling.ToolingValidation):Void {
		p.changeRequests.push({id:materia.tooling.ToolingIds.derive("",id),kind:kind,
			sourceIds:check.affectedIds.copy(),reason:check.message,blockingChecks:[check.rule]});
		var existing=p.validation.filter(item -> item.rule==check.rule);
		if (existing.length==0) p.validation.push(check);
		else if (existing.length!=1 || ToolingCodec.canonical(existing[0])!=ToolingCodec.canonical(check))
			throw "Conflicting design-change validation: "+check.rule;
	}

	public static function resinAtFill(document:ToolingDocument):Array<MoldBox> {
		var body=MoldRegions.boxes(MoldRegions.envelope(document));
		if (body==null) throw "Box resin query does not support source envelope: "+document.input.id;
		var wetted:Array<MoldBox>=[];
		for (piece in body) {
			var clipped=BoxAlgebra.copy(piece);
			clipped.height=Math.min(clipped.height,document.setup.fillHeightMm-clipped.z);
			if (clipped.height>0) wetted.push(clipped);
		}
		return BoxAlgebra.subtract(wetted,[for (feature in document.input.features) featureBox(document,feature)]);
	}

	public static function validate(p:CastingMoldPlan):Void {
		if (ToolingCodec.canonical(p.coreUnits)!=ToolingCodec.canonical(coreUnits(p.document.input,p.cores)))
			throw "Core units disagree with source feature lineage";
		switch p.construction {
			case SinglePocket:validateSinglePocket(p);return;
			case SharedBridges:
		}

		FoundationJointQualification.validate(p.document.input,p.options.foundationJointProcesses,p.validation);
		ReplicationJigs.validate(p.document.input,p.options.replicationJigs,p.options.precision,p.validation);
		InterfaceQualification.validate(p.document.input,p.validation);
		InsertQualifications.validateManufacturing(p.document.input.inserts,p.validation);
		InsertQualifications.validateAnchorage(p.document.input.inserts,p.validation);
		if (ToolingCodec.canonical(p.fabrication) != ToolingCodec.canonical(materia.tooling.ToolingValidation.ToolingReadiness.evaluate(p.validation)))
			throw "Stale fabrication readiness";
		if (Math.abs(p.document.setup.fillHeightMm - p.cavity.z - p.cavity.height) > 1e-7 || p.document.setup.freeboardMm != 0)
			throw "Box template needs full-height fill and zero freeboard";
		if (ToolingCodec.documentFingerprint(p.document) != p.sourceFingerprint)
			throw "Stale multi-core mold source";
		materia.tooling.panel.PanelAssemblyPlan.validate(p.panels);
		if (p.supportPanels.length > 0)
			materia.tooling.panel.PanelAssemblyPlan.validate({
				id: "shared-bridges",
				members: p.supportPanels,
				joints: []
			});
		MoldOptions.copy(p.options);
		if (ToolingCodec.canonical(p.handling) != ToolingCodec.canonical(CastHandling.plan(p.document, p.options)))
			throw "Stale source-derived handling plan";
		DatumFixtures.validate(p);
		var members = new Map<String, Bool>();
		function member(id:String):Void {
			materia.tooling.panel.PanelCodec.identifier(id);
			if (members.exists(id))
				throw "Duplicate mold member";
			members.set(id, true);
		}
		for (m in p.panels.members.concat(p.supportPanels))
			member(m.panel.id);
		for (c in p.cores) {
			member(c.id);
			if (c.layers.length == 0 || c.layers.length > 32)
				throw "Invalid EPS layers";
		}
		for (f in p.formers)
			member(f.volume.id);
		for (volume in MoldFixtures.allVolumes(p))
			member(volume.id);
		for (i in p.document.input.inserts)
			member(materia.tooling.ToolingIds.derive("insert-", i.id));
		var ruleIds = [for (v in p.validation) v.rule];
		for (strategy in CastingStrategies.resolve(p.document))
			if (strategy.strategy == CoreStrategy.ReusableFormer
				&& p.document.input.features.filter(feature -> feature.id == strategy.featureId)[0].draft.degrees == 0
				&& ruleIds.indexOf("tooling.former.draft." + strategy.featureId) < 0)
				throw "Zero-draft former needs a release validation";
		for (former in p.formers)
			if (former.featureIds.length == 0
				&& former.draftDegrees == 0
				&& ruleIds.indexOf("tooling.former.draft." + former.volume.id) < 0)
				throw "Zero-draft bed-backing former needs a release validation";
		for (seat in p.options.precision) {
			var source = p.document.input.inserts.filter(i -> i.id == seat.insertId)[0];
			var replication=InterfaceQualification.replicationRequirement(source,seat);
			var replicationChecks=p.validation.filter(check -> check.rule=="tooling.precision.replication."+source.id);
			if (replication==null ? replicationChecks.length!=0 : replicationChecks.length!=1
				|| ToolingCodec.canonical(replicationChecks[0])!=ToolingCodec.canonical(replication))
				throw "Replication requirement differs from source interfaces and process choices";
			var stock=InterfaceQualification.finishingStock(p.document.input,source);
			var decisions=p.validation.filter(decision -> decision.rule==stock.rule);
			if (decisions.length!=1 || ToolingCodec.canonical(decisions[0])!=ToolingCodec.canonical(stock))
				throw "Finishing stock decision differs from source geometry";
			var requests=p.changeRequests.filter(request -> request.id=="allowance-"+seat.insertId);
			if (stock.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass ? requests.length!=0
				: requests.length!=1 || requests[0].blockingChecks.indexOf(stock.rule)<0)
				throw "Finishing stock change request differs from source decision";
		}
		for (request in p.changeRequests)
			for (rule in request.blockingChecks)
				if (ruleIds.indexOf(rule) < 0)
					throw "Design change references unknown validation rule: " + rule;
		for (feature in p.features)
			if (feature.kind == UnsupportedFeature
				&& p.validation.filter(v -> v.status == "unsupported" && v.affectedIds.indexOf(feature.featureId) >= 0).length == 0)
				throw "Unsupported feature needs blocking validation";
		var ids = new Map<String, Bool>();
		for (f in p.features) {
			for (id in f.memberIds)
				if (!members.exists(id))
					throw "Unknown feature tooling member";
			if (ids.exists(f.featureId) || (f.memberIds.length == 0 && f.kind != UnsupportedFeature))
				throw "Invalid feature ownership";
			ids.set(f.featureId, true);
		}
		for (f in p.document.input.features)
			if (!ids.exists(f.id))
				throw "Unclassified source feature: " + f.id;
		if (p.features.length != p.document.input.features.length)
			throw "Unexpected feature classification";
		var seen = new Map<String, Bool>();
		for (o in p.openings) {
			var key = o.panelId + "/" + o.operationId;
			if (seen.exists(key))
				throw "Duplicate mold opening";
			seen.set(key, true);
			if (o.protection!=null && o.kind!=ToolingOpening)
				throw "Protected opening must be tooling-sealed";
			if (o.kind == ToolingOpening && o.sealingIds.length == 0)
				throw "Unsealed tooling opening";
			for (id in o.sealingIds)
				if (!members.exists(id))
					throw "Unknown sealing member";
		}
		for (m in p.panels.members)
			for (id in m.panel.contactOpenings)
				if (!seen.exists(m.panel.id + "/" + id))
					throw "Unclassified mold opening";
		var sealingProfiles:Array<MoldVolumeProfile>=[];
		for (f in p.formers) {
			MoldVolumeRecords.copy(f.volume);
			sealingProfiles=sealingProfiles.concat(f.volume.profiles);
		}
		for (core in p.cores) {
			if (core.support.kind==CoreSupportKind.RegisteredFlange && core.support.registrationProfiles.length!=1)
				throw "Registered EPS needs one exact wall sealing profile: "+core.id;
			sealingProfiles=sealingProfiles.concat(core.support.registrationProfiles);
		}
		for (profile in sealingProfiles) {
			var panels=p.panels.members.filter(m -> m.panel.id==profile.panelId);
			if (panels.length!=1) throw "Unknown former sealing panel";
			var operations=panels[0].panel.operations.filter(o -> o.id==profile.operationId);
			if (operations.length!=1
				|| ToolingCodec.canonical(operations[0].contour)!=ToolingCodec.canonical(profile.contour)
				|| ToolingCodec.canonical(panels[0].pose)!=ToolingCodec.canonical(profile.pose)
				|| profile.depth!=panels[0].panel.stock.measuredThickness)
				throw "Former sealing profile differs from panel operation";
		}
		var exactResin=requiresRegions(p.document);
		var resin = p.fixtures.length==0 || exactResin ? [] : resinAtFill(p.document);
		for (volume in MoldFixtures.allVolumes(p)) {
			MoldVolumeRecords.copy(volume);
			if (volume.profiles.length > 0 || volume.regions.length > 0)
				throw "Neutral datum/resin validation does not support datum profiles or regions";
			if (exactResin) {
				if (geometry==null) throw "Exact tooling geometry kernel required: "+volume.id;
				var region=materia.casting.CastingRegion.Union([for (piece in volume.pieces) MoldRegions.box(piece)]);
				if (geometry.resinIntersection(p.document,region)>1e-7)
					throw "Datum fixture intersects resin: "+volume.id;
			}
			for (piece in volume.pieces)
				for (resinPiece in resin)
					if (BoxAlgebra.intersection(piece, resinPiece) != null)
						throw "Datum fixture intersects resin: " + volume.id;
		}
	}
	/** Single-pocket construction: the individual bridge or routed bed socket. */


	public static function box(width:Float, depth:Float, height:Float, pose:AssemblyFrame):MoldBox {
		var points = [];
		for (x in [-width / 2, width / 2])
			for (y in [-depth / 2, depth / 2])
				for (z in [0.0, height])
					points.push(F.transformPoint(pose, x, y, z));
		var lo = {x: 1e30, y: 1e30, z: 1e30},
			hi = {x: -1e30, y: -1e30, z: -1e30};
		for (p in points) {
			lo.x = Math.min(lo.x, p.x);
			lo.y = Math.min(lo.y, p.y);
			lo.z = Math.min(lo.z, p.z);
			hi.x = Math.max(hi.x, p.x);
			hi.y = Math.max(hi.y, p.y);
			hi.z = Math.max(hi.z, p.z);
		}
		for (p in points)
			if (Math.min(Math.abs(p.x - lo.x), Math.abs(p.x - hi.x)) > 1e-7
				|| Math.min(Math.abs(p.y - lo.y), Math.abs(p.y - hi.y)) > 1e-7
				|| Math.min(Math.abs(p.z - lo.z), Math.abs(p.z - hi.z)) > 1e-7)
				throw "Box mold needs axis-aligned setup geometry";
		return {
			x: lo.x,
			y: lo.y,
			z: lo.z,
			width: hi.x - lo.x,
			depth: hi.y - lo.y,
			height: hi.z - lo.z
		};
	}

	public static function volume(b:MoldBox):Float
		return b.width * b.depth * b.height;

	public static function clipped(a:MoldBox, b:MoldBox):MoldBox {
		var x = Math.max(a.x, b.x),
			y = Math.max(a.y, b.y),
			z = Math.max(a.z, b.z);
		var out = {
			x: x,
			y: y,
			z: z,
			width: Math.min(a.x + a.width, b.x + b.width) - x,
			depth: Math.min(a.y + a.depth, b.y + b.depth) - y,
			height: Math.min(a.z + a.height, b.z + b.height) - z
		};
		if (out.width <= 0 || out.depth <= 0 || out.height <= 0)
			throw "Feature misses casting envelope";
		return out;
	}

	static function singlePocketOptions(d:ToolingDocument):CastingMoldOptions return {
		foundationJointProcesses:[],replicationJigs:[],precision:[for (insert in d.input.inserts) {
			insertId:insert.id,jigReference:null,masterReference:null,masterFlatnessMm:null,
			releaseSpec:null,holeProtectionReference:null,evidence:[]}],
		compaction:{method:Undeclared,bedInterfaceReference:null,frequencyHz:null,amplitudeMm:null,durationSeconds:null,evidence:[]},
		serviceOrientation:F.identity(),overlapRule:ReservedThenPriority,liftingEvidence:[]
	};

	public static function planSinglePocket(input:ToolingDocument):CastingMoldPlan {
		var d=ToolingCodec.copy(input),r=d.recipe;
		for (insert in d.input.inserts) switch insert.anchorage {
			case Bolted(_,_):throw "Unsupported non-box anchor occupancy: "+insert.id;
			default:
		}
		var b=envelope(d),t=r.panel.measuredThickness,c=r.jointClearance,seat=t/2;
		if (seat < r.minimumLigament)
			throw "Bed sockets leave insufficient stock";
		var result=seed(d,singlePocketOptions(d),b,b,[],RaisedWalls.NorthSouth);
		result.id="single-pocket-box";
		result.construction=SinglePocket;
		var assembly=result.panels,penetrations=MoldOpenings.records(result);
		var supportPanels=result.supportPanels,cores=result.cores;
		for (f in d.input.features)
			if (f.kind != CastingFeatureKind.InsertRecess) {
				var strategy = CastingStrategies.resolve(d).filter(s -> s.featureId == f.id)[0];
				if (strategy.strategy != CoreStrategy.SacrificialEPS)
					throw "Box template supports EPS only";
				var cb = featureBox(d,f);
				if (cb.x <= b.x || cb.y <= b.y || cb.x + cb.width >= b.x + b.width || cb.y + cb.depth >= b.y + b.depth)
					throw "EPS core needs an internal pocket accessible through fill face";
				cores.push(MoldCoreLayers.box(cb,f.id,r,CoreSupports.kind(cb,b),true));
			}
		if (cores.length != 1 || d.input.inserts.length != 1)
			throw "Single-pocket template needs one core and one insert";
		CoreSupports.build(cores[0], b, r, assembly.members[0], supportPanels, penetrations);
		var insert = d.input.inserts[0],
			dimensions = InsertBodies.box(insert.body, insert.id),
			ip = F.compose(d.setup.sourceToMold, insert.pose),
			ib = box(dimensions.width, dimensions.depth, dimensions.height, ip),
			face = F.compose(ip, F.translation(0, 0, dimensions.height)),
			normal = F.transformVector(face, 0, 0, 1);
		if (Math.abs(face.z - b.z) > 1e-7
			|| normal.z > -1 + 1e-8
			|| Math.abs(volume(clipped(ib, b)) - volume(ib)) > 1e-5)
			throw "Insert mounting face must seat on bed with fixing bore";
		result.openings=[for (record in penetrations) MoldOpenings.fromRecord(record)];
		for (core in cores) for (hardware in core.support.hardware) result.hardware.push(hardware);
		DatumFixtures.addPanel(result,insert,face,assembly.members[0]);
		var fixtureChecks=result.validation.copy();
		result.coreUnits=coreUnits(d.input,cores);
		result.features=[for (core in cores) {featureId:core.featureId,kind:EPSFeature,memberIds:[core.id]}];
		for (feature in d.input.features) if (feature.kind==CastingFeatureKind.InsertRecess) {
			var owner=feature.owner;
			if (owner==null) throw "Insert recess needs its source owner: "+feature.id;
			result.features.push({featureId:feature.id,kind:InsertFeature,memberIds:[owner]});
		}
		result.validation= [
				materia.tooling.ToolingValidation.ToolingReadiness.pending("core-restraint", [cores[0].id],
					"Adhesive, pad attachment and uplift capacity require qualification"),
				materia.tooling.ToolingValidation.ToolingReadiness.pending("insert-datums", [insert.id],
					"Independent datum accuracy and thread engagement require qualification"),
				materia.tooling.ToolingValidation.ToolingReadiness.pending("anchorage-finishing", [insert.id],
					"Anchorage and precision finishing require evidence"),
				materia.tooling.ToolingValidation.ToolingReadiness.pending("materials-process", [d.recipe.id],
					"Seals, resin/EPS compatibility and cure require evidence"),
				materia.tooling.ToolingValidation.ToolingReadiness.pending("loads-handling", [d.input.id],
					"Compaction, closure loads and handling require qualification")
			];
		result.validation=result.validation.concat(fixtureChecks);
		result.validation = result.validation.concat(InterfaceQualification.source(d.input))
			.concat(FoundationJointQualification.source(d.input));
		if (F.transformVector(d.setup.sourceToMold, 0, 0, 1).z < 0)
			result.changeRequests.push({
				id: "lifting-flip",
				kind: materia.tooling.casting.CastingMoldPlan.DesignChangeKind.LiftingPoints,
				sourceIds: [d.input.id],
				reason: "Provide qualified lifting points and handling for the inverted casting",
				blockingChecks: ["loads-handling"]
			});
		result.validation = result.validation.concat(InsertQualifications.check(d.input.inserts));
		result.validation = result.validation.concat(InsertQualifications.manufacturing(d.input.inserts));
		for (source in d.input.inserts) {
			var stock=InterfaceQualification.finishingStock(d.input,source);
			result.validation.push(stock);
			if (stock.status!=materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass)
				result.changeRequests.push({id:"allowance-"+source.id,
					kind:materia.tooling.casting.CastingMoldPlan.DesignChangeKind.InterfaceAllowance,
					sourceIds:[source.id],reason:stock.message,blockingChecks:[stock.rule]});
		}
		result.fabrication = materia.tooling.ToolingValidation.ToolingReadiness.evaluate(result.validation);
		validate(result);
		return result;
	}


	static function validateSinglePocket(plan:CastingMoldPlan):Void {
		InterfaceQualification.validate(plan.document.input,plan.validation);
		FoundationJointQualification.validate(plan.document.input,[],plan.validation);
		InsertQualifications.validateManufacturing(plan.document.input.inserts,plan.validation);
		InsertQualifications.validateAnchorage(plan.document.input.inserts,plan.validation);
		for (source in plan.document.input.inserts) {
			var stock=InterfaceQualification.finishingStock(plan.document.input,source);
			var checks=plan.validation.filter(check -> check.rule==stock.rule);
			if (checks.length!=1 || ToolingCodec.canonical(checks[0])!=ToolingCodec.canonical(stock))
				throw "Box mold finishing stock decision differs from source geometry: "+source.id;
			var requests=plan.changeRequests.filter(request -> request.id=="allowance-"+source.id);
			if (stock.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass ? requests.length!=0
				: requests.length!=1 || requests[0].blockingChecks.indexOf(stock.rule)<0)
				throw "Box mold finishing stock request differs from source geometry: "+source.id;
		}
		if (ToolingCodec.canonical(plan.fabrication) != ToolingCodec.canonical(materia.tooling.ToolingValidation.ToolingReadiness.evaluate(plan.validation)))
			throw "Stale fabrication readiness";
		if (Math.abs(plan.document.setup.fillHeightMm - plan.cavity.z - plan.cavity.height) > 1e-7 || plan.document.setup.freeboardMm != 0)
			throw "Box template needs full-height fill and zero freeboard";
		if (ToolingCodec.documentFingerprint(plan.document) != plan.sourceFingerprint)
			throw "Stale mold document";
		materia.tooling.panel.PanelAssemblyPlan.validate(plan.panels);
		if (plan.cores.length != 1 || plan.fixtures.length != 1 || plan.panels.members.length != 5)
			throw "Incomplete single-pocket mold";
		if (plan.supportPanels.length > 0)
			materia.tooling.panel.PanelAssemblyPlan.validate({
				id: "core-support",
				members: plan.supportPanels,
				joints: []
			});
		for (c in plan.cores) {
			if (c.layers.length!=1) throw "Single-pocket core needs one cutting layer";
			if (CoreSupports.kind(MoldCores.singleBox(c.cavity,c.id),plan.cavity)!=c.support.kind)
				throw "Core support disagrees with opening";
			var b = c.layers[0].block, f = MoldCores.singleBox(c.foamEnvelope,c.id), pose = c.layers[0].stockToMold, v = c.layers[0].wireDirection;
			for (size in [b.width, b.depth, b.height, f.width, f.depth, f.height])
				if (!Math.isFinite(size) || size <= 0)
					throw "Invalid core block";
			if (b.x != 0
				|| b.y != 0
				|| b.z != 0
				|| Math.abs(b.width - f.width) + Math.abs(b.depth - f.depth) + Math.abs(b.height - f.height) > 1e-7
				|| Math.abs(pose.x - f.x) + Math.abs(pose.y - f.y) + Math.abs(pose.z - f.z) > 1e-7
				|| Math.abs(pose.qx) + Math.abs(pose.qy) + Math.abs(pose.qz) > 1e-8
				|| Math.abs(Math.abs(pose.qw) - 1) > 1e-8
				|| false)
				throw "Invalid ruled core cutting definition";
			CoreSupports.validateCutting(c.layers[0], f, plan.document.recipe);
			CoreSupports.validateFoot(c, plan.document.recipe, plan.cavity);
			var support = c.support;
			if (support.kind == CoreSupportKind.Bridge) {
				var carrier = plan.supportPanels.filter(m -> m.panel.id == materia.tooling.ToolingIds.derive("", c.id, "-carrier"))[0];
				for (pad in plan.supportPanels)
					if (StringTools.endsWith(pad.panel.id, "-pad")) {
						var a = materia.casting.CastingProfileGeometry.bounds(materia.tooling.panel.PanelContours.outside(carrier.panel));
						var b = materia.casting.CastingProfileGeometry.bounds(materia.tooling.panel.PanelContours.outside(pad.panel));
						if (Math.min(carrier.pose.x + a.x + a.width, pad.pose.x + b.x + b.width) > Math.max(carrier.pose.x + a.x, pad.pose.x + b.x) + 1e-7
							&& Math.min(carrier.pose.y + a.y + a.height, pad.pose.y + b.y + b.height) > Math.max(carrier.pose.y + a.y, pad.pose.y + b.y) + 1e-7)
							throw "Core carrier overlaps bridge pad: " + carrier.panel.id + "/" + pad.panel.id;
						if (Math.min(plan.cavity.x + plan.cavity.width, pad.pose.x + b.x + b.width) > Math.max(plan.cavity.x, pad.pose.x + b.x) + 1e-7
							&& Math.min(plan.cavity.y + plan.cavity.depth, pad.pose.y + b.y + b.height) > Math.max(plan.cavity.y, pad.pose.y + b.y) + 1e-7)
							throw "Bridge pad covers cavity fill opening: " + pad.panel.id;
					}
			}
			if (support.kind != CoreSupportKind.Bridge && support.kind != CoreSupportKind.RoutedSocket)
				throw "Invalid core support kind";
			if (!Math.isFinite(support.adhesiveThickness)
				|| support.adhesiveThickness <= 0
				|| !Math.isFinite(support.bondAreaMm2)
				|| support.bondAreaMm2 <= 0)
				throw "Invalid core bond dimensions";
			for (id in support.panelIds)
				if (plan.supportPanels.concat(plan.panels.members).filter(m -> m.panel.id == id).length != 1)
					throw "Missing support panel";
			for (id in support.bondedPanelIds)
				if (support.panelIds.indexOf(id) < 0)
					throw "Unknown bonded support panel";
			if (support.kind == CoreSupportKind.Bridge
				&& (support.bondedPanelIds.length != 2 || support.panelIds.length != 4 || support.foot != null || support.socketOperationId != null))
				throw "Incomplete bridge";
			if (support.kind == CoreSupportKind.RoutedSocket
				&& (support.bondedPanelIds.length != 0 || support.panelIds.length != 1 || support.panelIds[0]!="bed" || support.foot == null || support.socketOperationId == null))
				throw "Incomplete bed socket";
			var l = c.coveringThickness, cb = MoldCores.singleBox(c.cavity,c.id), bridge = support.kind == CoreSupportKind.Bridge;
			if (!Math.isFinite(l)
				|| l < 0
				|| Math.abs(f.x - cb.x - l)
				+ Math.abs(f.y - cb.y - l)
				+ Math.abs(f.z - cb.z - (bridge ? l : 0))
				+ Math.abs(f.width + 2 * l - cb.width)
				+ Math.abs(f.depth + 2 * l - cb.depth)
				+ Math.abs(f.height + l + (bridge ? support.adhesiveThickness : 0) - cb.height) > 1e-7)
				throw "Core covering does not reproduce cavity";
		}
		for (fixture in plan.fixtures) DatumFixtures.validatePanel(plan,fixture);
		for (opening in plan.openings) {
			if (opening.protection!=null && opening.kind!=ToolingOpening)
				throw "Protected opening must be tooling-sealed";
		}
		var seen = new Map<String, Bool>();
		for (p in MoldOpenings.records(plan)) {
			var key = p.panelId + "/" + p.operationId;
			if (seen.exists(key))
				throw "Duplicate penetration";
			seen.set(key, true);
			var members = plan.panels.members.filter(m -> m.panel.id == p.panelId);
			if (members.length != 1 || members[0].panel.contactOpenings.indexOf(p.operationId) < 0)
				throw "Unknown penetration";
			if (p.kind != OutsideCasting && p.kind != InsertSealed && p.kind != CoreSealed)
				throw "Unsupported penetration class";
			if (p.kind == InsertSealed && (p.insertId == null || plan.fixtures.filter(f -> f.insertId == p.insertId).length != 1))
				throw "Unknown sealing insert";
			if (p.kind == InsertSealed)
				DatumFixtures.validatePanelSeal(plan, p);
			if (p.kind == CoreSealed
				&& (p.coreId == null
					|| plan.cores.filter(c -> c.id == p.coreId && c.support.socketOperationId == p.operationId).length != 1))
				throw "Unknown sealing core";
			if (!Math.isFinite(p.sealMargin) || p.sealMargin < 0 || StringTools.trim(p.sealMaterial).length == 0)
				throw "Invalid penetration seal";
		}
		for (m in plan.panels.members)
			for (id in m.panel.contactOpenings)
				if (!seen.exists(m.panel.id + "/" + id))
					throw "Unclassified contact opening " + m.panel.id + "/" + id;
	}
}
