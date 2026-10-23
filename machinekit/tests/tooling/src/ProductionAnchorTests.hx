import machinekit.tooling.InsertAnchorGeometry;
import machinekit.structural.CastInsert;
import machinekit.component.ComponentValues;

class ProductionAnchorTests {
	static var assertions = 0;
	static function check(ok:Bool,message:String):Void {assertions++; if (!ok) throw message;}

	static function checkMold(frame:machinekit.milling.MillFrame,foundation:machinekit.structural.MineralFoundation):Void {
		var document=MillMoldSetup.document(frame);
		document.input=machinekit.tooling.FoundationToolingAdapter.snapshot(foundation,document.input.id,document.input.revision);
		document.setup.sourceFingerprint=materia.tooling.ToolingCodec.inputFingerprint(document.input);
		var plan=materia.tooling.casting.CastingMoldPlanner.plan(document,MillMoldSetup.options(frame));
		var interference=machinekit.tooling.InsertAnchorGeometry.reviewMold(plan);
		check(interference.length>0,"production mold checks every derived anchor");
		for (finding in interference)
			check(finding.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass,
				finding.rule+": "+finding.message);
		var review=machinekit.tooling.CastingMoldCad.review(plan);
		check(review.symmetricDifferenceMm3<1e-6,"production mold reconstructs the exact anchored source resin");
		for (hardware in machinekit.tooling.CastingMoldCad.anchorComponents(plan))
			check(review.assemblyGroups.filter(group -> group.members.indexOf(hardware.id)>=0).length==1,
				"real anchor hardware belongs to exactly one checked insertion group: "+hardware.id);
		Sys.println("Production anchored mold: "+interference.length+" source anchors, exact reconstruction and complete swept graphs passed");
	}
	static function neighbours(foundation:machinekit.structural.MineralFoundation,
		row:machinekit.structural.MineralFoundation.CastInsertPlacement):Void {
		var parts:Array<cadkit.modeling.Part> = [];
		function own(part:cadkit.modeling.Part):cadkit.modeling.Part {parts.push(part);return part;}
		try {
			var local=own(machinekit.structural.InsertBodies.part(row.part.insertSpec().body));
			var placed=own(machinekit.assembly.PosedParts.moved(local,row.pose));
			var features=foundation.body.castingFeatures(),minimum=Math.POSITIVE_INFINITY,nearest="",overlap=0.0,conflict="";
			for (cavity in foundation.body.finishedCavities()) {
				var feature=features.filter(f -> f.id==cavity.featureId)[0];
				if (feature.owner==row.id) continue;
				var other=machinekit.structural.CastingCavityCad.part(cavity),common:cadkit.modeling.Part=null;
				try {
					common=placed.intersect(other);
					var volume=common.volume(),distance=placed.shape.distance(other.shape);
					if (volume>overlap) {overlap=volume;conflict=cavity.featureId;}
					if (distance<minimum) {minimum=distance;nearest=cavity.featureId;}
				} catch (error:Dynamic) {if (common!=null) common.close();other.close();throw error;}
				common.close();other.close();
			}
			check(overlap<1e-6,"enlarged source insert "+row.id+" intersects neighbouring feature "+conflict+": "+overlap+" mm3");
			Sys.println("INSERT-NEIGHBOUR "+row.id+" minimum "+minimum+" mm to "+nearest);
		} catch (error:Dynamic) {for (part in parts) part.close();throw error;}
		for (part in parts) part.close();
	}

	public static function main():Void {
		var rule = new machinekit.milling.MillCastingDetails().anchorRule;
		var plate = new CastInsert(CastInsert.boxValues(50,60,20).setToken("role",materia.casting.InsertSpec.InsertRole.GenericMount));
		var first = InsertAnchorGeometry.derive("plate",plate,rule,FootprintOnly,0);
		var second = InsertAnchorGeometry.derive("plate",plate,rule,FootprintOnly,0);
		check(materia.tooling.ToolingCodec.canonical(first.insertSpec()) == materia.tooling.ToolingCodec.canonical(second.insertSpec()),
			"footprint derivation is deterministic");
		check(switch first.insertSpec().anchorage {case Bolted(_,anchors): anchors.length>=2
			&& anchors.filter(anchor -> anchor.screw.thread.majorDiameter<5).length==0; default:false;},
			"unloaded steel receives a policy-compliant ISO 4162 anchor pair");
		var loaded=new CastInsert(CastInsert.precastHoles(CastInsert.boxValues(50,60,30),
			machinekit.structural.InsertHoles.tappedBlind(machinekit.structural.InsertHoles.metricThread(
				machinekit.standard.FastenerThreads.coarse(12).mating()),30))
			.setToken("role",materia.casting.InsertSpec.InsertRole.GenericMount));
		var sized=InsertAnchorGeometry.derive("loaded-plate",loaded,rule,FootprintOnly,0);
		var selected=switch sized.insertSpec().anchorage {case Bolted(r,a):{rules:r,anchors:a};default:throw "Missing load-sized anchors";};
		var admitted=0.0;
		for (anchor in selected.anchors) admitted+=anchor.screw.assemblyPreloadN;
		check(admitted>=materia.casting.InsertAnchorage.InsertAnchorageCodec.demandN(selected.rules),
			"the source M12 assembly preload is covered by the declared anchor steel-side admission");
		check(selected.anchors.length==4 && selected.anchors.filter(anchor -> anchor.screw.thread.majorDiameter==8).length==4,
			"demand and policy footprint choose four M8 anchors instead of the unloaded M5 pair");
		var unloaded=new CastInsert(CastInsert.boxValues(50,60,30).setToken("role",materia.casting.InsertSpec.InsertRole.GenericMount));
		check(InsertAnchorGeometry.minimumOuterLand("loaded-land",loaded,rule,0).x
			> InsertAnchorGeometry.minimumOuterLand("unloaded-land",unloaded,rule,0).x,
			"source placement reserves resin cover for the demand-sized flange rather than the smallest bolt");
		var error="";
		try InsertAnchorGeometry.derive("too-small",new CastInsert(CastInsert.boxValues(8,8,2).setToken("role",materia.casting.InsertSpec.InsertRole.GenericMount)),rule,FootprintOnly,0)
		catch (e:Dynamic) error=Std.string(e);
		check(error.indexOf("Insert too-small:")>=0,"unsatisfiable insert fails with its source ID");
		var hatchThread=machinekit.structural.InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(4).mating());
		var smallHatch=new CastInsert(CastInsert.precastHoles(CastInsert.boxValues(20,20,10),
			machinekit.structural.InsertHoles.tappedBlind(hatchThread,10))
			.setToken("role",materia.casting.InsertSpec.InsertRole.GenericMount));
		error="";
		try InsertAnchorGeometry.derive("small-hatch",smallHatch,rule,FootprintOnly,0)
		catch (e:Dynamic) error=Std.string(e);
		check(error.indexOf("Insert small-hatch:")>=0 && error.indexOf("thicker stock")>=0,
			"the old hatch stock cannot silently receive an undersized thread or a one-anchor fallback");
		var hatch=new CastInsert(CastInsert.precastHoles(CastInsert.boxValues(30,20,12),
			machinekit.structural.InsertHoles.tappedBlind(hatchThread,12))
			.setToken("role",materia.casting.InsertSpec.InsertRole.GenericMount));
		error="";
		try InsertAnchorGeometry.derive("thin-wall-hatch",hatch,rule,FootprintOnly,0)
		catch (e:Dynamic) error=Std.string(e);
		check(error.indexOf("Insert thin-wall-hatch:")>=0,
			"nonintersecting taps still fail when the service-hole material wall is missing");
		var largerHatch=new CastInsert(CastInsert.precastHoles(CastInsert.boxValues(44,20,12),
			machinekit.structural.InsertHoles.tappedBlind(hatchThread,12))
			.setToken("role",materia.casting.InsertSpec.InsertRole.GenericMount));
		var hatchPattern=InsertAnchorGeometry.derive("hatch-control",largerHatch,rule,FootprintOnly,0);
		check(switch hatchPattern.insertSpec().anchorage {case Bolted(r,a):a.length==2
			&& a.filter(anchor -> anchor.screw.thread.majorDiameter==5).length==2
			&& materia.casting.InsertAnchorage.InsertAnchorageCodec.demandN(r)==6030;default:false;},
			"the enlarged hatch control carries its M4 service preload through two policy-compliant M5 anchors");
		var longHoles:Array<materia.casting.InsertHole> = [];
		for (y in [-442.5,-295.0]) {
			var hole=machinekit.structural.InsertHoles.records(machinekit.structural.InsertHoles.tappedBlind(
				machinekit.structural.InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(5).mating()),20))[0];
			hole.id="service-"+longHoles.length;hole.frame.y=y;longHoles.push(hole);
		}
		var longSeat=new CastInsert(CastInsert.precastHoles(CastInsert.boxValues(40,900,20),
			machinekit.structural.InsertHoles.fromNeutral(longHoles))
			.setToken("role",materia.casting.InsertSpec.InsertRole.GuideSeat));
		var longPattern=InsertAnchorGeometry.derive("long-seat",longSeat,rule,FootprintOnly,0).insertSpec();
		var anchorIds=switch longPattern.anchorage {case Bolted(_,a):[for (anchor in a) anchor.holeId];default:throw "Missing long-seat pattern";};
		var stations=[for (hole in longPattern.holes) if (anchorIds.indexOf(hole.id)>=0) hole.frame.y];
		stations.sort(Reflect.compare);
		var largest=Math.max(stations[0]+450,450-stations[stations.length-1]);
		for (index in 1...stations.length) largest=Math.max(largest,stations[index]-stations[index-1]);
		check(largest<=rule.maximumPitchInShortWidths*40,
			"removing back-anchor sites beside service holes preserves the declared maximum station interval");
		for (width in [70.0,71.0]) {
			var holes=machinekit.structural.InsertHoles.records(machinekit.structural.InsertHoles.tappedBlind(
				machinekit.structural.InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(12).mating()),30));
			holes[0].frame.x=width/2+4.0-22;
			holes[0].frame.y=45.0/2+4.5-27;
			var compact=new CastInsert(CastInsert.precastHoles(CastInsert.boxValues(width,45,30),
				machinekit.structural.InsertHoles.fromNeutral(holes))
				.setToken("role",materia.casting.InsertSpec.InsertRole.LiftingMount));
			error="";
			var count=0;
			try {
				var selected=InsertAnchorGeometry.derive("compact-lifting",compact,rule,FootprintOnly,1000).insertSpec();
				count=switch selected.anchorage {case Bolted(_,anchors):anchors.length;default:0;};
			} catch (e:Dynamic) error=Std.string(e);
			check(width==70 ? error.indexOf("Insert compact-lifting:")>=0 : error=="" && count==4,
				"the compact source blank retains full service-hole walls and admits demand at 71 mm, rejecting 70 mm");
		}
		if (Sys.args().indexOf("--patterns-only")>=0) {
			Sys.println('ProductionAnchorTests: $assertions footprint assertions passed');
			return;
		}
		var sourceStart=Sys.time();
		var frame = MillMoldSetup.frame();
		Sys.println("ANCHOR-SOURCE-WALL-TIME "+(Sys.time()-sourceStart)+" seconds (complete frame construction)");
		var foundations=[frame.base,frame.column];
		var depth=Math.max(frame.layout.roof,frame.layout.casting.minimumColumnPadDepth(frame.layout.insertThickness));
		check(depth>frame.layout.casting.columnBoltAccessDepth,"source pad retains resin lands beside its declared access opening");
		for (foundation in foundations) {
			var steel=machinekit.design.AssemblyDesignChecks.check(foundation);
			check(!steel.hasErrors(),"production insert material walls: "+steel.items.map(item -> item.message).join("; "));
			var count=0;
			for (row in foundation.insertLocations()) {
				var spec=row.part.insertSpec();
				var raw=switch materia.casting.InsertFinishing.assess(spec.body,spec.interfaces) {
					case Resolved(value):value;
					case Invalid(reason),Unsupported(reason):throw reason;
				};
				var stock=materia.casting.InsertFinishing.box(raw);
				if (stock==null) throw "Production source insert must have rectangular stock: "+row.id;
				Sys.println("INSERT-STOCK "+(foundation==frame.base ? "base/" : "column/")+row.id+" "
					+stock.width+"x"+stock.depth+"x"+stock.height);
				var sourceHoles=switch spec.anchorage {case Bolted(_,anchors):
					var ids=[for (anchor in anchors) anchor.holeId];
					Sys.println("ANCHORS "+(foundation==frame.base ? "base/" : "column/")+row.id+" "+anchors.length+"x"+anchors[0].screw.thread.designation);
					spec.holes.filter(h -> ids.indexOf(h.id)<0);
					default:spec.holes;
				};
				if (row.part.parameterToken("role")==materia.casting.InsertSpec.InsertRole.LiftingMount
					|| row.part.parameterToken("role")==materia.casting.InsertSpec.InsertRole.HatchMount) neighbours(foundation,row);
				if (row.part.parameterToken("role")==materia.casting.InsertSpec.InsertRole.LiftingMount) {
					var service=materia.assembly.AssemblyFrames.transformPoint(row.pose,sourceHoles[0].frame.x,sourceHoles[0].frame.y,sourceHoles[0].frame.z);
					check(Math.abs(Math.abs(service.x)-(foundation.body.parameterNumber("width")/2-22))<1e-8
						&& Math.abs(Math.abs(service.y)-(foundation.body.parameterNumber("depth")/2-27))<1e-8,
						"enlarging the source lifting plate retains its existing service-hole position: "+row.id);
				}
				if (row.part.parameterToken("role")==materia.casting.InsertSpec.InsertRole.HatchMount) {
					var service=materia.assembly.AssemblyFrames.transformPoint(row.pose,sourceHoles[0].frame.x,sourceHoles[0].frame.y,sourceHoles[0].frame.z);
					var xy=frame.xyAxis;
					if (xy==null) throw "Hatch source position needs the mill XY layout";
					check(Math.abs(Math.abs(service.x)-75)<1e-8
						&& Math.abs(service.y-(xy.yServiceRear+2-frame.layout.columnCentreY))<1e-8
						&& Math.abs(service.z-((StringTools.endsWith(row.id,"Lower") ? xy.yHatchLowerZ : xy.yHatchUpperZ)-frame.layout.railPlane))<1e-8,
						"enlarging the source hatch insert retains its existing service-face hole position: "+row.id);
				}
				check(switch row.part.insertSpec().anchorage {case Bolted(_,anchors):count+=anchors.length;anchors.length>=2;default:false;},
					"every production insert has its derived anchors: "+row.id);
			}
			Sys.println("Production anchor count: "+count);
			var input=machinekit.tooling.FoundationToolingAdapter.snapshot(foundation,"production-anchor-test",1);
			for (finding in machinekit.tooling.InsertAnchorGeometry.review(input)) {
				Sys.println(finding.rule+" "+finding.status+" "+finding.message);
				check(finding.rule.indexOf(".capacity.")>=0
					? finding.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pending
					: finding.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass,"production anchor geometry/material input: "+finding.rule);
			}
		}
		if (Sys.args().indexOf("--check-mold")>=0) {
			checkMold(frame,foundations[0]);
		}
		Sys.println('ProductionAnchorTests: $assertions assertions passed');
	}
}
