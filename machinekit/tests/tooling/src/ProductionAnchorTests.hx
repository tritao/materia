import machinekit.structural.InsertAnchorPattern;
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
	public static function main():Void {
		var rule = new machinekit.milling.MillCastingDetails().anchorRule;
		var plate = new CastInsert(CastInsert.boxValues(50,60,20).setToken("role",materia.casting.InsertSpec.InsertRole.GenericMount));
		var first = InsertAnchorPattern.derive("plate",plate,rule,FootprintOnly);
		var second = InsertAnchorPattern.derive("plate",plate,rule,FootprintOnly);
		check(materia.tooling.ToolingCodec.canonical(first.insertSpec()) == materia.tooling.ToolingCodec.canonical(second.insertSpec()),
			"footprint derivation is deterministic");
		check(switch first.insertSpec().anchorage {case Bolted(_,anchors): anchors.length>=2
			&& anchors.filter(anchor -> anchor.screw.thread.majorDiameter!=3).length==0; default:false;},
			"smallest available policy-compliant thread and footprint pair are derived");
		var error="";
		try InsertAnchorPattern.derive("too-small",new CastInsert(CastInsert.boxValues(8,8,2).setToken("role",materia.casting.InsertSpec.InsertRole.GenericMount)),rule,FootprintOnly)
		catch (e:Dynamic) error=Std.string(e);
		check(error.indexOf("Insert too-small:")>=0,"unsatisfiable insert fails with its source ID");
		var frame = MillMoldSetup.frame();
		var foundations=[frame.base,frame.column];
		var depth=Math.max(frame.layout.roof,frame.layout.casting.minimumColumnPadDepth(frame.layout.insertThickness));
		check(Math.abs(depth-68)<1e-8,"pad depth is derived from policy edge wall, washer, access and resin cover");
		for (foundation in foundations) {
			var count=0;
			for (row in foundation.insertLocations()) {
				check(switch row.part.insertSpec().anchorage {case Bolted(_,anchors):count+=anchors.length;anchors.length>=2;default:false;},
					"every production insert has its derived anchors: "+row.id);
			}
			Sys.println("Production anchor count: "+count);
			var input=machinekit.tooling.FoundationToolingAdapter.snapshot(foundation,"production-anchor-test",1);
			for (finding in machinekit.tooling.InsertAnchorGeometry.review(input)) {
				Sys.println(finding.rule+" "+finding.status+" "+finding.message);
				check(finding.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass,"production anchor geometry: "+finding.rule);
			}
		}
		if (Sys.args().indexOf("--check-mold")>=0) {
			checkMold(frame,foundations[0]);
		}
		Sys.println('ProductionAnchorTests: $assertions assertions passed');
	}
}
