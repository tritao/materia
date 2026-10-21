import materia.casting.InsertSpec.InsertRole;
import materia.assembly.AssemblyFrames as F;

class LiftingSourceTests {
	static var assertions=0;
	static function check(ok:Bool,message:String):Void {assertions++;if (!ok) throw message;}
	public static function main():Void {
		var frame=MillMoldSetup.frame();
		for (foundation in [frame.base,frame.column]) {
			var rows=foundation.insertLocations().filter(row -> row.part.parameterToken("role")==InsertRole.LiftingMount);
			check(rows.length==4,"each mill foundation declares four source lifting connections");
			var mass=machinekit.milling.MillLiftingPattern.asCastMass(foundation),xs=[for (row in rows) row.pose.x],ys=[for (row in rows) row.pose.y];
			xs.sort(Reflect.compare);ys.sort(Reflect.compare);
			check(mass.mass>0 && mass.unaccounted.length==0 && mass.centreOfMass.x>xs[0]
				&& mass.centreOfMass.x<xs[xs.length-1] && mass.centreOfMass.y>ys[0]
				&& mass.centreOfMass.y<ys[ys.length-1],"final anchored casting CoM lies inside its lifting footprint");
			for (row in rows) {
				var spec=row.part.insertSpec(),occupied=materia.casting.InsertAnchorage.InsertAnchorageCodec.occupiedHoleIds(spec.anchorage);
				var service=spec.holes.filter(hole -> occupied.indexOf(hole.id)<0);
				check(service.length==1 && service[0].thread!=null && service[0].extent==materia.casting.InsertHole.HoleExtent.Blind,
					"lifting thread is a real blind source interface, separate from embedded anchor holes");
				var facet=machinekit.design.DesignInterfaces.of(row.part);
				check(facet!=null && facet.holes.filter(hole -> hole.id==service[0].id).length==1,
					"lifting fixture and service connection share the source HoleInterface");
				check(switch spec.anchorage {case Bolted(_,anchors):anchors.length>=2;default:false;},
					"lifting inserts receive the same production anchor rule");
				var owned=foundation.body.castingFeatures().filter(feature -> feature.owner==row.id);
				check(owned.length==1,"lifting steel owns exactly one source recess");
			}
			var diagnostics=machinekit.design.AssemblyDesignChecks.check(foundation);
			check(!diagnostics.hasErrors(),"source foundation interfaces: "+diagnostics.items.map(item -> item.message).join("; "));
			var input=machinekit.tooling.FoundationToolingAdapter.snapshot(foundation,"lifting-source",1);
			for (finding in machinekit.tooling.InsertAnchorGeometry.review(input))
				check(finding.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass,finding.rule+": "+finding.message);
		}
		var p=materia.tooling.casting.CastingMoldPlanner.plan(MillMoldSetup.document(frame),MillMoldSetup.options(frame));
		var bed=p.panels.members.filter(member -> member.panel.id=="bed")[0];
		var local=machinekit.tooling.ToolingPanel.fromPlan(bed.panel).geometry();
		var world=machinekit.assembly.PosedParts.moved(local,bed.pose);local.close();
		try {
			for (fixture in p.fixtures) if (fixture.insertId.indexOf("lift")==0) {
				var datum=machinekit.tooling.ToolingVolume.geometryOf(fixture.datum);
				try {
					var overlap=world.intersect(datum),volume=overlap.volume();overlap.close();
					check(volume<1e-5,"native bed/datum pair has no interior overlap: "+fixture.insertId+" "+volume);
				} catch (error:Dynamic) {datum.close();throw error;}
				datum.close();
			}
		} catch (error:Dynamic) {world.close();throw error;}
		world.close();
		var source=machinekit.tooling.CastingMoldCad.source(p);
		var rebuilt=machinekit.tooling.CastingMoldCad.reconstructed(p);
		try {
			var missing=source.subtractAll([rebuilt]),excess=rebuilt.subtractAll([source]);
			var difference=missing.volume()+excess.volume();missing.close();excess.close();
			check(difference<1e-5,"sealed fixture clearance bores are not resin: "+difference);
		} catch (error:Dynamic) {source.close();rebuilt.close();throw error;}
		source.close();rebuilt.close();
		Sys.println('LiftingSourceTests: $assertions assertions passed');
	}
}
