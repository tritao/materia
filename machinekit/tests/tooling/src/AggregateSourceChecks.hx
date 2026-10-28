import materia.assembly.AssemblyFrames as F;
import materia.tooling.ToolingCodec;
import materia.casting.InsertAnchorage.InsertAnchorageCodec;
import machinekit.tooling.CastingMoldCad;
import machinekit.tooling.CastingWallReview;

/** Approved aggregate corrections preserve the existing service interfaces. */
class AggregateSourceChecks {
	static var assertions=0;
	static function check(ok:Bool,message:String):Void {
		assertions++;
		if(!ok)throw message;
	}
	static function near(a:Float,b:Float,message:String):Void
		check(Math.abs(a-b)<1e-7,message+": "+a+" versus "+b);
	static function vector(actual:{x:Float,y:Float,z:Float},expected:Array<Float>,message:String):Void {
		near(actual.x,expected[0],message+" X");
		near(actual.y,expected[1],message+" Y");
		near(actual.z,expected[2],message+" Z");
	}
	static function main():Void {
		var reference:Dynamic=haxe.Json.parse(sys.io.File.getContent(
			"machinekit/tests/tooling/fixtures/foundation-design/aggregate-interface-reference.json"));
		check(reference.version==1,"Functional reference version");
		var frame=MillProject.smallFrame(),poses=frame.solvedPoses();
		vector(F.transformPoint(poses.get("base/casting"),0,0,0),[0,frame.layout.baseCentreY,0],"Base source frame");
		vector(F.transformPoint(poses.get("column/casting"),0,0,0),[0,frame.layout.columnCentreY,frame.layout.columnMountPlane],"Column source frame");
		for(id in ["base/casting","column/casting"])for(axis in [[1.0,0,0],[0.0,1,0],[0.0,0,1]])
			vector(F.transformVector(poses.get(id),axis[0],axis[1],axis[2]),axis,id+" orientation");
		var castings:Array<Dynamic>=reference.castings;
		for(casting in castings) {
			var foundation=casting.id=="base" ? frame.base : frame.column;
			var rows=foundation.insertLocations(),expectedRows:Array<Dynamic>=casting.inserts;
			for(expected in expectedRows) {
				var found=rows.filter(row->row.id==expected.id);
				check(found.length==1,"Service insert retained: "+expected.id);
				var row=found[0],spec=row.part.insertSpec();
				var occupied=InsertAnchorageCodec.occupiedHoleIds(spec.anchorage);
				var service=spec.holes.filter(hole->occupied.indexOf(hole.id)<0);
				var expectedHoles:Array<Dynamic>=expected.holes;
				check(service.length==expectedHoles.length,"Service-hole set retained: "+row.id);
				for(expectedHole in expectedHoles) {
					var holes=service.filter(hole->hole.id==expectedHole.id);
					check(holes.length==1,"Source service hole retained: "+row.id+"/"+expectedHole.id);
					var hole=holes[0],world=F.compose(row.pose,hole.frame);
					var entry:Array<Float>=expectedHole.entry,axis:Array<Float>=expectedHole.axis;
					vector(F.transformPoint(world,0,0,0),entry,row.id+"/"+hole.id+" entry");
					vector(F.transformVector(world,0,0,1),axis,row.id+"/"+hole.id+" axis");
					near(hole.depth,expectedHole.depth,row.id+"/"+hole.id+" depth");
					near(hole.diameter,expectedHole.diameter,row.id+"/"+hole.id+" diameter");
					check(ToolingCodec.canonical(hole.thread)==ToolingCodec.canonical(expectedHole.thread),row.id+"/"+hole.id+" thread unchanged");
				}
				var expectedInterfaces:Array<Dynamic>=expected.interfaces;
				check(spec.interfaces.length==expectedInterfaces.length,"Precision interface set retained: "+row.id);
				for(expectedInterface in expectedInterfaces) {
					var interfaces=spec.interfaces.filter(iface->iface.id==expectedInterface.id);
					check(interfaces.length==1,"Precision interface retained: "+row.id+"/"+expectedInterface.id);
					var iface=interfaces[0],local=switch iface.geometry {
						case Plane(at):at;
						case Line(_):throw "The frozen mill interface reference contains planes";
					};
					var world=F.compose(row.pose,local),normal=F.transformVector(world,0,0,1);
					var expectedNormal:Array<Float>=expectedInterface.normal;
					vector(normal,expectedNormal,row.id+"/"+iface.id+" normal");
					var expectedOrigin:Array<Float>=expectedInterface.origin;
					var expectedTangent:Array<Float>=expectedInterface.tangent;
					vector(F.transformPoint(world,0,0,0),expectedOrigin,row.id+"/"+iface.id+" datum origin");
					vector(F.transformVector(world,1,0,0),expectedTangent,row.id+"/"+iface.id+" datum tangent");
					near(normal.x*world.x+normal.y*world.y+normal.z*world.z,expectedInterface.planeOffsetMm,row.id+"/"+iface.id+" plane");
					near(iface.allowanceMm,expectedInterface.allowanceMm,row.id+"/"+iface.id+" finishing stock");
					near(iface.positionToleranceMm,expectedInterface.positionToleranceMm,row.id+"/"+iface.id+" tolerance");
					check(iface.route==expectedInterface.route,row.id+"/"+iface.id+" finishing route");
				}
			}
		}
		Sys.println("Aggregate service interfaces: "+assertions+" assertions passed");
		if(Sys.args().indexOf("--interfaces-only")>=0)return;
		for(kind in [MillMoldSetup.MillMoldCasting.Base,MillMoldSetup.MillMoldCasting.Column]) {
			var document=MillMoldSetup.document(frame,CastingShopRecipe.BOARD_THICKNESS_MM,kind);
			var plan=CastingMoldCad.plan(document,MillMoldSetup.options(frame,kind));
			var limits=CastingShopRecipe.process(document.recipe,"source-aggregate-check").castability;
			var report=CastingWallReview.fromPlan(plan,limits);
			var dir="machinekit/tests/tooling/build/aggregate-source-checks/"+document.input.id;
			machinekit.tooling.ToolingFiles.directory(dir);
			machinekit.tooling.ToolingProcessExport.writeAggregate(plan,report,dir);
			for(group in report.groups)if(group.status==materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Fail)
				Sys.println("AGGREGATE FAIL "+group.kind+" "+group.featurePair.join(" / ")+": "+group.minimumMeasuredMm+" < "+group.requiredMm);
			check(report.failures.length==0,"Aggregate source gate: "+document.input.id+"; see "+dir);
		}
		Sys.println("AggregateSourceChecks: "+assertions+" assertions passed");
	}
}
