import machinekit.tooling.CastingSectionContours as S;
import materia.casting.CastingProfileGeometry as G;
import materia.assembly.AssemblyFrames as F;
import cadkit.modeling.Part;
import cadkit.modeling.Scope;

class SectionContourTests {
	static var assertions=0;
	static function check(value:Bool,message:String):Void {assertions++;if (!value) throw message;}
	static function near(value:Float,expected:Float,message:String):Void
		check(Math.abs(value-expected)<1e-6,message+": "+value+" vs "+expected);
	public static function main():Void {
		var scope=new Scope();
		try {
			var outer=scope.own(Part.cylinder(20,40)),inner=scope.own(Part.cylinder(10,40));
			var ring=scope.own(outer.subtractAll([inner]));
			var section=S.section(ring,F.translation(0,0,20),"annulus");
			var summary=materia.tooling.ToolingCodec.canonical([for (row in section)
				{depth:row.depth,area:G.area(row.contour),bounds:G.bounds(row.contour)}]);
			check(section.length==2 && section[0].depth==0 && section[1].depth==1,
				"section retains outer material and its circular hole: "+summary);
			near(G.area(section[0].contour)-G.area(section[1].contour),300*Math.PI,"exact annular section area");
			check(section[0].contour.length==2 && section[0].contour.filter(segment -> segment.center==null).length==0,
				"full circles remain exact half arcs");
			var island=scope.own(Part.cylinder(5,40)),nested=scope.own(Part.fuseAll([ring,island]));
			var loops=S.section(nested,F.translation(0,0,20),"nested");
			check(loops.length==3 && loops[2].depth==2,"nested material islands retain their containment depth");
			near(G.area(loops[0].contour)-G.area(loops[1].contour)+G.area(loops[2].contour),325*Math.PI,"nested circular section area");
			check(S.section(ring,F.translation(0,0,60),"above-ring").length==0,"an absent section never becomes a stock-bounds profile");
			var block=scope.own(Part.box(40,40,40)),tool=scope.own(Part.cylinderSpan(10,30,45));
			var blind=scope.own(block.subtractAll([tool]));
			for (height in [20.0,30.0,35.0,40.0]) {
				var material=S.section(blind,F.translation(0,0,height),"blind-floor"),area=0.0;
				for (row in material) area+=(row.depth%2==0 ? 1 : -1)*G.area(row.contour);
				near(area,height<=30 ? 1600 : 1600-100*Math.PI,
					"a blind-hole floor is material, not a spurious inner wire at a critical level");
			}
			var placement=F.fromRotationMatrix(30,-17,8,[0,0,1,1,0,0,0,1,0]);
			var placed=scope.own(machinekit.assembly.PosedParts.moved(ring,placement));
			var posed=S.section(placed,F.compose(placement,F.translation(0,0,20)),"placed-ring");
			near(G.area(posed[0].contour)-G.area(posed[1].contour),300*Math.PI,"section plane follows the declared fixture frame");
			var draft=materia.casting.CastingDraftSections.sections("rounded-draft",
				materia.casting.CastingProfiles.roundedRectangle(-20,-15,40,30,4),F.identity(),60,2);
			var former=scope.own(machinekit.structural.CastingProfileCad.drafted(draft));
			for (height in [0.0,30.0,60.0]) {
				var profile=S.section(former,F.translation(0,0,height),"rounded-draft");
				var offset=(60-height)*Math.tan(2*Math.PI/180);
				check(profile.length==1,"draft section has one closed boundary");
				near(G.area(profile[0].contour),(40-2*offset)*(30-2*offset)-(4-Math.PI)*(4-offset)*(4-offset),
					"exact draft section agrees with the analytic changing radius");
			}
			var record:materia.tooling.casting.CastingMoldPlan.MoldVolume={id:"ring-former",material:FormerStock,
				pieces:[],profiles:[],cuts:[],regions:[materia.casting.CastingRegion.Difference(
					materia.casting.CastingRegion.Placed({solid:Cut(materia.casting.CastingSolid.Cylinder(40,40)),pose:F.identity()}),
					[materia.casting.CastingRegion.Placed({solid:Cut(materia.casting.CastingSolid.Cylinder(20,40)),pose:F.identity()})]) ]};
			var sheet=machinekit.tooling.ToolingSectionDrawing.sheet(record,F.identity(),[10,30]);
			var directional=machinekit.tooling.ToolingSectionDrawing.byDirection(record,{x:0,y:0,z:1});
			check(directional.sections.length==3 && Math.abs(directional.sections[0].plane.z)<1e-9
				&& Math.abs(directional.sections[2].plane.z-40)<1e-9,
				"drawing end sections use actual geometry extrema rather than inflated stock bounds: "
					+[for (view in directional.sections) view.plane.z].join(","));
			var drafted=materia.tooling.casting.MoldVolumeCodec.snapshot(record);
			drafted.id="draft-former";
			drafted.regions=[materia.casting.CastingRegion.Drafted("rounded-draft",draft.opening,draft.pose,draft.depth,2)];
			check(machinekit.tooling.ToolingSectionDrawing.byDirection(drafted,{x:0,y:0,z:1}).sections.length==3,
				"drafted walls support reproducible end and middle reference sections");
			check(sheet.sections.length==2 && sheet.sections[0].contours[1].depth==1,
				"finished section records remain available separately from paper layout");
			var dxf=cadkit.drawing.Dxf.read(cadkit.drawing.Dxf.write(sheet.drawing));
			check(dxf.curves.length==8,"DXF retains every exact arc from the section records");
			var outsideArea=0.0,insideArea=0.0;
			for (curve in dxf.curves) {
				if (curve.layer=="OUTSIDE_PROFILE") outsideArea+=curve.segment.area();
				if (curve.layer=="INSIDE_PROFILE") insideArea+=curve.segment.area();
			}
			near(outsideArea-insideArea,600*Math.PI,"DXF section geometry retains both annular material areas");
			check(cadkit.drawing.Svg.write(sheet.drawing).indexOf(" A ")>=0,"SVG uses existing exact circular drawing entities");
			var tilted=F.fromRotationMatrix(0,0,20,[1,0,0,0,Math.sqrt(0.5),-Math.sqrt(0.5),0,Math.sqrt(0.5),Math.sqrt(0.5)]);
			var failure="";
			try S.section(outer,tilted,"elliptical-cut") catch (error:Dynamic) failure=Std.string(error);
			check(failure.indexOf("elliptical-cut")>=0 && failure.indexOf("Unsupported section curve")>=0,
				"an elliptical section is reported by source ID instead of polygonized");
			scope.close();
		} catch (error:Dynamic) {scope.close();throw error;}
		Sys.println("SectionContourTests: "+assertions+" assertions passed");
	}
}
