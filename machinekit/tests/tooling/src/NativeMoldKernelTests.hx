import materia.assembly.AssemblyFrames as F;
import materia.casting.CastingRegion;
import materia.casting.CastingFeature;
import materia.casting.CastingPatterns as P;

class NativeMoldKernelTests {
	static var assertions=0;
	static function check(value:Bool,message:String):Void {assertions++;if (!value) throw message;}
	static function near(actual:Float,expected:Float,message:String,tolerance:Float=1e-6):Void
		check(Math.abs(actual-expected)<tolerance,message+": "+actual+" vs "+expected);
	public static function main():Void {
		var kernel=new machinekit.tooling.NativeCastingMoldKernel();
		var block=CastingRegion.Placed({solid:Cut(materia.casting.CastingSolid.Box(20,30,40)),pose:F.translation(10,20,30)});
		var properties=kernel.moments(block);
		near(properties.volumeMm3,24000,"native region volume");
		near(properties.firstMomentMm4.x,240000,"first X moment uses placed geometry");
		near(properties.firstMomentMm4.y,480000,"first Y moment uses placed geometry");
		near(properties.firstMomentMm4.z,1200000,"first Z moment uses placed geometry");
		properties.firstMomentMm4.x=-1;
		near(kernel.moments(block).firstMomentMm4.x,240000,"cached facts are detached from callers");
		var empty=kernel.moments(CastingRegion.Difference(block,[block]));
		check(empty.volumeMm3==0 && empty.firstMomentMm4.x==0 && empty.firstMomentMm4.y==0 && empty.firstMomentMm4.z==0,
			"empty material has zero moments rather than an invented centroid");
		var document=SinglePocketSetup.document();
		document.input.inserts=[];
		document.input.envelope=[{id:"body",profile:[{x:-60.0,y:-50.0},{x:60.0,y:-50.0},{x:60.0,y:50.0},{x:-60.0,y:50.0}],
			height:80,pose:F.identity()}];
		var feature=P.box("drafted-pocket",Lightening,[P.face("body.z-max")],40,30,61,0,0,20);
		feature.draft.degrees=2;
		document.input.features=[feature];
		document.setup.features=[];
		document.setup.sourceFingerprint=materia.tooling.ToolingCodec.inputFingerprint(document.input);
		document.setup.sourceToMold=F.fromRotationMatrix(5,-7,100,[1,0,0,0,-1,0,0,0,-1]);
		var offset=60*Math.tan(2*Math.PI/180);
		var removed=60*(1200-70*offset+4*offset*offset/3);
		var removedZ=80*removed-3600*(600-140*offset/3+offset*offset);
		var resin=960000-removed,sourceCenter=(38400000-removedZ)/resin;
		var mass=kernel.castingMass(document);
		near(mass.massKg,resin*document.recipe.mixDensityKgM3*1e-9,"drafted casting mass uses actual resin material");
		near(mass.centerOfMassMoldMm.x,5,"casting center follows source-to-mold X placement");
		near(mass.centerOfMassMoldMm.y,-7,"casting center follows source-to-mold Y placement");
		near(mass.centerOfMassMoldMm.z,100-sourceCenter,"casting center follows actual drafted cavity and inverted orientation");
		var returned=kernel.castingMass(document);
		returned.centerOfMassMoldMm.z=-100;
		near(kernel.castingMass(document).centerOfMassMoldMm.z,mass.centerOfMassMoldMm.z,"cached casting facts are detached from callers");
		var previous=materia.tooling.casting.CastingMoldPlanner.geometry;
		materia.tooling.casting.CastingMoldPlanner.setGeometry(kernel);
		try {
			var cavity=materia.tooling.casting.MoldRegions.feature(document,"drafted-pocket");
			near(materia.tooling.casting.MoldRegions.volume(cavity,"drafted-pocket"),removed,
				"planner region uses the same derived source cavity in mold coordinates");
			near(kernel.resinIntersection(document,cavity),0,"source cavity contains no resin for datum collision checks");
			var candidate=materia.tooling.casting.MoldRegions.box({x:45,y:0,z:40,width:5,depth:5,height:20});
			near(kernel.resinIntersection(document,candidate),500,"native datum checks intersect actual resin");
			var partial=materia.tooling.ToolingCodec.snapshot(document);
			partial.setup.fillHeightMm=50;
			near(kernel.resinIntersection(partial,candidate),250,"datum/resin intersection stops at the declared fill height");
			var triangle=CastingRegion.Placed({solid:Prism(materia.casting.CastingProfiles.polygon(
				[{x:0,y:0},{x:20,y:0},{x:0,y:20}]),40),pose:F.identity()});
			near(materia.tooling.casting.MoldRegions.volume(triangle,"triangle"),8000,"non-box planning volume uses exact geometry");
			var corner=materia.tooling.casting.MoldRegions.box({x:17,y:17,z:10,width:2,depth:2,height:5});
			check(!materia.tooling.casting.MoldRegions.overlaps(triangle,corner,0,"triangle-corner"),
				"overlapping stock bounds cannot create a false material collision");
			var handling=materia.tooling.casting.CastHandling.moments(document,{x:-55,y:-57,z:20,width:120,depth:100,height:80});
			near(handling.massKg,mass.massKg,"handling uses the exact non-box kernel rather than stock bounds");
			near(handling.centerOfMassMoldMm.z,mass.centerOfMassMoldMm.z,"handling preserves the exact non-box center");
		} catch (error:Dynamic) {
			materia.tooling.casting.CastingMoldPlanner.setGeometry(previous);
			throw error;
		}
		materia.tooling.casting.CastingMoldPlanner.setGeometry(previous);
		document.recipe.mixDensityKgM3/=2;
		near(kernel.castingMass(document).massKg,mass.massKg/2,"density changes cannot reuse stale mass facts");
		Sys.println("NativeMoldKernelTests: "+assertions+" assertions passed");
	}
}
