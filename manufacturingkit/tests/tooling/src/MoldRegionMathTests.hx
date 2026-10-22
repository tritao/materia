import materia.casting.CastingRegion;
import materia.casting.CastingFeature.CastingGeometrySolid;
import materia.tooling.casting.MoldRegions as R;
import materia.assembly.AssemblyFrames as F;

class MoldRegionMathTests {
	static var assertions=0;
	static function check(value:Bool,message:String):Void {assertions++;if (!value) throw message;}
	static function near(actual:Float,expected:Float,message:String):Void
		check(Math.abs(actual-expected)<1e-7,message+": "+actual+" vs "+expected);
	public static function main():Void run();
	public static function run():Void {
		var a=R.box({x:0,y:0,z:0,width:20,depth:30,height:40});
		var b=R.box({x:5,y:10,z:10,width:10,depth:10,height:10});
		near(R.volume(a,"stock-a"),24000,"box volume uses exact algebra");
		near(R.volume(CastingRegion.Union([a,b]),"union"),24000,"overlapping boxes are not counted twice");
		var cut=CastingRegion.Difference(a,[b]),properties=R.moments(cut,"cut");
		near(properties.volumeMm3,23000,"box subtraction retains exact material");
		near(properties.firstMomentMm4.x,230000,"first moment uses remaining material");
		near(properties.firstMomentMm4.z,465000,"removed material contributes its own centroid");
		var moved=R.moments(R.moved(cut,F.translation(7,-2,3)),"moved");
		near(moved.firstMomentMm4.x,391000,"placement moves the entire boolean expression");
		near(moved.firstMomentMm4.z,534000,"placement retains subtraction depth");
		var contact=R.box({x:20,y:10,z:10,width:10,depth:10,height:10});
		check(!R.overlaps(a,contact,0,"contact"),"zero-clearance contact has no interior overlap");
		contact=R.box({x:19.5,y:10,z:10,width:10,depth:10,height:10});
		check(R.overlaps(a,contact,0,"overlap"),"positive interior overlap is detected");
		check(!R.overlaps(a,contact,100,"tolerance"),"contact tolerance is explicit in volume units");
		var rectangle=CastingRegion.Placed({solid:CastingGeometrySolid.Prism(
			materia.casting.CastingProfiles.rectangle(-10,-15,20,30),40),pose:F.identity()});
		near(R.volume(rectangle,"rectangular-prism"),24000,"exact rectangular profiles retain the algebraic path");
		var triangle=CastingRegion.Placed({solid:CastingGeometrySolid.Prism(
			materia.casting.CastingProfiles.polygon([{x:0,y:0},{x:20,y:0},{x:0,y:20}]),40),pose:F.identity()});
		check(R.boxes(triangle)==null,"a triangular cavity is never replaced by its bounds");
		var failure="";
		try R.volume(triangle,"triangle-feature") catch (error:Dynamic) failure=Std.string(error);
		check(failure.indexOf("Exact tooling geometry kernel required: triangle-feature")>=0,
			"missing native geometry identifies the affected feature");
		var volume:materia.tooling.casting.CastingMoldPlan.MoldVolume={id:"profile-stock",material:FormerStock,
			pieces:[],regions:[],cuts:[],profiles:[{panelId:"wall",operationId:"access",pose:F.identity(),depth:10,
				contour:materia.casting.CastingProfiles.rectangle(0,0,20,20),
				subtract:[R.box({x:7.5,y:7.5,z:0,width:5,depth:5,height:10})]}]};
		near(R.volume(R.tooling(volume),volume.id),3750,"the shared tooling expression retains exact sealing-profile exclusions");
		volume.regions.push(R.box({x:7.5,y:7.5,z:0,width:5,depth:5,height:10}));
		near(R.volume(R.tooling(volume),volume.id),4000,"profile and independent solid terms form a union without double counting");
		Sys.println("MoldRegionMathTests: "+assertions+" assertions passed");
	}
}
