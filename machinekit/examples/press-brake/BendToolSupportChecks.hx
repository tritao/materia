import cadkit.modeling.Location;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.sheetmetal.BendZone;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Segment;
import cadkit.sheetmetal.WallOutline;
import cadkit.sheetmetal.SheetDemoParts;
import machinekit.pressbrake.ToolHorn;
import machinekit.pressbrake.ToolHornEnd;
import machinekit.pressbrake.ToolProfile;
import machinekit.pressbrake.ToolLayout;
import machinekit.pressbrake.ToolRow;
import machinekit.pressbrake.ToolSegment;
import machinekit.pressbrake.ToolStation;
import machinekit.pressbrake.ToolingCatalogue;
import machinekit.pressbrake.ToolContactPrism;
import machinekit.pressbrake.planning.BendCandidates;
import machinekit.pressbrake.planning.PressGeometry;
import machinekit.pressbrake.planning.PressBendToolBounds;
import machinekit.pressbrake.planning.PressBendToolSeparation;

/** Independent extrema and interior witnesses constrain the exact separation certificate. */
class BendToolSupportChecks {
	static var checks:Int = 0;
	public static function main():Void run();
	public static function run():Void {
		checks = 0;
		checkHornCandidates();
		var identity = Location.identity(), positive = zone(Math.PI/2), negative = zone(-Math.PI/2);
		range(positive,identity,0,0,5); range(positive,identity,1,0,10); range(positive,identity,2,-2,3);
		range(negative,identity,0,0,5); range(negative,identity,2,-5,0);
		var c = Math.sqrt(0.5), tilted = new Location(new Plane(new Vector(),new Vector(c,0,c),new Vector(-c,0,c)));
		range(positive,tilted,0,0,c*(5*(Math.sqrt(2)-1)+2));
		for (angle in [0.0,1e-8,-1e-8,Math.PI/2,-Math.PI/2,Math.PI,-Math.PI]) {
			var stock = zone(angle);
			for (placement in [identity,tilted,Location.translation(new Vector(7,-3,11)).compose(tilted)])
				for (axis in [Vector.X(),Vector.Y(),Vector.Z(),new Vector(1,2,3).normalized()]) {
					var bound = PressBendToolBounds.projectionAlong(stock,placement,axis);
					for (i in 0...201) for (along in [0.0,10.0]) for (depth in [0.0,1.0,2.0]) {
						var value = placement.plane.toWorld(stock.point(i/200,along,depth)).dot(axis);
						check(value >= bound[0]-1e-8 && value <= bound[1]+1e-8,"Exact support contains independent interior and boundary samples");
					}
				}
		}
		var hem = new BendZone(2,identity,Math.PI,2*Math.PI,10,2,2);
		range(hem,identity,0,0,2); range(hem,identity,2,-2,2);
		check(!PressBendToolSeparation.clear(hem,identity,tool(rect(4,-0.1,2,0.2),0.9,1.1)),
			"Zero-radius hem retains its interior material witness");
		check(PressBendToolSeparation.clear(positive,identity,tool(rect(1,-4,8,2),0,5)),"Exact face tangency remains clear");
		check(!PressBendToolSeparation.clear(positive,identity,tool(rect(-20,-20,40,40),-20,20)),"Containing tool cannot receive a clearance certificate");
		check(!PressBendToolSeparation.clear(positive,identity,tool(rect(2,0.5,2,1),2,4)),"Known annular material penetration cannot receive clearance");
		var disk = new WallOutline([Segment.arc(new Point2(4,1),2,0,2*Math.PI)]);
		check(!PressBendToolSeparation.clear(positive,identity,tool(disk,2,4)),"Rounded tool penetration retains material ownership");
		// These points lie strictly inside the strip. Each small tool contains its witness;
		// the certificate must refuse clearance across both signs and oblique placements.
		for (stock in [positive,negative]) for (placement in [identity,tilted]) for (i in 1...10) {
			var point = placement.plane.toWorld(stock.point(i/10,5,1));
			var obstacle = tool(rect(point.y-0.1,point.z-0.1,0.2,0.2),point.x-0.1,point.x+0.1);
			check(!PressBendToolSeparation.clear(stock,placement,obstacle),"Interior witness forbids a false separation certificate");
		}
		trace("Bend tool support checks passed: " + checks);
	}
	static function checkHornCandidates():Void {
		var sheet = SheetDemoParts.p1();
		var punch:ToolProfile = Punch(ToolingCatalogue.straight());
		var die = new ToolRow([new ToolSegment(SingleV(ToolingCatalogue.dies()[1]),200)],[]);
		var plain = new ToolStation("plain",new ToolRow([new ToolSegment(punch,100),new ToolSegment(punch,50)],[0]),die,700);
		var relieved = new ToolStation("relieved",new ToolRow([
			new ToolSegment(punch,100,new ToolHorn(ToolHornEnd.Left,25,25)),new ToolSegment(punch,50)],[0]),die,700);
		var plainGeometry = PressGeometry.reference(new ToolLayout(1600,[plain]),true,30);
		var hornGeometry = PressGeometry.reference(new ToolLayout(1600,[relieved]),true,30);
		check(BendCandidates.centers(sheet,sheet.bends()[0],plain,plainGeometry,true,false).indexOf(-25) < 0,
			"Unrelieved fixture has no event or midpoint at the horn transition alignment");
		check(BendCandidates.centers(sheet,sheet.bends()[0],relieved,hornGeometry,true,false).indexOf(-25) >= 0,
			"Horn boundary at -75 mm aligned with the -50 mm seam end contributes center -25 mm");
	}
	static function zone(angle:Float):BendZone return new BendZone(1,Location.identity(),angle,2*Math.PI,10,2,1);
	static function range(stock:BendZone,placement:Location,axis:Int,low:Float,high:Float):Void {
		var actual = PressBendToolBounds.projection(stock,placement,axis);
		check(Math.abs(actual[0]-low) <= 1e-8 && Math.abs(actual[1]-high) <= 1e-8,"Analytic coordinate extrema match independent dimensions");
	}
	static function rect(y:Float,z:Float,width:Float,height:Float):WallOutline return WallOutline.polygon([
		new Point2(y,z),new Point2(y+width,z),new Point2(y+width,z+height),new Point2(y,z+height)]);
	static function tool(section:WallOutline,first:Float,last:Float):ToolContactPrism return new ToolContactPrism(section,first,last);
	static function check(value:Bool,message:String):Void { checks++; if (!value) throw message; }
}
