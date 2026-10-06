import cadkit.modeling.Axis;
import cadkit.modeling.Location;
import cadkit.modeling.Vector;
import cadkit.sheetmetal.BendRule;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Segment;
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.SheetMaterial;
import cadkit.sheetmetal.WallEdge;
import cadkit.sheetmetal.WallOutline;
import cadkit.sheetmetal.process.AirBending;
import cadkit.sheetmetal.process.ProcessBendRule;
import machinekit.pressbrake.planning.BackgaugeStroke;
import machinekit.pressbrake.planning.BendOrientation;
import machinekit.pressbrake.planning.BendSetup;
import machinekit.pressbrake.planning.BendTimeModel;
import machinekit.pressbrake.planning.ConvexPrism;
import machinekit.pressbrake.planning.GaugeEdge;
import machinekit.pressbrake.planning.GeometricGripCheck;
import machinekit.pressbrake.planning.GripCheck;
import machinekit.pressbrake.planning.GripCandidateResult.GripAccepted;
import machinekit.pressbrake.planning.GripCandidateResult.GripRejected;
import machinekit.pressbrake.planning.GripperBox;
import machinekit.pressbrake.planning.PressGeometry;
import machinekit.pressbrake.planning.PressWallPrismQuery;
import machinekit.pressbrake.planning.PrismSatQuery;
import machinekit.pressbrake.planning.SheetGrip;

/** Whole-patch support and physical gripper clearance through registration, withdrawal and bending. */
class GripChecks {
	static var checks:Int = 0;
	public static function main():Void run();
	static function check(value:Bool,label:String):Void { checks++; if (!value) throw label; }
	static function box(x:Float,y:Float,z:Float,width:Float,length:Float,height:Float):ConvexPrism
		return new ConvexPrism("wall-probe",[new Vector(x,y,z),new Vector(x+width,y,z),
			new Vector(x+width,y+length,z),new Vector(x,y+length,z)],new Vector(0,0,height));
	static function diagnosis(code:String,action:Void->Void):SheetDiagnosis {
		checks++;
		try action() catch (failure:SheetDiagnosis) { if (failure.code == code) return failure; throw failure; }
		throw "Expected " + code;
	}
	public static function run():Void {
		checks = 0;
		var wallQuery = new PressWallPrismQuery();
		var ring = new WallOutline([Segment.arc(new Point2(0,0),5,0,2*Math.PI)],[[Segment.arc(new Point2(0,0),2,0,-2*Math.PI)]]);
		var material = box(2.5,-0.1,-1,0.2,0.2,0.5), cavity = box(-0.1,-0.1,-1,0.2,0.2,0.5);
		var tangent = box(5,-0.1,-1,1,0.2,0.5), penetrating = box(4.999,-0.01,-1,1,0.02,0.5);
		var surface = box(2.5,-0.1,0,0.2,0.2,1), below = box(2.5,-0.1,-3,0.2,0.2,0.5);
		var transforms = [Location.identity(),Location.translation(new Vector(31,17,23)).compose(Location.rotation(new Axis(new Vector(),new Vector(1,2,3)),0.7))];
		for (placement in transforms) {
			check(wallQuery.overlap(ring,1.5,placement,material.placed(placement)),"Exact curved wall retains physical material at arbitrary orientation");
			check(!wallQuery.overlap(ring,1.5,placement,cavity.placed(placement)),"Exact wall query preserves curved holes");
			check(!wallQuery.overlap(ring,1.5,placement,tangent.placed(placement)),"Outer circular boundary contact remains clear");
			check(wallQuery.overlap(ring,1.5,placement,penetrating.placed(placement)),"Small penetration of a circular wall boundary is detected");
			check(!wallQuery.overlap(ring,1.5,placement,surface.placed(placement)),"Gripper contact with a curved wall surface is not volume overlap");
			check(!wallQuery.overlap(ring,1.5,placement,below.placed(placement)),"Finite stock thickness is respected");
		}
		var press = new PressBrake(true,null,null,30), geometry = PressGeometry.of(press), station = press.toolLayout.stations()[0];
		var stock = new SheetMaterial("grip-steel",1.5,BendRule.Process(new ProcessBendRule(station.processTooling())),{uts:400,yield:250,youngs:205000});
		var sheet = new Sheet(stock), root = sheet.addWall(WallOutline.rectangle(60,100));
		var child = SheetDemoParts.flange(sheet,root,1,40,Math.PI/2,stock.defaultRadius(Math.PI/2));
		var planner = new BendSetup(geometry), bend = sheet.bends()[0], setup = planner.plan(sheet,sheet.flat(),bend.id,station,
			new BendOrientation(true,false,-10),new GaugeEdge(bend.wallB,new WallEdge(1)),new AirBending());
		var withdrawal = new BackgaugeStroke(setup.registration,20), envelope = new GripperBox(10,10,15);
		var geometric = new GeometricGripCheck(geometry,envelope,new PrismSatQuery()), checker:GripCheck = geometric;
		var grip = new SheetGrip(root.id,new Point2(10,50),true);
		check(envelope.supported(sheet,grip),"The complete patch lies on the chosen wall");
		checker.check(sheet,setup,grip,withdrawal); checks++;
		var underside = new SheetGrip(root.id,new Point2(10,50),false);
		checker.check(sheet,setup,underside,withdrawal); checks++;
		var unsupportedGrip = new SheetGrip(root.id,new Point2(1,50),true);
		var outcomes = geometric.checkCandidates(sheet,setup,[grip,unsupportedGrip,underside],withdrawal);
		check(outcomes.length == 3,"Batched grip clearance returns one typed result per authored candidate");
		switch outcomes[0] {
			case GripAccepted(value): check(BendTimeModel.sameGrip(value,grip),"Batched clearance retains the first accepted grip");
			case GripRejected(_,failure): throw failure;
		}
		switch outcomes[1] {
			case GripRejected(value,failure):
				check(BendTimeModel.sameGrip(value,unsupportedGrip) && failure.code == "press.grip-support",
					"Batched clearance retains each unsupported candidate and its diagnosis");
			case GripAccepted(_): throw "Batched clearance cannot accept an unsupported grip";
		}
		switch outcomes[2] {
			case GripAccepted(value): check(BendTimeModel.sameGrip(value,underside),"Batched clearance preserves candidate order");
			case GripRejected(_,failure): throw failure;
		}
		for (pose in setup.stroke()) {
			var frame = underside.frame(sheet,pose), expected = pose.wall(root.id).plane.toWorld(new Vector(10,50,-stock.thickness));
			check(frame.plane.origin.subtract(expected).length() < 1e-8
				&& Math.abs(frame.plane.normal.dot(pose.wall(root.id).plane.normal)+1) < 1e-8,
				"Underside grip follows the stock face and points away from material through the stroke");
		}
		check(!envelope.supported(sheet,new SheetGrip(root.id,new Point2(1,50),true)),"Patch crossing a wall boundary is refused");
		var oversized:GripCheck = new GeometricGripCheck(geometry,new GripperBox(10,10,350),new PrismSatQuery());
		var interference = diagnosis("press.grip-collision",function() {
			oversized.check(sheet,setup,new SheetGrip(root.id,new Point2(30,50),true),withdrawal);
		});
		check(interference.elements.indexOf("machine:ram") >= 0,"Gripper collision names the actual ram body");
		var candidates = geometric.candidates(sheet), repeated = geometric.candidates(sheet);
		check(candidates.length > 0 && candidates.length == repeated.length,"Supported grip candidates are deterministic");
		for (i in 0...candidates.length) {
			var a = candidates[i], b = repeated[i];
			check(a.wall == b.wall && a.center.x == b.center.x && a.center.y == b.center.y && a.faceA == b.faceA,"Candidate order and authored coordinates are stable");
		}
		var rounded = sheet.cutHole(root.id,[Segment.arc(new Point2(30,50),1,0,-2*Math.PI)]);
		var curvedSetup = planner.plan(sheet,sheet.flat(),bend.id,station,new BendOrientation(true,false,-10),
			new GaugeEdge(bend.wallB,new WallEdge(1)),new AirBending());
		checker.check(sheet,curvedSetup,grip,new BackgaugeStroke(curvedSetup.registration,20)); checks++;
		var punctured = sheet.cutHole(root.id,[Segment.arc(new Point2(12,52),1,0,-2*Math.PI)]);
		check(!envelope.supported(sheet,grip),"An off-center hole invalidates the patch despite solid material at its center");
		var unsupported = diagnosis("press.grip-support",function() { checker.check(sheet,curvedSetup,grip,withdrawal); });
		check(unsupported.elements.indexOf("wall:"+root.id) >= 0,"Unsupported grip names the authored wall");
		trace("Grip checks passed ("+checks+" assertions)");
	}
}
