import cadkit.modeling.Vector;
import cadkit.sheetmetal.BendRule;
import cadkit.sheetmetal.OutlineBoolean;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.SheetDiagnosis;
import cadkit.sheetmetal.SheetMaterial;
import cadkit.sheetmetal.WallEdge;
import cadkit.sheetmetal.WallOutline;
import cadkit.sheetmetal.process.AirBending;
import cadkit.sheetmetal.process.ProcessBendRule;
import machinekit.pressbrake.ToolHorn;
import machinekit.pressbrake.ToolLayout;
import machinekit.pressbrake.ToolRow;
import machinekit.pressbrake.ToolSegment;
import machinekit.pressbrake.ToolStation;
import machinekit.pressbrake.ToolingCatalogue;
import machinekit.pressbrake.planning.BendOrientation;
import machinekit.pressbrake.planning.BendSetup;
import machinekit.pressbrake.planning.ConvexPrism;
import machinekit.pressbrake.planning.GaugeEdge;
import machinekit.pressbrake.planning.PressCollisionBody;
import machinekit.pressbrake.planning.PressCollisionGeometry;
import machinekit.pressbrake.planning.PressCollisionOwner;
import machinekit.pressbrake.planning.PressGeometry;
import machinekit.pressbrake.planning.PressSheetPose;
import machinekit.pressbrake.planning.PrismSatQuery;

/** Independent press-frame material probes verify row placement, relief and evaluated sheet ownership. */
class PressCollisionGeometryChecks {
	static var checks:Int = 0;
	public static function main():Void run();
	static function check(value:Bool, label:String):Void { checks++; if (!value) throw label; }
	static function probe(point:Vector):ConvexPrism {
		var x = point.x, y = point.y, z = point.z, h = 0.001;
		return new ConvexPrism("probe", [new Vector(x-h,y-h,z-h), new Vector(x+h,y-h,z-h),
			new Vector(x+h,y+h,z-h), new Vector(x-h,y+h,z-h)], new Vector(0,0,2*h));
	}
	static function overlaps(bodies:Array<PressCollisionBody>, point:Vector):Bool {
		var query = new PrismSatQuery(), target = probe(point);
		for (body in bodies) for (piece in body.pieces()) if (query.overlap(piece,target)) return true;
		return false;
	}
	static function member(bodies:Array<PressCollisionBody>, id:String):PressCollisionBody {
		for (body in bodies) switch body.owner { case MachineMember(name): if (name == id) return body; case _: }
		throw "Missing physical member " + id;
	}
	static function minimum(body:PressCollisionBody, axis:Int):Float {
		var result = Math.POSITIVE_INFINITY;
		for (piece in body.pieces()) for (point in piece.vertices())
			result = Math.min(result, axis == 0 ? point.x : axis == 1 ? point.y : point.z);
		return result;
	}
	static function volume(body:PressCollisionBody):Float {
		var result = 0.0;
		for (piece in body.pieces()) {
			var vertices = piece.vertices(), count = Std.int(vertices.length / 2), area = new Vector();
			for (i in 0...count) area = area.add(vertices[i].subtract(vertices[0]).cross(vertices[(i + 1) % count].subtract(vertices[0])));
			result += Math.abs(area.dot(piece.extrusion)) / 2;
		}
		return result;
	}
	static function refusal(action:Void->Void):Void {
		try { action(); } catch (failure:SheetDiagnosis) { check(failure.code == "press.collision-travel", "Named collision travel refusal"); return; }
		throw "Expected collision travel refusal";
	}
	public static function run():Void {
		checks = 0;
		var punch = ToolingCatalogue.straight(), die = ToolingCatalogue.dies()[1];
		var station = new ToolStation("split", new ToolRow([new ToolSegment(Punch(punch),100),
			new ToolSegment(Punch(punch),50,new ToolHorn(Left,10,25))],[20]),
			new ToolRow([new ToolSegment(SingleV(die),100),new ToolSegment(SingleV(die),50)],[30]),700);
		var other = new ToolStation("neck",new ToolRow([new ToolSegment(Punch(ToolingCatalogue.gooseneck()),100)],[]),
			new ToolRow([new ToolSegment(SingleV(die),100)],[]),1100);
		var layout = new ToolLayout(1600,[station,other]), geometry = PressGeometry.reference(layout,true,30);
		var tools = PressCollisionGeometry.tools(geometry,0), contact = geometry.station("split");
		check(tools.length == 6,"All segments from both installed stations participate");
		check(overlaps(tools,new Vector(-50,0,contact.openTipHeight+5)),"Punch material starts at installed bed offset");
		check(!overlaps(tools,new Vector(10,0,contact.openTipHeight+5)),"Punch gap remains empty");
		check(overlaps(tools,new Vector(25,0,contact.openTipHeight+5)),"Horn retains its working nose");
		check(!overlaps(tools,new Vector(25,0,contact.openTipHeight+50)),"Horn mounting relief remains empty");
		check(overlaps(tools,new Vector(35,0,contact.openTipHeight+50)),"Unrelieved segment material remains");
		check(overlaps(tools,new Vector(-50,15,contact.dieTopHeight-10)),"Die section uses top and bed datums");
		check(!overlaps(tools,new Vector(20,15,contact.dieTopHeight-10)),"Independent die gap remains empty");
		check(overlaps(tools,new Vector(40,15,contact.dieTopHeight-10)),"Die segment after gap is correctly placed");
		var neck = geometry.station("neck");
		check(!overlaps(tools,new Vector(350,10,neck.openTipHeight+50)),"Gooseneck concavity remains clear");
		check(overlaps(tools,new Vector(350,-10,neck.openTipHeight+50)),"Adjacent station gooseneck web participates");
		var lowered = PressCollisionGeometry.tools(geometry,80);
		check(overlaps(lowered,new Vector(-50,0,contact.openTipHeight-75)),"Punch follows physical ram travel");
		check(overlaps(lowered,new Vector(-50,15,contact.dieTopHeight-10)),"Die remains fixed during the stroke");
		check(!overlaps(tools,new Vector(-50,0,contact.openTipHeight-75)),"Open punch does not occupy lowered material");
		var punches = 0, dies = 0;
		for (body in tools) {
			switch body.owner { case Punch(id,index): punches++; check(index >= 0 && (id == "split" || id == "neck"),"Typed punch ownership");
				case Die(id,index): dies++; check(index >= 0 && (id == "split" || id == "neck"),"Typed die ownership"); case _: throw "Wrong tool owner"; }
			check(body.error == 0.01,"Tool conservative error is retained");
		}
		check(punches == 3 && dies == 3,"Independent row owners");
		var moving = PressCollisionGeometry.machineMembers(geometry,80,50,20), initial = PressCollisionGeometry.machineMembers(geometry,0,0,0);
		check(minimum(member(moving,"ram"),2) == minimum(member(initial,"ram"),2)-80,"Ram envelope moves downward");
		check(minimum(member(moving,"fingerLeft"),1) == minimum(member(initial,"fingerLeft"),1)+50,"Finger X moves toward backgauge");
		check(minimum(member(moving,"fingerLeft"),2) == minimum(member(initial,"fingerLeft"),2)+20,"Finger R lifts physical envelope");
		check(minimum(member(moving,"fingerRight"),0) == 20,"Manual lateral finger setup is retained");
		var copies = moving[0].pieces(); var removed = copies.pop();
		check(moving[0].pieces().length == 1,"Body owns its piece array");
		refusal(function() { var result = PressCollisionGeometry.tools(geometry,101); });
		refusal(function() { var result = PressCollisionGeometry.machineMembers(geometry,0,-1,0); });
		refusal(function() { var result = PressCollisionGeometry.machineMembers(geometry,0,0,121); });

		var press = new PressBrake(true,null,null,30), actual = PressGeometry.of(press), installed = press.toolLayout.stations()[0];
		check(actual.installedTools == press.toolLayout,"Assembly owns installed tool specification");
		var actualMachine = PressCollisionGeometry.machineMembers(actual,80,50,20), frame = member(actualMachine,"frameRight");
		check(!overlaps([frame],new Vector(825,100,950)),"Open frame throat is not replaced by a filled envelope");
		check(overlaps([frame],new Vector(825,250,950)),"Rear frame web remains material");
		check(overlaps([frame],new Vector(825,100,700)),"Lower frame web remains material");
		check(overlaps([frame],new Vector(825,100,1200)),"Upper frame web remains material");
		check(volume(frame) == 50*(600*1600-330*320),"Independent frame material volume");
		for (entry in press.components()) if (entry.id == "frameRight") {
			var part = entry.component.geometry();
			var same = Math.abs(part.volume()-volume(frame)) < 1e-6; part.close();
			check(same,"Collision frame boxes preserve exact CAD material volume");
		}
		check(minimum(member(actualMachine,"bed"),2) == 600,"Bed follows its physical assembly placement");
		check(minimum(member(actualMachine,"gaugeX"),1) == 10,"Backgauge carriage follows X only");
		check(minimum(member(actualMachine,"gaugeX"),2) == 660,"Backgauge carriage does not follow R");
		check(minimum(member(actualMachine,"gaugeR"),2) == 810,"Backgauge lift follows physical R");
		check(minimum(member(actualMachine,"ramBracketLeft"),2) == 1120,"Ram bracket follows physical downward stroke");
		check(overlaps([member(actualMachine,"supportLeft")],new Vector(-450,-240,790)),"Fixed support participates in collision geometry");
		var withoutR = PressGeometry.of(new PressBrake(false,null,null,30));
		var absentR = true;
		for (body in PressCollisionGeometry.machineMembers(withoutR,0,0,0)) switch body.owner {
			case MachineMember(id): if (id == "gaugeR") absentR = false;
			case _:
		}
		check(absentR,"Optional R hardware is absent when not installed");
		refusal(function() { var result = PressCollisionGeometry.machineMembers(withoutR,0,0,1); });
		var stock = new SheetMaterial("collision-steel",1.5,BendRule.Process(new ProcessBendRule(installed.processTooling())),{uts:400,yield:250,youngs:205000});
		var sheet = SheetDemoParts.p2(stock), root = sheet.walls()[0], bend = sheet.bends()[0];
		var hole = WallOutline.polygon([new Point2(20,50),new Point2(30,50),new Point2(30,70),new Point2(20,70)]);
		var cuts = OutlineBoolean.regions(root.outline,hole,"probe-hole");
		check(cuts.length == 1,"Sheet hole keeps one connected wall");
		var changed = sheet.updateWallMaterial(root.id,cuts[0]);
		var setup = new BendSetup(actual).plan(sheet,sheet.flat(),bend.id,installed,new BendOrientation(true,false,-10),
			new GaugeEdge(bend.wallB,new WallEdge(1)),new AirBending());
		var foreign = new ToolStation(installed.id,new ToolRow([new ToolSegment(Punch(ToolingCatalogue.acute()),100)],[]),installed.dies,installed.offset);
		var refused = false;
		try { var result = new BendSetup(actual).plan(sheet,sheet.flat(),bend.id,foreign,new BendOrientation(true,false,-10),
			new GaugeEdge(bend.wallB,new WallEdge(1)),new AirBending()); }
		catch (failure:SheetDiagnosis) { refused = failure.code == "press.station-not-installed"; }
		check(refused,"An uninstalled specification cannot reuse a physical station identity");
		for (index in [0,18,36]) {
			var pose = setup.stroke()[index], bodies = PressCollisionGeometry.sheet(sheet,pose), walls:Array<PressCollisionBody> = [];
			check(bodies.length == sheet.walls().length+sheet.bends().length,"Every evaluated material owner participates");
			for (body in bodies) switch body.owner {
				case SheetWall(id): if (id == root.id) walls.push(body);
				case SheetBend(id): check(body.error <= 0.01+1e-8,"Placed bend retains bounded curvature error");
				case _: throw "Wrong sheet owner";
			}
			check(!overlaps(walls,pose.wall(root.id).plane.toWorld(new Vector(25,60,-0.75))),"Placed wall hole remains clear through loaded stroke");
			check(overlaps(walls,pose.wall(root.id).plane.toWorld(new Vector(10,60,-0.75))),"Placed wall material follows actual FK");
			var zones:Array<PressCollisionBody> = [];
			for (body in bodies) switch body.owner { case SheetBend(id): if (id == bend.id) zones.push(body); case _: }
			var zone = pose.evaluation.bend(bend.id);
			check(overlaps(zones,pose.placement.plane.toWorld(zone.point(0.3,60,0.75))),"Active bend pieces follow press placement");
		}
		var opposite = SheetDemoParts.p3(stock), negative = opposite.bends()[1];
		var prediction = new AirBending().predict(stock,installed.processTooling(),negative.angle,120);
		for (fraction in [0.0,0.5,1.0]) {
			var pose = PressSheetPose.at(opposite,opposite.flat().set(negative.id,prediction.loadedAngle*fraction),negative.id,
				prediction,installed.processTooling(),actual.station(installed.id),new BendOrientation(false,true,-10));
			var bodies = PressCollisionGeometry.sheet(opposite,pose), active:Array<PressCollisionBody> = [];
			for (body in bodies) switch body.owner { case SheetBend(id): if (id == negative.id) active.push(body); case _: }
			check(overlaps(active,pose.placement.plane.toWorld(pose.evaluation.bend(negative.id).point(0.3,60,0.75))),
				"Negative bend material follows face-B-up placement through loaded stroke");
		}
		trace("Press collision geometry checks passed ("+checks+" assertions)");
	}
}
