import cadkit.modeling.Vector;
import cadkit.modeling.Location;
import cadkit.modeling.Plane;
import cadkit.sheetmetal.BendRule;
import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Segment;
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.SheetDemoParts;
import cadkit.sheetmetal.SheetMaterial;
import cadkit.sheetmetal.WallOutline;
import cadkit.sheetmetal.process.AirBending;
import cadkit.sheetmetal.process.ProcessBendRule;
import machinekit.pressbrake.ToolContactPrism;
import machinekit.pressbrake.ToolHorn;
import machinekit.pressbrake.ToolHornEnd;
import machinekit.pressbrake.ToolProfiles;
import machinekit.pressbrake.ToolSegment;
import machinekit.pressbrake.ToolingCatalogue;
import machinekit.pressbrake.planning.BendOrientation;
import machinekit.pressbrake.planning.ConvexPrism;
import machinekit.pressbrake.planning.PressBendSection;
import machinekit.pressbrake.planning.PressBendPrismQuery;
import machinekit.pressbrake.planning.PressCollisionGeometry;
import machinekit.pressbrake.planning.PressCollisionBody;
import machinekit.pressbrake.planning.PressMaterialQuery;
import machinekit.pressbrake.planning.CoalPrismQuery;
import machinekit.pressbrake.planning.PressContactQuery;
import machinekit.pressbrake.planning.PressGeometry;
import machinekit.pressbrake.planning.PressPrismSection;
import machinekit.pressbrake.planning.PressSectionQuery;
import machinekit.pressbrake.planning.PressSheetPose;
import machinekit.pressbrake.planning.PressToolContact;

/** Exact rounded sections distinguish intended contact from physical material penetration. */
class PressContactChecks {
	static var checks:Int = 0;
	public static function main():Void run();
	static function check(value:Bool, label:String):Void { checks++; if (!value) throw label; }
	static function rect(x:Float,y:Float,w:Float,h:Float):WallOutline
		return WallOutline.polygon([new Point2(x,y),new Point2(x+w,y),new Point2(x+w,y+h),new Point2(x,y+h)]);
	static function circle(x:Float,y:Float,radius:Float):WallOutline
		return new WallOutline([Segment.arc(new Point2(x,y),radius,0,2*Math.PI)]);
	static function probe(point:Vector):ConvexPrism {
		var r = 0.001;
		return new ConvexPrism("material-probe", [new Vector(point.x-r,point.y-r,point.z-r),
			new Vector(point.x+r,point.y-r,point.z-r),new Vector(point.x+r,point.y+r,point.z-r),
			new Vector(point.x-r,point.y+r,point.z-r)],new Vector(0,0,2*r));
	}
	static function polygon(points:Array<machinekit.pressbrake.ToolProfilePoint>):WallOutline
		return WallOutline.polygon([for (point in points) new Point2(point.y,point.z)]);
	static function nativeVolume(segment:ToolSegment):Void {
		var expected = 0.0;
		for (prism in segment.contactPrisms()) expected += prism.section.area()*(prism.end-prism.start);
		var part = segment.geometry(), actual = part.volume(); part.close();
		check(Math.abs(actual-expected) <= Math.max(1e-6,actual*1e-9),"Exact contact material matches CAD volume for "+ToolProfiles.identity(segment.profile));
	}
	public static function run():Void {
		checks = 0;
		var query = new PressSectionQuery(), a = rect(0,0,10,10);
		check(query.overlap(a,a),"Identical material sections overlap");
		check(!query.overlap(a,rect(10,0,2,10)),"Shared boundary has no material overlap");
		check(!query.overlap(a,rect(10,10,2,2)),"Shared corner has no material overlap");
		check(query.overlap(a,rect(9.999,3,2,2)),"Small positive planar penetration is detected");
		check(!query.overlap(a,rect(10.001,3,2,2)),"Small planar separation remains clear");
		check(query.overlap(a,rect(3,3,1,1)),"Containment has material overlap");
		var disk = circle(0,0,3);
		check(!query.overlap(disk,rect(3,-0.1,1,0.2)),"Exact line/circle tangency is clear");
		check(query.overlap(disk,rect(2.999,-0.1,1,0.2)),"Line/circle penetration is detected");
		check(!query.overlap(disk,circle(6,0,3)),"Circle/circle tangency is clear");
		check(query.overlap(disk,circle(5.999,0,3)),"Thin circle/circle material lens is detected");
		var ring = new WallOutline([Segment.arc(new Point2(0,0),5,0,2*Math.PI)],[[Segment.arc(new Point2(0,0),2,0,-2*Math.PI)]]);
		check(!query.overlap(ring,circle(0,0,0.5)),"A section hole stays empty");
		check(query.overlap(ring,rect(2.5,-0.1,0.1,0.2)),"Material beside a section hole remains");
		var strip = query.projection(ring,-0.1,0.1);
		check(strip.length == 4 && strip[1] < -1.99 && strip[2] > 1.99,"Projection retains a hole spanning the selected strip");
		check(query.projection(ring,-4,4).length == 2,"Projection includes material elsewhere along a wider strip");

		for (profile in ToolProfiles.catalogueProfiles()) nativeVolume(new ToolSegment(profile,100));
		for (punch in ToolingCatalogue.punches()) {
			nativeVolume(new ToolSegment(Punch(punch),100,new ToolHorn(ToolHornEnd.Left,20,25)));
			nativeVolume(new ToolSegment(Punch(punch),100,new ToolHorn(ToolHornEnd.Right,20,25)));
		}
		for (end in [ToolHornEnd.Left,ToolHornEnd.Right])
			nativeVolume(new ToolSegment(Punch(ToolingCatalogue.straight()),50,new ToolHorn(end,25,1)));
		var punch = ToolingCatalogue.straight(), flat = rect(-1,-1,2,1);
		check(!query.overlap(punch.contactSection(),flat),"Exact punch nose merely touches flat stock");
		check(query.overlap(polygon(punch.collisionSection(0.01).points()),flat),"Fixture independently exposes false conservative nose penetration");
		var die = ToolingCatalogue.dies()[1], beta = 40*Math.PI/180;
		var center = new Point2(6-Math.sin(beta),-1+Math.cos(beta));
		var tangent = new Point2(Math.cos(beta),Math.sin(beta)), normal = new Point2(-Math.sin(beta),Math.cos(beta));
		var wall = WallOutline.polygon([new Point2(center.x-3*tangent.x,center.y-3*tangent.y),
			new Point2(center.x+3*tangent.x,center.y+3*tangent.y),
			new Point2(center.x+3*tangent.x+1.5*normal.x,center.y+3*tangent.y+1.5*normal.y),
			new Point2(center.x-3*tangent.x+1.5*normal.x,center.y-3*tangent.y+1.5*normal.y)]);
		check(!query.overlap(die.contactSection(),wall),"Exact rounded shoulder merely touches its tangent stock plane");
		check(query.overlap(polygon(die.collisionSection(0.01).points()),wall),"Fixture exposes conservative shoulder penetration");

		var rod = new ConvexPrism("skew-rod",[new Vector(0,0,0),new Vector(10,10,0),new Vector(10,11,0),new Vector(0,1,0)],new Vector(0,0,1));
		var section = PressPrismSection.of(rod,0,1);
		check(section != null && Math.abs(section.area()-2)<1e-8,"Prism projection is clipped to the actual tool bed interval");
		check(section != null && !query.overlap(section,rect(9,0.4,0.1,0.1)),"Clipped projection does not fill distant unoccupied material");
		check(PressPrismSection.of(rod,10,11) == null,"Bed endpoint contact has no positive extrusion overlap");

		var press = new PressBrake(true,null,null,30), geometry = PressGeometry.of(press), station = press.toolLayout.stations()[0];
		var tooling = station.processTooling(), physical = geometry.station(station.id), contactQuery = new PressContactQuery();
		var bendQuery = new PressBendPrismQuery();
		var materialQuery = new PressMaterialQuery(new CoalPrismQuery());
		var stock = new SheetMaterial("exact-contact-steel",1.5,BendRule.Process(new ProcessBendRule(tooling)),{uts:400,yield:250,youngs:205000});
		for (sign in [1.0,-1.0]) {
			var sheet = new Sheet(stock), root = sheet.addWall(WallOutline.rectangle(60,100));
			var child = SheetDemoParts.flange(sheet,root,1,40,sign*Math.PI/2,stock.defaultRadius(sign*Math.PI/2));
			var bend = sheet.bends()[0], prediction = new AirBending().predict(stock,tooling,bend.angle,100);
			var endpoint:Null<PressSheetPose> = null;
			for (i in 0...37) {
				var pose = PressSheetPose.at(sheet,sheet.flat().set(bend.id,prediction.loadedAngle*i/36),bend.id,prediction,tooling,physical,
					new BendOrientation(sign>0,sign<0,-10));
				endpoint = pose;
				var travel = physical.openTipHeight-physical.dieTopHeight+pose.depth;
				var bodies = PressCollisionGeometry.sheet(sheet,pose), tools = PressCollisionGeometry.toolContacts(geometry,travel);
				for (body in bodies) for (tool in tools)
					check(contactQuery.overlap(sheet,pose,body,tool) == false,"Both bend signs clear physical tool material through the entire loaded stroke");
				var cap = PressBendSection.of(bend,pose.evaluation.bend(bend.id),pose.placement,-100,100);
				if (cap == null || cap.length != 1) throw "Missing full-width bend cap";
				var zone = pose.evaluation.bend(bend.id), expected = zone.allowance*zone.thickness;
				var materialPoint = pose.placement.plane.toWorld(zone.point(0.5,50,0.75));
				for (body in bodies) {
					var far = new PressCollisionBody(MachineMember("far-probe"),[probe(new Vector(1e6,1e6,1e6))],0.1);
					check(materialQuery.overlap(sheet,pose,body,far) == false,"Approximation error still permits certified distant clearance");
					switch body.owner {
						case SheetBend(_):
							var exact = new PressCollisionBody(MachineMember("interior-probe"),[probe(materialPoint)],0);
							check(materialQuery.overlap(sheet,pose,body,exact) == true,"Exact bend refinement proves a machine material collision");
							var uncertain = new PressCollisionBody(MachineMember("approximate-probe"),[probe(materialPoint)],0.1);
							check(materialQuery.overlap(sheet,pose,body,uncertain) == null,"Approximate overlap is unresolved rather than an invented physical collision");
						case SheetWall(_):
							var exact = new PressCollisionBody(MachineMember("wall-copy"),body.pieces(),0);
							check(materialQuery.overlap(sheet,pose,body,exact) == true,"Exact wall pieces prove positive machine material overlap");
							var approximate = new PressCollisionBody(MachineMember("bounded-wall-copy"),body.pieces(),0.01);
							check(materialQuery.overlap(sheet,pose,body,approximate) == true,"Inset material proves collision despite a declared boundary error");
						case _:
					}
				}
				check(bendQuery.overlap(bend,zone,pose.placement,probe(materialPoint)) == true,"Exact bend query detects interior material through flat and loaded strokes");
				var outside = pose.placement.plane.toWorld(zone.point(0.5,0,0.75)).add(pose.placement.direction(zone.frame.direction(new Vector(0,110,0))));
				check(bendQuery.overlap(bend,zone,pose.placement,probe(outside)) == false,"Exact bend query clips the finite axis rather than extending it indefinitely");
				var rotated = new Location(new Plane(new Vector(17,23,31),new Vector(0,1,0),new Vector(0,0,1))).compose(pose.placement);
				check(bendQuery.overlap(bend,zone,rotated,probe(rotated.plane.toWorld(zone.point(0.5,50,0.75)))) == true,"A previously formed bend may have an axis perpendicular to the tool bed");
				if (zone.angle != 0) {
					var cavity = rotated.plane.toWorld(zone.axis().add(zone.frame.direction(new Vector(0,50,0))));
					check(bendQuery.overlap(bend,zone,rotated,probe(cavity)) == false,"The curved bend's empty interior is not filled by its convex envelope");
				}
				if (zone.angle != 0) expected = Math.abs(zone.angle)*zone.thickness*(zone.curvedInnerRadius()+zone.thickness/2);
				check(Math.abs(cap[0].area()-expected)<1e-6,"Independent exact annular-sector material area");
			}
			if (endpoint == null) throw "Missing loaded endpoint";
			var loaded = endpoint, travel = physical.openTipHeight-physical.dieTopHeight+loaded.depth;
			var bodies = PressCollisionGeometry.sheet(sheet,loaded), penetrated = false;
			for (body in bodies) for (tool in PressCollisionGeometry.toolContacts(geometry,travel+0.2))
				if (contactQuery.overlap(sheet,loaded,body,tool) == true) penetrated = true;
			check(penetrated,"Overclosed punch is a real material collision, not excused as intended contact");
			var highDie = false;
			for (body in bodies) for (tool in PressCollisionGeometry.toolContacts(geometry,travel)) switch tool.owner {
				case Die(_, _):
					var raised = new PressToolContact(tool.owner,[for (prism in tool.prisms()) prism.placed(0,0,0.2)]);
					if (contactQuery.overlap(sheet,loaded,body,raised) == true) highDie = true;
				case _:
			}
			check(highDie,"Raised die intersects actual stock and is diagnosed as material overlap");
			var allowance = sheet.bendAllowance(bend);
			var changed = sheet.cutBend(bend.id,rect(allowance*0.4,40,allowance*0.2,20),"exact-contact-hole");
			check(bendQuery.overlap(sheet.bend(bend.id),loaded.evaluation.bend(bend.id),loaded.placement,probe(new Vector())) == null,"Cut bends explicitly require the query retaining along-axis variation");
			var cutBodies = PressCollisionGeometry.sheet(sheet,loaded), materialPoint = loaded.placement.plane.toWorld(loaded.evaluation.bend(bend.id).point(0.5,50,0.75));
			var holeProbe = new PressToolContact(Punch("probe",0),[new ToolContactPrism(rect(materialPoint.y-0.001,materialPoint.z-0.001,0.002,0.002),materialPoint.x-0.01,materialPoint.x+0.01)]);
			var wideProbe = new PressToolContact(Punch("probe",0),[new ToolContactPrism(rect(materialPoint.y-0.001,materialPoint.z-0.001,0.002,0.002),-100,100)]);
			for (body in cutBodies) switch body.owner {
				case SheetBend(_):
					check(contactQuery.overlap(sheet,loaded,body,holeProbe) == false,"Exact bend projection preserves a developed hole inside the tool interval");
					check(contactQuery.overlap(sheet,loaded,body,wideProbe) == true,"Material outside the cut along a wider tool interval participates");
				case _:
			}
		}
		trace("Press contact checks passed ("+checks+" assertions)");
	}
}
