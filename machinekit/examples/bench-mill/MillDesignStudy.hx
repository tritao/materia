import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.PosedParts;
import machinekit.motion.GuideSupport;
import machinekit.motion.GuideSupport.GuideSupportWrench;
import machinekit.milling.*;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Review packet from actual component datums. Reference calculations explicitly
 * exclude foundation/joint compliance, preload, fatigue and modal qualification.
 */
class MillDesignStudy {
	static function descends(id:String, root:String, parents:Map<String,String>):Bool {
		var seen:Map<String,Bool> = [];
		while (id != null && !seen.exists(id)) { if (id == root) return true; seen.set(id,true); id = parents.get(id); }
		return false;
	}
	static function add(w:GuideSupportWrench, fx:Float, fy:Float, fz:Float, x:Float, y:Float, z:Float):Void {
		w.fx += fx; w.fy += fy; w.fz += fz;
		w.mx += y*fz-z*fy; w.my += z*fx-x*fz; w.mz += x*fy-y*fx;
	}
	public static function guideLoads(assembly:MachineAssembly, frame:MillFrame):Array<Dynamic> {
		if (frame.xyAxis == null || frame.zAxis == null) throw "Design load study needs all three mechanical axes";
		var model = new AssemblyModel("mm"); assembly.addTo(model, "");
		var state = new AssemblyState(model.definition("mill-reference-loads"));
		var l:MillXYStageLayout = frame.xyAxis, z:MillZAxisLayout = frame.zAxis;
		var parents = assembly.mateParents(), members = assembly.components();
		var studies = [
			{id:"x", joint:"xyStage/x", rail:"xyStage/railXFront", screw:"xyStage/screwX", root:"xyStage/blockX0",
			 blocks:["xyStage/blockX0","xyStage/blockX1","xyStage/blockX2","xyStage/blockX3"]},
			{id:"y", joint:"xyStage/y", rail:"xyStage/railYLeft", screw:"xyStage/screwY", root:"xyStage/blockY0",
			 blocks:["xyStage/blockY0","xyStage/blockY1","xyStage/blockY2","xyStage/blockY3"]},
			{id:"z", joint:"zAxis/z", rail:"zAxis/railLeft", screw:"zAxis/screw", root:"zAxis/blockLeftLower",
			 blocks:["zAxis/blockLeftLower","zAxis/blockLeftUpper","zAxis/blockRightLower","zAxis/blockRightUpper"]}];
		var results:Array<Dynamic> = [];
		for (study in studies) {
			var moving = members.filter(m -> descends(m.id,study.root,parents));
			var masses = [for (m in moving) m.component.massProperties()];
			var total = 0.0; for (mass in masses) total += mass.mass;
			var worst = 0.0;
			for (x in [-l.xHalfTravel,l.xHalfTravel]) for (y in [-l.yHalfTravel,l.yHalfTravel]) for (q in [z.lower,0.0]) {
				state.setJoint("xyStage/x",x); state.setJoint("xyStage/y",y); state.setJoint("zAxis/z",q); state.forwardKinematics();
				var inverse = F.inverse(state.worldPose(study.rail));
				var points = [for (id in study.blocks) { var p = state.worldConnector(id,"top"); F.transformPoint(inverse,p.x,p.y,p.z); }];
				var cx = 0.0, cy = 0.0, cz = 0.0;
				var minX = points[0].x, maxX = minX, minZ = points[0].z, maxZ = minZ;
				for (p in points) { cx += p.x/4; cy += p.y/4; cz += p.z/4;
					minX=Math.min(minX,p.x); maxX=Math.max(maxX,p.x); minZ=Math.min(minZ,p.z); maxZ=Math.max(maxZ,p.z); }
				var shaft = state.worldPose(study.screw), drive = F.transformPoint(inverse,shaft.x,shaft.y,shaft.z);
				var support = new GuideSupport((maxX-minX)*0.001,(maxZ-minZ)*0.001,(drive.x-cx)*0.001,(drive.y-cy)*0.001);
				var tool = state.worldConnector("spindle/tool","tip");
				var tip = F.transformPoint(inverse,tool.x,tool.y,study.id=="z"?tool.z:l.tableTop+50);
				for (loadCase in 0...8) {
					var w:GuideSupportWrench={fx:0,fy:0,fz:0,mx:0,my:0,mz:0};
					var ax=(loadCase&1)==0?-1.0:1.0, ay=(loadCase&2)==0?-1.0:1.0, az=(loadCase&4)==0?-1.0:1.0;
					for (i in 0...moving.length) {
						var mass=masses[i], c=mass.centreOfMass;
						var world=F.transformPoint(state.worldPose(moving[i].id),c.x,c.y,c.z);
						var p=F.transformPoint(inverse,world.x,world.y,world.z);
						var force=F.transformVector(inverse,-mass.mass*ax,-mass.mass*ay,mass.mass*(-9.80665-az));
						add(w,force.x,force.y,force.z,(p.x-cx)*0.001,(p.y-cy)*0.001,(p.z-cz)*0.001);
					}
					if (study.id!="z") {
						var p=F.transformPoint(inverse,0,l.centreY,l.tableTop+50);
						var force=F.transformVector(inverse,-20*ax,-20*ay,20*(-9.80665-az));
						add(w,force.x,force.y,force.z,(p.x-cx)*0.001,(p.y-cy)*0.001,(p.z-cz)*0.001);
					}
					for (cuttingCase in 0...8) {
						var cut:GuideSupportWrench={fx:w.fx,fy:w.fy,fz:w.fz,mx:w.mx,my:w.my,mz:w.mz};
						var force=F.transformVector(inverse,(cuttingCase&1)==0?-500:500,(cuttingCase&2)==0?-500:500,(cuttingCase&4)==0?-500:500);
						add(cut,force.x,force.y,force.z,(tip.x-cx)*0.001,(tip.y-cy)*0.001,(tip.z-cz)*0.001);
						worst=Math.max(worst,support.requireCapacity(cut,study.id=="z"?z.block.spec.staticLoadCapacity:l.block.spec.staticLoadCapacity,2));
					}
				}
			}
			results.push({axis:study.id,cadMovingMassKg:total,worstStaticUtilization:worst});
		}
		return results;
	}
	public static function write(assembly:MachineAssembly, frame:MillFrame, directory:String):Void {
		var folder=directory+"/design-review";
		if (!sys.FileSystem.exists(folder)) sys.FileSystem.createDirectory(folder);
		var ids=["frame/base/casting","frame/column/casting","xyStage/saddle","xyStage/table","xyStage/motorMountX","zAxis/head","zAxis/carriage"];
		var poses:Map<String,AssemblyFrame> = assembly.solvedPoses(), cache=new PosedParts();
		var bounds:Array<Dynamic> = [];
		try for (id in ids) {
			var m=assembly.components().filter(row->row.id==id)[0];
			var local=cache.local(m.component), box=PosedParts.boxOf(local);
			local.shape.exportStep(folder+"/"+StringTools.replace(id,"/","-")+".step");
			bounds.push({member:id,localBoundsMm:box});
		} catch(e:Dynamic) {cache.close();throw e;}
		cache.close();
		// Section-only comparison of the actual deck depth and two 34x48 ribs.
		var xy:machinekit.milling.MillXYStageLayout=frame.xyAxis;
		var zLayout:MillZAxisLayout=frame.zAxis;
		var depth=xy.tableDepth;
		var area=depth*40+2*34*48, centroid=(depth*40*20+2*34*48*(-24))/area;
		var deckI=depth*Math.pow(40,3)/12;
		var ribbedI=deckI+depth*40*Math.pow(20-centroid,2)+2*(34*Math.pow(48,3)/12+34*48*Math.pow(-24-centroid,2));
		var report={status:"concept review; manufacturing release blocked", units:"mm N kg",
			assumptions:{cuttingForcePerWorldAxisN:500,payloadKg:20,payloadHeightAboveTableMm:50,accelerationEnvelopeMetresPerSecondSquared:1,guideStaticSafetyFactor:2,
				beltCordRigidityPerWidthNPerMm:5000},
			guideLoads:guideLoads(assembly,frame),
			zHead:{carriageThicknessMm:zLayout.carriageThickness,guideBlockVerticalCentersMm:2*zLayout.blockSpan,
				rearFlangeWidthMm:zLayout.carriageWidth,noseWidthMm:120,headFasteners:"4 x M6 x 16; nominal blind taps",
				releaseSpace:{diameterMm:MillSpindleServiceSpace.DIAMETER,heightMm:MillSpindleServiceSpace.HEIGHT,
					standoffAboveHeadMm:MillSpindleServiceSpace.STANDOFF,status:"project allocation; no actuator fitted or release load qualification"}},
			atc:{status:"candidate study; not fitted",candidate:"PMK 100MCB30SSS short-nose BT30",housingDiameterMm:100,
				publishedL8Mm:264,publishedMaximumRpm:12000,publishedClampingForceN:2800,
				source:"https://pmkindia.com/datasheets/spindles_belt-driven-spindles-for-machining-center.pdf",
				compatibility:"current 70 mm ER20 head bore and spindle drive must be redesigned for this candidate",
				missing:["certified drawing and gauge datum", "release actuator dimensions, force, stroke and pressure", "release reaction path",
					"pulley/journal fit", "mass and bearing load limits", "tool-clamped/released feedback and orientation requirements"],
				note:"Clamping force is not the actuator release force; no actuator sized from it."},
			services:{status:"interface requirements; hardware and controller integration pending",
				lubrication:["fixed-base manifold", "independent moving X/Y/Z guide and screw circuits", "accessible fittings and metered flow", "protected flexible transitions"],
				protection:["X/Y/Z way covers with compressed/extended envelopes", "separate dry drive bays and wet chip area", "removable chip tray and coolant drain", "lifting and anchoring access"],
				controls:["independent home/limit feedback", "physical stops and verified stopping distance", "Z brake release/engagement feedback", "guard and emergency-stop integration"],
				atcReleasePermissives:["verified zero spindle speed", "orientation confirmed if required by chosen interface", "exchange pose confirmed", "tool supported", "air supply confirmed", "coherent clamp sensors"],
				spindleRunPermissives:["tool clamped confirmed", "release command off", "release sensor off", "guards closed", "no control fault"],
				failureBehavior:"inhibit rotation on contradictory feedback or lost clamp confirmation; selected mechanism must retain the tool on loss of power/air"},
			tableSection:{bareDeckIyyMm4:deckI,ribbedIyyMm4:ribbedI,ratio:ribbedI/deckI,scope:"idealized unperforated section; no joint, guide or casting compliance"},
			conceptStepParts:bounds,
			stages:[
				{step:1,status:"concept geometry",result:"250/160/250 travel; motor bays within base perimeter; service and enclosure envelope pending"},
				{step:2,status:"concept geometry and motion checks",result:"Y rear coaxial drive beneath column with 60 mm roof and removable end hatches; X 1:1 HTD5M return into saddle; preload and belt life pending"},
				{step:3,status:"concept geometry",result:"four accessible foot pockets, bored steel seats and M16 foot envelopes; anchorage/load ratings pending"},
				{step:4,status:"concept geometry and section comparison",result:"integral table ribs replace isolated risers; full stiffness qualification pending"},
				{step:5,status:"supplier selection required",result:"BT30 is a candidate, not an installed spindle; current ER20 cartridge remains manual"},
				{step:6,status:"revised head and reserved release space",result:"wider block span and 20 mm carriage; tapered windowed head with bolted flange; final head bore and release mechanism await spindle drawings"},
				{step:7,status:"partial nominal interfaces",result:"X return plate/pocket and foot bores modeled; complete drilling, fasteners and anchor calculations pending"},
				{step:8,status:"partial protection",result:"motor hoods modeled; lubrication, way covers, cables, coolant and control interlocks specified in README but not installed"},
				{step:9,status:"reference checks and concept STEP packet",result:"static guide admission and continuous structural sweeps; FEA, thermal/modal and manufacturing drawings pending"}],
			releaseBlockers:["verified ATC spindle/drawbar/actuator and motor drawings", "release force, stroke, air pressure, sensor and reaction-path data",
				"complete fastener and insert anchorage design", "foot ratings, floor support and stability", "whole-machine stiffness, thermal and modal qualification",
				"validated lubrication/coolant/guards and controls", "material, mould, cure and post-cast machining process qualification"]};
		sys.io.File.saveContent(folder+"/interfaces.csv", "interface,pattern,status\nBK12 and BF12 pedestal bolts,2 x M5 x 55 per housing,nominal modeled\nX motor plate anchors,4 x M6 x 25,nominal modeled\nY service hatches,4 x M4 x 10 per hatch into steel anchors,nominal modeled\nZ head flange,4 x M6 x 16 with counterbores and blind carriage taps,nominal modeled\nLeveling feet,M16 stem through 18 mm steel bore,concept geometry and supplier rating pending\nGuide and block screws,catalog pattern with accessible head seats,drilling and fasteners pending\nSpindle and actuator,selected supplier drawing required,unresolved\n");
		sys.io.File.saveContent(folder+"/review.json",haxe.Json.stringify(report,null,"  "));
		Sys.println("Mill concept design review: "+folder+"/review.json");
	}
}
