package machinekit.gantry;

import machinekit.assembly.AxisBuilder;
import machinekit.assembly.AxisBuilder.AxisSpec;
import machinekit.gantry.GantrySpec.GantryDrive;
import machinekit.gantry.RotaryHead.CHead;
import machinekit.gantry.RotaryHead.CaHead;
import cadkit.modeling.AssemblyState;
import machinekit.gantry.GantryParts.GantryExtrusion;
import machinekit.gantry.GantryParts.GantryPlate;
import machinekit.gantry.GantryParts.GantryStiffenedPlate;
import machinekit.gantry.GantryParts.GantryIdlerPin;
import machinekit.gantry.GantryParts.GantryBeltClamp;
import machinekit.gantry.GantryParts.GantryNutMount;
import machinekit.gantry.GantryParts.GantryMotorSupport;
import machinekit.gantry.GantryParts.GantryBoredBracket;
import machinekit.gantry.GantryParts.GantrySwitchTrigger;
import machinekit.component.MachineComponent;
import machinekit.motion.LinearRail;
import machinekit.motion.LinearRailBlock;
import machinekit.motion.NemaStepper;
import machinekit.motion.AxisMotor;
import machinekit.transmission.RackGeometry;
import machinekit.component.ComponentValues;
import machinekit.motion.MotorDriver;
import machinekit.motion.PowerSupply;
import machinekit.motion.LeadScrew;
import machinekit.motion.LeadScrewNut;
import machinekit.motion.ScrewSupportUnit;
import machinekit.motion.ShaftCoupling;
import machinekit.transmission.TimingBelt;
import machinekit.transmission.TimingPulley;
import machinekit.transmission.Rack;
import machinekit.transmission.SpurGear;
import machinekit.robotics.RobotFlange;
import machinekit.structural.TSlotExtrusion;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

private typedef GantryBox = {min:Array<Float>, max:Array<Float>};

/** Three translational axes assembled from real guides, drives and a standard robot tool flange. */
class Gantry extends AxisBuilder {
	static inline var MOTOR_PLATE_THICKNESS:Float = 6;
	static inline var DRIVE_CLEARANCE:Float = 3;
	public final spec:GantrySpec;
	public final axes:Array<AxisSpec>;
	public final flangeZero:AssemblyFrame;
	public final railMargin:Float;
	public final head:Null<RotaryHead>;
	var headMountZero:Null<AssemblyFrame> = null;
	var driverIndex:Int = 0;

	public function new(spec:GantrySpec) {
		super();
		this.spec = spec;
		head = switch spec.head { case None: null; case C: new CHead(); case CA: new CaHead(); };
		var headDepth = head == null ? 0.0 : head.flangeZero.z;
		var headRadius = head == null ? 0.0 : head.radialEnvelopeMm;
		axes = [{id: "x", lower: 0.0, upper: spec.travelX, initial: 0.0},
			{id: "y", lower: 0.0, upper: spec.travelY, initial: 0.0, rackingTolerance: spec.dualY ? spec.racking : null},
			{id: "z", lower: 0.0, upper: spec.travelZ, initial: 0.0}];
		var frame = spec.frameSection();
		var beam = spec.frameSection(true);
		var guide = LinearRailBlock.metric(spec.railProfile).spec;
        var guideY=spec.railProfileY==spec.railProfile?guide:LinearRailBlock.metric(spec.railProfileY).spec;
        if(spec.yBlockSpacing>0&&spec.yBlockSpacing<guideY.blockLength+3)throw "Y guide blocks need physical separation";
		var flange = new RobotFlange(50);
		// The flange's rear rim must clear the column's projecting rail by 3 mm.
		var toolReach = Math.max(spec.toolReach, Math.max(flange.flangeDiameter / 2, headRadius) + 3 - (guide.blockHeight - guide.railHeight + 4));
		railMargin = Math.max(Math.max(Math.max(80, Math.max(frame.size / 2 + 66, 89)), guide.railEndMargin + guide.blockLength / 2 + 20), guideY.railEndMargin + guideY.blockLength / 2 + spec.yBlockSpacing / 2 + 20);
        var xGuideMargin=Math.max(Math.max(80,Math.max(frame.size/2+66,89)),guide.railEndMargin+guide.blockLength/2+20);
		var guideOvertravel = (spec.frontGuides==null?railMargin:xGuideMargin) - guide.railEndMargin - guide.blockLength / 2;
		var zDriveX = 40 + Math.max((AxisMotor.of(spec.motorPart("z")).face + 12) / 2, 30) + 12 + 3;
        if(spec.frontGuides!=null)zDriveX=Math.max(zDriveX,
            (spec.frontGuides.zRailSpacing+guide.railWidth+40)/2+(AxisMotor.of(spec.motorPart("z")).face+12)/2+3);
		var sideRoom = Math.max(Math.max(spec.frontGuides==null?railMargin:xGuideMargin, headRadius + frame.size / 2 + 3), zDriveX + Math.max((AxisMotor.of(spec.motorPart("z")).face + 12) / 2, 30) + frame.size / 2 + 3);
		var left = -sideRoom - spec.sideExtension - guideOvertravel;
		var right = spec.travelX + sideRoom + spec.sideExtension + guideOvertravel;
		// Keep the front crossmember ahead of the full vertical carriage sweep.
		var columnY = -(beam.size / 2 + frame.height / 2 + 40);
		var carriageFront = columnY - frame.height / 2 - guide.blockHeight - 8 - toolReach;
		var switchSpec = machinekit.motion.ProximitySwitch.catalog().get("GENERIC-INDUCTIVE-M8");
		var sensorFront = columnY - frame.height / 2 - guide.blockHeight - 3 - switchSpec.length - switchSpec.sensingDistance - 6;
		var flangeFront = columnY - frame.height / 2 - guide.blockHeight - 4 - toolReach - Math.max(flange.flangeDiameter / 2, headRadius);
		var front = Math.min(-railMargin, Math.min(flangeFront, Math.min(carriageFront, sensorFront)) - frame.size / 2 - 3 - guideOvertravel) - spec.frontExtension;
		var back = spec.travelY + railMargin;
		var frameZ = spec.travelZ + 350 + spec.frameLift + headDepth;
		var endAllowance = Math.max(frame.size / 2, AxisMotor.of(spec.motorPart("y")).shaftLength + 6);
		var up = [0.0, 0, 1], alongX = [1.0, 0, 0], alongY = [0.0, 1, 0];
		place("frameFront", frame.member(right - left - frame.size),
			AxisBuilder.orient(left + frame.size / 2, front, frameZ, up, alongX));
		attach("frameBack", frame.member(right - left - frame.size),
			AxisBuilder.orient(left + frame.size / 2, back, frameZ, up, alongX), "frameFront");
		for (side in [-1, 1]) {
			var name = side < 0 ? "Left" : "Right", x = side < 0 ? left : right;
			attach('frame$name', frame.member(back - front + 2 * endAllowance),
				AxisBuilder.orient(x, front - endAllowance, frameZ, up, alongY), "frameFront");
			if (!spec.tableMounted) for (end in [0, 1]) {
				var id = 'post$name${end == 0 ? "Front" : "Back"}';
				attach(id, frame.member(frameZ - frame.height / 2),
					AxisBuilder.orient(x, end == 0 ? front : back, 0, [0.0, 1, 0], up), "frameFront");
			}
		}
		var railYTop = frameZ + frame.height / 2 + spec.railPadHeight + guideY.railHeight;
		for (side in [-1, 1]) {
			var name = side < 0 ? "Left" : "Right", x = side < 0 ? left : right;
			if(spec.railPadHeight>0) {
                var pad=new GantryPlate("rail pad",frame.size,spec.travelY+2*railMargin,spec.railPadHeight);pad.setMaterial("steel");
                attach('railPadY$name',pad,AssemblyFrames.translation(x,spec.travelY/2,frameZ+frame.height/2),'frame$name');
            }
            attach('railY$name', LinearRail.metric(spec.railProfileY, spec.travelY + 2 * railMargin),
				AxisBuilder.orient(x, -railMargin, railYTop, up, alongY), 'frame$name');
		}
		// One translational leader; the opposite guide is rigidly held by the beam.
		slide(axes[1], "railYLeft", "blockYLeft", LinearRailBlock.metric(spec.railProfileY),
			AxisBuilder.orient(left, spec.yBlockSpacing==0?0:-spec.yBlockSpacing/2, railYTop, up, alongY), {x: 0.0, y: 1.0, z: 0.0},
            spec.yBlockSpacing==0?null:[AxisBuilder.orient(left,spec.yBlockSpacing/2,railYTop,up,alongY)]);
		var feetZ = railYTop + guideY.blockHeight - guideY.railHeight;
        var footDepth=spec.yBlockSpacing==0?spec.driveFootprint("y",60):Math.max(spec.driveFootprint("y",60),spec.yBlockSpacing+guideY.blockLength);
        var footPlate=new GantryPlate("beam foot",spec.driveFootprint("y",60),footDepth,spec.beamFootThickness);
        if(spec.beamFootMaterial!=null)footPlate.setMaterial(spec.beamFootMaterial);
        attach("beamFootLeft",footPlate,AssemblyFrames.translation(left,0,feetZ),"blockYLeft");
        var beamZ=feetZ+spec.beamFootThickness+beam.height/2;
		attach("beam", beam.member(right - left + 2 * endAllowance),
			AxisBuilder.orient(left - endAllowance, 0, beamZ, up, alongX), "beamFootLeft");
        var rightFootPlate=new GantryPlate("beam foot",spec.driveFootprint("y",60),footDepth,spec.beamFootThickness);
        if(spec.beamFootMaterial!=null)rightFootPlate.setMaterial(spec.beamFootMaterial);
        attach("beamFootRight",rightFootPlate,AssemblyFrames.translation(right,0,feetZ),"beam");
		attach("blockYRight", LinearRailBlock.metric(spec.railProfileY),
			AxisBuilder.orient(right, spec.yBlockSpacing==0?0:-spec.yBlockSpacing/2, railYTop, up, alongY), "beamFootRight");
        if(spec.yBlockSpacing>0)for(side in ["Left","Right"])attach('blockY${side}Rear',LinearRailBlock.metric(spec.railProfileY),
            AxisBuilder.orient(side=="Left"?left:right,spec.yBlockSpacing/2,railYTop,up,alongY),'beamFoot$side');
        var xPlateZ:Float,columnLength:Float,zPlateY:Float,mountZero:AssemblyFrame;
        if(spec.frontGuides!=null) {
            var layout=spec.frontGuides;
            if(layout.xRailSpacing>beam.height-40)throw "X rail seats must fit the vertical beam face";
            if(layout.xBlockSpacing<guide.blockLength+3||layout.zBlockSpacing<guide.blockLength+3||layout.zRailSpacing<guide.blockWidth+3)
                throw "Front guide blocks need physical separation";
            var railY=-beam.size/2-guide.railHeight;
            var plateY=-beam.size/2-guide.blockHeight;
            var railLength=spec.travelX+2*xGuideMargin+layout.xBlockSpacing;
            for(row in 0...2) {
                var id=row==0?"railX":"railXLower",z=beamZ+(row==0?1:-1)*layout.xRailSpacing/2;
                attach(id,LinearRail.metric(spec.railProfile,railLength),
                    orient(-xGuideMargin-layout.xBlockSpacing/2,railY,z,[0.0,-1,0],alongX),"beam");
            }
            var first=orient(-layout.xBlockSpacing/2,railY,beamZ+layout.xRailSpacing/2,[0.0,-1,0],alongX);
            slide(axes[0],"railX","blockX",LinearRailBlock.metric(spec.railProfile),first,{x:1.0,y:0.0,z:0.0},
                [orient(layout.xBlockSpacing/2,railY,first.z,[0.0,-1,0],alongX)]);
            var plateWidth=Math.max(layout.xBlockSpacing+guide.blockLength+30,layout.zRailSpacing+guide.blockWidth+30);
            var lowerZ=beamZ;
            var upperZ=beamZ+layout.zBlockSpacing;
            var bottom=Math.min(lowerZ-guide.blockLength/2-30,beamZ-layout.xRailSpacing/2-guide.blockWidth/2-30);
            var top=Math.max(beamZ+layout.xRailSpacing/2+guide.blockWidth/2+30,upperZ+guide.blockLength/2+30);
            attach("xCarriage",new GantryPlate("X carriage",plateWidth,top-bottom,layout.carriageThickness),
                orient(0,plateY,(bottom+top)/2,[0.0,0,1],[0.0,-1,0]),"blockX");
            for(row in 0...2)for(side in 0...2)if(row!=0||side!=0) {
                var id='blockX${row}${side}';
                attach(id,LinearRailBlock.metric(spec.railProfile),
                    orient((side==0?-1:1)*layout.xBlockSpacing/2,railY,beamZ+(row==0?1:-1)*layout.xRailSpacing/2,[0.0,-1,0],alongX),"xCarriage");
            }
            var movingY=plateY-layout.carriageThickness-guide.blockHeight;
            var zRailY=movingY+guide.railHeight;
            var margin=guide.railEndMargin+guide.blockLength/2+20;
            var railBottom=lowerZ-margin;
            columnLength=spec.travelZ+layout.zBlockSpacing+2*margin;
            var fixedBlocks:Array<AssemblyFrame> = [];
            for(side in 0...2)for(row in 0...2) {
                var pose=orient((side==0?-1:1)*layout.zRailSpacing/2,zRailY,row==0?lowerZ:upperZ,[0.0,1,0],up);
                var id=side==0&&row==0?"blockZ":'blockZ${side}${row}';
                attach(id,LinearRailBlock.metric(spec.railProfile),pose,"xCarriage");
                if(side==0)fixedBlocks.push(pose);
            }
            slideRail(axes[2],"xCarriage","railZ",LinearRail.metric(spec.railProfile,columnLength),
                orient(-layout.zRailSpacing/2,zRailY,railBottom,[0.0,1,0],up),{x:0.0,y:0.0,z:-1.0},fixedBlocks);
            var zWidth=layout.zRailSpacing+guide.railWidth+40;
            var zPlate:GantryPlate=layout.zRibDepth==null?new GantryPlate("Z carriage",zWidth,columnLength,layout.movingThickness):
                new GantryStiffenedPlate(zWidth,columnLength,layout.movingThickness,cast layout.zRibDepth,cast layout.zRibWidth,cast layout.zRibWall);
            attach("zCarriage",zPlate,
                orient(0,movingY,railBottom+columnLength/2,[0.0,0,1],[0.0,-1,0]),"railZ");
            attach("railZRight",LinearRail.metric(spec.railProfile,columnLength),
                orient(layout.zRailSpacing/2,zRailY,railBottom,[0.0,1,0],up),"zCarriage");
            var shelfY=movingY-layout.movingThickness-60,shelfZ=railBottom+220;
            // Front-guide carriages expose a mounting datum on the vertical plate.
            // Router saddles mount here directly, without an open horizontal shelf.
            mountZero=orient(0,movingY-layout.movingThickness,shelfZ-flange.thickness,[0.0,1,0],[0.0,0,-1]);
            xPlateZ=top;zPlateY=shelfY;
        } else {
		var railXTop = beamZ + beam.height / 2 + guide.railHeight;
		attach("railX", LinearRail.metric(spec.railProfile, spec.travelX + 2 * railMargin),
			AxisBuilder.orient(-railMargin, 0, railXTop, up, alongX), "beam");
		slide(axes[0], "railX", "blockX", LinearRailBlock.metric(spec.railProfile),
			AxisBuilder.orient(0, 0, railXTop, up, alongX), {x: 1.0, y: 0.0, z: 0.0});
		xPlateZ = railXTop + guide.blockHeight - guide.railHeight;
		// The Z column is in front of the beam, joined by the carriage's top plate.
		attach("xCarriage", new GantryPlate("X carriage", spec.driveFootprint("x",80), -columnY + 80, 12),
			AssemblyFrames.translation(0, columnY / 2, xPlateZ), "blockX");
		columnLength = spec.travelZ + 2 * railMargin;
		attach("zColumn", frame.member(columnLength),
			AxisBuilder.orient(0, columnY, xPlateZ - columnLength, [0.0, 1, 0], up), "xCarriage");
		var railZFace = columnY - frame.height / 2 - guide.railHeight;
		attach("railZ", LinearRail.metric(spec.railProfile, columnLength),
			AxisBuilder.orient(0, railZFace, xPlateZ, [0.0, -1, 0], [0.0, 0, -1]), "zColumn");
		slide(axes[2], "railZ", "blockZ", LinearRailBlock.metric(spec.railProfile),
			AxisBuilder.orient(0, railZFace, xPlateZ - railMargin, [0.0, -1, 0], [0.0, 0, -1]),
			{x: 0.0, y: 0.0, z: -1.0});
		zPlateY = railZFace - (guide.blockHeight - guide.railHeight) - 4;
		var carriageHeight = 100.0;
		// Retain 3 mm of plate above the upper guide-block mounting row.
		var carriageDrop = carriageHeight - guide.blockHolePitchC / 2 - 3;
		attach("zCarriage", new GantryPlate("Z carriage", spec.driveFootprint("z",80), 8 + toolReach, carriageHeight),
			AssemblyFrames.translation(0, zPlateY - toolReach / 2, xPlateZ - railMargin - carriageDrop), "blockZ");
		mountZero = AxisBuilder.orient(0, zPlateY - toolReach, xPlateZ - railMargin - carriageDrop - flange.thickness,
			[0.0, 1, 0], [0.0, 0, -1]);
        }
        if(spec.frontGuides==null)attach("flange",flange,mountZero,"zCarriage");
        else if(head!=null)throw "Front-guide output requires a plate-mounted tool";
		if (head == null) {
			flangeZero = mountZero;
            if(spec.frontGuides==null)exposeConnector("toolFlange","flange","face");
            else {seatAt("zCarriage","toolMount",mountZero);exposeConnector("toolFlange","zCarriage","toolMount");}
		} else {
			include("head", head);
			addMate("head-mount", "fixed", "flange", "face", "head/mount", "mount");
			exposeConnector("toolFlange", "head/outputFlange", "face");
			var state = new AssemblyState(definition());
			headMountZero = state.worldPose("head/mount");
			flangeZero = state.worldPose("head/outputFlange");
		}
        var power=axisPower(),headRegen=head==null?0.0:head.regenerativePower;
        var sourceAmps=new machinekit.power.DcCurrent(Math.max(20,1.1*(power.peak+2*headRegen)/spec.supplyVoltage.volts()));
        var requestedCurrent=spec.supplyCurrent;
        if(requestedCurrent!=null)sourceAmps=requestedCurrent;
        if(!Math.isFinite(sourceAmps.amps())||sourceAmps.amps()<=0)throw 'Invalid gantry source current ${sourceAmps.amps()}, request ${requestedCurrent}, peak ${power.peak}';
		attach("powerSupply", new PowerSupply(spec.supplyVoltage, sourceAmps,
			(spec.dualY ? 4 : 3) + (head == null ? 0 : head.rotaryJoints.length), 120, 60, 30,
			headRegen+power.regeneration),
			AssemblyFrames.translation(left - 180, front, 30), "frameFront");
		buildDrive(axes[1], spec.driveY, "YLeft", "frameLeft", "beamFootLeft",
			[left - 70, 0.0, feetZ + 6], alongY);
		if (spec.dualY) buildDrive(axes[1], spec.driveY, "YRight", "frameRight", "beamFootRight",
			[right + 70, 0.0, feetZ + 6], alongY);
		buildDrive(axes[0], spec.driveX, "X", "beam", "xCarriage", [0.0, 70, spec.frontGuides==null?xPlateZ+6:beamZ+150], alongX);
		// Belt endpoints fit the column's full length; the clamp remains on the
		// carriage's upper edge. The screw nut and rack pinion sit 40 mm lower.
		var zDriveDrop = switch spec.driveZ { case Belt(_, _, _): 0.0; case _: 40.0; };
		// Leave the corner bracket's 12 mm arm outside the 80 mm carriage.
		buildDrive(axes[2], spec.driveZ, "Z", spec.frontGuides==null?"zColumn":"xCarriage", "zCarriage",
			[zDriveX, zPlateY, xPlateZ - railMargin - zDriveDrop], [0.0, 0, -1]);
		if (head != null) {
			var drivers:Array<String> = [];
			for (joint in head.rotaryJoints) {
				var id = joint.id + "HeadDriver";
				attach(id, new MotorDriver("GENERIC-SERVO-AMP", head.driverPhaseCurrent),
					AssemblyFrames.translation(-railMargin - 180, -railMargin - 130 - 125 * driverIndex, 30), "frameFront");
				driverIndex++;
				connectPorts(id + "-power", "powerSupply", 'power$driverIndex', id, "power");
				drivers.push(id);
			}
			head.bindDrives(this, "head", drivers);
		}
		buildSwitches(axes[1], "YLeft", "frameLeft", "beamFootLeft", 0, -1, alongY);
		if (spec.dualY) buildSwitches(axes[1], "YRight", "frameRight", "beamFootRight", 0, 1, alongY);
		buildSwitches(axes[0], "X", "beam", "xCarriage", 1, -1, alongX);
		buildSwitches(axes[2], "Z", spec.frontGuides==null?"zColumn":"xCarriage", "zCarriage", 0, -1, [0.0, 0, -1]);
		homeAfter("x", ["z"]);
		homeAfter("y", ["z"]);
        if(spec.cableChain!=null)buildCableChains();
	}

    public final cableChainBindings:Array<{axis:String,member:String,carrier:String,pose:AssemblyFrame,chain:machinekit.motion.CableChain}> = [];
    function buildCableChains():Void {
        for(axis in ["y","x"]) {
            var travel=axis=="y"?spec.travelY:spec.travelX,carrier=axis=="y"?"beam":"xCarriage",fixed=axis=="y"?"frameLeft":"beam";
            var start=zeroPose(carrier),r=spec.cableChain.n("bendRadius"),direction=axisDirection(axis);
            var offset=axis=="x"?spec.cableChainOffsetX:spec.cableChainOffsetY;
            var origin=[start.x+offset.x+direction[0]*travel/2,start.y+offset.y+direction[1]*travel/2,start.z+offset.z-2*r];
            var pose=AxisBuilder.orient(origin[0],origin[1],origin[2],direction,[0.0,0,1]);
            var member="cableChain"+axis.toUpperCase(),values=spec.cableChain.values().setNumber("travel",travel);
            var chain=new machinekit.motion.CableChain(values);
            attach(member,new GantryCableChain(chain),pose,fixed);
            cableChainBindings.push({axis:axis,member:member,carrier:carrier,pose:pose,chain:chain});
        }
    }
    public function cableChainEnd(axis:String,position:Float):AssemblyFrame {
        for(binding in cableChainBindings)if(binding.axis==axis) {
            var chain=binding.chain;
            return AssemblyFrames.compose(binding.pose,chain.movingFrame(position));
        }
        throw "No cable chain for axis "+axis;
    }
    public function cableChainGeometry(axis:String,position:Float):cadkit.modeling.Part {
        for(binding in cableChainBindings)if(binding.axis==axis)return binding.chain.geometryAt(position);
        throw "No cable chain for axis "+axis;
    }

	/** Side-mounted steel trigger and switches carried by the actual guide's fixed structure. */
	function buildSwitches(axis:AxisSpec, suffix:String, fixed:String, moving:String,
			outboardAxis:Int, sign:Int, direction:Array<Float>):Void {
		var drive = suffix == "X" ? spec.driveX : suffix == "Z" ? spec.driveZ : spec.driveY;
		var driveJoint = switch drive {
			case Screw(_): "screw" + suffix + "-turn";
			case Belt(_, _, _): "pulley" + suffix + "-turn";
			case Rack(_, _, _) | HelicalRack(_,_,_,_): "pinion" + suffix + "-turn";
		};
		var bounds = mountBounds(component(moving), zeroPose(moving));
		var point = [for (i in 0...3) (bounds.min[i] + bounds.max[i]) / 2];
		point[outboardAxis] = (sign < 0 ? bounds.min[outboardAxis] : bounds.max[outboardAxis]) + sign * 18;
		var dimensions = [10.0, 10.0, 6.0];
        var customTrigger:Null<MachineComponent> = null;
		dimensions[outboardAxis] = 40;
		for (i in 0...3) if (Math.abs(direction[i]) > 0.5)
			dimensions[i] = bounds.max[i] - bounds.min[i] + 6;
        if(suffix=="X" && spec.frontGuides==null && switch spec.motorZ {case Stepper(_):false; case _:true;}) {
            // Keep the sensor's descending leg ahead of a wide Z motor.
            var motor=mountBounds(component("motorZ"),zeroPose("motorZ"));
            if(motor.min[1]<bounds.min[1]) {
                point[1]=Math.min(point[1],motor.min[1]-18);
                dimensions[1]=2*(bounds.min[1]-point[1]+3);
            }
        }
        if(spec.frontGuides!=null&&suffix=="X") {
            point[1]=bounds.max[1]+10;dimensions[1]=26;
            point[2]=bounds.max[2]-3;dimensions[2]=6;
        }
        if(spec.frontGuides!=null&&suffix=="Z") {
            // Keep the positive-limit bracket above neighbouring tool forks.
            point[2]=bounds.min[2]+66;dimensions[2]=6;
        }
		if (StringTools.startsWith(suffix, "Y")) {
			point[outboardAxis] = (sign < 0 ? bounds.min[outboardAxis] : bounds.max[outboardAxis]) + sign * 7;
			dimensions[outboardAxis] = 20;
			point[2] = bounds.min[2] + 2;
            if(bounds.max[0]-bounds.min[0]>70)point[outboardAxis]=(sign<0?bounds.max[outboardAxis]:bounds.min[outboardAxis])-sign*7;
		}
		// Z workpieces rise in front of the column. Keep the fixed switches
		// alongside it; the outboard trigger stays clear of the rear beam.
		if (suffix == "Z" && spec.frontGuides==null) {
			// A 26 mm tab carries the sideways sensor field and overlaps the carriage
			// by 3 mm without reaching the Y rails or frame at the X travel corners.
			point[outboardAxis] = (sign < 0 ? bounds.min[outboardAxis] : bounds.max[outboardAxis]) + sign * 10;
			dimensions[outboardAxis] = 26;
			var column = mountBounds(component(fixed), zeroPose(fixed));
			var rearBeam = mountBounds(component("beam"), zeroPose("beam"));
			var rearFoot = mountBounds(component("beamFootLeft"), zeroPose("beamFootLeft"));
			var rearLimit = Math.min(rearBeam.min[1], rearFoot.min[1]);
			// End the tab below the foot and its Y sensor mounts at the zero corner.
			point[2] = bounds.min[2] + 6; dimensions[2] = 6;
			// The trigger touches the carriage's rear by 3 mm and stops
			// 3 mm short of the beam and its feet, throughout the Z stroke.
			point[1] = Math.min(column.max[1] - 10, (rearLimit + bounds.max[1] - 6) / 2);
			dimensions[1] = 2 * (point[1] - bounds.max[1] + 3);
            if(column.min[0]<bounds.min[0]-3) {
                point[0]=column.min[0]-13;dimensions[0]=20;
                var bridgeWidth=bounds.min[0]+3-(point[0]-dimensions[0]/2);
                var bridgeDepth=column.min[1]-bounds.max[1];
                customTrigger=new GantrySwitchTrigger(dimensions[0],dimensions[1],dimensions[2],bridgeWidth,bridgeDepth,
                    (bridgeWidth-dimensions[0])/2,(bounds.max[1]-3+column.min[1]-3)/2-point[1]);
            }
		}

		var target:MachineComponent = customTrigger==null?new GantryPlate("switch trigger", dimensions[0], dimensions[1], dimensions[2]):customTrigger;
		target.setMaterial("steel");
		var trigger = "switchTrigger" + suffix;
		attach(trigger, target, AssemblyFrames.translation(point[0], point[1], point[2] - dimensions[2] / 2), moving);
		var half = 0.0;
		for (i in 0...3) half += Math.abs(direction[i]) * dimensions[i] / 2;
		var room = axisOvertravel(axis.id);
		if (room <= 1) throw "Gantry switches need more than 1 mm of guide overtravel";
		for (kind in ["home", "negativeLimit", "positiveLimit"]) {
			var side = kind == "positiveLimit" ? 1 : -1;
			var travel = (side < 0 ? axis.lower : axis.upper) + side * room * (kind == "home" ? 0.25 : 0.75);
			var id = "switch" + suffix + kind;
			var sensor = new machinekit.motion.ProximitySwitch();
			// Sense perpendicular to travel so the trigger can pass the tube safely.
			var normal = suffix == "X" || suffix == "Z" ? [0.0, 1, 0] : [0.0, 0, 1];
			var face = [for (i in 0...3) point[i] + direction[i] * (travel + side * half) - normal[i] * dimensions[i] / 2];
			var contact = [for (i in 0...3) face[i] - direction[i] * travel];
			addMemberConnector(trigger, kind + suffix, AssemblyFrames.translation(
				contact[0] - point[0], contact[1] - point[1], contact[2] - point[2] + dimensions[2] / 2));
			var origin = [for (i in 0...3) face[i] - normal[i] * (sensor.spec.length + sensor.spec.sensingDistance)];
			var pose = AxisBuilder.orient(origin[0], origin[1], origin[2], direction, normal);
			var mount = id + "Mount";
			mountSwitch(mount, fixed, moving, trigger, suffix, origin, pose, sensor.spec.diameter);
			attach(id, sensor, pose, mount);
			addTripSwitch(id, axis.id, id, {instanceId: trigger, connectorName: kind + suffix},
				side, kind == "home" ? "home" : "limit", suffix == "YRight" ? 2 : 1, driveJoint);
		}
	}

	/** Route bored brackets around the complete moving envelope, including homing travel. */
	function mountSwitch(id:String, fixed:String, moving:String, trigger:String, suffix:String,
			origin:Array<Float>, sensorPose:AssemblyFrame, diameter:Float):Void {
		var host = mountBounds(component(fixed), zeroPose(fixed));
		var carriage = mountBounds(component(moving), zeroPose(moving));
		var target = mountBounds(component(trigger), zeroPose(trigger));
		var half = 6.0, clearance = 3.0;
		var path:Array<Array<Float>>;
		if (suffix == "X") {
			var carried = movingBounds(moving);
			var rear = carried.max[1], top = carried.max[2];
			// Also clear the fixed X drive where a field bracket passes its motor.
			for (member in ["motorX", "motorXPlate"]) {
				var envelope = mountBounds(component(member), zeroPose(member));
				rear = Math.max(rear, envelope.max[1]); top = Math.max(top, envelope.max[2]);
			}
			var rearPost = rear + half + clearance, overhead = top + half + clearance;
			var baseZ = host.min[2] + half;
			path = [[origin[0], host.max[1] - half, baseZ], [origin[0], rearPost, baseZ],
				[origin[0], rearPost, overhead], [origin[0], origin[1], overhead], origin];
		} else if (suffix == "Z") {
			var overhead = target.max[2] + axisOvertravel("z") + half + clearance;
			var anchorZ = Math.max(host.min[2] + half, Math.min(host.max[2] - half, overhead));
			path = [[host.min[0] + half, host.min[1] + half, anchorZ],
				[host.min[0] + half, host.min[1] + half, overhead],
				[origin[0], host.min[1] + half, overhead], [origin[0], origin[1], overhead], origin];
		} else {
			var anchor = [for (i in 0...3) Math.max(host.min[i], Math.min(host.max[i], origin[i]))];
			path = [anchor, origin];
		}
		var lo = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY];
		var hi = [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY];
		var sections:Array<Array<Float>> = [];
		for (index in 1...path.length) {
			var a = path[index - 1], b = path[index];
			var lower = [for (i in 0...3) Math.min(a[i], b[i]) - half];
			var upper = [for (i in 0...3) Math.max(a[i], b[i]) + half];
			for (i in 0...3) { lo[i] = Math.min(lo[i], lower[i]); hi[i] = Math.max(hi[i], upper[i]); }
			sections.push(lower.concat(upper));
		}
		var pose = AssemblyFrames.translation((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]);
		for (section in sections) for (i in 0...3) {
			var offset = i == 2 ? lo[2] : (lo[i] + hi[i]) / 2;
			section[i] -= offset; section[i + 3] -= offset;
		}
		attach(id, new GantryBoredBracket(id, hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2],
			AssemblyFrames.compose(AssemblyFrames.inverse(pose), sensorPose), diameter + 0.5, sections), pose, fixed);
	}

	/** All descendants carried by X, including the Z carriage and its guide allowance. */
	function movingBounds(moving:String):GantryBox {
		var definition = this.definition();
		var state = new AssemblyState(definition);
		var lo = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY];
		var hi = [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY];
		for (member in components()) {
			var cursor = member.id, carried = false, movesZ = false;
			for (_ in 0...definition.joints.length + 1) {
				if (cursor == moving) { carried = true; break; }
				var parent:Null<String> = null;
				for (joint in definition.joints) if (joint.child == cursor && joint.role == materia.assembly.AssemblyDefinition.AssemblyJointRole.Tree) {
					parent = joint.parent; if (joint.id == "z") movesZ = true;
				}
				if (parent == null) break;
				cursor = parent;
			}
			if (!carried || !member.component.hasGeometry()) continue;
			var box:GantryBox;
			if (head != null && StringTools.startsWith(member.id, "head/")) {
				var origin:AssemblyFrame = cast headMountZero;
				box = {min: [origin.x - head.radialEnvelopeMm, origin.y - head.radialEnvelopeMm, origin.z - head.axialEnvelopeMm],
					max: [origin.x + head.radialEnvelopeMm, origin.y + head.radialEnvelopeMm, origin.z]};
			} else box = mountBounds(member.component, state.worldPose(member.id));
			if (movesZ) { box.max[2] += axisOvertravel("z"); box.min[2] -= spec.travelZ + axisOvertravel("z"); }
			for (i in 0...3) { lo[i] = Math.min(lo[i], box.min[i]); hi[i] = Math.max(hi[i], box.max[i]); }
		}
		return {min: lo, max: hi};
	}

    /** The shared bus is explicit; winding and peak torque determine phase current. */
    function driverPart(part:MachineComponent):MotorDriver {
        if(Std.isOfType(part,NemaStepper)) {
            var motor:NemaStepper=cast part, rating=motor.rating();
            if(rating==null)throw "Gantry stepper needs rated drive data";
            var family=rating.ratedCurrent<=2 && spec.supplyVoltage.volts()<=29?"GENERIC-TMC2209":"GENERIC-DM542";
            return new MotorDriver(family,new machinekit.power.PhaseRmsCurrent(Math.min(rating.ratedCurrent,family=="GENERIC-TMC2209"?2.0:3.0)),spec.microsteps);
        }
        var drive=part.motorDrive();
        if(drive==null)throw "Gantry motor has no drive ratings";
        var winding=drive.electricalRating();
        if(winding==null)throw "Servo needs a winding rating";
        var actuator=drive.actuator("sizing","joint",spec.supplyVoltage.volts(),0.5),peak=actuator.peakTorque;
        if(peak==null)throw "Servo needs peak torque";
        var family=spec.supplyVoltage.volts()>=450?"GENERIC-400V-SERVO-AMP":spec.supplyVoltage.volts()>=200?"GENERIC-AC-SERVO-AMP":"GENERIC-LV-SERVO-AMP";
        return new MotorDriver(family,new machinekit.power.PhaseRmsCurrent(peak/winding.torqueConstant));
    }
    function axisPower():{peak:Float,regeneration:Float} {
        var peak=0.0,regeneration=0.0;
        for(axis in ["x","y","z"]) {
            if(GantrySpec.isStepper(spec.motorFor(axis)))continue;
            var part=spec.motorPart(axis),drive=part.motorDrive(),winding=drive.electricalRating();
            if(winding==null)continue;
            var driver=driverPart(part),load=new machinekit.motion.ServoElectricalLoad(drive.actuator("sizing","joint",spec.supplyVoltage.volts(),0.5),winding,driver);
            var count=axis=="y"&&spec.dualY?2:1;peak+=count*load.peakPower;regeneration+=count*load.regenerativePower;
        }
        return {peak:peak,regeneration:regeneration};
    }
    function addDriver(motorId:String):String {
        var id=motorId+"Driver";
        attach(id,driverPart(component(motorId)),AssemblyFrames.translation(-railMargin-180,-railMargin-130-125*driverIndex,30),"frameFront");
        driverIndex++;connectPorts(id+"-power","powerSupply",'power$driverIndex',id,"power");return id;
    }

	function mountMotor(id:String, parent:String, face:AssemblyFrame, ?clearanceDiameter:Float):MachineComponent {
        var motor=spec.motorPart(StringTools.endsWith(id,"Z")?"z":StringTools.startsWith(id,"motorY")?"y":"x"), mount=AxisMotor.of(motor);
		var plateSize = Math.max(mount.face + 12, clearanceDiameter == null ? 0 : clearanceDiameter + 10);
		var plate = new GantryPlate("motor mount", plateSize, plateSize, MOTOR_PLATE_THICKNESS, motor, AssemblyFrames.identity());
		var support = supportMotor(id + "Support", parent, plate, face, motor, clearanceDiameter);
		attach(id + "Plate", plate, face, support);
		attach(id, motor, face, id + "Plate");
		return motor;
	}

	/** World bounds of an extrusion or mounting plate at its zero pose. */
	static function mountBounds(member:MachineComponent, pose:AssemblyFrame):GantryBox {
		var min:Array<Float>, max:Array<Float>;
		if (Std.isOfType(member, GantryExtrusion)) {
			var extrusion:GantryExtrusion = cast member;
			min = [-extrusion.profile.size / 2, -extrusion.profile.height / 2, 0.0];
			max = [extrusion.profile.size / 2, extrusion.profile.height / 2, extrusion.length];
		} else if (Std.isOfType(member, GantryPlate)) {
			var plate:GantryPlate = cast member;
			min = [-plate.width / 2, -plate.depth / 2, 0.0]; max = [plate.width / 2, plate.depth / 2, plate.height];
		} else if (Std.isOfType(member, NemaStepper)) {
			var motor:NemaStepper = cast member;
			min = [-motor.variant.bodyFace / 2, -motor.variant.bodyFace / 2, -motor.bodyLength];
			max = [motor.variant.bodyFace / 2, motor.variant.bodyFace / 2, motor.variant.shaftLength];
		} else {
			var part = member.geometry(machinekit.component.ComponentDetail.Envelope);
			try {
				var box = part.shape.bounds(), lower = box.get_min(), upper = box.get_max();
				min = [lower.get_x(), lower.get_y(), lower.get_z()];
				max = [upper.get_x(), upper.get_y(), upper.get_z()]; part.close();
			} catch (error:Dynamic) { part.close(); throw error; }
		}
		var lo = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY];
		var hi = [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY];
		for (x in [min[0], max[0]]) for (y in [min[1], max[1]]) for (z in [min[2], max[2]]) {
			var point = AssemblyFrames.transformPoint(pose, x, y, z), coordinates = [point.x, point.y, point.z];
			for (axis in 0...3) { lo[axis] = Math.min(lo[axis], coordinates[axis]); hi[axis] = Math.max(hi[axis], coordinates[axis]); }
		}
		return {min: lo, max: hi};
	}

	function supportMotor(id:String, parent:String, plate:GantryPlate, face:AssemblyFrame, motor:MachineComponent, ?clearanceDiameter:Float):String {
		var host = mountBounds(component(parent), zeroPose(parent)), target = mountBounds(plate, face);
        if(spec.frontGuides!=null&&id=="motorXSupport") {
            // Route the carrier over the fixed rack rather than filling its swept volume.
            var top=target.max[2]+20,loY=host.max[1],hiY=target.min[1];
            var pose=AssemblyFrames.translation((target.min[0]+target.max[0])/2,(loY+hiY)/2,target.min[2]);
            var width=target.max[0]-target.min[0],depth=hiY-loY,height=top-target.min[2];
            var sections=[[-width/2,-depth/2,height-12,width/2,depth/2,height],
                [-width/2,depth/2-12,0.0,width/2,depth/2,height]];
            attach(id,new GantryMotorSupport(id,width,depth,height,motor,
                AssemblyFrames.compose(AssemblyFrames.inverse(pose),face),sections,clearanceDiameter),pose,parent);
            return id;
        }
		var gaps:Array<Int> = [];
		for (axis in 0...3) if (target.min[axis] > host.max[axis] + 1e-7 || host.min[axis] > target.max[axis] + 1e-7) gaps.push(axis);
		if (gaps.length == 0) return parent;
		if (gaps.length == 2) return supportMotorCorner(id, parent, host, target, face, motor, gaps, clearanceDiameter);
		if (gaps.length > 2) throw "A gantry motor bracket needs a common supporting plane";
		var gapAxis = gaps[0];
		var min:Array<Float> = [], max:Array<Float> = [];
		for (axis in 0...3) {
			if (axis == gapAxis) {
				min.push(Math.min(host.max[axis], target.max[axis])); max.push(Math.max(host.min[axis], target.min[axis]));
			} else {
				min.push(Math.max(host.min[axis], target.min[axis])); max.push(Math.min(host.max[axis], target.max[axis]));
			}
			if (max[axis] - min[axis] <= 1e-7) throw "Gantry motor bracket has no supporting contact area";
		}
        // A thicker beam foot must not lift the bridge into the fixed Y rack.
        if(spec.frontGuides!=null&&id=="motorYRightSupport")max[2]=Math.min(max[2],host.min[2]+16);
		var pose = AssemblyFrames.translation((min[0] + max[0]) / 2, (min[1] + max[1]) / 2, min[2]);
		attach(id, new GantryMotorSupport(id, max[0] - min[0], max[1] - min[1], max[2] - min[2], motor,
			AssemblyFrames.compose(AssemblyFrames.inverse(pose), face), null, clearanceDiameter), pose, parent);
		return id;
	}

	/** Two intersecting arms support a motor beside a frame corner without passing through its body. */
	function supportMotorCorner(id:String, parent:String, host:GantryBox, target:GantryBox,
			face:AssemblyFrame, motor:MachineComponent, gaps:Array<Int>, ?clearanceDiameter:Float):String {
		var a = gaps[0], b = gaps[1], c = 3 - a - b;
		var signA = target.min[a] > host.max[a] ? 1.0 : -1.0;
		var signB = target.min[b] > host.max[b] ? 1.0 : -1.0;
		var hostA = signA > 0 ? host.max[a] : host.min[a];
		var targetA = signA > 0 ? target.min[a] : target.max[a];
		var hostB = signB > 0 ? host.max[b] : host.min[b];
		var stripeA = Math.min(12, Math.abs(targetA - hostA));
		var stripeB = Math.min(12, host.max[b] - host.min[b]);
		var loC = Math.max(host.min[c], target.min[c]), hiC = Math.min(host.max[c], target.max[c]);
		if (hiC - loC <= 1e-7) throw "Corner motor bracket has no common contact area";
		var firstMin = [0.0, 0, 0], firstMax = [0.0, 0, 0];
		firstMin[a] = Math.min(hostA, targetA); firstMax[a] = Math.max(hostA, targetA);
		firstMin[b] = Math.min(hostB, hostB - signB * stripeB); firstMax[b] = Math.max(hostB, hostB - signB * stripeB);
		firstMin[c] = loC; firstMax[c] = hiC;
		var secondMin = firstMin.copy(), secondMax = firstMax.copy();
		secondMin[a] = Math.min(targetA, targetA - signA * stripeA); secondMax[a] = Math.max(targetA, targetA - signA * stripeA);
		secondMin[b] = Math.min(target.min[b], firstMin[b]); secondMax[b] = Math.max(target.max[b], firstMax[b]);
		var min = [for (axis in 0...3) Math.min(firstMin[axis], secondMin[axis])];
		var max = [for (axis in 0...3) Math.max(firstMax[axis], secondMax[axis])];
		var pose = AssemblyFrames.translation((min[0] + max[0]) / 2, (min[1] + max[1]) / 2, min[2]);
		var sections:Array<Array<Float>> = [];
		for (pair in [{lo: firstMin, hi: firstMax}, {lo: secondMin, hi: secondMax}])
			sections.push([pair.lo[0] - pose.x, pair.lo[1] - pose.y, pair.lo[2] - pose.z,
				pair.hi[0] - pose.x, pair.hi[1] - pose.y, pair.hi[2] - pose.z]);
		attach(id, new GantryMotorSupport(id, max[0] - min[0], max[1] - min[1], max[2] - min[2], motor,
			AssemblyFrames.compose(AssemblyFrames.inverse(pose), face), sections, clearanceDiameter), pose, parent);
		return id;
	}

	/** A plate bridges the actual gap from the carriage edge to the clamp jaw. */
	function supportClamp(id:String, parent:String, point:Array<Float>, direction:Array<Float>,
			transverse:Array<Float>, normal:Array<Float>, beltWidth:Float):String {
		var halfNormal = (beltWidth + 4) / 2;
		var radiusX = Math.abs(direction[0]) * 10 + Math.abs(transverse[0]) * 6 + Math.abs(normal[0]) * halfNormal;
		var radiusY = Math.abs(direction[1]) * 10 + Math.abs(transverse[1]) * 6 + Math.abs(normal[1]) * halfNormal;
		var pose = zeroPose(parent);
		var height = direction[2] == 0 ? 4.0 : 20.0;
		var z = direction[2] == 0 ? pose.z : point[2] - height / 2;
		return supportBox(id, parent, point, radiusX, radiusY, height, z);
	}

	function supportBox(id:String, parent:String, point:Array<Float>, radiusX:Float, radiusY:Float,
			height:Float, z:Float):String {
		var carrier:GantryPlate = cast component(parent);
		var pose = zeroPose(parent);
		var dx = point[0] - pose.x, dy = point[1] - pose.y;
		var gapX = Math.abs(dx) - carrier.width / 2 - radiusX;
		var gapY = Math.abs(dy) - carrier.depth / 2 - radiusY;
		if (gapX <= 0 && gapY <= 0) return parent;
		var width:Float, depth:Float, x:Float, y:Float;
		if (gapX > gapY) {
			var sign = dx < 0 ? -1.0 : 1.0;
			var host = pose.x + sign * carrier.width / 2, jaw = point[0] - sign * radiusX;
			width = Math.abs(jaw - host); depth = Math.max(12, Math.abs(dy) + 8);
			x = (host + jaw) / 2; y = (point[1] + pose.y) / 2;
		} else {
			var sign = dy < 0 ? -1.0 : 1.0;
			var host = pose.y + sign * carrier.depth / 2, jaw = point[1] - sign * radiusY;
			width = 20; depth = Math.abs(jaw - host); x = point[0]; y = (host + jaw) / 2;
		}
		attach(id, new GantryPlate("drive support bracket", width, depth, height), AssemblyFrames.translation(x, y, z), parent);
		return id;
	}

	function buildDrive(axis:AxisSpec, drive:GantryDrive, suffix:String, fixed:String, moving:String,
			origin:Array<Float>, direction:Array<Float>):Void {
		var transverse:Array<Float> = direction[2] == 0 ? [0.0, 0, 1] : [1.0, 0, 0];
		// Mirror right-side transverse drives to keep the motor body outboard.
		var mirroredDrive = suffix == "YRight" && (switch drive { case Rack(_, _, _) | HelicalRack(_,_,_,_) | Belt(_, _, _): true; case _: false; });
		if (mirroredDrive) transverse = [for (value in transverse) -value];
		var normal = [direction[1] * transverse[2] - direction[2] * transverse[1],
			direction[2] * transverse[0] - direction[0] * transverse[2],
			direction[0] * transverse[1] - direction[1] * transverse[0]];
		var motorId = "motor" + suffix;
		var selectedMotor=AxisMotor.of(spec.motorPart(axis.id)),reducer=spec.reducerFor(axis.id);
        var shaft=selectedMotor.shaftLength+(reducer==null?0:reducer.n("length")+MOTOR_PLATE_THICKNESS);
		function point(distance:Float):Array<Float> return [origin[0] + direction[0] * distance,
			origin[1] + direction[1] * distance, origin[2] + direction[2] * distance];
		switch drive {
			case Screw(thread):
                var driveMargin=axis.id=="z"&&spec.frontGuides!=null?95.0:railMargin;
				var ball = thread.family == machinekit.motion.LeadScrewThread.LeadScrewThreadFamily.Ball;
				var unit = ball ? new ScrewSupportUnit(true, thread.screwDiameter) : null;
				var coupling = new ShaftCoupling(selectedMotor.shaftDiameter, thread.screwDiameter);
				// Short-shaft motors cannot bury a long sleeve inside their pilot or mounting plate.
				if (ball) {
					var engagement = Math.min(coupling.length / 2, shaft - MOTOR_PLATE_THICKNESS - DRIVE_CLEARANCE);
					coupling = new ShaftCoupling(selectedMotor.shaftDiameter, thread.screwDiameter, null, 2 * engagement);
				}
				var grip = coupling.length / 2;
				// The bearing follows the coupling's screw-side grip. Keep its far face clear of the
				// nut at the guide's full declared overtravel, without reducing usable guide travel.
				var endRoom = 0.0;
				if (unit != null && !(axis.id=="z"&&spec.frontGuides!=null)) {
					var bounds = mountBounds(component(moving), zeroPose(moving));
					var leading = Math.POSITIVE_INFINITY;
					for (x in [bounds.min[0], bounds.max[0]]) for (y in [bounds.min[1], bounds.max[1]])
						for (z in [bounds.min[2], bounds.max[2]])
							leading = Math.min(leading, (x-origin[0])*direction[0] + (y-origin[1])*direction[1] + (z-origin[2])*direction[2]);
					// The switch trigger extends 3 mm beyond the carriage along travel (buildSwitches).
					leading -= axisOvertravel(axis.id) + 3;
					endRoom = Math.max(0, grip + unit.length + DRIVE_CLEARANCE - driveMargin - leading);
				}
				var start = point(-driveMargin - endRoom), face = point(-driveMargin - endRoom - shaft);
				mountMotor(motorId, fixed, AxisBuilder.orient(face[0], face[1], face[2], transverse, direction), reducer == null ? null : reducer.n("diameter") + 2);
				var screwId = "screw" + suffix;
				driveScrew(axis, screwId, motorId, new LeadScrew(thread, axis.upper + 2 * driveMargin + endRoom),
					AxisBuilder.orient(start[0], start[1], start[2], transverse, direction), direction, true, addDriver, coupling,reducer);
				if (unit != null) {
					// Generic screws retain their nominal cylindrical end, unlike the SFU1605's 12 mm journal.
					// A fixed bearing fits that actual shaft; its foot is carried back to the motor mounting plate.
					var bearing = point(-driveMargin - endRoom + grip + unit.length / 2);
					var bearingPose = AxisBuilder.orient(bearing[0], bearing[1], bearing[2], transverse, direction);
					var motorPlate:GantryPlate = cast component(motorId + "Plate");
					var foot = unit.connector("mount").frame.y;
					var thickness = motorPlate.height;
					var begin = point(-driveMargin - endRoom - shaft + thickness);
					var bracketLength = shaft - thickness + grip + unit.length;
					var drop = reducer == null ? 0 : Math.max(0, reducer.n("diameter") / 2 + DRIVE_CLEARANCE + foot);
					var bracketPose = AxisBuilder.orient(begin[0] + transverse[0] * (foot - thickness / 2 - drop),
						begin[1] + transverse[1] * (foot - thickness / 2 - drop),
						begin[2] + transverse[2] * (foot - thickness / 2 - drop), transverse, direction);
					var bracketId = screwId + "FixedBracket";
					attach(bracketId, new GantryPlate("fixed bearing bracket", unit.width, thickness, bracketLength),
						bracketPose, motorId + "Plate");
					if (drop > 0) {
						var riser = AxisBuilder.orient(bearing[0] + transverse[0] * (foot - drop / 2) - direction[0] * unit.length / 2,
							bearing[1] + transverse[1] * (foot - drop / 2) - direction[1] * unit.length / 2,
							bearing[2] + transverse[2] * (foot - drop / 2) - direction[2] * unit.length / 2, transverse, direction);
						attach(bracketId + "Riser", new GantryPlate("fixed bearing riser", unit.width, drop, unit.length), riser, bracketId);
					}
					attach(screwId + "Fixed", unit, bearingPose, bracketId);
				}
				var nut:LeadScrewNut = cast component(screwId + "Nut");
				var faceDistance = nut.bodyLength + nut.flangeThickness;
				var nutFace = point(faceDistance);
				var mount = new GantryNutMount(nut);
				var size = mount.size;
				var localX = [transverse[1] * direction[2] - transverse[2] * direction[1],
					transverse[2] * direction[0] - transverse[0] * direction[2],
					transverse[0] * direction[1] - transverse[1] * direction[0]];
				var centre = [nutFace[0] + direction[0] * GantryNutMount.THICKNESS / 2,
					nutFace[1] + direction[1] * GantryNutMount.THICKNESS / 2,
					nutFace[2] + direction[2] * GantryNutMount.THICKNESS / 2];
				var radiusX = (Math.abs(localX[0]) + Math.abs(transverse[0])) * size / 2 + Math.abs(direction[0]) * GantryNutMount.THICKNESS / 2;
				var radiusY = (Math.abs(localX[1]) + Math.abs(transverse[1])) * size / 2 + Math.abs(direction[1]) * GantryNutMount.THICKNESS / 2;
				var supportZ = direction[2] == 0 ? zeroPose(moving).z : centre[2] - GantryNutMount.THICKNESS / 2;
				var parent = supportBox(screwId + "NutSupport", moving, centre, radiusX, radiusY, GantryNutMount.THICKNESS, supportZ);
				var mountPose = AxisBuilder.orient(nutFace[0], nutFace[1], nutFace[2], transverse, direction);
				attach(screwId + "NutMount", mount, mountPose, parent);
				mountNut(screwId, screwId + "NutMount", mountPose);
			case Belt(profile, teeth, width):
				// The clamp's 20 mm jaw must stop before the idler's 60 mm plate.
				var endRoom = Math.max(0, 30 + 10 + 3 - (railMargin - axisOvertravel(axis.id)));
				var start = point(-railMargin), end = point(axis.upper + railMargin + endRoom);
				var face = [start[0] - normal[0] * (shaft - width), start[1] - normal[1] * (shaft - width), start[2] - normal[2] * (shaft - width)];
				var motor = mountMotor(motorId, fixed, AxisBuilder.orient(face[0], face[1], face[2], transverse, normal));
				var beltId = "belt" + suffix;
				var belt = TimingBelt.twoPulley(profile, teeth, teeth, axis.upper + 2 * railMargin + endRoom, width);
				var plane = AxisBuilder.orient(start[0], start[1], start[2], transverse, normal);
				var pulleyId = "pulley" + suffix, idlerId = "idler" + suffix;
				twoPulleyAxis({axis: axis, beltId: beltId, belt: belt, beltPose: plane, beltParent: fixed,
					motorId: motorId, pulleyId: pulleyId, pulley: new TimingPulley(profile, teeth, AxisMotor.of(motor).shaftDiameter, width),
					idlerId: idlerId, idler: new TimingPulley(profile, teeth, AxisMotor.of(motor).shaftDiameter, width),
					idlerPose: AxisBuilder.orient(end[0], end[1], end[2], transverse, normal), about: normal,
					strand: 0, travelX: 1.0, travelY: 0.0, driverForMotor: addDriver,
					mountIdler: () -> {
						var mount = AxisBuilder.orient(end[0] - normal[0] * 6, end[1] - normal[1] * 6,
							end[2] - normal[2] * 6, transverse, normal);
						var plate = new GantryPlate("idler mount", 60, 60, 6);
						var support = supportMotor(idlerId + "Support", fixed, plate, mount, motor);
						attach(idlerId + "Plate", plate, mount, support);
						attach(idlerId + "Pin", new GantryIdlerPin(AxisMotor.of(motor).shaftDiameter, width),
							AxisBuilder.orient(end[0], end[1], end[2], transverse, normal), idlerId + "Plate");
						return idlerId + "Pin";
					}});
				var strand = belt.strands()[0];
				var clampPoint = AssemblyFrames.transformPoint(plane, railMargin, strand.startY, width / 2);
				var clampId = "clamp" + suffix;
				var clampParent = supportClamp(clampId + "Support", moving,
					[clampPoint.x, clampPoint.y, clampPoint.z], direction, transverse, normal, width);
				attach(clampId, new GantryBeltClamp(width, belt.thickness),
					{x: clampPoint.x, y: clampPoint.y, z: clampPoint.z, qx: plane.qx, qy: plane.qy, qz: plane.qz, qw: plane.qw}, clampParent);
				addBeltPath({belt: beltId, clamp: {instanceId: clampId, connectorName: "belt"},
					wraps: [{instanceId: pulleyId, connectorName: "attach-" + pulleyId},
						{instanceId: idlerId, connectorName: "attach-" + idlerId}]});
			case Rack(moduleSize, teeth, width) | HelicalRack(moduleSize,teeth,width,_):
				var face = [origin[0] - normal[0] * (shaft - width), origin[1] - normal[1] * (shaft - width), origin[2] - normal[2] * (shaft - width)];
				var helix:Null<Float> = switch drive {case HelicalRack(_,_,_,angle):angle;case _:null;};
                var pinion:MachineComponent=helix==null?new SpurGear(moduleSize,teeth,width,SpurGear.STANDARD_PRESSURE_ANGLE,0,0,selectedMotor.shaftDiameter):
                    new machinekit.transmission.HelicalPinion(new ComponentValues().setNumber("normalModule",moduleSize).setInteger("teeth",teeth).setNumber("faceWidth",width).setNumber("helixAngle",helix).setNumber("bore",selectedMotor.shaftDiameter));
                var wheel=RackGeometry.pinion(pinion);
				mountMotor(motorId, moving, AxisBuilder.orient(face[0], face[1], face[2], transverse, normal), Math.max(wheel.diameter+2*moduleSize,reducer==null?0:reducer.n("diameter")) + 2);
				// driveRack places the rack under the pinion, phases a gap to the pinion's tooth and derives the sense.
				var rackPitch = Math.PI * wheel.moduleSize;
				var rackMargin = Math.ceil(railMargin / rackPitch) * rackPitch;
				var rackTeeth = Std.int(Math.ceil((axis.upper + 2 * rackMargin) / rackPitch));
				var pinionPose = AxisBuilder.orient(origin[0], origin[1], origin[2], transverse, normal);
				driveRack(axis, "pinion" + suffix, motorId, pinion, pinionPose, [-transverse[0], -transverse[1], -transverse[2]],
					"rack" + suffix, helix==null?new Rack(moduleSize,rackTeeth,width):
                        new machinekit.transmission.HelicalRack(new ComponentValues().setNumber("normalModule",moduleSize).setInteger("teeth",rackTeeth).setNumber("faceWidth",width).setNumber("helixAngle",helix)),
                    fixed,addDriver,reducer,spec.preload);
		}
	}
}
