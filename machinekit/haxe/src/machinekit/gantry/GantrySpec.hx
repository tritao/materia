package machinekit.gantry;

import machinekit.motion.LeadScrewThread;
import machinekit.motion.LeadScrewThread.LeadScrewThreadFamily;
import machinekit.transmission.TimingBeltProfile;
import machinekit.transmission.ValueBasis;

/** Mechanical drive choice: ratios are derived from these physical parts. */
enum GantryDrive {
	Screw(thread:LeadScrewThread);
	Belt(profile:TimingBeltProfile, teeth:Int, width:Float);
	Rack(moduleSize:Float, pinionTeeth:Int, width:Float);
	HelicalRack(normalModule:Float,pinionTeeth:Int,width:Float,helixAngle:Float);
}

/** Motor selection is explicit; a braked motor exposes its physical brake for commissioning. */
enum GantryMotor {
    Stepper(frame:Int);
    Servo(designation:String);
    BrakedServo(power:String);
}
enum GantryStructure {
    Aluminium;
    Steel(frame:machinekit.structural.RectTube,beam:machinekit.structural.RectTube);
}
/** Front-face four-block X carriage with fixed Z blocks and moving Z rails. */
typedef FrontGuideLayout = {
    var xRailSpacing:Float;
    var xBlockSpacing:Float;
    var zRailSpacing:Float;
    var zBlockSpacing:Float;
    var carriageThickness:Float;
    var movingThickness:Float;
    @:optional var zRibDepth:Float;
    @:optional var zRibWidth:Float;
    @:optional var zRibWall:Float;
}

typedef GantryOptions = {
    @:optional var frontGuides:FrontGuideLayout;
    @:optional var beamFootThickness:Null<Float>;
    @:optional var beamFootWelded:Bool;
    @:optional var beamFootMaterial:String;
    @:optional var railProfileY:String;
    @:optional var yBlockSpacing:Null<Float>;
    @:optional var railPadHeight:Null<Float>;
    @:optional var motor:GantryMotor;
    @:optional var motorX:GantryMotor;
    @:optional var motorY:GantryMotor;
    @:optional var motorZ:GantryMotor;
    @:optional var reducer:machinekit.motion.PlanetaryReducer;
    @:optional var reducerX:machinekit.motion.PlanetaryReducer;
    @:optional var reducerY:machinekit.motion.PlanetaryReducer;
    @:optional var reducerZ:machinekit.motion.PlanetaryReducer;
    @:optional var preload:machinekit.transmission.PreloadedPinionMount;
    @:optional var structure:GantryStructure;
    @:optional var cableChain:machinekit.motion.CableChain;
    @:optional var cableChainOffsetX:materia.assembly.AssemblyDefinition.AssemblyVector;
    @:optional var cableChainOffsetY:materia.assembly.AssemblyDefinition.AssemblyVector;
    @:optional var supplyCurrent:Null<machinekit.power.DcCurrent>;
}

/** Optional rotary head; the translational assembly exposes the flange without one. */
enum GantryHead {
	None;
	C;
	CA;
}

/** Travel is in millimetres. Frame and guide dimensions describe parts, not virtual joints. */
class GantrySpec {
	public final travelX:Float;
	public final travelY:Float;
	public final travelZ:Float;
	public final driveX:GantryDrive;
	public final driveY:GantryDrive;
	public final driveZ:GantryDrive;
	public final dualY:Bool;
	public final railProfile:String;
	public final motorFrame:Int;
    public final frontGuides:Null<FrontGuideLayout>;
    public final beamFootThickness:Float;
    public final beamFootWelded:Bool;
    public final beamFootMaterial:Null<String>;
    public final railProfileY:String;
    public final yBlockSpacing:Float;
    public final railPadHeight:Float;
    public final motor:GantryMotor;
    public final motorX:GantryMotor;
    public final motorY:GantryMotor;
    public final motorZ:GantryMotor;
    public final reducer:Null<machinekit.motion.PlanetaryReducer>;
    public final reducerX:Null<machinekit.motion.PlanetaryReducer>;
    public final reducerY:Null<machinekit.motion.PlanetaryReducer>;
    public final reducerZ:Null<machinekit.motion.PlanetaryReducer>;
    public final preload:Null<machinekit.transmission.PreloadedPinionMount>;
    public final structure:GantryStructure;
    public final cableChain:Null<machinekit.motion.CableChain>;
    public final cableChainOffsetX:materia.assembly.AssemblyDefinition.AssemblyVector;
    public final cableChainOffsetY:materia.assembly.AssemblyDefinition.AssemblyVector;
    public final supplyCurrent:Null<machinekit.power.DcCurrent>;
	public final frameProfile:String;
	public final beamProfile:String;
	public final tableMounted:Bool;
	/** Extra frame length ahead of Y travel, clearing an overhanging tool's vertical sweep. */
	public final frontExtension:Float;
	/** Extra space on each X side between the tool travel and the Y frame guides. */
	public final sideExtension:Float;
	/** Extra frame elevation above the default work envelope, in millimetres. */
	public final frameLift:Float;
	/** Requested forward flange reach, in millimetres; enlarged as needed to clear the Z column. */
	public final toolReach:Float;
	public final head:GantryHead;
	/** Allowed side-to-side displacement; a stated design assumption until measured. */
	public final racking:Float;
	public final rackingBasis:ValueBasis;
	public final supplyVoltage:machinekit.power.DcVoltage;
	public final microsteps:Int;
	/** Provenance for the authored machine layout and electrical settings. */
	public final assumedFields:Array<String>;

	public function new(travelX:Float = 1500, travelY:Float = 1000, travelZ:Float = 500,
			?driveX:GantryDrive, ?driveY:GantryDrive, ?driveZ:GantryDrive, dualY:Bool = true,
			railProfile:String = "MGN12C", motorFrame:Int = 23,
			frameProfile:String = "HFS5-4040", beamProfile:String = "HFS5-4040",
			tableMounted:Bool = false, head:GantryHead = None, racking:Float = 0.5,
			?supplyVoltage:machinekit.power.DcVoltage, microsteps:Int = 16, frontExtension:Float = 0, sideExtension:Float = 0, frameLift:Float = 0, toolReach:Float = 0, ?options:GantryOptions) {
		for (value in [travelX, travelY, travelZ, racking])
			if (!Math.isFinite(value) || value <= 0) throw "Gantry dimensions and electrical limits must be finite and positive";
		if (microsteps <= 0) throw "Gantry microsteps must be positive";
		if (!Math.isFinite(frontExtension) || frontExtension < 0) throw "Gantry front extension must be finite and non-negative";
		if (!Math.isFinite(sideExtension) || sideExtension < 0) throw "Gantry side extension must be finite and non-negative";
		if (!Math.isFinite(frameLift) || frameLift < 0) throw "Gantry frame lift must be finite and non-negative";
        var selected:GantryOptions=options==null?{motor:null}:options;
        beamFootThickness=selected.beamFootThickness==null?12:selected.beamFootThickness;
        beamFootMaterial=selected.beamFootMaterial;
        beamFootWelded=selected.beamFootWelded==true;
        if(beamFootWelded&&beamFootMaterial!="steel")throw "Welded gantry feet require steel feet";
        if(!Math.isFinite(beamFootThickness)||beamFootThickness<=0)throw "Invalid beam foot thickness";
        frontGuides=selected.frontGuides;
        if(frontGuides!=null)for(value in [frontGuides.xRailSpacing,frontGuides.xBlockSpacing,frontGuides.zRailSpacing,
                frontGuides.zBlockSpacing,frontGuides.carriageThickness,frontGuides.movingThickness])
            if(!Math.isFinite(value)||value<=0)throw "Front guide dimensions must be finite and positive";
        if(frontGuides!=null&&frontGuides.zRibDepth!=null) {
            var ribs:Array<Null<Float>> = [frontGuides.zRibDepth,frontGuides.zRibWidth,frontGuides.zRibWall];
            for(v in ribs)
                if(v==null||!Math.isFinite(cast v)||(cast v:Float)<=0)throw "Z stiffeners require depth, width and wall";
        }
        railProfileY=selected.railProfileY==null?railProfile:selected.railProfileY;
        yBlockSpacing=selected.yBlockSpacing==null?0:selected.yBlockSpacing;
        railPadHeight=selected.railPadHeight==null?0:selected.railPadHeight;
        if(!Math.isFinite(yBlockSpacing)||yBlockSpacing<0||!Math.isFinite(railPadHeight)||railPadHeight<0)throw "Invalid Y guide mounting dimensions";
        motor=selected.motor==null?Stepper(motorFrame):selected.motor;
        motorX=selected.motorX==null?motor:selected.motorX;
        motorY=selected.motorY==null?motor:selected.motorY;
        motorZ=selected.motorZ==null?motor:selected.motorZ;
        reducer=selected.reducer;reducerX=selected.reducerX==null?reducer:selected.reducerX;
        reducerY=selected.reducerY==null?reducer:selected.reducerY;reducerZ=selected.reducerZ;preload=selected.preload;
        structure=selected.structure==null?Aluminium:selected.structure;
        cableChain=selected.cableChain;supplyCurrent=selected.supplyCurrent;
        cableChainOffsetX=selected.cableChainOffsetX==null?{x:0.0,y:0.0,z:0.0}:selected.cableChainOffsetX;
        cableChainOffsetY=selected.cableChainOffsetY==null?{x:0.0,y:0.0,z:0.0}:selected.cableChainOffsetY;
        for(offset in [cableChainOffsetX,cableChainOffsetY])for(value in [offset.x,offset.y,offset.z])
            if(!Math.isFinite(value))throw "Cable-chain mounting offsets must be finite";
        if(supplyVoltage==null && (!isStepper(motorX)||!isStepper(motorY)||!isStepper(motorZ)))throw "Servo gantries need an explicit DC bus voltage";
		this.travelX = travelX; this.travelY = travelY; this.travelZ = travelZ;
		this.driveX = driveX == null ? Belt(TimingBeltProfile.GT2, 20, 9) : driveX;
		this.driveY = driveY == null ? Belt(TimingBeltProfile.GT2, 20, 9) : driveY;
		this.driveZ = driveZ == null ? Screw(new LeadScrewThread(MetricTrapezoidal, 12, 3)) : driveZ;
		this.dualY = dualY;
		this.railProfile = railProfile; this.motorFrame = motorFrame;
		this.frameProfile = frameProfile; this.beamProfile = beamProfile;
		this.tableMounted = tableMounted; this.head = head;
		this.frontExtension = frontExtension;
		this.sideExtension = sideExtension;
		this.frameLift = frameLift;
		if (!Math.isFinite(toolReach) || toolReach < 0) throw "Gantry tool reach must be finite and non-negative";
		this.toolReach = toolReach;
		this.racking = racking; rackingBasis = ValueBasis.Assumed;
		this.supplyVoltage = supplyVoltage == null ? new machinekit.power.DcVoltage(24) : supplyVoltage;
		this.microsteps = microsteps;
        for(axis in ["x","y","z"])validateReduction(axis);
		assumedFields = ["travelX", "travelY", "travelZ", "driveX", "driveY", "driveZ", "dualY",
			"railProfile", "motorFrame", "frameProfile", "beamProfile", "tableMounted", "head",
			"racking", "supplyVoltage", "microsteps", "frontExtension", "sideExtension", "frameLift", "toolReach", "railProfileY", "yBlockSpacing", "railPadHeight", "motor", "motorX", "motorY", "motorZ", "reducer", "reducerX", "reducerY", "reducerZ", "preload", "structure", "cableChain", "cableChainOffsetX", "cableChainOffsetY", "supplyCurrent"];
	}
    function validateReduction(axis:String):Void {
        if(reducerFor(axis)==null)return;
        var drive=axis=="x"?driveX:axis=="y"?driveY:driveZ;
        switch drive {
            case Belt(_,_,_):throw "Reducer stage requires a screw or rack drive";
            case _:
        }
        var choice=motorFor(axis);
        switch choice {
            case BrakedServo(_):throw "A braked servo requires a direct shaft joint";
            case _:
        }
    }
    public static function isStepper(choice:GantryMotor):Bool return switch choice {case Stepper(_):true;case _:false;};
    public function motorFor(axis:String):GantryMotor return axis=="z"?motorZ:axis=="y"?motorY:motorX;
    public function motorPart(axis:String):machinekit.component.MachineComponent return switch motorFor(axis) {
        case Stepper(frame):machinekit.motion.NemaStepper.frame(frame);
        case Servo(name):new machinekit.motion.AxisServoMotor(name);
        case BrakedServo(power):new machinekit.motion.BrakeServo(new machinekit.component.ComponentValues().setToken("power",power));
    };
    public function reducerFor(axis:String):Null<machinekit.motion.PlanetaryReducer> return axis=="z"?reducerZ:axis=="y"?reducerY:reducerX;
    public function frameSection(beam:Bool=false):GantrySection return switch structure {
        case Aluminium:new GantrySection(machinekit.structural.TSlotExtrusion.forProfile(beam?beamProfile:frameProfile));
        case Steel(frame,member):new GantrySection(null,beam?member:frame);
    };
    public function driveFootprint(axis:String,baseline:Float):Float {
        var stage=reducerFor(axis);
        if(isStepper(motorFor(axis))&&stage==null)return baseline;
        return Math.max(baseline,Math.max(machinekit.motion.AxisMotor.of(motorPart(axis)).face+12,stage==null?0:stage.n("diameter")+12));
    }

}
