import machinekit.gantry.GantrySpec;
import machinekit.gantry.GantrySpec.GantryDrive;
import machinekit.gantry.GantrySpec.GantryMotor;
import machinekit.gantry.GantrySpec.GantryStructure;
import machinekit.motion.*;
import machinekit.motion.LeadScrewThread.LeadScrewThreadFamily;
import machinekit.motion.LeadScrewThread.LeadScrewHand;
import machinekit.component.ComponentValues;
import machinekit.structural.RectTube;

/** Authored layout assumptions in mm. Work length excludes the rear tool-change zone. */
class SheetRouterSpec {
    public final workX:Float = 1600;
    public final workY:Float;
    public final travelZ:Float = 200;
    public final rackZone:Float = 300;
    public final longestTool:Float = 120;
    public final sheetThickness:Float = 40;
    /** 1610, 390 mm fixed/free shaft; rerun critical-speed and torque gates at 10 m/min. */
    public final rapidZMmPerMinute:Float = 10000;
    public function new(workY:Float = 2600) {
        if(!Math.isFinite(workY)||workY<2600)throw "Sheet router work length must be at least 2600 mm";
        this.workY=workY;
    }
    public function gantry():GantrySpec {
        var rack:GantryDrive=HelicalRack(1.5,20,20,Math.acos(3*Math.PI/10));
        var screw:GantryDrive=Screw(new LeadScrewThread(Ball,16,10,1,RightHand,13));
        return new GantrySpec(workX,workY+rackZone,travelZ,rack,rack,screw,true,"HGR20",23,
            "HFS5-4040","HFS5-4040",true,None,0.5,new machinekit.power.DcVoltage(560),16,300,100,200,180,{
                railProfileY:"HGR25",yBlockSpacing:280,railPadHeight:12,
                beamFootThickness:30,beamFootMaterial:"steel",beamFootWelded:true,
                frontGuides:{xRailSpacing:150,xBlockSpacing:300,zRailSpacing:180,zBlockSpacing:200,carriageThickness:40,movingThickness:20,zRibDepth:120,zRibWidth:25,zRibWall:6},
                motor:Servo("GENERIC-HV-SERVO-750W"),motorX:Servo("GENERIC-HV-SERVO-750W"),motorZ:BrakedServo("750"),
                reducer:new PlanetaryReducer(new ComponentValues().setNumber("backlash",3)),reducerX:new PlanetaryReducer(new ComponentValues().setNumber("bore",19).setNumber("ratio",7.5).setNumber("backlash",3)),
                preload:new machinekit.transmission.PreloadedPinionMount(new ComponentValues().setNumber("springForce",550).setNumber("springRate",4000).setNumber("stroke",0.5)),
                structure:Steel(new RectTube(100,100,6),new RectTube(100,200,6.3)),cableChain:new CableChain(),
                cableChainOffsetY:{x:-80.0,y:0.0,z:150.0},cableChainOffsetX:{x:0.0,y:200.0,z:400.0}
            });
    }
}
