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
    public final travelZ:Float = 250;
    public final rackZone:Float = 300;
    public final longestTool:Float = 120;
    public final sheetThickness:Float = 40;
    public function new(workY:Float = 2600) {
        if(!Math.isFinite(workY)||workY<2600)throw "Sheet router work length must be at least 2600 mm";
        this.workY=workY;
    }
    public function gantry():GantrySpec {
        var rack:GantryDrive=HelicalRack(1.5,20,20,Math.acos(3*Math.PI/10));
        var screw:GantryDrive=Screw(new LeadScrewThread(Ball,16,5,1,RightHand,13));
        return new GantrySpec(workX,workY+rackZone,travelZ,rack,rack,screw,true,"HGR20",23,
            "HFS5-4040","HFS5-4040",true,None,0.5,new machinekit.power.DcVoltage(560),16,300,100,200,180,{
                railProfileY:"HGR25",yBlockSpacing:140,railPadHeight:12,
                motor:Servo("GENERIC-HV-SERVO-750W"),motorX:Servo("GENERIC-HV-SERVO-400W"),motorZ:BrakedServo("400"),
                reducer:new PlanetaryReducer(),reducerX:new PlanetaryReducer(new ComponentValues().setNumber("bore",14)),
                preload:new machinekit.transmission.PreloadedPinionMount(),
                structure:Steel(new RectTube(100,100,6),new RectTube(100,200,6.3)),cableChain:new CableChain(),
                cableChainOffsetY:{x:-80.0,y:0.0,z:150.0},cableChainOffsetX:{x:0.0,y:200.0,z:250.0}
            });
    }
}
