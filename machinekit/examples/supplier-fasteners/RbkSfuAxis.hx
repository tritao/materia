import machinekit.assembly.MachineAssembly;
import machinekit.assembly.RbkMotorMount;
import machinekit.assembly.Transmission;
import machinekit.assembly.Sense;
import machinekit.component.*;
import machinekit.motion.*;
import machinekit.standard.*;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import cadkit.modeling.AssemblyState;

typedef AxisHardware = {
    var id:String;
    var size:String;
    var length:Float;
    var grip:Float;
    var engagement:Float;
    var availableDepth:Float;
    var interfacePair:String;
    var through:Bool;
    var nutStack:Float;
}

/** Worked supplier-intended SFU1605/RBK12-57/BF12 axis, 200 mm travel.
 * The 0.5 mm CAD stack discrepancy is retained as evidence, never custom machining.
 */
class RbkSfuAxis extends MachineAssembly {
    public final screw:VallderSfu1605Screw;
    public final nut:VallderSfu1605Nut;
    public final mount:RbkMotorMount;
    public final hardware:Array<AxisHardware> = [];
    public static function registry():ComponentRegistry {
        var r=MachineKitComponents.standard();r.register(RbkAxisPart.recipeType());r.register(RbkAxisCoupling.axisRecipeType());return r;
    }
    public function new() {
        super();
        mount=new RbkMotorMount("RBK12-57");screw=new VallderSfu1605Screw(400);nut=new VallderSfu1605Nut();
        addComponent("base",new RbkAxisPart("base"));
        addComponent("rbk",mount);addMate("rbk-base","fixed","base","rbk","rbk","base");
        addComponent("motor",NemaStepper.model("OPENBUILDS-NEMA23-56"));addMate("motor-mount","fixed","rbk","motor","motor","mountFace");
        addComponent("screw",screw);addMate("screwTurn","continuous","base","screwSeat","screw","raceStart");
        addComponent("coupling",new RbkAxisCoupling());addMate("coupling-screw","fixed","screw","input","coupling","screwEnd");
        addConstraint("motor-coupling-axis","continuous","motor","shaftTip","coupling","motorEnd");
        addComponent("bf12",new VallderBf12Support());addMate("bf12-base","fixed","base","floating","bf12","base");
        addComponent("outputRing",RetainingRing.forShaft(10));addMate("ring-groove","fixed","screw","outputRing","outputRing","seat");
        for(side in ["Left","Right"]) {
            addComponent("riser"+side,new RbkAxisPart("riser"+side));
            addMate("riser-base-"+side,"fixed","base","riser"+side,"riser"+side,"base");
            addComponent("rail"+side,LinearRail.metric("HGR15",300));
            addMate("rail-riser-"+side,"fixed","riser"+side,"rail","rail"+side,"base");
        }
        addComponent("bracket",new RbkAxisPart("nutBracket"));
        addMateOnAxis("travel","prismatic","base","travel","bracket","slide",{x:0,y:1,z:0},0,
            {lower:0,upper:200,velocity:null,effort:null});
        addComponent("carriage",new RbkAxisPart("carriage"));addMate("plate-bracket","fixed","bracket","plate","carriage","base");
        addComponent("ballNut",nut);addMate("nut-bracket","fixed","bracket","nut","ballNut","mountFace");
        for(side in ["Left","Right"]) {
            addComponent("block"+side,LinearRailBlock.metric("HGR15"));
            addMate("block-carriage-"+side,"fixed","carriage","guide"+side,"block"+side,"top");
            addConstraint("block-rail-"+side,"prismatic","rail"+side,"axis","block"+side,"rail");
        }
        // Kinematics only: the RBK's deep-groove bearing axial stiffness/preload is
        // not specified. Do not substitute the generic preloaded BK support law.
        addCoupling("lead","travel","screwTurn",2*Math.PI/nut.thread.signedLead());
        // All fasteners have installed geometry and independent BOM entries. Threaded-host
        // penetration is intentional; no preload or load rating is inferred.
        var meta:Dynamic=mount.interfaceMetadata(),baseHoles:Array<Dynamic> = meta.base_holes;
        for(i in 0...baseHoles.length) {
            var h=baseHoles[i],recess:Float=h.counterbore_depth_mm==null ? 0 : h.counterbore_depth_mm;
            through("rbkBolt"+i,"M6",h.z==40 ? 50 : 60,"base",F.alongY(h.x,h.top_y_mm-recess,h.z,0,1,0),
                h.z==40 ? 39 : 47,"rbk/base");
        }
        for(i in 0...2) through("bfBolt"+i,"M6",55,"base",F.alongY(i==0 ? -23 : 23,31,433.5,0,1,0),43,"bf12/base");
        for(side in [-1,1]) for(z in [110.0,390]) through('riser-$side-$z',"M6",60,"base",F.alongY(side*83,37,z,0,1,0),49,"riser/base");
        for(side in ["Left","Right"]) for(z in [20.0,80,140,200,260]) {
            var x=side=="Left" ? -67.0 : 67.0;
            bolt('rail-$side-$z',"M4",20,"base",F.alongY(x,46.7,z+100,0,1,0),9.7,12,"rail/riser");
        }
        for(p in NemaStepper.model("OPENBUILDS-NEMA23-56").boltPattern()) {
            var id='motor-${hardware.length}';
            bolt(id,"M4",12,"base",F.alongY(p.x,25+p.y,-5.6,0,0,-1),5.6,12,"motor/rbk");
            attach(id+"-washer",FlatWasher.metric("M4"),"base",F.alongY(p.x,25+p.y,-5.6,0,0,1));
        }
        var points=nut.boltPattern();
        for(i in 0...points.length) {
            var p=points[i];through("nutBolt"+i,"M5",30,"bracket",F.alongY(p.x,25+p.y,0,0,0,-1),20,"nut/bracket");
        }
        for(x in [-30.0,30]) for(z in [-10.0,10]) through('foot-$x-$z',"M6",30,"carriage",F.alongY(x,10,z,0,1,0),20,"carriage/bracket");
        for(side in [-67.0,67]) for(x in [-13.0,13]) for(z in [-13.0,13])
            bolt('block-$side-$x-$z',"M4",16,"carriage",F.alongY(side+x,10,z,0,1,0),10,6,"carriage/block (assumed 6 mm thread depth)");
        bolt("clampA","M4",10,"coupling",F.alongY(0,12.5,6,0,1,0),9.825,10,"coupling/motor shaft contact");
        bolt("clampB","M4",8,"coupling",F.alongY(0,12.5,34,0,1,0),7.5,8,"coupling/screw shaft contact");
    }
    function attach(id:String,part:MachineComponent,parent:String,pose:AssemblyFrame):Void {
        addComponent(id,part);
        addMemberConnector(parent,id+"-seat",pose);
        addMemberConnector(id,"installation",Solids.axial(0,0,0));
        addMate(id+"-fixed","fixed",parent,id+"-seat",id,"installation");
    }
    function bolt(id:String,size:String,length:Float,parent:String,seat:AssemblyFrame,grip:Float,depth:Float,pair:String):Void {
        var part=SocketHeadCapScrew.metric(size,length);attach(id,part,parent,seat);
        hardware.push({id:id,size:size,length:length,grip:grip,engagement:length-grip,availableDepth:depth,interfacePair:pair,through:false,nutStack:0});
    }
    function through(id:String,size:String,length:Float,parent:String,seat:AssemblyFrame,grip:Float,pair:String):Void {
        bolt(id,size,length,parent,seat,grip,0,pair);
        var washer=FlatWasher.metric(size),nut=HexNut.metric(size);
        var record=hardware[hardware.length-1];
        record.through=true;record.availableDepth=0;record.nutStack=washer.thickness+nut.spec.height;
        var w=F.compose(seat,F.translation(0,-grip-washer.thickness,0));
        var n=F.compose(seat,F.translation(0,-grip-washer.thickness-nut.spec.height,0));
        attach(id+"-washer",washer,parent,w);attach(id+"-nut",nut,parent,n);
    }
    public function setTravel(state:AssemblyState,value:Float):Void {
        if(!Math.isFinite(value) || value<0 || value>200) throw "Travel must be between 0 and 200 mm";
        state.setJoint("travel",value);
    }
}
