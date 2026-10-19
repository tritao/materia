package machinekit.assembly;

import machinekit.assembly.MachineAssembly;
import machinekit.assembly.RbkMotorMount;
import machinekit.component.*;
import machinekit.motion.*;
import machinekit.standard.*;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import cadkit.modeling.AssemblyState;

typedef AxisClearance = {
    var travel:Float;
    var a:String;
    var b:String;
    var intersectionMm3:Float;
}

typedef AxisHardware = {
    var id:String;
    var size:String;
    var length:Float;
    var grip:Float;
    var engagement:Float;
    var availableDepth:Float;
    var interfacePair:String;
    var minimumEngagement:Float;
}

/** Reusable purchased-support twin-rail axis for the checked RBK12/SFU16/HGR15 interface.
 * Component selection and fabrication layout are explicit inputs; purchasing stays in SourcingPlan.
 * Unlike the generic LinearAxis rod/rail envelope, this template includes the actual support stack
 * and installation hardware. The supplier CAD's 0.5 mm stack discrepancy is retained.
 */
class TwinRailScrewAxis extends MachineAssembly {
    public final screw:LeadScrew;
    public final nut:LeadScrewNut;
    public final mount:RbkMotorMount;
    public final hardware:Array<AxisHardware> = [];
    public final layout:ScrewAxisLayout;
    public final selected:ScrewAxisComponents;
    public function new(layout:ScrewAxisLayout,selected:ScrewAxisComponents,?fabricate:String->MachineComponent,?coupling:ShaftCoupling) {
        super();selected.check(layout);this.layout=layout;this.selected=selected;
        mount=selected.mount;screw=selected.screw;nut=selected.nut;
        if(fabricate==null)fabricate=kind -> new ScrewAxisPart(kind,"v2-tapped",layout);
        if(coupling==null)coupling=new ScrewAxisCoupling(selected.motor.variant.shaftLength);
        if(Math.abs(coupling.boreA-selected.motor.variant.shaftDiameter)>1e-6 || Math.abs(coupling.boreB-10)>1e-6 ||
           Math.abs(coupling.connector("motorEnd").frame.z-(selected.motor.variant.shaftLength-12))>1e-6 ||
           Math.abs(coupling.connector("screwEnd").frame.z-28.5)>1e-6 || coupling.length!=40 || coupling.outerDiameter!=25)
            throw "Incompatible axis coupling bores or engagement connectors";
        addComponent("base",fabricate("base"));
        addComponent("rbk",mount);addMate("rbk-base","fixed","base","rbk","rbk","base");
        addComponent("motor",selected.motor);addMate("motor-mount","fixed","rbk","motor","motor","mountFace");
        addComponent("screw",screw);addMate("screwTurn","continuous","base","screwSeat","screw","raceStart");
        addComponent("coupling",coupling);addMate("coupling-screw","fixed","screw","input","coupling","screwEnd");
        addConstraint("motor-coupling-axis","continuous","motor","shaftTip","coupling","motorEnd");
        addComponent("bf12",selected.support);addMate("bf12-base","fixed","base","floating","bf12","base");
        addComponent("outputRing",RetainingRing.forShaft(10));addMate("ring-groove","fixed","screw","outputRing","outputRing","seat");
        for(side in ["Left","Right"]) {
            addComponent("riser"+side,fabricate("riser"+side));
            addMate("riser-base-"+side,"fixed","base","riser"+side,"riser"+side,"base");
            addComponent("rail"+side,selected.rail);
            addMate("rail-riser-"+side,"fixed","riser"+side,"rail","rail"+side,"base");
        }
        addComponent("bracket",fabricate("nutBracket"));
        addMateOnAxis("travel","prismatic","base","travel","bracket","slide",{x:0,y:1,z:0},0,
            {lower:0,upper:layout.travel,velocity:null,effort:null});
        addComponent("carriage",fabricate("carriage"));addMate("plate-bracket","fixed","bracket","plate","carriage","base");
        addComponent("ballNut",nut);addMate("nut-bracket","fixed","bracket","nut","ballNut","mountFace");
        for(side in ["Left","Right"]) {
            addComponent("block"+side,selected.block);
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
            var grip:Float=h.top_y_mm-recess;
            tappedBolt("rbkBolt"+i,"M6",h.z==40 ? 40 : 45,"base",F.alongY(h.x,grip,h.z,0,1,0),
                grip,12,"rbk/base",h.z==40);
        }
        for(i in 0...2) tappedBolt("bfBolt"+i,"M6",40,"base",F.alongY(i==0 ? -23 : 23,31,layout.floatingZ(),0,1,0),31,12,"bf12/base");
        for(side in [-1,1]) for(z in layout.riserBolts()) tappedBolt('riser-$side-$z',"M6",40,"base",F.alongY(side*(layout.halfSpacing()+ScrewAxisPart.RISER_BOLT_OFFSET),ScrewAxisPart.riserBoltSeat(),z,0,1,0),ScrewAxisPart.riserBoltSeat(),12,"riser/base");
        for(side in ["Left","Right"]) for(z in selected.rail.holePositions) {
            var x=side=="Left" ? -layout.halfSpacing() : layout.halfSpacing();
            bolt('rail-$side-$z',"M4",20,"base",F.alongY(x,46.7,z+100,0,1,0),9.7,12,"rail/riser",6);
        }
        for(p in selected.motor.boltPattern()) {
            var id='motor-${hardware.length}';
            var grip=selected.motor.variant.mountHoleDepth+(selected.motorWasher ? 0.8 : 0);
            bolt(id,"M4",12,"base",F.alongY(p.x,25+p.y,-grip,0,0,-1),grip,12,"motor/rbk");
            if(selected.motorWasher)attach(id+"-washer",FlatWasher.metric("M4"),"base",F.alongY(p.x,25+p.y,-grip,0,0,1));
        }
        var points=nut.boltPattern();
        for(i in 0...points.length) {
            var p=points[i];tappedBolt("nutBolt"+i,"M5",20,"bracket",F.alongY(p.x,25+p.y,0,0,0,-1),10,10,"nut/bracket",true);
        }
        for(x in [-30.0,30]) for(z in [-10.0,10]) tappedBolt('foot-$x-$z',"M6",20,"carriage",F.alongY(x,10,z,0,1,0),10,10,"carriage/bracket");
        for(side in [-layout.halfSpacing(),layout.halfSpacing()]) for(x in [-13.0,13]) for(z in [-13.0,13])
            bolt('block-$side-$x-$z',"M4",12,"carriage",F.alongY(side+x,7.5,z,0,1,0),7.5,selected.blockThreadDepth,"carriage/block",4);
        bolt("clampA","M4",10,"coupling",F.alongY(0,12.5,6,0,1,0),9.825,10,"coupling/motor shaft contact");
        bolt("clampB","M4",8,"coupling",F.alongY(0,12.5,34,0,1,0),7.5,8,"coupling/screw shaft contact");
        for(h in hardware)if(!StringTools.startsWith(h.interfacePair,"coupling/") &&
            (h.engagement<h.minimumEngagement || h.engagement>h.availableDepth))
            throw "Incompatible axis screw engagement: "+h.id;
    }
    function attach(id:String,part:MachineComponent,parent:String,pose:AssemblyFrame):Void {
        addComponent(id,part);
        addMemberConnector(parent,id+"-seat",pose);
        addMemberConnector(id,"installation",Solids.axial(0,0,0));
        addMate(id+"-fixed","fixed",parent,id+"-seat",id,"installation");
    }
    function bolt(id:String,size:String,length:Float,parent:String,seat:AssemblyFrame,grip:Float,depth:Float,pair:String,minimum:Float=0):Void {
        var part=SocketHeadCapScrew.metric(size,length);attach(id,part,parent,seat);
        hardware.push({id:id,size:size,length:length,grip:grip,engagement:length-grip,availableDepth:depth,interfacePair:pair,minimumEngagement:minimum==0 ? 1.3*part.diameter : minimum});
    }
    /** Threads live in the fabricated receiving part; the purchased part keeps its clearance holes. */
    function tappedBolt(id:String,size:String,length:Float,parent:String,seat:AssemblyFrame,grip:Float,depth:Float,pair:String,washer:Bool=false):Void {
        var headSeat=seat,stack=grip;
        if(washer) {
            var flat=FlatWasher.metric(size);
            attach(id+"-washer",flat,parent,seat);
            stack+=flat.thickness;headSeat=F.compose(seat,F.translation(0,flat.thickness,0));
        }
        bolt(id,size,length,parent,headSeat,stack,depth,pair,1.5*SocketHeadCapScrew.metric(size,length).diameter);
    }
    /** Nominal solid intersections at start, midpoint and end. Thread penetration is intentional;
     * only installation pairs which must clear are tested. This is not a load/preload qualification.
     */
    public function installationClearances():Array<AxisClearance> {
        var state=new AssemblyState(definition()),cache=new PosedParts();
        var entries:Map<String,MachineComponent> = [];
        for(entry in components())entries.set(entry.id,entry.component);
        var pairs=[["rbk","screw"],["rbk","coupling"],["rbk","base"],["bf12","screw"],["bf12","base"],
            ["coupling","motor"],["coupling","screw"],["ballNut","screw"],["ballNut","bracket"],["bracket","base"],
            ["bracket","riserLeft"],["bracket","riserRight"],["carriage","rbk"],["carriage","bf12"],
            ["blockLeft","railLeft"],["blockRight","railRight"],["blockLeft","riserLeft"],["blockRight","riserRight"],
            ["blockLeft","carriage"],["blockRight","carriage"],["blockLeft","bracket"],["blockRight","bracket"]];
        for(h in hardware) {
            if(h.interfacePair=="motor/rbk"){pairs.push(["motor",h.id]);if(entries.exists(h.id+"-washer"))pairs.push(["motor",h.id+"-washer"]);}
            if(h.interfacePair=="riser/base")for(side in ["Left","Right"])pairs.push(["block"+side,h.id]);
            if(h.interfacePair=="carriage/block")pairs.push(["carriage",h.id]);
            if(h.interfacePair=="nut/bracket")pairs.push(["ballNut",h.id]);
            if(h.interfacePair=="rbk/base")pairs.push(["rbk",h.id]);
            if(h.interfacePair=="bf12/base")pairs.push(["bf12",h.id]);
        }
        var result:Array<AxisClearance> = [];
        try {
            for(travel in [0.0,layout.travel/2,layout.travel]) {
                setTravel(state,travel);var poses=solvedPoses(state);
                for(pair in pairs)result.push({travel:travel,a:pair[0],b:pair[1],
                    intersectionMm3:cache.overlap(entries.get(pair[0]),poses.get(pair[0]),entries.get(pair[1]),poses.get(pair[1]))});
            }
            cache.close();return result;
        } catch(error:Dynamic){cache.close();throw error;}
    }
    public function validateInstallation():Void {
        validate();
        for(row in installationClearances())if(row.intersectionMm3>0.001)
            throw "Axis installation clash "+row.a+"/"+row.b+" at "+row.travel+" mm: "+row.intersectionMm3+" mm3";
    }
    public function setTravel(state:AssemblyState,value:Float):Void {
        if(!Math.isFinite(value) || value<0 || value>layout.travel) throw "Travel outside configured axis limits";
        state.setJoint("travel",value);
    }
}
