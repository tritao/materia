import machinekit.gantry.Gantry;
import machinekit.gantry.GantryParts.GantryPlate;
import machinekit.assembly.AxisBuilder;
import machinekit.component.*;
import machinekit.routing.*;
import machinekit.structural.*;
import machinekit.structural.WeldedFrame.FrameWeld;
import machinekit.control.ControlCabinet;
import machinekit.pneumatic.*;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;

/** Full-sheet reference machine; dimensions and bought-part ratings remain assumed. */
class SheetRouter extends Gantry {
    public final routerSpec:SheetRouterSpec;
    public final spindle:AtcSpindle;
    public final rack:ToolRack;
    public final shoe:DustShoe;
    public final table:VacuumTable;
    public final gaugeZero:AssemblyFrame;
    public final boardTop:Float;
    public final workOrigin:AssemblyFrame;
    public final rackPose:AssemblyFrame;
    public final base:WeldedFrame;
    public function new(?layout:SheetRouterSpec) {
        var chosen=layout==null?new SheetRouterSpec():layout;
        super(chosen.gantry());routerSpec=chosen;
        homeAfter("y",["z","x"]);
        bindMotorBrake("motorZ","motorZ","sheet-router/z-brake.release","sheet-router/z-brake.released",250,100);
        // The spindle stands upright, ahead of the slide; gauge is below its body.
        spindle=new AtcSpindle();
        var reach=spindle.n("diameter")/2+18;
        gaugeZero=F.translation(flangeZero.x,flangeZero.y-reach,flangeZero.z-240);
        attach("spindleMount",new SheetSpindleSaddle(spindle.n("diameter"),reach),gaugeZero,"zCarriage");
        attach("spindle",spindle,gaugeZero,"spindleMount");
        attach("spindleRelease",spindle.releaseCylinder(),F.compose(gaugeZero,F.translation(0,0,spindle.n("length"))),"spindle");
        exposeConnector("spindleGauge","spindle","gauge");
        var holder=new ToolHolder();attach("holder",holder,gaugeZero,"spindle");
        var collet=new Collet(new ComponentValues().setNumber("toolDiameter",12));attach("collet",collet,F.compose(gaugeZero,F.translation(0,0,-holder.n("gaugeLength"))),"holder");
        attach("longestTool",new SheetRouterTool(chosen.longestTool-holder.n("gaugeLength")),
            F.compose(gaugeZero,F.translation(0,0,-holder.n("gaugeLength"))),"holder");
        exposeConnector("toolTip","longestTool","tip");
        boardTop=gaugeZero.z-chosen.travelZ-chosen.longestTool;
        workOrigin=F.translation(0,gaugeZero.y,boardTop);
        table=new VacuumTable(new ComponentValues().setNumber("width",chosen.workX).setNumber("length",chosen.workY));
        var board=new Spoilboard(new ComponentValues().setNumber("width",chosen.workX).setNumber("length",chosen.workY));
        var bedBottom=boardTop-board.n("thickness")-table.n("height");
        var left=zeroPose("frameLeft").x,right=zeroPose("frameRight").x;
        var front=zeroPose("frameFront").y,back=zeroPose("frameBack").y;
        base=buildBase(left,right,front,back,zeroPose("frameFront").z-50,bedBottom);
        attach("base",base,F.identity(),"frameFront");
        buildChainSupports();
        var foot=new LevellingFoot();
        for(side in 0...2)for(end in 0...3) {
            var x=side==0?left:right,y=front+(back-front)*end/2;
            attach('foot${side}${end}',foot,F.translation(x,y,0),"base");
            attach('footCap${side}${end}',new SheetFootCap(),F.translation(x,y,88),"base");
        }
        attach("vacuumTable",table,F.translation(chosen.workX/2,gaugeZero.y+chosen.workY/2,bedBottom),"base");
        attach("spoilboard",board,F.translation(chosen.workX/2,gaugeZero.y+chosen.workY/2,boardTop-board.n("thickness")),"vacuumTable");
        for(i in 0...table.zoneCount()) {
            var valve=table.zoneValve(i),zone=F.compose(zeroPose("vacuumTable"),table.zoneFrame(i));
            attach('zoneValve${i+1}',valve,F.translation(zone.x,zone.y,bedBottom-valve.n("length")),"base");
            connectPorts('zone${i+1}', 'zoneValve${i+1}',"outlet","vacuumTable",'zone${i+1}');
        }
        attach("vacuumBlower",new VacuumBlower(),F.translation(right+300,(front+back)/2,0),"base");
        attach("vacuumHeader",new SheetVacuumHeader(),F.translation(right+300,(front+back)/2+600,100),"base");
        connectPorts("vacuumFeed","vacuumBlower","vacuum","vacuumHeader","input");
        for(i in 1...7)connectPorts('vacuumBranch$i',"vacuumHeader",'out$i','zoneValve$i',"inlet");
        // Forks open towards -Y, so their approach remains inside the extra Y travel.
        rack=new ToolRack(new ComponentValues().setNumber("forkHeight",160).setNumber("coverStroke",600));
        rackPose=F.translation(chosen.workX/2,gaugeZero.y+chosen.workY+260,boardTop-20);
        attach("toolRack",rack,rackPose,"base");
        attach("rackSupport",new GantryPlate("rack support",rack.parameterCount("pockets")*rack.n("pitch"),180,rackPose.z-bedBottom),
            F.translation(rackPose.x,rackPose.y,bedBottom),"base");
        for(i in 0...rack.parameterCount("pockets")) {
            var pocket=F.compose(rackPose,rack.pocketFrame(i));
            var pose:AssemblyFrame={x:pocket.x,y:pocket.y-rack.n("depth")/2-50,z:pocket.z-12,qx:0,qy:0,qz:1,qw:0};
            attach('fork${i+1}',new ToolFork(),pose,"toolRack");
            attach('forkRiser${i+1}',new GantryPlate("fork riser",80,12,pose.z-rackPose.z-rack.n("height")),
                F.translation(pose.x,rackPose.y-rack.n("depth")/2+6,rackPose.z+rack.n("height")),"toolRack");
        }
        attach("setterPad",new GantryPlate("setter pedestal",100,100,boardTop-bedBottom),F.translation(100,rackPose.y,bedBottom),"base");
        attach("setter",new ToolLengthSetter(),F.translation(100,rackPose.y,boardTop),"setterPad");
        exposeConnector("setterTrigger","setter","trigger");
        attach("airSupply",new AirSupply(),F.translation(right+300,front,100),"base");
        attach("airManifold",new PneumaticManifold(2),F.translation(right+300,front,230),"base");
        connectPorts("airFeed","airSupply","air","airManifold","input");
        shoe=new DustShoe(new ComponentValues().setNumber("diameter",150).setNumber("brushLength",40).setNumber("hoseRise",20).setNumber("hoseOffset",120).setNumber("hoseAngle",3*Math.PI/2));
        var shoeZero=F.compose(gaugeZero,F.translation(0,0,40));
        lift("shoeLift","shoe",shoe,shoeZero,"spindle",shoe.liftCylinder(),shoe.n("liftStroke"),0);
        var coverZ=Math.max(rackPose.z+rack.n("forkHeight")+60,zeroPose("frameBack").z+spec.frameSection().height/2+5);
        var cover=new GantryPlate("rack cover",rack.parameterCount("pockets")*rack.n("pitch")+20,300,8);
        lift("rackCoverLift","rackCover",cover,F.translation(rackPose.x,rackPose.y-20+rack.n("coverStroke"),coverZ),
            "toolRack",rack.coverCylinder(),rack.n("coverStroke"),0);
        var cabinetPose=F.translation(right+600,front+600,0);
        attach("vfd",new machinekit.power.Vfd(),F.compose(cabinetPose,F.translation(-180,0,200)),"base");
        attach("brakeResistor",new machinekit.power.BrakeResistor(),F.compose(cabinetPose,F.translation(400,0,1050)),"base");
        attach("mainBreaker",new machinekit.power.CircuitBreaker(),F.compose(cabinetPose,F.translation(0,0,600)),"base");
        attach("mainContactor",new machinekit.power.Contactor(),F.compose(cabinetPose,F.translation(80,0,600)),"base");
        attach("safetyRelay",new machinekit.control.SafetyRelay(),F.compose(cabinetPose,F.translation(180,0,600)),"base");
        attach("emergencyStop",new machinekit.control.EmergencyStopButton(),F.compose(cabinetPose,F.translation(0,200,900)),"base");
        connectPorts("spindleMotor","vfd","motor","spindle","motor");
        connectPorts("vfdBrake","vfd","brake_resistor","brakeResistor","brake");
        // Safety contacts are physical parts only; SR17 authors and validates their wiring.
        var io:Array<{id:String,part:AssumedCatalogComponent}> = [];
        for(member in components())if(Std.isOfType(member.component,AssumedCatalogComponent))io.push({id:member.id,part:cast member.component});
        attach("cabinet",ControlCabinet.forParts(io),F.translation(right+600,front+600,0),"base");
        var connections=describe().machine.portConnections;
        for(member in components())for(port in member.component.ports())if(port.required==true) {
            var connected=false;
            for(edge in connections)if((edge.fromInstance==member.id&&edge.fromPort==port.name)||
                (edge.toInstance==member.id&&edge.toPort==port.name))connected=true;
            if(!connected)exposePort(member.id+"-"+port.name,member.id,port.name);
        }

    }
    function lift(joint:String,id:String,part:MachineComponent,pose:AssemblyFrame,parent:String,cylinder:PneumaticCylinder,stroke:Float,initial:Float):Void {
        var along=joint=="rackCoverLift"?[0.0,-1.0,0.0]:[0.0,0.0,-1.0];
        var anchorX=joint=="rackCoverLift"?120.0:-120.0;
        var anchorZ=joint=="rackCoverLift"?60.0:40.0;
        var up=joint=="rackCoverLift"?[0.0,0.0,1.0]:[0.0,1.0,0.0];
        hang(id,part,pose,parent);
        seatAt(parent,joint+"Axis",F.translation(pose.x+anchorX,pose.y,pose.z+anchorZ));
        addMemberConnector(id,joint+"Axis",F.translation(anchorX,0,anchorZ));
        addMateOnAxis(joint,"prismatic",parent,joint+"Axis",id,joint+"Axis",{x:along[0],y:along[1],z:along[2]},initial,
            {lower:0.0,upper:stroke,velocity:null,effort:null});
        var c=joint+"Cylinder",v=joint+"Valve";
        // Cylinder and guided tip share the same axis and default extension.
        var extension=initial-cylinder.bodyLength-cylinder.stemLength;
        var body=AxisBuilder.orient(pose.x+anchorX+along[0]*extension,pose.y+along[1]*extension,pose.z+anchorZ+along[2]*extension,up,along);
        attach(c,cylinder,body,parent);
        attach(joint+"Bracket",new GantryPlate("lift rod bracket",80,20,anchorZ),
            F.translation(pose.x+anchorX,pose.y,pose.z),id);
        attach(joint+"Rod",cylinder.movingRod(),AxisBuilder.orient(pose.x+anchorX,pose.y,pose.z+anchorZ,up,along),id);
        attach(v,new SolenoidValve(),F.translation(pose.x+(joint=="shoeLift"?-220:220),pose.y,pose.z+200),parent);
        connectPorts(joint+"Air","airManifold",joint=="shoeLift"?"out1":"out2",v,"P");
        connectPorts(joint+"A",v,"A",c,"A");connectPorts(joint+"B",v,"B",c,"B");
        addCylinder(joint+"Drive",joint,c,v);
        addSwitch(joint+"Retracted",joint,{lower:0.0,upper:0.25,velocity:null,effort:null},0.1);
        addSwitch(joint+"Extended",joint,{lower:stroke-0.25,upper:stroke,velocity:null,effort:null},0.1);
    }
    function buildChainSupports():Void {
        var beam=zeroPose("beam"),frame=zeroPose("frameLeft"),carriage=zeroPose("xCarriage");
        function box(x0:Float,y0:Float,z0:Float,x1:Float,y1:Float,z1:Float):SheetChainBox
            return {x:(x0+x1)/2,y:(y0+y1)/2,z:z0,w:x1-x0,d:y1-y0,h:z1-z0};
        function support(id:String,parent:String,boxes:Array<SheetChainBox>,end:AssemblyFrame,?cuts:Array<SheetChainBox>):Void
            attach(id,new SheetChainSupport(id,boxes,end,cuts),F.identity(),parent);
        for(binding in cableChainBindings) {
            var end=cableChainEnd(binding.axis,0),fixed=binding.pose;
            if(binding.axis=="y") {
                support("chainYMovingBracket","beam",[
                    box(beam.x-100,beam.y-40,beam.z+100,beam.x+60,beam.y+40,beam.z+112),
                    box(end.x-20,end.y-30,beam.z+112,end.x+20,end.y,end.z-15.5),
                    box(end.x-25,end.y-10,end.z-17.5,end.x+25,end.y,end.z+17.5)
                ],end,[box(end.x-20,end.y-11,end.z-12.5,end.x+20,end.y+1,end.z+12.5)]);
                var post=frame.x-350;
                support("chainYFixedBracket","frameLeft",[
                    box(post-20,fixed.y-20,frame.z+38,frame.x-50,fixed.y+20,frame.z+50),
                    box(post-20,fixed.y-20,frame.z+50,post+20,fixed.y+20,fixed.z-15.5),
                    box(post-20,fixed.y-20,fixed.z-27.5,fixed.x+25,fixed.y+20,fixed.z-15.5)
                ],fixed);
            } else {
                var plate:machinekit.gantry.GantryParts.GantryPlate=cast component("xCarriage");
                var carriageTop=carriage.z+plate.depth/2;
                support("chainXMovingBracket","xCarriage",[
                    box(carriage.x-40,carriage.y-40,carriageTop-12,carriage.x+40,carriage.y+40,carriageTop),
                    box(carriage.x-35,carriage.y-15,carriageTop,carriage.x-5,carriage.y+15,end.z-15.5),
                    box(end.x-40,carriage.y-20,end.z-27.5,end.x,end.y+25,end.z-15.5),
                    box(end.x-10,end.y-25,end.z-17.5,end.x,end.y+25,end.z+17.5)
                ],end,[box(end.x-11,end.y-20,end.z-12.5,end.x+1,end.y+20,end.z+12.5)]);
                var post=beam.y+420;
                support("chainXFixedBracket","beam",[
                    box(fixed.x-20,beam.y+50,beam.z,fixed.x+20,post+20,beam.z+12),
                    box(fixed.x-20,post-20,beam.z+12,fixed.x+20,post+20,fixed.z-15.5),
                    box(fixed.x-20,fixed.y-25,fixed.z-27.5,fixed.x+20,post+20,fixed.z-15.5)
                ],fixed);
            }
        }
    }
    function buildBase(left:Float,right:Float,front:Float,back:Float,top:Float,bed:Float):WeldedFrame {
        var frame=new FrameAssembly(),ids:Array<String> = [],welds:Array<FrameWeld> = [];
        var tube=new RectTube(100,100,6),cross=new RectTube(100,80,4);
        function member(id:String,a:Array<Float>,b:Array<Float>,profile:RectTube):Void {
            frame.point(id+"a",a[0],a[1],a[2]);frame.point(id+"b",b[0],b[1],b[2]);
            frame.member(id,id+"a",id+"b",profile);ids.push(id);
        }
        for(side in 0...2) {
            var x=side==0?left:right;
            member('lower$side',[x,front,180],[x,back,180],tube);
            for(end in 0...3) {
                var y=front+(back-front)*end/2;
                member('post$side$end',[x,y,100],[x,y,top],tube);
                welds.push({a:'lower$side',b:'post$side$end',legSize:6});
            }
        }
        for(i in 0...7) {
            var y=workOrigin.y+routerSpec.workY*i/6;
            member('tableCross$i',[left,y,bed-40],[right,y,bed-40],cross);
            // Side stringers meet every table crossmember.
            for(side in 0...2)welds.push({a:'tableSide$side',b:'tableCross$i',legSize:5});
        }
        member("rackCross",[left,workOrigin.y+routerSpec.workY+260,bed-40],[right,workOrigin.y+routerSpec.workY+260,bed-40],cross);
        for(side in 0...2) {
            welds.push({a:'tableSide$side',b:"rackCross",legSize:5});
            var x=side==0?left:right;
            member('tableSide$side',[x,front,bed-40],[x,back,bed-40],cross);
            for(end in 0...3)welds.push({a:'tableSide$side',b:'post$side$end',legSize:5});
        }
        return new WeldedFrame('sheet-router-${routerSpec.workY}',frame,ids,[],welds);
    }
    /** Axis coordinate needed to bring the spindle gauge to a world position. */
    public function gaugeCoordinates(world:AssemblyFrame):Array<Float> return [world.x-gaugeZero.x,world.y-gaugeZero.y,gaugeZero.z-world.z];
}

class SheetRouterTool extends MachineComponent {
    public final length:Float;
    public function new(length:Float) {super('REFERENCE-ROUTER-TOOL-$length','Longest assumed router tool envelope','steel',true);this.length=length;addConnector("tip",Axis,F.translation(0,0,-length));}
    override public function hasGeometry():Bool return true;
    override public function geometry(detail:ComponentDetail=Preview):Part return Part.cylinderSpan(6,-length,0);
}

/** Two spaced clamps react spindle moments into a vertical plate through a bolt couple. */
class SheetSpindleSaddle extends MachineComponent {
    public final diameter:Float;
    public final reach:Float;
    public function new(diameter:Float,reach:Float) {
        super('SHEET-SPINDLE-SADDLE-$diameter-$reach',"Two-level spindle clamp saddle","aluminium 6061",true);
        this.diameter=diameter;this.reach=reach;
        for(x in [-50.0,50.0])for(z in [110.0,230.0])addConnector('mount$x-$z',Mount,F.translation(x,reach,z));
    }
    override public function hasGeometry():Bool return true;
    override public function geometry(detail:ComponentDetail=Preview):Part return Solids.building([],parts->{
        var blank=Part.box(160,16,160),back=blank.translated(new Vector(0,reach-8,90));blank.close();parts.push(back);
        for(z in [100.0,220.0]) {
            var ring=Part.cylinderSpan(diameter/2+20,z,z+20);parts.push(ring);
            var web=Part.box(120,reach,20),placed=web.translated(new Vector(0,reach/2,z));web.close();parts.push(placed);
        }
        var body=Solids.union(parts),tools:Array<Part> = [Part.cylinderSpan(diameter/2+0.2,89,251)];
        for(z in [100.0,220.0]) {
            var blank=Part.box(2,diameter+40,22),slit=blank.translated(new Vector(0,-(diameter+40)/2,z-1));blank.close();tools.push(slit);
        }
        for(x in [-50.0,50.0])for(z in [110.0,230.0])tools.push(Part.cylinderAlong(3.3,new Vector(x,reach+1,z),new Vector(0,-1,0),18));
        // Trim the ring flats to the actual plate mounting plane.
        var cutter=Part.box(diameter+100,100,200),rear=cutter.translated(new Vector(0,reach+50,80));cutter.close();tools.push(rear);
        return Solids.cut(body,tools);
    });
}

/** Assumed common vacuum header; zone isolation stays in the six separate valves. */
class SheetVacuumHeader extends MachineComponent {
    public function new() {
        super("SHEET-VACUUM-HEADER-6","Six-zone vacuum distribution header","aluminium 6061",true);
        addPort({name:"input",kind:Vacuum,role:Consumer,iface:Thread("G2"),required:true});
        for(i in 1...7) {addPort({name:'out$i',kind:Vacuum,role:Supply,iface:Thread("G2"),required:true});addBridge("input",'out$i');}
    }
    override public function hasGeometry():Bool return true;
    override public function geometry(detail:ComponentDetail=Preview):Part return Solids.cut(Part.box(500,100,100),[Part.cylinderAlong(30,new Vector(-251,0,50),new Vector(1,0,0),502)]);
}

/** Nominal M20 threaded leg cap: bore is the semantic thread's major-diameter envelope. */
class SheetFootCap extends MachineComponent {
    public function new() {super("SHEET-LEG-CAP-M20","Assumed M20 threaded leg cap","steel",true);}
    override public function hasGeometry():Bool return true;
    override public function geometry(detail:ComponentDetail=Preview):Part return Solids.cut(Part.box(100,100,12),[Part.cylinderSpan(10,-1,13)]);
}

typedef SheetChainBox = {var x:Float;var y:Float;var z:Float;var w:Float;var d:Float;var h:Float;}

/** Welded chain support envelopes; feet sit outside the moving motor housings. */
class SheetChainSupport extends MachineComponent {
    final boxes:Array<SheetChainBox>;
    final cuts:Array<SheetChainBox>;
    public function new(id:String,boxes:Array<SheetChainBox>,end:AssemblyFrame,?cuts:Array<SheetChainBox>) {
        super("SHEET-"+id,"Cable-chain support "+id,"steel",true);this.boxes=boxes;this.cuts=cuts==null?[]:cuts;
        for(box in boxes)if(box.w<=0||box.d<=0||box.h<=0)throw "Invalid chain support dimensions";
        addConnector("chain",Mount,end);
    }
    override public function hasGeometry():Bool return true;
    override public function geometry(detail:ComponentDetail=Preview):Part return Solids.building([],parts->{
        for(box in boxes) {var blank=Part.box(box.w,box.d,box.h);var placed=blank.translated(new Vector(box.x,box.y,box.z));blank.close();parts.push(placed);}
        var body=Solids.union(parts),tools:Array<Part> = [];
        for(box in cuts) {var blank=Part.box(box.w,box.d,box.h);var placed=blank.translated(new Vector(box.x,box.y,box.z));blank.close();tools.push(placed);}
        return Solids.cut(body,tools);
    });
}
