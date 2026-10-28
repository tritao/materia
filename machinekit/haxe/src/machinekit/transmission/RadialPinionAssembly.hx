package machinekit.transmission;

import machinekit.assembly.AxisBuilder;
import machinekit.component.*;
import machinekit.structural.*;
import machinekit.standard.*;
import machinekit.motion.SteppedShaft;
import machinekit.gantry.GantryParts.GantryPlate;
import machinekit.transmission.PinionPreloadHardware.PinionPreloadAdjuster;
import machinekit.transmission.PinionPreloadHardware.PinionDiscSpringPack;
import machinekit.transmission.PinionPreloadHardware.PinionSpringCup;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;
import materia.casting.InsertHole;

/** Builds the fixed anchor first, then a bounded passive lever carrying motor,
 * reducer and pinion. No second rigid motor support bypasses the hinge/spring. */
class RadialPinionAssembly {
    public static inline final ARM=80.0;
    public static inline final SPRING_ARM=56.0;
    public static function lowerAngle():Float return -Math.asin(0.1/ARM);
    public static function upperAngle():Float return Math.asin(0.5/ARM);
    public static function build(builder:AxisBuilder,id:String,motorId:String,hostId:String,motor:MachineComponent,
        oldFace:AssemblyFrame,pinionCentre:AssemblyFrame,spec:PreloadedPinionMount,hostFace:AssemblyFrame,topMount:Bool):Void {
        var shaft=F.transformVector(oldFace,0,0,1),up=[0.0,0,1];
        var face=AxisBuilder.orient(oldFace.x-9*shaft.x,oldFace.y-9*shaft.y,oldFace.z-9*shaft.z,up,[shaft.x,shaft.y,shaft.z]);
        var along=F.transformVector(face,1,0,0);
        var frame=F.compose(F.translation(-ARM*along.x,-ARM*along.y,-ARM*along.z),face);
        var inverse=F.inverse(frame),centre=F.transformPoint(inverse,pinionCentre.x,pinionCentre.y,pinionCentre.z);
        if(Math.abs(centre.x-ARM)>1e-6||Math.abs(centre.y)>1e-6)throw "Pivot carrier does not meet the pinion shaft";
        var contact=F.transformPoint(inverse,hostFace.x,hostFace.y,hostFace.z);
        if(topMount&&Math.abs(contact.y-24)>1e-6)throw "Y anchor requires the declared foot top 24 mm above shaft";
        var carrier=new RadialPinionCarrier(motor,spec,centre.z),anchor=new PinionPivotAnchor(contact.z,topMount);
        var anchorId=id+"Anchor",carrierId=id+"Preload";
        builder.attach(anchorId,anchor,frame,hostId);
        builder.hang(carrierId,carrier,frame,anchorId);
        builder.addMateOnAxis(id+"-preload-pivot","revolute",anchorId,'to-$carrierId',carrierId,'attach-$carrierId',shaft,0,
            {lower:lowerAngle(),upper:upperAngle(),velocity:null,effort:null});
        builder.setPassive(id+"-preload-pivot",true);
        builder.attach(motorId,motor,face,carrierId);
        function at(local:AssemblyFrame):AssemblyFrame return F.compose(frame,local);
        function put(name:String,part:MachineComponent,local:AssemblyFrame,parent:String):Void builder.attach(id+name,part,at(local),parent);
        var pin=new SteppedShaft([{diameter:10.0,length:42.0}],null,null,[
            {name:"ringA",z0:2.0,width:1.1,diameter:9.6},{name:"ringB",z0:38.0,width:1.1,diameter:9.6}]);
        put("PivotPin",pin,F.translation(0,0,-13),anchorId);
        for(name in ["ringA","ringB"])builder.mount(id+name,RetainingRing.forShaft(10),"seat",id+"PivotPin",name);
        put("PivotBush",new Bushing(10.05,14,15),F.identity(),carrierId);
        // Select/finish these bronze thrust shims against the measured clevis
        // gap: 0.01..0.02 mm total endplay, not the empty 1 mm gaps per side.
        put("PivotThrustFront",new Bushing(10.1,22,0.99),F.translation(0,0,-1),anchorId);
        put("PivotThrustRear",new Bushing(10.1,22,0.99),F.translation(0,0,15.01),anchorId);
        var springAxis=AxisBuilder.orient(-56,-29,7.5,[1.0,0,0],[0.0,1,0]);
        put("SpringAdjuster",new PinionPreloadAdjuster(),springAxis,anchorId);
        put("SpringAdjusterWasher",FlatWasher.metric("M10"),AxisBuilder.orient(-56,-43,7.5,[1.0,0,0],[0.0,-1,0]),anchorId);
        put("SpringAdjusterNut",HexNut.metric("M10"),AxisBuilder.orient(-56,-45,7.5,[1.0,0,0],[0.0,-1,0]),anchorId);
        put("SpringPack",new PinionDiscSpringPack(),AxisBuilder.orient(-56,-25,7.5,[1.0,0,0],[0.0,1,0]),anchorId);
        put("SpringCup",new PinionSpringCup(),AxisBuilder.orient(-56,-13,7.5,[1.0,0,0],[0.0,1,0]),carrierId);
        var upperTip=(10-56*Math.sin(lowerAngle()))/Math.cos(lowerAngle())+3*Math.abs(Math.tan(lowerAngle()));
        var lowerTip=(-10-56*Math.sin(upperAngle()))/Math.cos(upperAngle())-3*Math.abs(Math.tan(upperAngle()));
        put("UpperStop",SocketSetScrew.forThread(FastenerThreads.coarse(6),35),AxisBuilder.orient(-56,upperTip+35,-16,[1.0,0,0],[0.0,1,0]),anchorId);
        put("UpperStopNut",HexNut.metric("M6"),AxisBuilder.orient(-56,34,-16,[1.0,0,0],[0.0,1,0]),anchorId);
        put("LowerStop",SocketSetScrew.forThread(FastenerThreads.coarse(6),35),AxisBuilder.orient(-56,lowerTip-35,-16,[1.0,0,0],[0.0,-1,0]),anchorId);
        put("LowerStopNut",HexNut.metric("M6"),AxisBuilder.orient(-56,-35,-16,[1.0,0,0],[0.0,-1,0]),anchorId);
        // Servo flange holes belong to the actual motor; no catalogue pitch is
        // repeated here. Through screws clamp flange and lever, clear of reducer.
        var flange=machinekit.motion.AxisMotor.of(motor).flange;
        if(flange.thread!=null || flange.holes.length!=4)throw "This carrier requires a four-hole drilled motor flange";
        var sizes:Array<String>=[];
        for(size in ["M3","M4","M5","M6"]) {
            var data=SocketHeadCapScrew.catalog().get(size);
            if(flange.holeDiameter>=data.clearanceFine-1e-6 && flange.holeDiameter<=data.clearanceCoarse+1e-6)sizes.push(size);
        }
        if(sizes.length!=1)throw "Motor flange needs an explicit compatible through-fastener selection";
        var motorSize=sizes[0],motorFlat=FlatWasher.metric(motorSize),motorNut=HexNut.metric(motorSize);
        var motorLength=5*Math.ceil((RadialPinionCarrier.THICKNESS+flange.flangeThickness+2*motorFlat.thickness+motorNut.height+2)/5);
        for(i in 0...flange.holes.length) {
            var h=flange.holes[i],x=ARM+h.x,y=h.y;
            put('MotorWasher$i',motorFlat,F.translation(x,y,15),carrierId);
            put('MotorBolt$i',SocketHeadCapScrew.metric(motorSize,motorLength,Steel88),F.translation(x,y,15+motorFlat.thickness),carrierId);
            put('MotorRearWasher$i',motorFlat,AxisBuilder.orient(x,y,-flange.flangeThickness,[1.0,0,0],[0.0,0,-1]),carrierId);
            put('MotorNut$i',motorNut,AxisBuilder.orient(x,y,-flange.flangeThickness-motorFlat.thickness,[1.0,0,0],[0.0,0,-1]),carrierId);
        }
        // Transfer-drilled host holes at the declared mounting face. Preserve
        // existing drilling and material, including the aluminium X carriage.
        var host:GantryPlate=cast builder.component(hostId),drilling:Array<InsertHole>=[];
        if(Std.isOfType(host,MountDrilledPlate))drilling=cast(host,MountDrilledPlate).holes();
        var washer=FlatWasher.metric("M8");
        for(bore in anchor.bores)if(StringTools.startsWith(bore.id,"host-")) {
            var direction=F.transformVector(bore.frame,0,0,1);
            var hostEntry=F.compose(bore.frame,F.translation(0,0,bore.depth));
            var worldEntry=at(hostEntry),local=F.compose(F.inverse(builder.zeroPose(hostId)),worldEntry);
            var record=MountHostThreads.blindM8(host.materialSpec(),host.height,local,id+"-"+bore.id);drilling.push(record);
            var screw=SocketHeadCapScrew.metric("M8",topMount?25:35,Steel88);
            var grip=bore.depth+washer.thickness,engagement=screw.length-grip;
            var threadEnd:Float=cast record.threadDepth,rules=materia.casting.InsertHole.InsertHoleCodec.threadLimits(record);
            if(engagement<MountHostThreads.minimumEngagement(host.materialSpec()) || threadEnd-engagement<rules.minTipGap || record.depth-engagement<2.5)
                throw "Anchor screw does not fit its blind host thread";
            // 15% upper tightening scatter plus a conservative 6 kN external
            // increment. Model reactions and installation evidence must replace it.
            if(MountHostThreads.strippingCapacityN(host.materialSpec(),engagement)<9000*1.15+6000)
                throw "Anchor host thread fails stripping screen";
            // Head +Z points away from the contact; shank points into host.
            var outward=AxisBuilder.orient(bore.frame.x,bore.frame.y,bore.frame.z,[direction.y==0?0.0:1.0,direction.y==0?1.0:0.0,0],[-direction.x,-direction.y,-direction.z]);
            put(bore.id+"Washer",washer,outward,anchorId);
            put(bore.id+"Bolt",screw,F.compose(outward,F.translation(0,0,washer.thickness)),anchorId);

        }
        builder.replaceComponent(hostId,new MountDrilledPlate(new ComponentValues().setNumber("width",host.width)
            .setNumber("depth",host.depth).setNumber("height",host.height).setToken("hostMaterial",host.materialSpec())
            .setToken("holes",InsertHoles.fromNeutral(drilling))));
    }
}
