package machinekit.assembly;

import machinekit.motion.*;
import machinekit.component.MachineComponent;

/** Selected engineering parts, independent of supplier offers.
 * This template supports the drawing-checked RBK12/SFU16 end stack; it does not
 * treat all products called BF12 or NEMA23 as interchangeable.
 */
class ScrewAxisComponents {
    public final mount:RbkMotorMount;
    public final motor:NemaStepper;
    public final screw:LeadScrew;
    public final nut:LeadScrewNut;
    public final support:VallderBf12Support;
    public final rail:LinearRail;
    public final block:LinearRailBlock;
    public final blockThreadDepth:Float;
    public function new(mount:RbkMotorMount,motor:NemaStepper,screw:LeadScrew,nut:LeadScrewNut,
        support:VallderBf12Support,rail:LinearRail,block:LinearRailBlock,blockThreadDepth:Float) {
        this.mount=mount;this.motor=motor;this.screw=screw;this.nut=nut;this.support=support;
        this.rail=rail;this.block=block;this.blockThreadDepth=blockThreadDepth;
    }
    static function same(a:Float,b:Float,label:String):Void {
        if(!Math.isFinite(a) || Math.abs(a-b)>1e-6)throw "Incompatible axis "+label;
    }
    public function check(layout:ScrewAxisLayout):Void {
        if(mount.model!="RBK12-57")throw "Axis requires the checked RBK12-57 interface";
        same(mount.shaftHeight,25,"mount shaft height");same(mount.pilotDiameter,motor.spec.pilotDiameter,"motor pilot");
        var points=motor.boltPattern();
        var diameter=motor.variant.mountHoleDiameter;
        var holeDiameter:Float=diameter==null ? machinekit.standard.SocketHeadCapScrew.catalog().get(motor.variant.mountScrew).clearanceMedium : diameter;
        var radialClearance=(holeDiameter-4)/2;
        for(i in 0...points.length) {
            var seat=mount.connector("motorBolt"+(i+1)).frame;
            // Compare patterns as a set: catalog and CAD order need not match.
            var found=false;
            for(p in points)if(Math.sqrt(Math.pow(p.x-seat.x,2)+Math.pow(p.y-(seat.y-25),2))<=radialClearance)found=true;
            if(!found)throw "Incompatible axis motor mounting pattern";
        }
        if(points.length!=4 || motor.variant.tappedMount)throw "Axis motor requires four flange clearance holes";
        same(motor.variant.shaftDiameter,6.35,"motor shaft/coupling bore");
        if(motor.variant.shaftLength<20 || motor.variant.shaftLength>27)throw "Axis motor shaft does not engage the coupling safely";
        if(motor.variant.pilotHeight>1.6)throw "Axis motor pilot exceeds the checked recess";
        // An M4x12 with a 0.8 mm washer must leave 6..12 mm in the bracket.
        var engagement=12-motor.variant.mountHoleDepth-0.8;
        if(engagement<6 || engagement>12)throw "Axis motor flange gives insufficient screw engagement";
        same(screw.totalLength,layout.screwLength,"screw length");
        if(screw.thread.designation!=nut.thread.designation)throw "Incompatible axis screw/nut thread";
        same(screw.thread.screwDiameter,16,"screw diameter");
        same(screw.connector("raceStart").frame.z,54,"fixed screw end stack");
        same(screw.connector("fixedJournalStart").frame.z,29,"fixed journal start");
        same(screw.connector("floatingSeat").frame.z,layout.screwLength-11,"floating journal start");
        same(nut.bodyDiameter,28,"nut barrel");same(nut.bodyLength,46,"nut length");
        same(nut.flangeThickness,10,"nut flange thickness");same(nut.flangeOffset,10,"nut flange position");
        same(nut.flangeDiameter,48,"nut flange diameter");
        var holes=nut.boltPattern(),angles=[45,90,135,225,270,315];
        if(holes.length!=6 || nut.mountScrew!="M5")throw "Incompatible axis nut mounting pattern";
        for(a in angles) {
            var found=false;
            for(p in holes)if(Math.abs(p.x-19*Math.cos(a*Math.PI/180))<1e-6 && Math.abs(p.y-19*Math.sin(a*Math.PI/180))<1e-6)found=true;
            if(!found)throw "Incompatible axis nut mounting pattern";
        }
        var r=rail.spec,b=block.spec;
        same(rail.length,layout.railLength,"rail length");
        same(r.railWidth,15,"rail width");same(r.railHeight,15,"rail height");
        same(r.railHolePitch,60,"rail hole pitch");same(r.railEndMargin,20,"rail end margin");
        same(b.railWidth,r.railWidth,"block/rail width");same(b.railHeight,r.railHeight,"block/rail height");
        same(b.blockHeight,28,"block assembly height");same(b.blockHolePitchB,26,"block longitudinal pattern");
        same(b.blockHolePitchC,26,"block transverse pattern");
        if(b.blockMountScrew.split("x")[0]!="M4" || b.blockLength>80 || b.blockWidth>34)throw "Incompatible axis block mounting envelope";
        if(!Math.isFinite(blockThreadDepth) || blockThreadDepth<5)throw "Axis block needs at least 5 mm known thread depth";
        for(component in ([mount,motor,screw,nut,support,rail,block]:Array<MachineComponent>))
            if(component.componentType()==null)throw "Axis components need saveable engineering recipes";
    }
}
