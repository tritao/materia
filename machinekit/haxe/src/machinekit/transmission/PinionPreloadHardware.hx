package machinekit.transmission;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.threads.ThreadedFace.ThreadStart;
import machinekit.component.*;

/** Machined spring adjuster, M10x1.5 stem. Threads have a nominal envelope;
 * the unverified disc-stack ratings are carried separately by the carrier spec. */
class PinionPreloadAdjuster extends MachineComponent {
    public static inline final WRENCH_AF=20.0;
    public static inline final HEAD_HEIGHT=4.0;
    public function new() {super("PINION-SPRING-ADJUSTER-M10X1.5","M10 pinion spring seat adjuster","steel C45");}
    override public function hasGeometry():Bool return true;
    override public function geometry(detail:ComponentDetail=Preview):Part {
        var stem=Solids.named(Part.cylinderSpan(5,-30,0),"stem");
        var threaded=new Part(cadkit.threads.Threads.attach(stem.shape,"stem:cyl.side",machinekit.standard.FastenerThreads.coarse(10),AxisMaximum,30));stem.close();
        // Real 20 mm wrench flats; a plain round seat cannot set preload.
        return Solids.union([threaded,Part.prism(cadkit.modeling.Polygon.regular(6,WRENCH_AF),0,HEAD_HEIGHT),Part.cylinderSpan(4,4,17)]);
    }
}

/** Assumed disc-spring pack envelope, OD22/ID10, loaded height12 mm.
 * Supplier curve, fatigue and solid-height confirmation remain open sourcing inputs. */
class PinionDiscSpringPack extends MachineComponent {
    public final loadedHeight:Float;
    public function new(loadedHeight:Float=12) {
        if(!Math.isFinite(loadedHeight)||loadedHeight<11.5||loadedHeight>12.1375)throw "Disc pack outside assumed working range";
        super("PINION-DISC-PACK-ASSUMED-"+loadedHeight,"Assumed preloaded disc-spring pack","spring steel");this.loadedHeight=loadedHeight;
    }
    override public function hasGeometry():Bool return true;
    override public function geometry(detail:ComponentDetail=Preview):Part return Solids.cut(Part.cylinder(11,loadedHeight),[
        Part.cylinderSpan(5,-0.1,loadedHeight+0.1)]);
}

/** Upper spring cup fixed to the carrier's rear lug. The centre clears the
 * adjuster's guide pilot, so it does not bypass the spring's axial compliance. */
class PinionSpringCup extends MachineComponent {
    public function new() {super("PINION-SPRING-CUP-24X3","Machined pinion spring cup","steel C45");}
    override public function hasGeometry():Bool return true;
    override public function geometry(detail:ComponentDetail=Preview):Part return Solids.cut(Part.cylinder(12,3),[
        Part.cylinderSpan(4.25,-0.1,3.1)]);
}
