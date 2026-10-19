import machinekit.assembly.*;
import machinekit.component.*;
import machinekit.motion.*;

/** Original worked Vallder axis, retaining its saved fabrication and coupling identities. */
class RbkSfuAxis extends TwinRailScrewAxis {
    public static function registry():ComponentRegistry {
        var r=MachineKitComponents.standard();r.register(RbkAxisPart.recipeType());r.register(RbkAxisCoupling.axisRecipeType());return r;
    }
    public static function componentsFor(layout:ScrewAxisLayout,motorModel:String="OPENBUILDS-NEMA23-56",motorWasher:Bool=true):ScrewAxisComponents {
        return new ScrewAxisComponents(new RbkMotorMount("RBK12-57"),NemaStepper.model(motorModel),
            new VallderSfu1605Screw(layout.screwLength),new VallderSfu1605Nut(),new VallderBf12Support(),
            LinearRail.metric("HGR15",layout.railLength),new HiwinHgh15ca(),HiwinHgh15ca.MOUNT_THREAD_DEPTH,motorWasher);
    }
    public function new() {
        var layout=new ScrewAxisLayout();
        super(layout,componentsFor(layout),kind -> new RbkAxisPart(kind),new RbkAxisCoupling());
    }
}
