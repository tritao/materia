package machinekit.assembly;

import machinekit.motion.ShaftCoupling;
import machinekit.component.*;

/** Existing rigid sleeve generator, 6.35/10 bores, D25 L40. Installed engagement
 * 8.6 mm motor and 11.5 mm screw; 19.9 mm gap. Reference design, no supplier SKU.
 */
class ScrewAxisCoupling extends ShaftCoupling {
    static var recipe:Null<ComponentType>;
    public final motorShaftLength:Float;
    public function new(motorShaftLength:Float=20.6) {
        super(6.35,10,25,40,[{z:6.0,angle:Math.PI/2},{z:34.0,angle:Math.PI/2}]);
        if(!Math.isFinite(motorShaftLength) || motorShaftLength<20 || motorShaftLength>27)throw "Unsupported motor shaft engagement";
        this.motorShaftLength=motorShaftLength;
        addConnector("motorEnd",Axis,Solids.axial(0,0,motorShaftLength-12));
        addConnector("screwEnd",Axis,Solids.axial(0,0,28.5));
    }
    public static function axisRecipeType():ComponentType {
        if(recipe==null) recipe=new ComponentType("machinekit.assembly.screw-axis-coupling",[ComponentRecipeSupport.length("motorShaftLength",20.6)],v -> new ScrewAxisCoupling(v.number("motorShaftLength")),true);
        return recipe;
    }
    override public function componentType():Null<ComponentType> return axisRecipeType();
    override public function values():ComponentValues return new ComponentValues().setNumber("motorShaftLength",motorShaftLength).setToken("material",materialSpec());
}
