import machinekit.motion.ShaftCoupling;
import machinekit.component.*;

/** Existing rigid sleeve generator, 6.35/10 bores, D25 L40. Installed engagement
 * 8.6 mm motor and 11.5 mm screw; 19.9 mm gap. Reference design, no supplier SKU.
 */
class RbkAxisCoupling extends ShaftCoupling {
    static var recipe:Null<ComponentType>;
    public function new() {
        super(6.35,10,25,40,[{z:6.0,angle:Math.PI/2},{z:34.0,angle:Math.PI/2}]);
        addConnector("motorEnd",Axis,Solids.axial(0,0,8.6));
        addConnector("screwEnd",Axis,Solids.axial(0,0,28.5));
    }
    public static function axisRecipeType():ComponentType {
        if(recipe==null) recipe=new ComponentType("example.rbk-axis.coupling",[],_ -> new RbkAxisCoupling(),true);
        return recipe;
    }
    override public function componentType():Null<ComponentType> return axisRecipeType();
    override public function values():ComponentValues return new ComponentValues().setToken("material",materialSpec());
}
