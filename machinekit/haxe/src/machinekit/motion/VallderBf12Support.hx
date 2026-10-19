package machinekit.motion;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.assembly.RbkMotorMount;
import materia.assembly.AssemblyFrames as F;

/** SKU1013002 pinned CAD, including its 6000 bearing. Y=0 base, Y=25 shaft;
 * bearing front Z=0 and back Z=8. Housing Z=-6..14. No 12 mm bore assumption.
 */
class VallderBf12Support extends MachineComponent {
    public static inline final ASSET:String="76ccf59477cad0fa0a33462842c34c0b3ff0eed99c2c60e70666bcad40a23c86";
    static var recipe:Null<ComponentType>;
    public function new() {
        super("VALLDER-BF12-CAD-v1","Vallder BF12 support with 10 mm bore and included bearing","unspecified");
        lockManufacturingMaterial("unspecified");
        addConnector("base",Mount,F.translation(0,0,4));
        addConnector("bearingFront",Axis,Solids.axial(0,25,0));
        addConnector("bearingBack",Axis,Solids.axial(0,25,8));
        for(i in 0...2) addConnector('baseBolt${i+1}',Face,F.alongY(i==0 ? -23 : 23,31,4,0,1,0));
    }
    override public function hasGeometry():Bool return true;
    override public function geometry(detail:ComponentDetail=Preview):Part {
        var imported=new SupplierCadComponent(RbkMotorMount.assetDirectory()+"/cad/assets/"+ASSET+".step",ASSET,designation,description);
        try {var raw=imported.geometry();try {var posed=raw.translated(new Vector(0,25,4));raw.close();imported.close();return posed;}
            catch(e:Dynamic){raw.close();throw e;}}
        catch(e:Dynamic){imported.close();throw e;}
    }
    public static function drawingRecipeType():ComponentType {
        if(recipe==null) recipe=new ComponentType("machinekit.motion.vallder-bf12-support",[],_ -> new VallderBf12Support(),true,false,null,false);
        return recipe;
    }
    override public function componentType():Null<ComponentType> return drawingRecipeType();
    override public function values():ComponentValues return new ComponentValues();
}
