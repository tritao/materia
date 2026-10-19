package machinekit.motion;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;

/** SKU1035001 table/drawing: L1=46, B=10, L2=10, D1=28, D2=27.8,
 * D3=48, D4=38, six D5=5.5 holes, H1 flats=40. The calculator's 42 is not used.
 * Preload, accuracy and lubrication-port drilling are not inferred.
 */
class VallderSfu1605Nut extends LeadScrewNut {
    static var recipe:Null<ComponentType>;
    public function new() {
        super(BallScrew.race(),6,true,BallNut,{bodyDiameter:28,bodyLength:46,flangeDiameter:48,flangeThickness:10,
            flangeOffset:10,boltCircleDiameter:38,mountScrew:"M5"});
        setMaterial("steel");
    }
    override public function boltPattern():Array<{x:Float,y:Float}> return [for(degrees in [45,90,135,225,270,315]) {
        x:19*Math.cos(degrees*Math.PI/180),y:19*Math.sin(degrees*Math.PI/180)}];
    override public function geometry(detail:ComponentDetail=Preview):Part {
        var body=super.geometry(detail);
        return Solids.building([body],owned -> {
            var tail=Solids.cut(Part.cylinderSpan(14.2,20,46.1),[Part.cylinderSpan(13.9,19.9,46.2)]);owned.push(tail);
            var tools=[tail];
            for(side in [-1,1]) {
                var raw=Part.box(10,60,10.2),flat=raw.translated(new Vector(side*25,0,9.9));raw.close();owned.push(flat);tools.push(flat);
            }
            return Solids.cut(body,tools);
        });
    }
    public static function drawingRecipeType():ComponentType {
        if(recipe==null) recipe=new ComponentType("machinekit.motion.vallder-sfu1605-nut",[],_ -> new VallderSfu1605Nut());
        return recipe;
    }
    override public function componentType():Null<ComponentType> return drawingRecipeType();
    override public function values():ComponentValues return new ComponentValues().setToken("material",materialSpec());
}
