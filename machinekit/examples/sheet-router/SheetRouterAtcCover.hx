import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;

/** Three-millimetre steel pan with upward 30 mm edge returns. Upward returns
 * retain the flat underside's clearance while stiffening the wider ATC lid.
 * The fabricator supplies bend radii/development for the selected tooling.
 */
class SheetRouterAtcCover extends MachineComponent {
    public final width:Float;
    public final depth:Float=300;
    public final web:Float=3;
    public final returnHeight:Float=30;
    public function new(width:Float) {
        super('SHEET-ROUTER-COVER-$width',"Stiffened ATC rack cover","steel",true);this.width=width;
    }
    override public function hasGeometry():Bool return true;
    override public function geometry(detail:ComponentDetail=Preview):Part return Solids.building([],parts->{
        parts.push(Part.box(width,depth,web));
        for(sign in [-1,1]) {
            var blank=Part.box(width,web,returnHeight),placed=blank.translated(new Vector(0,sign*(depth-web)/2,web));blank.close();parts.push(placed);
        }
        return Solids.union(parts);
    });
}
