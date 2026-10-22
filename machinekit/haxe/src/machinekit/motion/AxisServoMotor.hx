package machinekit.motion;

import cadkit.modeling.Part;
import machinekit.component.*;
import machinekit.motion.ServoMotor.ServoMounting;
import machinekit.component.ComponentRecipeSupport as P;

/** Servo housing with its declared mounting flange and pilot. Existing generic
 * servo envelopes stay unchanged; this part is selected by the linear-axis builder.
 * The rotating steel shaft is a separate part on the driven joint. */
class AxisServoMotor extends ServoMotor {
    final source:ServoMotor;
    static var recipe:Null<ComponentType>;
    public function new(designation:String) {
        var selected=ServoMotor.model(designation);
        if(selected.mounting()==null)throw 'Axis servo needs a declared flange: '+designation;
        super(selected.rating,selected.ratingsBasis);source=selected;
    }
    override public function mounting():Null<ServoMounting> return source.mounting();
    override public function geometry(detail:ComponentDetail=Preview):Part return withInterface(source.geometry(detail));

    /** Mass of the base motor's body with this flange and pilot, as before the body was drawn in detail. */
    override public function massGeometry():Null<Part> return withInterface(source.massGeometry());

    function withInterface(base:Part):Part {
        var face=source.mounting();
        if(face==null){base.close();throw 'Axis servo lost its mounting interface';}
        return Solids.building([],parts->{
            parts.push(base);
            var blank=Part.box(face.face,face.face,face.flangeThickness);
            var flange=blank.translated(new cadkit.modeling.Vector(0,0,-face.flangeThickness));blank.close();
            var holes=[for(hole in face.holes)Solids.named(Part.cylinderSpan(face.holeDiameter/2,-face.flangeThickness-1,1,hole.x,hole.y),"mount-bolt")];
            parts.push(Solids.cut(Solids.named(flange,"mounting-flange"),holes));
            var pilot=Solids.named(Part.cylinder(face.pilotDiameter/2,face.pilotHeight),"pilot");
            parts.push(Solids.cut(pilot,[Solids.named(Part.cylinderSpan((rating.shaftDiameter+0.2)/2,-1,face.pilotHeight+1),"shaft-clearance")]));
            return Solids.union(parts);
        });
    }
    public static function recipeType():ComponentType {
        if(recipe==null)recipe=new ComponentType("machinekit.motion.axis-servo-motor",[
            P.choice("designation",[for(row in ServoMotor.ratings())row.designation],"GENERIC-HV-SERVO-750W")
        ],v->new AxisServoMotor(v.token("designation")));
        return recipe;
    }
    override public function componentType():Null<ComponentType> return recipeType();
    override public function values():ComponentValues return new ComponentValues().setToken("designation",source.rating.designation).setToken("material",materialSpec());
}
