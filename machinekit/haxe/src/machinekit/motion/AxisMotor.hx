package machinekit.motion;

import cadkit.modeling.Part;
import machinekit.component.MachineComponent;

/** Mechanical interface of an axis motor. The owning part supplies its dimensions
 * and mounting tool; builders never infer a servo interface from its body shape. */
class AxisMotor {
    public final part:MachineComponent;
    public final face:Float;
    public final shaftDiameter:Float;
    public final shaftLength:Float;
    final cutout:Float->Part;
    public function new(part:MachineComponent,face:Float,shaftDiameter:Float,shaftLength:Float,cutout:Float->Part) {
        if(part.motorDrive()==null)throw 'Axis motor '+part.designation+' has no drive ratings';
        for(value in [face,shaftDiameter,shaftLength])if(!Math.isFinite(value)||value<=0)throw 'Invalid axis motor interface';
        this.part=part;this.face=face;this.shaftDiameter=shaftDiameter;this.shaftLength=shaftLength;this.cutout=cutout;
    }
    public function mountingCutout(depth:Float):Part return cutout(depth);
    public static function of(part:MachineComponent):AxisMotor {
        var result=part.axisMotor();
        if(result==null)throw 'Component '+part.designation+' does not declare an axis motor interface';
        return result;
    }
}
