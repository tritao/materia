package machinekit.transmission;

import machinekit.component.MachineComponent;

/** The pitch geometry shared by straight and normal-module helical rack drives. */
class RackGeometry {
    public static function pinion(part:MachineComponent):{diameter:Float,moduleSize:Float,teeth:Int,width:Float,helix:Float,hand:Int} {
        if(Std.isOfType(part,SpurGear)) {
            var p:SpurGear=cast part;
            return {diameter:p.pitchDiameter,moduleSize:p.moduleSize,teeth:p.teeth,width:p.faceWidth,helix:0.0,hand:0};
        }
        if(Std.isOfType(part,HelicalPinion)) {
            var p:HelicalPinion=cast part;
            return {diameter:p.pitchDiameter(),moduleSize:p.transverseModule(),teeth:p.parameterCount('teeth'),width:p.n('faceWidth'),helix:p.n('helixAngle'),hand:p.handSign()};
        }
        throw new TransmissionDesignError('Rack drive needs a straight or helical pinion');
    }
    public static function rack(part:MachineComponent):{length:Float,width:Float,teeth:Int,helix:Float,hand:Int} {
        if(Std.isOfType(part,GearRack)) {
            var r:GearRack=cast part;
            return {length:r.spec.lengthMm,width:r.spec.faceWidthMm,teeth:r.spec.teeth,helix:r.helixAngle(),hand:1};
        }
        if(Std.isOfType(part,Rack)) {
            var r:Rack=cast part;return {length:r.length,width:r.faceWidth,teeth:r.teethCount,helix:0.0,hand:0};
        }
        if(Std.isOfType(part,HelicalRack)) {
            var r:HelicalRack=cast part;
            return {length:r.length(),width:r.n('faceWidth'),teeth:r.parameterCount('teeth'),helix:r.n('helixAngle'),hand:r.parameterToken('hand')=='right'?1:-1};
        }
        throw new TransmissionDesignError('Rack drive needs a straight or helical rack');
    }
    public static function relation(pinion:MachineComponent,rack:Null<MachineComponent>,alignment:Float):TransmissionRelation {
        if(Std.isOfType(rack,GearRack))return relation(pinion,cast(rack,GearRack).nominalRack(),alignment);
        if(Std.isOfType(pinion,SpurGear)) {
            if(rack!=null&&!Std.isOfType(rack,Rack))throw new TransmissionDesignError('Straight pinion needs a straight rack');
            return Rack.relation(cast pinion,cast rack,alignment);
        }
        if(!Std.isOfType(pinion,HelicalPinion)||rack==null||!Std.isOfType(rack,HelicalRack))
            throw new TransmissionDesignError('Helical drive needs both matching physical rack and pinion');
        return cast(rack,HelicalRack).relation(cast pinion,alignment);
    }
}
