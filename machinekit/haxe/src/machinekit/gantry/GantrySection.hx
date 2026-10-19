package machinekit.gantry;

import machinekit.structural.TSlotExtrusion;
import machinekit.structural.RectTube;
import machinekit.structural.RectTubeMember;
import machinekit.component.MachineComponent;
import machinekit.gantry.GantryParts.GantryExtrusion;

/** Cross-section used by layout and by the actual cut member. */
class GantrySection {
    public final size:Float;
    public final height:Float;
    final extrusion:Null<TSlotExtrusion>;
    final tube:Null<RectTube>;
    public function new(?extrusion:TSlotExtrusion,?tube:RectTube) {
        if((extrusion==null)==(tube==null))throw 'Select one structural section';
        this.extrusion=extrusion;this.tube=tube;
        if(extrusion!=null){size=extrusion.size;height=extrusion.height;}
        else {var section:RectTube=cast tube;size=section.width;height=section.height;}
    }
    public function member(length:Float):MachineComponent {
        var profile=extrusion;
        if(profile!=null)return new GantryExtrusion(profile,length);
        var section:RectTube=cast tube;
        return new RectTubeMember(section.width,section.height,section.wall,length);
    }
}
