import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.structural.TSlotExtrusion;

/** Shared layout checks run independently of downloaded supplier geometry. */
class TSlotFamilyChecks {
    static function check(value:Bool,message:String):Void {if(!value) throw message;}
    public static function run():Void {
        var cases=[{system:"TSLOT-30-SLOT8",width:30.0,height:30.0,slots:4,bores:1},
            {system:"TSLOT-40-SLOT8",width:40.0,height:40.0,slots:4,bores:1},
            {system:"TSLOT-40-SLOT8",width:40.0,height:80.0,slots:6,bores:2},
            {system:"TSLOT-40-SLOT8",width:80.0,height:80.0,slots:8,bores:4}];
        for(c in cases) {
            var profile=TSlotExtrusion.forSystem(c.system,c.width,c.height),count=0;
            for(face in 0...4) count+=profile.slotPositions(face).length;
            check(count==c.slots && profile.boreCentres().length==c.bores,"Shared system slot/bore layout");
            var shape=profile.geometry(50),bounds=shape.shape.bounds();
            check(shape.valid() && shape.solidCount()==1,"Generic multi-slot profile remains one valid solid");
            check(Math.abs(bounds.get_max().get_x()-c.width/2)<0.001 &&
                  Math.abs(bounds.get_max().get_y()-c.height/2)<0.001,"Generic profile dimensions");
            if(c.height==80 && c.width==40) {
                var probe=Part.box(0.2,0.2,1),slotProbe=probe.translated(new Vector(19.9,20,25)),
                    webProbe=probe.translated(new Vector(19.9,0,25));
                var gap=shape.intersect(slotProbe),web=shape.intersect(webProbe);
                check(gap.volume()<0.000001 && web.volume()>0.001,"Long face has two slots and an intervening web");
                gap.close();web.close();slotProbe.close();webProbe.close();probe.close();
            }
            shape.close();
            var offsets=profile.slotPositions(0);offsets[0]=999;
            check(profile.slotPositions(0)[0]!=999,"Slot positions are defensive copies");
        }
        var rejected=false;
        try TSlotExtrusion.forSystem("TSLOT-40-SLOT8",40,60) catch(_:Dynamic) rejected=true;
        check(rejected,"Reject dimensions outside the modular grid");
        // Existing table hardware retains the legacy geometry and catalog interface.
        var legacy=TSlotExtrusion.forProfile("HFS5-2020"),old=legacy.geometry(25);
        check(old.valid() && old.solidCount()==1 && legacy.designation=="HFS5-2020" &&
              legacy.slotWidth==6 && legacy.slotPositions(0).length==1,"Legacy catalog profile remains supported");
        check(Math.abs(legacy.modeledSlotDepth()-3.5)<0.000001,"Legacy hardware pocket depth remains unchanged");
        old.close();
        for(name in ["HFS5-2040","HFS5-2060","HFS5-4040"]) {
            var profile=TSlotExtrusion.forProfile(name),part=profile.geometry(25);
            check(part.valid() && part.solidCount()==1 && profile.designation==name,"Legacy rectangular/profile catalog geometry");
            part.close();
        }
    }
}
