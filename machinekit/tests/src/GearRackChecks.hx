import machinekit.transmission.GearRack;
import machinekit.transmission.HelicalPinion;
import machinekit.component.ComponentValues;
import machinekit.transmission.ValueBasis;
import machinekit.transmission.GearRackSpec.GearRackCatalog;
import machinekit.transmission.RackGeometry;
import machinekit.component.MachineKitComponents;
import machinekit.assembly.PosedParts;
import materia.assembly.AssemblyFrames as F;

class GearRackChecks {
    static function check(ok:Bool,label:String):Void {if(!ok)throw 'Gear rack: $label';}
    static function near(a:Float,b:Float,label:String):Void check(Math.abs(a-b)<1e-8,label);
    public static function main():Void {
        var count=0;
        for(code in GearRackCatalog.catalog().designations()) {
            var rack=GearRack.catalogue(code),s=rack.spec;
            near(rack.massProperties().mass,s.massKg,"datasheet mass");
            check(rack.mountingConnectors().length==s.holeCount,"supplier hole count");
            for(i in 0...s.holeCount) {
                var c=rack.connector('mount${i+1}').frame,n=F.transformVector(c,0,1,0);
                near(c.z,62.5+125*i,"supplier pitch and end margin");near(c.x,s.faceWidthMm,"support face");near(n.x,-1,"mount normal into rack");
            }
            near(rack.connector("pinPilot1").frame.z,31.7,"supplier first pin pilot");
            near(rack.connector("pinPilot2").frame.z,s.lengthMm-31.7,"supplier last pin pilot");
            near(RackGeometry.rack(rack).length,s.lengthMm,"mesh length");
            near(rack.connector("back").frame.y,-s.pitchHeightMm,"back datum");
            check(rack.mountingThread().majorDiameter==6,"catalogue M6 fastener interface");
            var recipe=GearRack.recipeType();
            var reload=MachineKitComponents.defaultRegistry().byId(recipe.id).create(rack.values());
            near(reload.massProperties().mass,s.massKg,"saved recipe mass");
            var geometry=rack.geometry();check(geometry.valid(),"drilled catalogue section valid");
            var moved=PosedParts.moved(geometry,F.translation(4,5,6));check(moved.valid(),"transformed catalogue section valid");moved.close();geometry.close();count++;
        }
        near(GearRack.catalogue("29-15-105").publishedFeedForceN(20),6000,"C-44 HPR rating");
        near(GearRack.catalogue("29-15-055").publishedFeedForceN(20),3000,"500 mm mounting derating");
        near(GearRack.catalogue("29-20-105").publishedFeedForceN(15),8000,"C-45 HPR standard steel column, not case-hardened column");
        var physical=GearRack.catalogue("29-15-105");
        var pinion=new HelicalPinion(new ComponentValues().setToken("hand","left"));
        var relation=RackGeometry.relation(pinion,physical,1);
        near(relation.ratio,2*Math.PI/100,"100 mm/turn for reviewed module-1.5 geometry");
        check(relation.basis("stiffness")==ValueBasis.Assumed,"published capacity is not a mesh stiffness");
        var wrongHand=false;
        try RackGeometry.relation(new HelicalPinion(),physical,1) catch(_:Dynamic)wrongHand=true;
        check(wrongHand,"matching hands refused");
        var refused=false;try GearRack.catalogue("29-15-105").publishedFeedForceN(19) catch(_:Dynamic)refused=true;
        check(refused,"unreviewed rating row refused");
        Sys.println('PASS: $count catalogue rack recipes, connectors, declared mass, ratings, geometry and reload');
    }
}
