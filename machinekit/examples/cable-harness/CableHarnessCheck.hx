import machinekit.electrical.CableHarness;
import cadkit.modeling.Part;

class CableHarnessCheck {
    static function require(ok:Bool, message:String):Void {if (!ok) throw message;}
    static function main() {
        var root = Sys.getEnv("MATERIA_ROOT");
        if (root == null) throw "Set MATERIA_ROOT";
        var directory = root + "/build/suppliers/lcsccable/example/";
        var text = sys.io.File.getContent(directory + "geometry.json");
        var harness = new CableHarness(text), part = harness.geometry();
        try {
            require(part.valid(),"Invalid cable solids");
            require(part.solidCount()==4,"Two wires and two housing envelopes expected");
            var expected = 2*Math.PI*0.7*0.7*200+2*7.3*7.75*5.7;
            require(Math.abs(part.volume()-expected)<1e-5,"Cable volume mismatch");
            part.exportStep(directory+"harness.step");
            var roundtrip = new Part(cadkit.Shape.importStep(directory+"harness.step"));
            try {
                require(roundtrip.valid() && roundtrip.solidCount()==4,"STEP roundtrip failed");
                require(Math.abs(roundtrip.volume()-expected)<1e-5,"STEP volume changed");
            } catch(error:Dynamic) {roundtrip.close();throw error;}
            roundtrip.close();
            // Moving both connector and route endpoints must change geometry without changing volume.
            var moved:Dynamic=haxe.Json.parse(text);
            var wires:Array<Dynamic>=moved.wires, ends:Array<Dynamic>=moved.ends;
            for(wire in wires) {var route:Array<Array<Float>>=wire.route;for(p in route) p[2]+=20;}
            for(end in ends) {var p:Array<Float>=end.origin;p[2]+=20;}
            var shifted=new CableHarness(haxe.Json.stringify(moved)).geometry();
            try {
                require(shifted.valid() && Math.abs(shifted.volume()-expected)<1e-5,"Placement changed cable volume");
                var before=part.shape.bounds(),after=shifted.shape.bounds();
                require(Math.abs(after.get_min().get_z()-before.get_min().get_z()-20)<1e-6,"Placement did not move bounds");
                require(Math.abs(after.get_max().get_z()-before.get_max().get_z()-20)<1e-6,"Placement did not move upper bounds");
            }
            catch(error:Dynamic) {shifted.close();throw error;}
            shifted.close();
            var invalid:Dynamic=haxe.Json.parse(text);
            var invalidWires:Array<Dynamic>=invalid.wires;invalidWires[0].diameter_mm=-1;
            var rejected=false;
            try {new CableHarness(haxe.Json.stringify(invalid));} catch(error:Dynamic) {rejected=true;}
            require(rejected,"Invalid wire diameter accepted");
            sys.io.File.saveContent(directory+"cad-checks.json",haxe.Json.stringify({valid:true,solids:4,
                volume_mm3:part.volume(),step_roundtrip:true,placement_check:true,
                geometry_status:"nominal_envelopes",manufacturing_qualified:false},null,"  ")+"\n");
            Sys.println("Cable reference CAD: 4 valid solids; STEP roundtrip and placement checks passed");
        } catch(error:Dynamic) {part.close();throw error;}
        part.close();
    }
}
