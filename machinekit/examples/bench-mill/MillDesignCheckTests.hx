import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.Diagnostics;
import machinekit.component.Solids;
import machinekit.design.DesignChecks as D;

/** Deliberately bad physical examples, not just assertions of current constants. */
class MillDesignCheckTests {
	public static function run():Int {
		var count=0;
		for(gap in [-0.2,0.0,0.5,2.9,3.1]) {
			var out=new Diagnostics(); D.minimum(out,"edge ligament",gap,3);
			if(out.hasErrors()!=(gap<3)) throw "Minimum ligament must reject close holes as well as breakthroughs";
			count++;
		}
		for(thickness in [10.0,11.5,12.0]) {
			var out=new Diagnostics(); D.blindFastener(out,"M5",16,10,8,8.5,1.5,thickness,5,1,2);
			if(out.hasErrors()!=(thickness<12)) throw "Blind tap check missed drill-point/back-wall stack";
			count++;
		}
		for(length in [14.0,16.0,18.0,22.0]) {
			var out=new Diagnostics(); D.blindFastener(out,"M5",length,10,8,8.5,1.5,12,5,1,2);
			if(out.hasErrors()!=(length!=16)) throw "Blind screw check missed short engagement or bottoming";
			count++;
		}
		for(x in [0.0,5.0,7.0]) {
			var host=Solids.cut(Part.box(20,20,10),[Part.cylinderSpan(2,-1,11,x,0)]);
			var ring=Solids.cut(Part.cylinderSpan(5,0,10,x,0),[Part.cylinderSpan(2,-1,11,x,0)]);
			var out=new Diagnostics(); D.material(out,"3 mm radial wall",ring,host);
			if(out.hasErrors()!=(x>5)) throw "Actual-solid ligament check missed thin outside wall";
			count++; ring.close(); host.close();
		}
		// An internal pocket can violate the ligament even far from outside edges.
		var host=Solids.cut(Part.box(30,30,10),[Part.cylinderSpan(2,-1,11),Part.cylinderSpan(2,-1,11,5,0)]);
		var ring=Solids.cut(Part.cylinderSpan(5,0,10),[Part.cylinderSpan(2,-1,11)]);
		var thin=new Diagnostics(); D.material(thin,"nearby pocket",ring,host);
		if(!thin.hasErrors()) throw "Ligament check missed nearby cavity"; count++; host.close(); ring.close();
		// A seated connector fits, but its approach crosses an obstruction.
		var obstacle=Part.box(10,10,10).translated(new Vector(0,20,0));
		var seated=Part.box(4,4,4), approach=Part.box(4,44,4);
		var clear=new Diagnostics(), blocked=new Diagnostics();
		D.access(clear,"seated",seated,[{id:"wall",part:obstacle}]);
		D.access(blocked,"approach",approach,[{id:"wall",part:obstacle}]);
		if(clear.hasErrors() || !blocked.hasErrors()) throw "Access check must reject obstructed approach";
		count++; obstacle.close(); seated.close(); approach.close();
		var axes=new Diagnostics();
		D.coaxial(axes,"shifted screw",materia.assembly.AssemblyFrames.translation(0,0,0),materia.assembly.AssemblyFrames.translation(1,0,0));
		if(!axes.hasErrors()) throw "Shifted fastener axis accepted"; count++;
		var angles=new Diagnostics();
		D.coaxial(angles,"tilted screw",materia.assembly.AssemblyFrames.translation(0,0,0),materia.assembly.AssemblyFrames.fromRotationMatrix(0,0,0,[0,0,1,0,1,0,-1,0,0]));
		if(!angles.hasErrors()) throw "Tilted fastener axis accepted"; count++;
		return count;
	}
}
