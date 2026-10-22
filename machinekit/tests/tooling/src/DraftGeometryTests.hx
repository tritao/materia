import materia.casting.CastingPatterns as P;
import materia.casting.CastingFeature;
import materia.casting.CastingDraftSections;
import materia.assembly.AssemblyFrames as F;
import cadkit.modeling.Part;
import cadkit.modeling.Model;
import cadkit.modeling.Vector;
import cadkit.modeling.Scope;

class DraftGeometryTests {
	static var assertions=0;
	static function check(value:Bool,message:String):Void {
		assertions++;
		if (!value) throw message;
	}
	public static function main():Void run();
	public static function run():Void {
		var body:Array<materia.casting.CastingPrism> = [{id:"body",profile:[{x:-60.0,y:-50.0},{x:60.0,y:-50.0},
			{x:60.0,y:50.0},{x:-60.0,y:50.0}],height:80,pose:F.identity()}];
		var feature=P.box("drafted-former",ServiceOpening,[P.face("body.z-max")],40,30,61,0,0,20);
		feature.draft.degrees=2;
		var sections=CastingDraftSections.derive(body,feature),offset=60*Math.tan(2*Math.PI/180);
		Scope.run(scope -> {
			var former=scope.own(machinekit.structural.CastingProfileCad.drafted(sections));
			var cavity=scope.own(machinekit.structural.CastingCavityCad.part(materia.casting.CastingCavities.derive(body,[feature])[0]));
			var missing=scope.own(former.subtractAll([cavity])),extra=scope.own(cavity.subtractAll([former]));
			check(missing.volume()+extra.volume()<1e-7,"authoritative source cavity realizes the same drafted section geometry");
			var expression=materia.casting.CastingCavities.region(materia.casting.CastingCavities.derive(body,[feature])[0]);
			var restored=scope.own(machinekit.structural.CastingRegionCad.part(materia.casting.CastingRegion.CastingRegionCodec.snapshot(
				haxe.Json.parse(haxe.Json.stringify(materia.casting.CastingRegion.CastingRegionCodec.record(expression))))));
			var lost=scope.own(cavity.subtractAll([restored])),added=scope.own(restored.subtractAll([cavity]));
			check(lost.volume()+added.volume()<1e-7,"serialized finished-region expression reconstructs the source cavity exactly");
			var volume:materia.tooling.casting.CastingMoldPlan.MoldVolume={id:"drafted-former",material:
				materia.tooling.casting.CastingMoldPlan.MoldPartMaterial.FormerStock,pieces:[],profiles:[],cuts:[],regions:[expression]};
			var encoded=materia.tooling.casting.MoldVolumeCodec.encode(volume);
			var component=machinekit.tooling.ToolingVolume.fromVolume(materia.tooling.casting.MoldVolumeCodec.decode(encoded));
			var rebuilt=scope.own(component.geometry());
			var gap=scope.own(cavity.subtractAll([rebuilt])),excess=scope.own(rebuilt.subtractAll([cavity]));
			check(gap.volume()+excess.volume()<1e-7,"portable tooling volume rebuilds its exact region without the source document");
			var expected=60*(40*30-(40+30)*offset+4*offset*offset/3);
			check(former.valid() && Math.abs(former.volume()-expected)<1e-6,"drafted rectangular former has analytic wall-slope volume");
			var envelope=scope.own(Part.box(120,100,80)),resin=scope.own(envelope.subtractAll([former]));
			var sweep=scope.own(machinekit.tooling.SweptRemoval.sweep(former,new Vector(0,0,100)));
			var overlap=scope.own(new Model(sweep.shape.common(resin.shape)));
			check(overlap.shape.volume()<1e-7,"exact continuous sweep clears resin toward the declared opening");
			var reverse=scope.own(machinekit.tooling.SweptRemoval.sweep(former,new Vector(0,0,-100)));
			var blocked=scope.own(new Model(reverse.shape.common(resin.shape)));
			check(blocked.shape.volume()>1,"reverse withdrawal is blocked by actual casting material");
			return former;
		}).close();
		feature.cornerRadius=4;
		Scope.run(scope -> {
			var rounded=scope.own(machinekit.structural.CastingProfileCad.drafted(CastingDraftSections.derive(body,feature)));
			var squareVolume=60*(1200-70*offset+4*offset*offset/3);
			var expected=squareVolume-(4-Math.PI)*60*(16-4*offset+offset*offset/3);
			check(rounded.valid() && Math.abs(rounded.volume()-expected)<1e-5,"rounded draft loft retains exact changing corner radii");
			var swept=scope.own(machinekit.tooling.SweptRemoval.sweep(rounded,new Vector(0,0,100)));
			var openingArea=1200-(4-Math.PI)*16;
			check(Math.abs(swept.volume()-expected-100*openingArea)<1e-4,
				"rounded draft uses an exact conical sweep with the analytic nested-section volume");
			return rounded;
		}).close();
		Scope.run(scope -> {
			var lower=scope.own(machinekit.structural.CastingProfileCad.wire(materia.casting.CastingProfiles.rectangle(-20,-15,40,30)));
			var upper=scope.own(cadkit.modeling.Curve.polyline([new Vector(-19,-15,60),new Vector(20,-15,60),
				new Vector(20,15,60),new Vector(-20,15,60)],true));
			var twisted=scope.own(Part.loft([lower,upper],true)),nonplanar=0;
			for (index in 0...twisted.shape.subshapeCount(CadKit.ShapeKind.Face)) {
				var face=twisted.shape.subshape(CadKit.ShapeKind.Face,index);
				if (face.surfaceKind()==CadKit.SurfaceKind.BSpline) nonplanar++;
				face.close();
			}
			check(twisted.valid() && nonplanar>0,"twisted bilinear patches cannot be classified as planes from corner normals");
			var failure="";
			try scope.own(machinekit.tooling.SweptRemoval.sweep(twisted,new Vector(0,0,100)))
			catch (error:Dynamic) failure=Std.string(error);
			check(failure.indexOf("Unsupported removal boundary surface")>=0,"unsupported curved sweep still rejects a twisted loft");
			return twisted;
		}).close();
		Scope.run(scope -> {
			var a=materia.casting.CastingRegion.Placed({solid:materia.casting.CastingFeature.CastingGeometrySolid.Cut(
				materia.casting.CastingSolid.Box(20,20,20)),pose:F.identity()});
			var b=materia.casting.CastingRegion.Placed({solid:materia.casting.CastingFeature.CastingGeometrySolid.Cut(
				materia.casting.CastingSolid.Box(20,20,20)),pose:F.translation(0,0,20)});
			var contact=scope.own(machinekit.structural.CastingRegionCad.part(materia.casting.CastingRegion.Intersection([a,b])));
			check(contact.isEmpty() && contact.volume()==0,"face-only region intersection is empty solid material");
			var remainder=scope.own(machinekit.structural.CastingRegionCad.part(materia.casting.CastingRegion.Union([
				materia.casting.CastingRegion.Difference(a,[a]),b])));
			check(Math.abs(remainder.volume()-8000)<1e-7,"empty subtraction operands do not invent material in a region union");
			return remainder;
		}).close();
		Sys.println("DraftGeometryTests: "+assertions+" assertions passed");
	}
}
