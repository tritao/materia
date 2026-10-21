import materia.assembly.AssemblyFrames as F;
import materia.casting.InterfaceRequirement;
import materia.casting.InterfaceStock;

/** Synthetic layers check geometry, without choosing a production finishing process. */
class InterfaceStockTests {
	static var assertions = 0;
	static function check(ok:Bool,message:String):Void {
		assertions++;
		if (!ok) throw message;
	}
	static function requirement():InterfaceRequirement return {
		id:"stock-face",geometry:Plane(F.translation(0,0,20)),route:InterfaceRoute.Ground,allowanceMm:1,finishingProcessErrorMm:null,
		permittedFinishes:[InterfaceFinish.Ground],positionToleranceMm:0.01,
		datumReference:"primary",relationshipIds:[]
	};
	public static function run():Void {
		var r=requirement();
		switch InterfaceStocks.assess(Primitive(Box(40,30,21)),r) {
			case Remove(layer):
				check(layer.x == -20 && layer.y == -15 && layer.z == 20
					&& layer.width == 40 && layer.depth == 30 && layer.height == 1,"exact whole-face removal layer");
			default: throw "Expected finishing stock";
		}
		r.allowanceMm=2;
		check(switch InterfaceStocks.assess(Primitive(Box(40,30,21)),r) {
			case Invalid(reason): reason.indexOf("modeled layer")>=0; default:false;
		},"a declared allowance cannot substitute for geometry");
		check(switch InterfaceStocks.assess(Primitive(Box(40,30,22)),r) {
			case Remove(layer):layer.height==2; default:false;
		},"matching geometry clears allowance mismatch");
		r=requirement();r.geometry=Plane(F.identity());r.allowanceMm=21;
		check(switch InterfaceStocks.assess(Primitive(Box(40,30,21)),r) {
			case Invalid(reason):reason.indexOf("retained")>=0; default:false;
		},"removal must retain material");
		r=requirement();r.allowanceMm=0;
		check(switch InterfaceStocks.assess(Primitive(Box(40,30,20)),r) {
			case Invalid(reason):reason.indexOf("positive")>=0; default:false;
		},"zero finishing allowance remains unresolved");
		r=requirement();r.route=InterfaceRoute.AsSupplied;r.allowanceMm=0;r.permittedFinishes=[InterfaceFinish.AsSupplied];
		check(switch InterfaceStocks.assess(Primitive(Box(40,30,20)),r) {case NoLayer:true;default:false;},"as-supplied boundary");
		check(switch InterfaceStocks.assess(Primitive(Box(40,30,21)),r) {case Invalid(_):true;default:false;},"interior plane is not an as-made boundary");
		check(switch InterfaceStocks.assess(Primitive(Cylinder(20,20)),r) {case NoLayer:true;default:false;},"exact cylinder end cap needs no box approximation");
		r=requirement();r.route=InterfaceRoute.Replicated;r.permittedFinishes=[JigMaster];
		switch InterfaceStocks.assess(Primitive(Box(40,30,19)),r) {
			case Replicate(layer):check(layer.z==19 && layer.height==1 && layer.width==40,"replication reserves an addition layer");
			default:throw "Expected replication gap";
		}
		r=requirement();
		check(switch InterfaceStocks.assess(Primitive(Cylinder(20,21)),r) {case Unsupported(_):true;default:false;},"cylindrical finishing requires an exact profile model");
		var raw=materia.casting.InsertSpec.InsertBody.Primitive(Box(40,30,21));
		var ground=requirement();
		var replica=requirement();replica.id="replicated-face";
		replica.geometry=Plane(F.translation(0,0,22));replica.route=InterfaceRoute.Replicated;
		replica.allowanceMm=2;replica.permittedFinishes=[JigMaster];
		switch materia.casting.InsertFinishing.assess(raw,[ground,replica]) {
			case Resolved(steel):
				var box=materia.casting.InsertFinishing.box(steel);
				check(box!=null && box.height==20 && box.z==0,"service steel excludes the finishing layer");
				check(materia.casting.BoxAlgebra.volume(steel.removedSteel)==1200,"removal uses exact disjoint box algebra");
				check(steel.replicatedLayers.length==1 && steel.replicatedLayers[0].region.z==20
					&& steel.replicatedLayers[0].region.height==2,"replication starts at the finished steel, not the raw stock");
			default:throw "Expected resolved steel and a separate replication region";
		}
		var conflict=requirement();conflict.id="conflicting-face";
		conflict.geometry=Plane(F.translation(0,0,19));conflict.allowanceMm=2;
		check(switch materia.casting.InsertFinishing.assess(raw,[ground,conflict]) {
			case Invalid(issues):issues[0].interfaceIds.indexOf("stock-face")>=0;
			default:false;
		},"conflicting finished faces retain their source interface IDs");
		var underside=requirement();underside.id="finished-underside";
		underside.geometry=Plane(F.fromRotationMatrix(0,0,1,[1,0,0,0,-1,0,0,0,-1]));
		switch materia.casting.InsertFinishing.assess(raw,[ground,underside]) {
			case Resolved(steel):
				var box=materia.casting.InsertFinishing.box(steel);
				check(box!=null && box.z==1 && box.height==19,"opposed finishing faces preserve the authored coordinate frame");
				check(materia.casting.BoxAlgebra.volume(steel.removedSteel)==2400,"combined removal has no double-counted stock");
			default:throw "Expected compatible opposite-face removal";
		}
		var material=materia.casting.InsertMaterial.InsertMaterials.specification(materia.casting.InsertMaterial.C45Bright);
		var manufacturing=materia.casting.InsertManufacturing.InsertManufacturingCodec.draft(material);
		manufacturing.holeRoutes=[{holeId:"fixing",route:materia.casting.InsertManufacturing.InsertHoleRoute.BeforeCasting}];
		var holes=TestInsertHoles.through(5,20);
		var spec:materia.casting.InsertSpec={body:raw,holes:holes,
			origin:materia.casting.InsertOrigin.Fabricated(manufacturing),interfaces:[ground]};
		var before=materia.casting.InsertStages.holes(spec,materia.casting.InsertStages.InsertStage.BeforeCasting);
		check(before.length==1 && before[0].frame.z==21 && before[0].depth==21,
			"pre-cast drilling extends through disposable finishing stock");
		check(holes[0].frame.z==20 && holes[0].depth==20,"raw drilling derivation preserves the source service hole");
		var service=materia.casting.InsertStages.holes(spec,materia.casting.InsertStages.InsertStage.Service);
		check(service[0].frame.z==20 && service[0].depth==20,"service drilling remains on the finished steel face");
		Sys.println('InterfaceStockTests: $assertions assertions passed');
	}
}
