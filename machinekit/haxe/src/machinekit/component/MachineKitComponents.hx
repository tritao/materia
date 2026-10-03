package machinekit.component;

import machinekit.standard.Bushing;
import machinekit.standard.DeepGrooveBearing;
import machinekit.standard.FlatWasher;
import machinekit.standard.HexBolt;
import machinekit.standard.HexNut;
import machinekit.standard.ParallelKey;
import machinekit.standard.RetainingRing;
import machinekit.standard.ShaftCollar;
import machinekit.standard.SocketHeadCapScrew;
import machinekit.motion.FlangeBearingHousing;
import machinekit.motion.LeadScrew;
import machinekit.motion.LeadScrewNut;
import machinekit.motion.LinearBearing;
import machinekit.motion.LinearRail;
import machinekit.motion.LinearRailBlock;
import machinekit.motion.NemaStepper;
import machinekit.motion.MotorDriver;
import machinekit.motion.PowerSupply;
import machinekit.motion.PillowBlock;
import machinekit.robotics.RobotFlange;
import machinekit.robotics.EndEffectorPlate;
import machinekit.robotics.Pedestal;
import machinekit.transmission.Sprocket;
import machinekit.transmission.SpurGear;
import machinekit.transmission.Rack;
import machinekit.transmission.TimingBelt;
import machinekit.transmission.TimingPulley;

/** Registered, editable single-part generators. Parts whose inputs include lists, such as
 * SteppedShaft and ShaftCoupling, use text inputs for structured feature lists.
 */
class MachineKitComponents {
	static var types:Null<Array<ComponentType>>;
	static final extensions:Array<ComponentType> = [];

	/** Applications register their component recipes before loading saved assemblies. */
	public static function register(type:ComponentType):Void {
		for (existing in entries()) if (existing.id == type.id) throw 'Duplicate component type "${type.id}"';
		for (existing in extensions) if (existing.id == type.id) throw 'Duplicate component type "${type.id}"';
		extensions.push(type);
	}

	public static function all():Array<ComponentType> return entries().concat(extensions);

	public static function byId(id:String):ComponentType {
		for (type in entries()) if (type.id == id) return type;
		for (type in extensions) if (type.id == id) return type;
		throw 'Unknown machine component type "$id"';
	}

	static function entries():Array<ComponentType> {
		if (types == null) types = build();
		return types;
	}

	static function build():Array<ComponentType> {
		var result:Array<ComponentType> = [
			DeepGrooveBearing.recipeType(),
			SocketHeadCapScrew.recipeType(),
			HexBolt.recipeType(),
			HexNut.recipeType(),
			FlatWasher.recipeType(),
			ParallelKey.recipeType(),
			RetainingRing.recipeType(),
			ShaftCollar.recipeType(),
			Bushing.recipeType(),
			LinearBearing.recipeType(),
			PillowBlock.recipeType(),
			LinearRail.recipeType(),
			LinearRailBlock.recipeType(),
			NemaStepper.namedRecipeType(),
			NemaStepper.genericRecipeType(),
			MotorDriver.recipeType(),
			PowerSupply.recipeType(),
			FlangeBearingHousing.recipeType(),
			LeadScrew.recipeType(),
			LeadScrewNut.recipeType(),
			RobotFlange.recipeType(),
			EndEffectorPlate.recipeType(),
			Pedestal.recipeType(),
			Sprocket.chainRecipeType(),
			Sprocket.genericRecipeType(),
			SpurGear.recipeType(),
			Rack.recipeType(),
			TimingPulley.standardRecipeType(),
			TimingPulley.customRecipeType(),
			TimingBelt.pairRecipeType()
		];
		for (type in MachineKitAdditionalRecipes.all()) result.push(type);
		return result;
	}
}
