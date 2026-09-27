package machinekit.component;

import machinekit.component.ComponentParameterType.*;
import machinekit.component.ComponentValue.*;
import machinekit.catalog.CatalogIndex;

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
import machinekit.standard.BearingFit.BearingHousingFit;
import machinekit.motion.LeadScrew;
import machinekit.motion.LeadScrewNut;
import machinekit.motion.LeadScrewThread;
import machinekit.motion.LeadScrewThread.LeadScrewThreadFamily;
import machinekit.motion.LeadScrewThread.LeadScrewHand;
import machinekit.motion.LinearBearing;
import machinekit.motion.LinearRail;
import machinekit.motion.LinearRailBlock;
import machinekit.motion.LinearRailSystem;
import machinekit.motion.NemaStepper;
import machinekit.motion.PillowBlock;
import machinekit.robotics.RobotFlange;
import machinekit.robotics.EndEffectorPlate;
import machinekit.robotics.Pedestal;
import machinekit.transmission.Sprocket;
import machinekit.transmission.TimingPulley;
import machinekit.transmission.TimingBeltProfile;
import materia.project.MaterialLibrary;

/** Registered, editable single-part generators. Parts whose inputs include lists, such as
 * SteppedShaft and ShaftCoupling, remain code-only until a list input type exists.
 */
class MachineKitComponents {
	static var types:Null<Array<ComponentType>>;

	public static function all():Array<ComponentType> return entries().copy();

	public static function byId(id:String):ComponentType {
		for (type in entries()) if (type.id == id) return type;
		throw 'Unknown machine component type "$id"';
	}

	public static function forComponent(component:MachineComponent):Null<ComponentType> {
		for (type in entries()) if (type.matches(component)) return type;
		return null;
	}

	static function entries():Array<ComponentType> {
		if (types == null) types = build();
		return types;
	}

	static function build():Array<ComponentType> {
		var result:Array<ComponentType> = [];
		result.push(new ComponentType("machinekit.standard.deep-groove-bearing",
			[catalog("designation", DeepGrooveBearing.catalog(), "608"), flag("shielded", true)],
			v -> DeepGrooveBearing.metric(v.token("designation"), v.boolean("shielded")),
			c -> new ComponentValues().set("designation", Token(cast(c, DeepGrooveBearing).spec.designation))
				.set("shielded", Boolean(cast(c, DeepGrooveBearing).shielded)),
			c -> Std.isOfType(c, DeepGrooveBearing), true));
		result.push(new ComponentType("machinekit.standard.socket-head-cap-screw",
			[catalog("size", SocketHeadCapScrew.catalog(), "M5"), length("length", 20),
				choice("material", ["steel 12.9", "steel", "stainless steel"], "steel 12.9")],
			v -> SocketHeadCapScrew.metric(v.token("size"), v.number("length"), v.token("material")),
			c -> new ComponentValues().set("size", Token(cast(c, SocketHeadCapScrew).spec.size))
				.set("length", Number(cast(c, SocketHeadCapScrew).length))
				.set("material", Token(MaterialLibrary.require(c.materialId).physical.spec)),
			c -> Std.isOfType(c, SocketHeadCapScrew), true));
		result.push(new ComponentType("machinekit.standard.hex-bolt",
			[catalog("size", HexBolt.catalog(), "M5"), length("length", 20)],
			v -> HexBolt.metric(v.token("size"), v.number("length")),
			c -> new ComponentValues().set("size", Token(cast(c, HexBolt).spec.size))
				.set("length", Number(cast(c, HexBolt).length)),
			c -> Std.isOfType(c, HexBolt), true));
		result.push(new ComponentType("machinekit.standard.hex-nut",
			[catalog("size", HexNut.catalog(), "M5")],
			v -> HexNut.metric(v.token("size")),
			c -> new ComponentValues().set("size", Token(cast(c, HexNut).spec.size)),
			c -> Std.isOfType(c, HexNut), true));
		result.push(new ComponentType("machinekit.standard.flat-washer",
			[catalog("size", FlatWasher.catalog(), "M5")],
			v -> FlatWasher.metric(v.token("size")),
			c -> new ComponentValues().set("size", Token(cast(c, FlatWasher).spec.size)),
			c -> Std.isOfType(c, FlatWasher), true));
		result.push(new ComponentType("machinekit.standard.parallel-key",
			[catalog("size", ParallelKey.catalog(), "2x2"), length("length", 10)],
			v -> ParallelKey.metric(v.token("size"), v.number("length")),
			c -> new ComponentValues()
				.set("size", Token('${Dimension.format(cast(c, ParallelKey).spec.width)}x${Dimension.format(cast(c, ParallelKey).spec.height)}'))
				.set("length", Number(cast(c, ParallelKey).length)),
			c -> Std.isOfType(c, ParallelKey), true));
		result.push(new ComponentType("machinekit.standard.retaining-ring",
			[catalog("shaft", RetainingRing.catalog(), "8")],
			v -> RetainingRing.forShaft(Std.parseFloat(v.token("shaft"))),
			c -> new ComponentValues().set("shaft", Token(Dimension.format(cast(c, RetainingRing).spec.shaftDiameter))),
			c -> Std.isOfType(c, RetainingRing), true));
		result.push(new ComponentType("machinekit.standard.shaft-collar",
			[catalog("bore", ShaftCollar.catalog(), "8")],
			v -> ShaftCollar.forShaft(Std.parseFloat(v.token("bore"))),
			c -> new ComponentValues().set("bore", Token(Dimension.format(cast(c, ShaftCollar).spec.boreDiameter))),
			c -> Std.isOfType(c, ShaftCollar), true));
		result.push(new ComponentType("machinekit.standard.bushing",
			[length("boreDiameter", 8), length("outerDiameter", 12), length("length", 10)],
			v -> new Bushing(v.number("boreDiameter"), v.number("outerDiameter"), v.number("length")),
			c -> new ComponentValues().set("boreDiameter", Number(cast(c, Bushing).boreDiameter))
				.set("outerDiameter", Number(cast(c, Bushing).outerDiameter)).set("length", Number(cast(c, Bushing).length)),
			c -> Std.isOfType(c, Bushing)));
		result.push(new ComponentType("machinekit.motion.linear-bearing",
			[catalog("designation", LinearBearing.catalog(), "LM8UU")],
			v -> LinearBearing.metric(v.token("designation")),
			c -> new ComponentValues().setToken("designation", cast(c, LinearBearing).spec.designation),
			c -> Std.isOfType(c, LinearBearing), true));
		result.push(new ComponentType("machinekit.motion.pillow-block",
			[catalog("designation", PillowBlock.catalog(), "UCP204")],
			v -> PillowBlock.metric(v.token("designation")),
			c -> new ComponentValues().setToken("designation", cast(c, PillowBlock).spec.designation),
			c -> Std.isOfType(c, PillowBlock), true));
		result.push(new ComponentType("machinekit.motion.linear-rail",
			[catalog("profile", LinearRailSystem.catalog(), "MGN12C"), length("length", 100)],
			v -> new LinearRail(LinearRailSystem.catalog().get(v.token("profile")), v.number("length")),
			c -> new ComponentValues().setToken("profile", cast(c, LinearRail).spec.designation)
				.setNumber("length", cast(c, LinearRail).length),
			c -> Std.isOfType(c, LinearRail)));
		result.push(new ComponentType("machinekit.motion.linear-rail-block",
			[catalog("profile", LinearRailSystem.catalog(), "MGN12C")],
			v -> new LinearRailBlock(LinearRailSystem.catalog().get(v.token("profile"))),
			c -> new ComponentValues().setToken("profile", cast(c, LinearRailBlock).spec.designation),
			c -> Std.isOfType(c, LinearRailBlock), true));
		result.push(new ComponentType("machinekit.motion.nema-stepper",
			[catalog("model", NemaStepper.variantCatalog(), "17HS19-1684S1")],
			v -> NemaStepper.model(v.token("model")),
			c -> new ComponentValues().setToken("model", cast(c, NemaStepper).variant.designation),
			c -> Std.isOfType(c, NemaStepper) &&
				NemaStepper.variantCatalog().designations().indexOf(cast(c, NemaStepper).variant.designation) >= 0, true));
		result.push(new ComponentType("machinekit.motion.generic-nema-stepper",
			[choice("frame", ["17", "23", "34"], "17"), length("bodyLength", 48)],
			v -> NemaStepper.frame(Std.parseInt(v.token("frame")), v.number("bodyLength")),
			c -> new ComponentValues().setToken("frame", Std.string(cast(c, NemaStepper).spec.frame))
				.setNumber("bodyLength", cast(c, NemaStepper).bodyLength),
			c -> Std.isOfType(c, NemaStepper) &&
				cast(c, NemaStepper).variant.designation.indexOf("GENERIC-NEMA") == 0));
		result.push(new ComponentType("machinekit.motion.flange-bearing-housing",
			[catalog("bearing", DeepGrooveBearing.catalog(), "608"), flag("shielded", true),
				choice("fit", ["Slip", "Transition", "Interference"], "Slip")],
			v -> new FlangeBearingHousing(DeepGrooveBearing.metric(v.token("bearing"), v.boolean("shielded")),
				fit(v.token("fit"))),
			c -> new ComponentValues().setToken("bearing", cast(c, FlangeBearingHousing).bearing.spec.designation)
				.setBoolean("shielded", cast(c, FlangeBearingHousing).bearing.shielded)
				.setToken("fit", Std.string(cast(c, FlangeBearingHousing).fit)),
			c -> Std.isOfType(c, FlangeBearingHousing)));
		result.push(new ComponentType("machinekit.motion.lead-screw",
			threadParameters().concat([length("length", 100)]),
			v -> new LeadScrew(thread(v), v.number("length")),
			c -> threadValues(cast(c, LeadScrew).thread).setNumber("length", cast(c, LeadScrew).totalLength),
			c -> Std.isOfType(c, LeadScrew)));
		result.push(new ComponentType("machinekit.motion.lead-screw-nut",
			threadParameters().concat([count("boltCount", 4)]),
			v -> new LeadScrewNut(thread(v), v.integer("boltCount")),
			c -> threadValues(cast(c, LeadScrewNut).thread).setInteger("boltCount", cast(c, LeadScrewNut).boltCount),
			c -> Std.isOfType(c, LeadScrewNut)));
		result.push(new ComponentType("machinekit.robotics.robot-flange",
			[length("pitchCircleDiameter", 40), count("boltCount", 4)],
			v -> new RobotFlange(v.number("pitchCircleDiameter"), v.integer("boltCount")),
			c -> new ComponentValues().setNumber("pitchCircleDiameter", cast(c, RobotFlange).spec.pitchCircle)
				.setInteger("boltCount", cast(c, RobotFlange).boltCount),
			c -> Std.isOfType(c, RobotFlange)));
		result.push(new ComponentType("machinekit.robotics.end-effector-plate",
			[length("flangePitchCircle", 40), count("flangeBoltCount", 4), length("thickness", 12),
				length("toolBoltCircleDiameter", 65), count("toolBoltCount", 4),
				catalog("toolMountScrew", SocketHeadCapScrew.catalog(), "M5")],
			v -> new EndEffectorPlate(new RobotFlange(v.number("flangePitchCircle"), v.integer("flangeBoltCount")),
				v.number("thickness"), v.number("toolBoltCircleDiameter"), v.integer("toolBoltCount"), v.token("toolMountScrew")),
			c -> new ComponentValues().setNumber("flangePitchCircle", cast(c, EndEffectorPlate).flange.spec.pitchCircle)
				.setInteger("flangeBoltCount", cast(c, EndEffectorPlate).flange.boltCount)
				.setNumber("thickness", cast(c, EndEffectorPlate).thickness)
				.setNumber("toolBoltCircleDiameter", cast(c, EndEffectorPlate).toolBoltCircleDiameter)
				.setInteger("toolBoltCount", cast(c, EndEffectorPlate).toolBoltCount)
				.setToken("toolMountScrew", cast(c, EndEffectorPlate).toolMountScrew),
			c -> Std.isOfType(c, EndEffectorPlate)));
		result.push(new ComponentType("machinekit.robotics.pedestal",
			[length("flangePitchCircle", 40), count("flangeBoltCount", 4), length("height", 150),
				length("columnDiameter", 60), count("floorBoltCount", 4), length("baseThickness", 12),
				length("anchorCircleDiameter", 90), length("gussetHeight", 0), length("gussetThickness", 4),
				count("gussetCount", 4), length("levelingFootDiameter", 0),
				length("levelingFootHeight", 4), length("cablePathDiameter", 0)],
			v -> new Pedestal(new RobotFlange(v.number("flangePitchCircle"), v.integer("flangeBoltCount")),
				v.number("height"), v.number("columnDiameter"), v.integer("floorBoltCount"),
				{baseThickness: v.number("baseThickness"), anchorCircleDiameter: v.number("anchorCircleDiameter"),
				gussetHeight: v.number("gussetHeight"), gussetThickness: v.number("gussetThickness"),
				gussetCount: v.integer("gussetCount"), levelingFootDiameter: v.number("levelingFootDiameter"),
				levelingFootHeight: v.number("levelingFootHeight"), cablePathDiameter: v.number("cablePathDiameter")}),
			c -> new ComponentValues().setNumber("flangePitchCircle", cast(c, Pedestal).flange.spec.pitchCircle)
				.setInteger("flangeBoltCount", cast(c, Pedestal).flange.boltCount)
				.setNumber("height", cast(c, Pedestal).height)
				.setNumber("columnDiameter", cast(c, Pedestal).columnDiameter)
				.setInteger("floorBoltCount", cast(c, Pedestal).floorBoltCount)
				.setNumber("baseThickness", cast(c, Pedestal).baseThickness)
				.setNumber("anchorCircleDiameter", cast(c, Pedestal).floorBoltCircleDiameter)
				.setNumber("gussetHeight", cast(c, Pedestal).gussetHeight)
				.setNumber("gussetThickness", cast(c, Pedestal).gussetThickness)
				.setInteger("gussetCount", cast(c, Pedestal).gussetCount)
				.setNumber("levelingFootDiameter", cast(c, Pedestal).levelingFootDiameter)
				.setNumber("levelingFootHeight", cast(c, Pedestal).levelingFootHeight)
				.setNumber("cablePathDiameter", cast(c, Pedestal).cablePathDiameter),
			c -> Std.isOfType(c, Pedestal)));
		result.push(new ComponentType("machinekit.transmission.sprocket",
			[length("pitch", 6.35), count("teeth", 20), length("boreDiameter", 8),
				length("thickness", 5), length("rollerDiameter", 3.3), catalog("chain", Sprocket.chainCatalog(), "ANSI25")],
			v -> new Sprocket(v.number("pitch"), v.integer("teeth"), v.number("boreDiameter"),
				v.number("thickness"), v.number("rollerDiameter"), v.token("chain")),
			c -> new ComponentValues().setNumber("pitch", cast(c, Sprocket).pitch)
				.setInteger("teeth", cast(c, Sprocket).teeth).setNumber("boreDiameter", cast(c, Sprocket).boreDiameter)
				.setNumber("thickness", cast(c, Sprocket).thickness)
				.setNumber("rollerDiameter", cast(c, Sprocket).rollerDiameter)
				.setToken("chain", cast(c, Sprocket).chain),
			c -> Std.isOfType(c, Sprocket) && cast(c, Sprocket).chain != null));
		result.push(new ComponentType("machinekit.transmission.generic-sprocket",
			[length("pitch", 6.35), count("teeth", 20), length("boreDiameter", 8),
				length("thickness", 5), length("rollerDiameter", 3.96875)],
			v -> new Sprocket(v.number("pitch"), v.integer("teeth"), v.number("boreDiameter"),
				v.number("thickness"), v.number("rollerDiameter")),
			c -> new ComponentValues().setNumber("pitch", cast(c, Sprocket).pitch)
				.setInteger("teeth", cast(c, Sprocket).teeth)
				.setNumber("boreDiameter", cast(c, Sprocket).boreDiameter)
				.setNumber("thickness", cast(c, Sprocket).thickness)
				.setNumber("rollerDiameter", cast(c, Sprocket).rollerDiameter),
			c -> Std.isOfType(c, Sprocket) && cast(c, Sprocket).chain == null));
		result.push(new ComponentType("machinekit.transmission.timing-pulley",
			[choice("profile", ["GT2", "HTD3M", "HTD5M", "HTD8M", "HTD14M", "T5", "XL"], "GT2"),
				count("teeth", 20), length("boreDiameter", 5), length("thickness", 6)],
			v -> new TimingPulley(profile(v.token("profile")),
				v.integer("teeth"), v.number("boreDiameter"), v.number("thickness")),
			c -> new ComponentValues().setToken("profile", Std.string(cast(c, TimingPulley).beltProfile))
				.setInteger("teeth", cast(c, TimingPulley).teeth)
				.setNumber("boreDiameter", cast(c, TimingPulley).boreDiameter)
				.setNumber("thickness", cast(c, TimingPulley).thickness),
			c -> Std.isOfType(c, TimingPulley) && switch (cast(c, TimingPulley).beltProfile) {
				case Custom(_, _, _): false;
				default: true;
			}));
		result.push(new ComponentType("machinekit.transmission.custom-timing-pulley",
			[choice("family", ["CUSTOM"], "CUSTOM"), length("pitch", 2),
				length("pitchLineDifferential", 0.254), count("teeth", 20),
				length("boreDiameter", 5), length("thickness", 6)],
			v -> new TimingPulley(Custom(v.token("family"), v.number("pitch"),
				v.number("pitchLineDifferential")), v.integer("teeth"),
				v.number("boreDiameter"), v.number("thickness")),
			c -> {
				var pulley:TimingPulley = cast c;
				var values = new ComponentValues().setInteger("teeth", pulley.teeth)
					.setNumber("boreDiameter", pulley.boreDiameter).setNumber("thickness", pulley.thickness);
				switch pulley.beltProfile {
					case Custom(family, pitch, differential):
						values.setToken("family", family).setNumber("pitch", pitch)
							.setNumber("pitchLineDifferential", differential);
					default: throw "Expected custom timing pulley";
				}
				return values;
			},
			c -> Std.isOfType(c, TimingPulley) && switch (cast(c, TimingPulley).beltProfile) {
				case Custom(_, _, _): true;
				default: false;
			}));
		return result;
	}

	static function threadParameters():Array<ComponentParameter> return [
		choice("family", ["MetricTrapezoidal", "Acme"], "MetricTrapezoidal"),
		length("screwDiameter", 8), length("pitch", 2), count("starts", 1),
		choice("hand", ["RightHand", "LeftHand"], "RightHand")];

	static function thread(v:ComponentValues):LeadScrewThread return new LeadScrewThread(
		family(v.token("family")), v.number("screwDiameter"),
		v.number("pitch"), v.integer("starts"), hand(v.token("hand")));

	static function threadValues(t:LeadScrewThread):ComponentValues return new ComponentValues()
		.setToken("family", Std.string(t.family)).setNumber("screwDiameter", t.screwDiameter)
		.setNumber("pitch", t.pitch).setInteger("starts", t.starts).setToken("hand", Std.string(t.hand));

	static function fit(value:String):BearingHousingFit return switch value {
		case "Slip": Slip;
		case "Transition": Transition;
		case "Interference": Interference;
		default: throw 'Unknown bearing fit "$value"';
	};
	static function family(value:String):LeadScrewThreadFamily return switch value {
		case "MetricTrapezoidal": MetricTrapezoidal;
		case "Acme": Acme;
		default: throw 'Unknown thread family "$value"';
	};
	static function hand(value:String):LeadScrewHand return switch value {
		case "RightHand": RightHand;
		case "LeftHand": LeftHand;
		default: throw 'Unknown thread hand "$value"';
	};
	static function profile(value:String):TimingBeltProfile return switch value {
		case "GT2": GT2;
		case "HTD3M": HTD3M;
		case "HTD5M": HTD5M;
		case "HTD8M": HTD8M;
		case "HTD14M": HTD14M;
		case "T5": T5;
		case "XL": XL;
		default: throw 'Unknown belt profile "$value"';
	};

	static function length(name:String, value:Float):ComponentParameter
		return new ComponentParameter(name, Length, Number(value), "mm", 0);

	static function flag(name:String, value:Bool):ComponentParameter
		return new ComponentParameter(name, Bool, Boolean(value));

	static function count(name:String, value:Int):ComponentParameter
		return new ComponentParameter(name, Count, Integer(value), null, 0);

	static function choice(name:String, options:Array<String>, value:String):ComponentParameter
		return new ComponentParameter(name, Choice(options), Token(value));

	static function catalog(name:String, index:CatalogIndex, value:String):ComponentParameter
		return new ComponentParameter(name, CatalogDesignation(index), Token(value));
}
