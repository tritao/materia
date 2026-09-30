package machinekit.component;

import haxe.Json;
import machinekit.component.ComponentParameterType.*;
import cadkit.modeling.Vector;
import machinekit.motion.ShaftCoupling;
import machinekit.motion.SteppedShaft;
import machinekit.motion.LeadScrewNut;
import machinekit.assembly.LinearAxis.Carriage;
import machinekit.pneumatic.PneumaticManifold;
import machinekit.pneumatic.RoutedHose;
import machinekit.pneumatic.SuctionCup;
import machinekit.pneumatic.VacuumControlValve;
import machinekit.pneumatic.VacuumGenerator;
import machinekit.pneumatic.VacuumPressureSensor;
import machinekit.pneumatic.schmalz.SchmalzPushInFitting;
import machinekit.pneumatic.schmalz.SchmalzSuctionCup;
import machinekit.pneumatic.schmalz.SchmalzVacuumGenerator;
import machinekit.pneumatic.schmalz.SchmalzVacuumHose;
import machinekit.robotics.ArmJoint;
import machinekit.robotics.ArmLink;
import machinekit.robotics.FrameBar;
import machinekit.robotics.ParallelGripper;
import machinekit.robotics.RobotFlange;
import machinekit.robotics.ToolChangerMaster;
import machinekit.robotics.ToolChangerTool;
import machinekit.robotics.schmalz.SchmalzSxtMaster;
import machinekit.robotics.schmalz.SchmalzSxtTool;

/** Recipes for library parts that previously had only constructor APIs. */
class MachineKitAdditionalRecipes {
	static var types:Null<Array<ComponentType>>;
	static function n(name:String, value:Float):ComponentParameter return ComponentRecipeSupport.length(name, value);
	static function s(name:String, value:Float):ComponentParameter return ComponentRecipeSupport.scalar(name, value);
	static function optionalScalar(name:String):ComponentParameter
		return new ComponentParameter(name, machinekit.component.ComponentParameterType.Optional(Scalar), ComponentValue.Unset);
	static function optionalText(name:String):ComponentParameter
		return new ComponentParameter(name, machinekit.component.ComponentParameterType.Optional(Text), ComponentValue.Unset);
	static function i(name:String, value:Int):ComponentParameter return ComponentRecipeSupport.count(name, value);
	static function t(name:String, value:String):ComponentParameter return ComponentRecipeSupport.text(name, value);
	static function c(name:String, values:Array<String>, value:String):ComponentParameter
		return ComponentRecipeSupport.choice(name, values, value);
	static function cat(name:String, index:machinekit.catalog.CatalogIndex, value:String):ComponentParameter
		return ComponentRecipeSupport.catalog(name, index, value);

	public static function all():Array<ComponentType> return entries().copy();
	static function entries():Array<ComponentType> {
		if (types == null) types = build();
		return types;
	}

	static function route(text:String):Array<Vector> {
		var points:Array<Dynamic> = Json.parse(text);
		return [for (p in points) new Vector(cast Reflect.field(p, "x"),
			cast Reflect.field(p, "y"), cast Reflect.field(p, "z"))];
	}
	public static function routeText(points:Array<Vector>):String
		return Json.stringify([for (p in points) {x: p.x, y: p.y, z: p.z}]);
	static function defaultRoute():String return '[{"x":0,"y":0,"z":0},{"x":0,"y":0,"z":100}]';

	static function interfaceParameters(prefix:String):Array<ComponentParameter> return [
		c(prefix + "Kind", ["PushIn", "Thread", "Plug", "Coupling", "Unspecified"], "PushIn"),
		n(prefix + "Size", 6), t(prefix + "Name", ""), i(prefix + "Channel", 0)
	];

	static function interfaceFrom(values:ComponentValues, prefix:String):PortInterface
		return switch values.token(prefix + "Kind") {
			case "PushIn": PortInterface.PushIn(values.number(prefix + "Size"));
			case "Thread": PortInterface.Thread(values.token(prefix + "Name"));
			case "Plug": PortInterface.Plug(values.token(prefix + "Name"), values.integer(prefix + "Channel"));
			case "Coupling": PortInterface.Coupling(values.token(prefix + "Name"), values.integer(prefix + "Channel"));
			case _: PortInterface.Unspecified;
		};

	public static function interfaceValues(values:ComponentValues, prefix:String, iface:PortInterface):Void
		switch iface {
			case PushIn(size): values.setToken(prefix + "Kind", "PushIn").setNumber(prefix + "Size", size);
			case Thread(name): values.setToken(prefix + "Kind", "Thread").setToken(prefix + "Name", name);
			case Plug(name, pins): values.setToken(prefix + "Kind", "Plug").setToken(prefix + "Name", name).setInteger(prefix + "Channel", pins);
			case Coupling(key, channel): values.setToken(prefix + "Kind", "Coupling").setToken(prefix + "Name", key).setInteger(prefix + "Channel", channel);
			case Unspecified: values.setToken(prefix + "Kind", "Unspecified");
		}

	static function setScrews(text:String):Array<machinekit.motion.ShaftCoupling.ShaftCouplingSetScrew> {
		var rows:Array<Dynamic> = Json.parse(text);
		return [for (row in rows) {z: cast Reflect.field(row, "z"), angle: cast Reflect.field(row, "angle")}];
	}

	static function build():Array<ComponentType> return [
		new ComponentType("machinekit.pneumatic.manifold", [i("outlets", 2)],
			v -> new PneumaticManifold(v.integer("outlets")), true),
		new ComponentType("machinekit.pneumatic.suction-cup", [n("diameter", 40), n("height", 18),
			optionalScalar("effectiveAreaMm2"), optionalScalar("ratedMomentNm"),
			optionalText("catalogDesignation"), optionalText("catalogDescription")]
			.concat(interfaceParameters("vacuumInterface")),
			v -> new SuctionCup(v.number("diameter"), v.number("height"),
				v.optionalNumber("effectiveAreaMm2"), v.optionalNumber("ratedMomentNm"),
				v.optionalToken("catalogDesignation"), interfaceFrom(v, "vacuumInterface"),
				v.optionalToken("catalogDescription")), true),
		new ComponentType("machinekit.pneumatic.vacuum-generator", [optionalScalar("ratedVacuumKpa"),
			optionalText("catalogDesignation"), optionalText("catalogDescription")]
			.concat(interfaceParameters("airInterface")).concat(interfaceParameters("vacuumInterface")),
			v -> new VacuumGenerator(v.optionalNumber("ratedVacuumKpa"),
				v.optionalToken("catalogDesignation"), interfaceFrom(v, "airInterface"),
				interfaceFrom(v, "vacuumInterface"), v.optionalToken("catalogDescription")), true),
		new ComponentType("machinekit.pneumatic.vacuum-control-valve", [n("tubeOdMm", 4)],
			v -> new VacuumControlValve(v.number("tubeOdMm")), true),
		new ComponentType("machinekit.pneumatic.vacuum-pressure-sensor", [n("tubeOdMm", 4)],
			v -> new VacuumPressureSensor(v.number("tubeOdMm")), true),
		new ComponentType("machinekit.pneumatic.routed-hose", [t("designation", "HOSE"), t("route", defaultRoute()),
			n("outerDiameterMm", 4), n("innerDiameterMm", 2), s("massPerMetreKg", 0.01),
			c("service", ["Vacuum", "Pneumatic"], "Vacuum")],
			v -> new RoutedHose(v.token("designation"), route(v.token("route")),
				v.number("outerDiameterMm"), v.number("innerDiameterMm"), v.number("massPerMetreKg"),
				v.token("service") == "Vacuum" ? PortKind.Vacuum : PortKind.Pneumatic,
				"polyurethane PU", false), true, false, ["route"]),
		new ComponentType("machinekit.pneumatic.schmalz-push-in-fitting",
			[cat("designation", SchmalzPushInFitting.catalog(), "10.08.02.00203")],
			v -> new SchmalzPushInFitting(v.token("designation")), true),
		new ComponentType("machinekit.pneumatic.schmalz-suction-cup",
			[cat("designation", SchmalzSuctionCup.catalog(), "10.01.01.11400")],
			v -> new SchmalzSuctionCup(v.token("designation")), true),
		new ComponentType("machinekit.pneumatic.schmalz-vacuum-generator",
			[cat("designation", SchmalzVacuumGenerator.catalog(), "10.02.01.00563")],
			v -> new SchmalzVacuumGenerator(v.token("designation")), true),
		new ComponentType("machinekit.pneumatic.schmalz-vacuum-hose",
			[cat("stock", SchmalzVacuumHose.catalog(), "10.07.09.00001"), t("route", defaultRoute())],
			v -> new SchmalzVacuumHose(v.token("stock"), route(v.token("route"))), true, false, ["route"]),
		new ComponentType("machinekit.robotics.frame-bar", [n("width", 20), n("depth", 20), n("length", 100)],
			v -> new FrameBar(v.number("width"), v.number("depth"), v.number("length")), true),
		new ComponentType("machinekit.robotics.arm-joint", [n("diameter", 100), n("length", 70), n("flangePitchCircle", 0)],
			v -> new ArmJoint(v.number("diameter"), v.number("length"),
				v.number("flangePitchCircle") > 0 ? new RobotFlange(v.number("flangePitchCircle")) : null), true),
		new ComponentType("machinekit.robotics.arm-link", [n("length", 300), n("diameter", 80), n("wall", 5),
			n("collarDiameter", 100), c("startAxis", ["+X", "-X", "+Z"], "+Z"), c("endAxis", ["+X", "-X", "+Z"], "+Z"),
			n("endJointLength", 0)],
			v -> new ArmLink(v.number("length"), v.number("diameter"), v.number("wall"), v.number("collarDiameter"),
				ArmLink.axisFromToken(v.token("startAxis")), ArmLink.axisFromToken(v.token("endAxis")),
				v.number("endJointLength")), true),
		new ComponentType("machinekit.robotics.parallel-gripper", [n("width", 40), n("depth", 20),
			n("length", 60), n("stroke", 30)],
			v -> new ParallelGripper(v.number("width"), v.number("depth"), v.number("length"), v.number("stroke")), true),
		new ComponentType("machinekit.robotics.tool-changer-master", [i("airChannels", 2),
			n("diameter", 60), n("thickness", 15)],
			v -> new ToolChangerMaster(v.integer("airChannels"), v.number("diameter"), v.number("thickness")), true),
		new ComponentType("machinekit.robotics.tool-changer-tool", [i("airChannels", 2),
			n("diameter", 60), n("thickness", 12)],
			v -> new ToolChangerTool(v.integer("airChannels"), v.number("diameter"), v.number("thickness")), true),
		new ComponentType("machinekit.robotics.schmalz-sxt-master",
			[cat("designation", SchmalzSxtMaster.catalog(), "10.07.13.00013")],
			v -> new SchmalzSxtMaster(v.token("designation")), true),
		new ComponentType("machinekit.robotics.schmalz-sxt-tool",
			[cat("designation", SchmalzSxtTool.catalog(), "10.07.13.00018")],
			v -> new SchmalzSxtTool(v.token("designation")), true),
		new ComponentType("machinekit.motion.stepped-shaft",
			[t("sections", '[{"diameter":10,"length":100}]'), t("faces", "[]"),
				t("keyways", "[]"), t("grooves", "[]"), t("shaftDetail", "{}")],
			v -> SteppedShaft.fromRecipe(v), true),
		new ComponentType("machinekit.motion.shaft-coupling", [n("boreA", 8), n("boreB", 8),
			n("outerDiameter", 20), n("length", 30), t("setScrews", '[{"z":7.5,"angle":0},{"z":22.5,"angle":0}]')],
			v -> new ShaftCoupling(v.number("boreA"), v.number("boreB"), v.number("outerDiameter"),
				v.number("length"), setScrews(v.token("setScrews"))), true),
		new ComponentType("machinekit.assembly.carriage", [n("boreDiameter", 8), n("width", 70),
			n("length", 50), n("guideSpacing", 25), n("guideSeatDiameter", 12),
			c("guideSeatFit", ["Slip", "Transition", "Interference"], "Slip"),
			ComponentRecipeSupport.flag("hasRailMount", false), s("railMountY", 0),
			i("nutBoltCount", 4)].concat(ComponentRecipeSupport.threadParameters()),
			v -> new Carriage(v.number("boreDiameter"), v.number("width"), v.number("length"),
				v.number("guideSpacing"), v.number("guideSeatDiameter"),
				new LeadScrewNut(ComponentRecipeSupport.thread(v), v.integer("nutBoltCount")),
				ComponentRecipeSupport.fit(v.token("guideSeatFit")),
				v.boolean("hasRailMount") ? v.number("railMountY") : null), true)
	];

	public static function byId(id:String):ComponentType {
		for (recipe in entries()) if (recipe.id == id) return recipe;
		throw 'Unknown additional MachineKit recipe "$id"';
	}
}
