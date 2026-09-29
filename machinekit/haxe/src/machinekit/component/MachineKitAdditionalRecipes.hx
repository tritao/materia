package machinekit.component;

import haxe.Json;
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
import machinekit.robotics.FrameBar;
import machinekit.robotics.ParallelGripper;
import machinekit.robotics.ToolChangerMaster;
import machinekit.robotics.ToolChangerTool;
import machinekit.robotics.schmalz.SchmalzSxtMaster;
import machinekit.robotics.schmalz.SchmalzSxtTool;

/** Recipes for library parts that previously had only constructor APIs. */
class MachineKitAdditionalRecipes {
	static var types:Null<Array<ComponentType>>;
	static function n(name:String, value:Float):ComponentParameter return ComponentRecipeSupport.length(name, value);
	static function s(name:String, value:Float):ComponentParameter return ComponentRecipeSupport.scalar(name, value);
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
	static function routeText(points:Array<Vector>):String
		return Json.stringify([for (p in points) {x: p.x, y: p.y, z: p.z}]);
	static function defaultRoute():String return '[{"x":0,"y":0,"z":0},{"x":0,"y":0,"z":100}]';

	static function setScrews(text:String):Array<machinekit.motion.ShaftCoupling.ShaftCouplingSetScrew> {
		var rows:Array<Dynamic> = Json.parse(text);
		return [for (row in rows) {z: cast Reflect.field(row, "z"), angle: cast Reflect.field(row, "angle")}];
	}

	static function build():Array<ComponentType> return [
		new ComponentType("machinekit.pneumatic.manifold", [i("outlets", 2)],
			v -> new PneumaticManifold(v.integer("outlets")), true),
		new ComponentType("machinekit.pneumatic.suction-cup", [n("diameter", 40), n("height", 18),
			s("effectiveAreaMm2", 0), s("ratedMomentNm", 0)],
			v -> new SuctionCup(v.number("diameter"), v.number("height"),
				v.number("effectiveAreaMm2") == 0 ? null : v.number("effectiveAreaMm2"),
				v.number("ratedMomentNm") == 0 ? null : v.number("ratedMomentNm")), true),
		new ComponentType("machinekit.pneumatic.vacuum-generator", [s("ratedVacuumKpa", 0)],
			v -> new VacuumGenerator(v.number("ratedVacuumKpa") == 0 ? null : v.number("ratedVacuumKpa")), true),
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
				"polyurethane PU", false), true),
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
			v -> new SchmalzVacuumHose(v.token("stock"), route(v.token("route"))), true),
		new ComponentType("machinekit.robotics.frame-bar", [n("width", 20), n("depth", 20), n("length", 100)],
			v -> new FrameBar(v.number("width"), v.number("depth"), v.number("length")), true),
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

	public static function typeFor(component:MachineComponent):Null<ComponentType> {
		var all = entries();
		if (Std.isOfType(component, SchmalzSuctionCup)) return all[7];
		if (Std.isOfType(component, SchmalzVacuumGenerator)) return all[8];
		if (Std.isOfType(component, SchmalzVacuumHose)) return all[9];
		if (Std.isOfType(component, SchmalzPushInFitting)) return all[6];
		if (Std.isOfType(component, SchmalzSxtMaster)) return all[14];
		if (Std.isOfType(component, SchmalzSxtTool)) return all[15];
		if (Std.isOfType(component, PneumaticManifold)) return all[0];
		if (Std.isOfType(component, SuctionCup)) return all[1];
		if (Std.isOfType(component, VacuumGenerator)) return all[2];
		if (Std.isOfType(component, VacuumControlValve)) return all[3];
		if (Std.isOfType(component, VacuumPressureSensor)) return all[4];
		if (Std.isOfType(component, RoutedHose)) return all[5];
		if (Std.isOfType(component, FrameBar)) return all[10];
		if (Std.isOfType(component, ParallelGripper)) return all[11];
		if (Std.isOfType(component, ToolChangerMaster)) return all[12];
		if (Std.isOfType(component, ToolChangerTool)) return all[13];
		if (Std.isOfType(component, SteppedShaft)) return all[16];
		if (Std.isOfType(component, ShaftCoupling)) return all[17];
		if (Std.isOfType(component, Carriage)) return all[18];
		return null;
	}

	public static function valuesFor(component:MachineComponent):ComponentValues {
		var values = new ComponentValues();
		if (Std.isOfType(component, SchmalzSuctionCup)) values.setToken("designation", (cast component : SchmalzSuctionCup).spec.designation);
		else if (Std.isOfType(component, SchmalzVacuumGenerator)) values.setToken("designation", (cast component : SchmalzVacuumGenerator).spec.designation);
		else if (Std.isOfType(component, SchmalzPushInFitting)) values.setToken("designation", (cast component : SchmalzPushInFitting).spec.designation);
		else if (Std.isOfType(component, SchmalzVacuumHose)) {
			var hose:SchmalzVacuumHose = cast component;
			values.setToken("stock", hose.stock.designation).setToken("route", routeText(hose.route));
		} else if (Std.isOfType(component, SchmalzSxtMaster)) values.setToken("designation", (cast component : SchmalzSxtMaster).spec.designation);
		else if (Std.isOfType(component, SchmalzSxtTool)) values.setToken("designation", (cast component : SchmalzSxtTool).spec.designation);
		else if (Std.isOfType(component, PneumaticManifold)) values.setInteger("outlets", (cast component : PneumaticManifold).outlets);
		else if (Std.isOfType(component, SuctionCup)) {
			var cup:SuctionCup = cast component;
			values.setNumber("diameter", cup.diameter).setNumber("height", cup.height)
				.setNumber("effectiveAreaMm2", cup.effectiveAreaMm2 == null ? 0 : cup.effectiveAreaMm2)
				.setNumber("ratedMomentNm", cup.ratedMomentNm == null ? 0 : cup.ratedMomentNm);
		} else if (Std.isOfType(component, VacuumGenerator)) {
			var generator:VacuumGenerator = cast component;
			values.setNumber("ratedVacuumKpa", generator.ratedVacuumKpa == null ? 0 : generator.ratedVacuumKpa);
		} else if (Std.isOfType(component, VacuumControlValve)) {
			var valve:VacuumControlValve = cast component;
			values.setNumber("tubeOdMm", switch valve.port("vacuumIn").iface { case PushIn(d): d; case _: 0; });
		} else if (Std.isOfType(component, VacuumPressureSensor)) {
			var sensor:VacuumPressureSensor = cast component;
			values.setNumber("tubeOdMm", switch sensor.port("vacuumIn").iface { case PushIn(d): d; case _: 0; });
		} else if (Std.isOfType(component, RoutedHose)) {
			var hose:RoutedHose = cast component;
			values.setToken("designation", hose.designation).setToken("route", routeText(hose.route))
				.setNumber("outerDiameterMm", hose.outerDiameterMm).setNumber("innerDiameterMm", hose.innerDiameterMm)
				.setNumber("massPerMetreKg", hose.massPerMetreKg).setToken("service", Std.string(hose.serviceKind));
		} else if (Std.isOfType(component, FrameBar)) {
			var bar:FrameBar = cast component;
			values.setNumber("width", bar.width).setNumber("depth", bar.depth).setNumber("length", bar.length);
		} else if (Std.isOfType(component, ParallelGripper)) {
			var grip:ParallelGripper = cast component;
			values.setNumber("width", grip.width).setNumber("depth", grip.depth)
				.setNumber("length", grip.length).setNumber("stroke", grip.stroke);
		} else if (Std.isOfType(component, ToolChangerMaster)) {
			var changer:ToolChangerMaster = cast component;
			values.setInteger("airChannels", changer.airChannels).setNumber("diameter", changer.diameter)
				.setNumber("thickness", changer.thickness);
		} else if (Std.isOfType(component, ToolChangerTool)) {
			var changer:ToolChangerTool = cast component;
			values.setInteger("airChannels", changer.airChannels).setNumber("diameter", changer.diameter)
				.setNumber("thickness", changer.thickness);
		} else if (Std.isOfType(component, SteppedShaft)) return (cast component : SteppedShaft).recipeValues();
		else if (Std.isOfType(component, ShaftCoupling)) {
			var coupling:ShaftCoupling = cast component;
			values.setNumber("boreA", coupling.boreA).setNumber("boreB", coupling.boreB)
				.setNumber("outerDiameter", coupling.outerDiameter).setNumber("length", coupling.length)
				.setToken("setScrews", Json.stringify(coupling.setScrews));
		} else if (Std.isOfType(component, Carriage)) {
			var carriage:Carriage = cast component;
			values = ComponentRecipeSupport.threadValues(carriage.nut.thread);
			values.setNumber("boreDiameter", carriage.boreDiameter).setNumber("width", carriage.width)
				.setNumber("length", carriage.length).setNumber("guideSpacing", carriage.guideSpacing)
				.setNumber("guideSeatDiameter", carriage.guideSeatDiameter)
				.setToken("guideSeatFit", Std.string(carriage.guideSeatFit))
				.setBoolean("hasRailMount", carriage.hasRailMount)
				.setNumber("railMountY", carriage.railMountY)
				.setInteger("nutBoltCount", carriage.nut.boltCount);
		} else throw 'No recipe values for "${component.designation}"';
		return values.setToken("material", component.materialSpec());
	}
}
