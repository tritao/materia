package machinekit.component;

import haxe.Json;
import cadkit.modeling.Vector;
import machinekit.catalog.CatalogIndex;
import machinekit.component.ComponentParameterType.*;
import machinekit.component.ComponentValue.*;
import machinekit.motion.LeadScrewThread;
import machinekit.motion.LeadScrewThread.LeadScrewThreadFamily;
import machinekit.motion.LeadScrewThread.LeadScrewHand;
import machinekit.standard.BearingFit.BearingHousingFit;
import machinekit.standard.ClearanceFit;
import machinekit.transmission.TimingBeltProfile;

/** Shared parameter constructors and parsed enum inputs for component recipes. */
class ComponentRecipeSupport {
	public static function threadParameters():Array<ComponentParameter> return [
		choice("family", ["MetricTrapezoidal", "Acme", "Ball"], "MetricTrapezoidal"),
		length("screwDiameter", 8), length("pitch", 2), count("starts", 1),
		choice("hand", ["RightHand", "LeftHand"], "RightHand")];

	public static function thread(v:ComponentValues):LeadScrewThread return new LeadScrewThread(
		family(v.token("family")), v.number("screwDiameter"),
		v.number("pitch"), v.integer("starts"), hand(v.token("hand")));

	public static function threadValues(t:LeadScrewThread):ComponentValues return new ComponentValues()
		.setToken("family", Std.string(t.family)).setNumber("screwDiameter", t.screwDiameter)
		.setNumber("pitch", t.pitch).setInteger("starts", t.starts).setToken("hand", Std.string(t.hand));

	public static function fit(value:String):BearingHousingFit return switch value {
		case "Slip": Slip;
		case "Transition": Transition;
		case "Interference": Interference;
		default: throw 'Unknown bearing fit "$value"';
	};
	public static function clearanceFit(value:String):ClearanceFit return switch value {
		case "Fine": Fine;
		case "Medium": Medium;
		case "Coarse": Coarse;
		default: throw 'Unknown clearance fit "$value"';
	};
	public static function family(value:String):LeadScrewThreadFamily return switch value {
		case "MetricTrapezoidal": MetricTrapezoidal;
		case "Acme": Acme;
		case "Ball": Ball;
		default: throw 'Unknown thread family "$value"';
	};
	public static function hand(value:String):LeadScrewHand return switch value {
		case "RightHand": RightHand;
		case "LeftHand": LeftHand;
		default: throw 'Unknown thread hand "$value"';
	};
	public static function profile(value:String):TimingBeltProfile return switch value {
		case "GT2": GT2;
		case "HTD3M": HTD3M;
		case "HTD5M": HTD5M;
		case "HTD8M": HTD8M;
		case "HTD14M": HTD14M;
		case "T5": T5;
		case "XL": XL;
		default: throw 'Unknown belt profile "$value"';
	};

	public static function length(name:String, value:Float):ComponentParameter
		return new ComponentParameter(name, Length, Number(value), "mm", 0);

	public static function scalar(name:String, value:Float, ?minimum:Float, ?maximum:Float):ComponentParameter
		return new ComponentParameter(name, Scalar, Number(value), "1", minimum, maximum);

	public static function flag(name:String, value:Bool):ComponentParameter
		return new ComponentParameter(name, Bool, Boolean(value));

	public static function toolDepth(value:Float):ComponentParameter
		return new ComponentParameter("depth", Length, Number(value), "mm", 0.001);

	public static function count(name:String, value:Int):ComponentParameter
		return new ComponentParameter(name, Count, Integer(value), null, 0);

	public static function text(name:String, value:String):ComponentParameter
		return new ComponentParameter(name, Text, Token(value));

	public static function choice(name:String, options:Array<String>, value:String):ComponentParameter
		return new ComponentParameter(name, Choice(options), Token(value));

	public static function catalog(name:String, index:CatalogIndex, value:String):ComponentParameter
		return new ComponentParameter(name, CatalogDesignation(index), Token(value));

	public static function optionalScalar(name:String):ComponentParameter
		return new ComponentParameter(name, Optional(Scalar), Unset);

	public static function optionalText(name:String):ComponentParameter
		return new ComponentParameter(name, Optional(Text), Unset);

	/** Inputs describing a port's mating interface, under one name prefix. */
	public static function interfaceParameters(prefix:String):Array<ComponentParameter> return [
		choice(prefix + "Kind", ["PushIn", "Thread", "Plug", "Coupling", "Unspecified"], "PushIn"),
		length(prefix + "Size", 6), text(prefix + "Name", ""), count(prefix + "Channel", 0)
	];

	public static function interfaceFrom(values:ComponentValues, prefix:String):PortInterface
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

	public static function route(text:String):Array<Vector> {
		var points:Array<Dynamic> = Json.parse(text);
		return [for (p in points) new Vector(cast Reflect.field(p, "x"),
			cast Reflect.field(p, "y"), cast Reflect.field(p, "z"))];
	}

	public static function routeText(points:Array<Vector>):String
		return Json.stringify([for (p in points) {x: p.x, y: p.y, z: p.z}]);

	public static function defaultRoute():String return '[{"x":0,"y":0,"z":0},{"x":0,"y":0,"z":100}]';
}
