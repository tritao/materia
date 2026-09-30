package machinekit.robotics;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import materia.assembly.AssemblyFrames;

/** Direction of a joint axis, in the frame of the link that carries the joint. */
enum ArmAxis {
	PlusX;
	MinusX;
	PlusZ;
}

/** Hollow tube link of a serial arm along local +Z, closed at both ends, with a collar where it
 * meets the previous joint. Its `start` connector sits at the origin with the joint axis given by
 * `startAxis`; its `end` connector carries the next joint module, whose axis is `endAxis`. A
 * lateral end joint is centred on the tube's end point, so `end` is offset by half the module
 * length (`endJointLength`) against the axis direction.
 */
class ArmLink extends MachineComponent {
	public final length:Float;
	public final diameter:Float;
	public final wall:Float;
	public final collarDiameter:Float;
	public final startAxis:ArmAxis;
	public final endAxis:ArmAxis;
	public final endJointLength:Float;

	static inline var COLLAR_THICKNESS:Float = 6;

	public function new(length:Float, diameter:Float, wall:Float, collarDiameter:Float, startAxis:ArmAxis,
			endAxis:ArmAxis, endJointLength:Float) {
		if (!Math.isFinite(length) || !Math.isFinite(diameter) || !Math.isFinite(wall) ||
			!Math.isFinite(collarDiameter) || !Math.isFinite(endJointLength) || endJointLength < 0)
			throw "Arm link needs finite dimensions";
		if (!(wall > 0) || !(length > 2 * wall) || !(diameter > 2 * wall))
			throw "Arm link needs a wall thinner than its radius and length";
		var text = '${Dimension.format(diameter)}x${Dimension.format(length)}';
		super('ARM-LINK-D$text-W${Dimension.format(wall)}', 'Arm link tube, $text mm', "aluminium 6061", true);
		this.length = length;
		this.diameter = diameter;
		this.wall = wall;
		this.collarDiameter = collarDiameter;
		this.startAxis = startAxis;
		this.endAxis = endAxis;
		this.endJointLength = endJointLength;
		var start = direction(startAxis);
		addConnector("start", Mount, AssemblyFrames.alongY(0, 0, 0, start.x, start.y, start.z));
		var end = direction(endAxis);
		var offset = endAxis == PlusZ ? 0 : endJointLength / 2;
		addConnector("end", Mount, AssemblyFrames.alongY(-offset * end.x, -offset * end.y,
			length - offset * end.z, end.x, end.y, end.z));
	}

	public static function direction(axis:ArmAxis):{x:Float, y:Float, z:Float}
		return switch axis {
			case PlusX: {x: 1, y: 0, z: 0};
			case MinusX: {x: -1, y: 0, z: 0};
			case PlusZ: {x: 0, y: 0, z: 1};
		};

	/** Recipe token for an axis: "+X", "-X" or "+Z". */
	public static function axisToken(axis:ArmAxis):String
		return switch axis {
			case PlusX: "+X";
			case MinusX: "-X";
			case PlusZ: "+Z";
		};

	public static function axisFromToken(token:String):ArmAxis
		return switch token {
			case "+X": PlusX;
			case "-X": MinusX;
			case "+Z": PlusZ;
			default: throw 'Unknown arm axis "$token"';
		};

	public static function recipeType():ComponentType
		return machinekit.component.MachineKitAdditionalRecipes.byId("machinekit.robotics.arm-link");

	/** Subclasses must declare their own recipe and saved values. */
	override public function componentType():Null<ComponentType>
		return Std.isExactType(this, ArmLink) ? recipeType() : null;

	override public function values():ComponentValues
		return new ComponentValues().setNumber("length", length).setNumber("diameter", diameter)
			.setNumber("wall", wall).setNumber("collarDiameter", collarDiameter)
			.setToken("startAxis", axisToken(startAxis)).setToken("endAxis", axisToken(endAxis))
			.setNumber("endJointLength", endJointLength).setToken("material", materialSpec());

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var start = direction(startAxis);
		var collar = Part.cylinderAlong(collarDiameter / 2, new Vector(0, 0, 0),
			new Vector(start.x, start.y, start.z), COLLAR_THICKNESS);
		var body = Solids.union([Part.cylinderSpan(diameter / 2, 0, length), collar]);
		if (detail == Envelope) return body;
		return Solids.cut(body, [Part.cylinderSpan(diameter / 2 - wall, wall, length - wall)]);
	}
}
