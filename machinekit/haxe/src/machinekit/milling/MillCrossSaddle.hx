package machinekit.milling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.PosedParts;
import materia.assembly.AssemblyFrames as F;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Cross slide with a bottom Z datum and optional through-Y nut tunnel. */
class MillCrossSaddle extends MachineComponent {
	static var recipe:Null<ComponentType>;

	public static function parameters():Array<ComponentParameter> return [
		P.length("width", 450),
		P.length("depth", 174),
		P.length("height", 80),
		P.length("bore", 54), P.flag("motorPocket", true), P.flag("sidePockets", false)
	];

	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.milling.cross-saddle", parameters(), v -> new MillCrossSaddle(v));
		return recipe;
	}

	public function new(?values:ComponentValues) {
		var v = P.resolve(parameters(), values);
		if (v.number("bore") + 20 > v.number("height") || v.number("bore") >= v.number("width")) throw "Saddle tunnel leaves insufficient material";
		if (v.boolean("sidePockets") && (v.number("width") < 450 || v.number("depth") < 174 || v.number("height") < 56))
			throw "Saddle side pockets need continuous chords and end webs";
		super("MILL-CROSS-SADDLE", "Machined cross saddle", "cast iron", false, v);
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addConnector("top", Face, Solids.axial(0, 0, v.number("height")));
	}

	override public function componentType():Null<ComponentType> return recipeType();

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var w = parameterNumber("width"), d = parameterNumber("depth"), h = parameterNumber("height");
		var tools:Array<Part> = [];
		if (parameterNumber("bore") > 0) tools.push(Part.cylinderAlongY(parameterNumber("bore") / 2, -d / 2 - 1, d / 2 + 1, 0, h / 2));
		for (x in [-35.0, 35.0]) for (z in [-20.0, 20.0]) tools.push(Part.cylinderAlongY(2.5, d / 2 - 16, d / 2 + 1, x, h / 2 + z));
		if (values().boolean("motorPocket")) {
			var bore = Part.cylinderSpan(33, -226, -64);
			tools.push(PosedParts.moved(bore, F.fromRotationMatrix(0, 0, 40, [0,0,1, 1,0,0, 0,1,0]))); bore.close();
			tools.push(Part.box(12, 84, 82).translated(new Vector(-201, 0, -1)));
			tools.push(Part.box(17, 40, 42).translated(new Vector(-218.5, 0, 40)));
			for (y in [-32.0, 32.0]) for (z in [10.0, 70.0]) {
				var hole = Part.cylinderSpan(2.5, -196, -182);
				tools.push(PosedParts.moved(hole, F.fromRotationMatrix(0, y, z, [0,0,1, 1,0,0, 0,1,0]))); hole.close();
			}
		}
		for (x in [-32.0, 32.0]) tools.push(Part.cylinderAlongY(2.5, -d / 2 - 1, -d / 2 + 16, x, 12));
		// Shallow side pockets retain continuous top/bottom chords, end webs and the central nut attachment.
		if (values().boolean("sidePockets")) for (side in [-1, 1])
			for (interval in [[-205.0,-115.0],[-105.0,-55.0],[55.0,130.0],[140.0,205.0]])
				tools.push(Part.box(interval[1]-interval[0], 13, h-28)
					.translated(new Vector((interval[0]+interval[1])/2, side*(d/2-5.5), 14)));
		return Solids.cut(Part.box(w, d, h), tools);
	}
}
