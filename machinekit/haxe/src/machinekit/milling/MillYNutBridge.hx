package machinekit.milling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.PosedParts;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.motion.BallNut;
import materia.assembly.AssemblyFrames as F;

/** Gusseted Y nut bridge. Shaft +Z points toward the front, +Y is up.
 * The upper shoe meets the saddle underside; the vertical flange meets its front.
 * Rise is measured from the nut axis to the saddle underside. Nominal holes.
 */
class MillYNutBridge extends MachineComponent {
	static var recipe:Null<ComponentType>;
	public static function parameters():Array<ComponentParameter> return [P.length("rise", 128)];
	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.milling.y-nut-bridge", parameters(), v -> new MillYNutBridge(v));
		return recipe;
	}
	public function new(?values:ComponentValues) {
		var v = P.resolve(parameters(), values);
		if (v.number("rise") < 60) throw "Y bridge leaves insufficient space for its nut and gussets";
		super("MILL-Y-NUT-BRIDGE", "Gusseted bridge from lowered Y screw to saddle", "steel", false, v);
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var rise = parameterNumber("rise");
		var pieces = [Part.box(84, rise + 54, 12).translated(new Vector(0, (rise - 6) / 2, 0)),
			Part.box(84, 12, 72).translated(new Vector(0, rise - 6, -60))];
		for (side in [-1, 1]) {
			var rib = Part.prism([new Vector(0, 30), new Vector(-60, rise - 12), new Vector(0, rise - 12)], -3, 3);
			pieces.push(PosedParts.moved(rib, F.fromRotationMatrix(side * 39, 0, 0, [0,0,-1, 0,1,0, 1,0,0]))); rib.close();
		}
		var tools = [Part.cylinderSpan(9, -1, 13)];
		for (p in new BallNut().boltPattern()) tools.push(Part.cylinderSpan(2.75, -1, 13, p.x, p.y));
		for (x in [-32.0, 32.0]) tools.push(Part.cylinderSpan(3.3, -1, 13, x, rise + 12));
		for (x in [-25.0, 25.0]) tools.push(Part.cylinderAlongY(2.5, rise - 13, rise + 1, x, -30));
		return Solids.cut(Solids.union(pieces), tools);
	}
}
