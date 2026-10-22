package machinekit.milling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.motion.BallNut;

/** Welded steel carriage plate and central nut shelf. Origin: rear plate face,
 * centre X, bottom Z; +Y goes back toward the recessed screw. Nominal holes.
 */
class MillZCarriage extends MachineComponent {
	public static inline var GUSSET_HEIGHT:Float = 20;
	public static inline var SHELF_MARGIN:Float = 6;
	static var recipe:Null<ComponentType>;
	public static function parameters():Array<ComponentParameter> return [
			P.choice("nutModel",["generic1605","SFU2005"],"generic1605"), P.length("width", 234), P.length("height", 160), P.length("thickness", 16),
			P.length("nutReach", 53), P.length("nutCentreZ", 80), P.length("nutPlate", 10),
			P.length("headBoltX", 65), P.length("headBoltZ", 42),
			P.length("railSpacing", 180), P.length("blockSpan", 32.7), P.text("profile", "HGH15CA")];
	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.milling.z-carriage", parameters(), v -> new MillZCarriage(v));
		return recipe;
	}
	public static function selectedNut(model:String):machinekit.motion.LeadScrewNut return model=="SFU2005"?new machinekit.motion.SfuNut(model,16):new BallNut();
	static function guide(profile:String):machinekit.motion.LinearRailBlock return
		machinekit.motion.LinearRailBlock.metric(profile);

	public function new(?values:ComponentValues) {
		var v = P.resolve(parameters(), values), block = guide(v.token("profile")), nut = selectedNut(v.token("nutModel"));
		var width = v.number("width"), height = v.number("height"), t = v.number("thickness");
		if (width <= v.number("railSpacing") + block.spec.blockWidth ||
			height <= 2 * v.number("blockSpan") + block.spec.blockLength || t <= 0 ||
			v.number("nutReach") <= nut.flangeDiameter / 2 || v.number("nutPlate") <= 0 ||
			v.number("nutCentreZ") + nut.bodyLength / 2 - nut.flangeOffset - nut.flangeThickness - v.number("nutPlate") - GUSSET_HEIGHT <= 0)
			throw "Z carriage leaves insufficient plate or nut-shelf material";
		super("MILL-Z-CARRIAGE", "Welded Z carriage with central ball-nut shelf", v.get("material") == null ? "steel" : v.token("material"), false, v);
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var w = parameterNumber("width"), h = parameterNumber("height"), t = parameterNumber("thickness");
		var nut = selectedNut(parameterToken("nutModel")), reach = parameterNumber("nutReach"), shelf = parameterNumber("nutPlate");
		var face = parameterNumber("nutCentreZ") + nut.bodyLength / 2 - nut.flangeOffset - nut.flangeThickness;
		var shelfWidth = nut.flangeDiameter + 2 * SHELF_MARGIN;
		var pieces = [Part.box(w, t, h).translated(new Vector(0, -t / 2, 0)),
			Part.box(shelfWidth, reach + shelfWidth / 2, shelf).translated(new Vector(0, (reach + shelfWidth / 2) / 2, face - shelf))];
		for (side in [-1, 1]) pieces.push(Part.box(6, Math.min(reach / 2, 25), GUSSET_HEIGHT)
			.translated(new Vector(side * (shelfWidth / 2 - 3), Math.min(reach / 2, 25) / 2, face - shelf - GUSSET_HEIGHT)));
		var body = Solids.union(pieces);
		var tools = [Part.cylinderSpan(nut.bodyDiameter / 2 + 0.2, face - shelf - 1, face + 1, 0, reach)];
		for (point in nut.boltPattern()) tools.push(Part.cylinderSpan(nut.mountScrew=="M6"?3.35:2.75, face - shelf - 1, face + 1, point.x, reach + point.y));
		var block = guide(parameterToken("profile"));
		for (side in [-1, 1]) for (span in [-parameterNumber("blockSpan"), parameterNumber("blockSpan")])
			for (x in [-block.spec.blockHoleSpacingAcross / 2, block.spec.blockHoleSpacingAcross / 2])
				for (z in [-block.spec.blockHoleSpacingAlong / 2, block.spec.blockHoleSpacingAlong / 2])
					tools.push(Part.cylinderAlongY(block.spec.blockMountThread == "M6" ? 3.3 : 2.25, -t - 1, 1, side * parameterNumber("railSpacing") / 2 + x,
						parameterNumber("nutCentreZ") + span + z));
		// Blind nominal M6 taps match the head flange; retain the rear face of the carriage.
		for (x in [-parameterNumber("headBoltX"), parameterNumber("headBoltX")])
			for (z in [h / 2 - parameterNumber("headBoltZ"), h / 2 + parameterNumber("headBoltZ")])
				tools.push(Part.cylinderAlongY(2.5, -t - 1, -t + 12, x, z));
		return Solids.cut(body, tools);
	}
}
