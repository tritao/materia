package machinekit.milling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** T-slot table with integral longitudinal ribs and a recessed X nut bracket seat.
 * Origin is the underside of the deck; ribs extend down to the guide blocks. */
class MillCrossTable extends MachineComponent {
	static var recipe:Null<ComponentType>;

	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.milling.cross-table", [P.length("depth", 200), P.length("ribSpacing", 124), P.length("width",400), P.length("thickness",40), P.length("ribHeight",48), P.length("ribWidth",34), P.flag("nutSeat",true)], v -> new MillCrossTable(v.number("depth"), v.number("ribSpacing"),v.number("width"),v.number("thickness"),v.number("ribHeight"),v.number("ribWidth"),v.boolean("nutSeat")));
		return recipe;
	}

	public function new(depth:Float = 200, ribSpacing:Float = 124, width:Float=400, thickness:Float=40, ribHeight:Float=48, ribWidth:Float=34, nutSeat:Bool=true) {
		for(v in [width,thickness,ribHeight,ribWidth])if(!Math.isFinite(v)||v<=0)throw "Table dimensions must be finite and positive";
		if(width<400||thickness<40||ribWidth<34||ribSpacing+ribWidth>depth-12 ||
      (nutSeat&&ribSpacing-ribWidth<86))throw "Table ribs and slots need retained edge margins";
		if (!Math.isFinite(depth) || !Math.isFinite(ribSpacing) || depth < 200 || ribSpacing < 124) throw "Table must retain its slots, drive corridor and rib edge margins";
		super("MILL-CROSS-TABLE", "Ribbed T-slot table", "cast iron", false, new ComponentValues().setNumber("depth", depth).setNumber("ribSpacing", ribSpacing).setNumber("width",width).setNumber("thickness",thickness).setNumber("ribHeight",ribHeight).setNumber("ribWidth",ribWidth).setBoolean("nutSeat",nutSeat));
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addConnector("top", Face, Solids.axial(0, 0, thickness));
	}

	override public function componentType():Null<ComponentType> return recipeType();

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var width=parameterNumber("width"),thickness=parameterNumber("thickness"),ribHeight=parameterNumber("ribHeight");
		var tools:Array<Part> = [];
		if(values().boolean("nutSeat"))tools.push(Part.box(42, 86, 13).translated(new Vector(20, 0, -1)));
		var slots=Std.int((parameterNumber("depth")/2-20)/40);
		for (slot in -slots...slots+1) {
			var y=slot*40.0;
			tools.push(Part.box(width+2, 10, 9).translated(new Vector(0, y, thickness-8)));
			tools.push(Part.box(width+2, 18, 6).translated(new Vector(0, y, thickness-14)));
		}
		if(values().boolean("nutSeat"))for (y in [-25.0, 25.0]) {
			tools.push(Part.cylinderSpan(3.3, 11, 41, 20, y));
			tools.push(Part.cylinderSpan(5.5, 33, 41, 20, y));
		}
		var pieces = [Part.box(width, parameterNumber("depth"), thickness)];
		for (side in [-1, 1]) pieces.push(Part.box(width, parameterNumber("ribWidth"), ribHeight)
			.translated(new Vector(0, side * parameterNumber("ribSpacing") / 2, -ribHeight)));
		return Solids.cut(Solids.union(pieces), tools);
	}
}
