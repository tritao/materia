package machinekit.transmission;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** Timing belt pulley with a simplified tooth outline (shallow straight-flank grooves cut into a
 * near-pitch-diameter cylinder, not the true rounded-trapezoid GT2/HTD profile), extruded along
 * local +Z. CAD frame and connectors match `Sprocket`.
 */
class TimingPulley extends MachineComponent {
	public final pitch:Float;
	public final teeth:Int;
	public final boreDiameter:Float;
	public final thickness:Float;
	public final pitchDiameter:Float;
	public final outsideDiameter:Float;
	public final grooveDiameter:Float;

	public function new(pitch:Float, teeth:Int, boreDiameter:Float, thickness:Float) {
		if (!(pitch > 0)) throw "Timing pulley needs a positive belt pitch";
		if (teeth < 8) throw "Timing pulley needs at least 8 teeth";
		if (!(boreDiameter > 0)) throw "Timing pulley needs a positive bore diameter";
		if (!(thickness > 0)) throw "Timing pulley needs a positive thickness";
		var pitchDia = pitch * teeth / Math.PI;
		var grooveDepth = 0.2 * pitch;
		var outsideDia = pitchDia;
		var grooveDia = outsideDia - 2 * grooveDepth;
		if (!(grooveDia > boreDiameter))
			throw "Timing pulley groove diameter must clear the bore; use fewer teeth or a smaller bore";
		var pitchText = formatDecimal(pitch);
		super('PULLEY-P$pitchText-${teeth}T', 'Timing pulley, $pitchText mm pitch, ${teeth} teeth', "aluminium 6061");
		this.pitch = pitch;
		this.teeth = teeth;
		this.boreDiameter = boreDiameter;
		this.thickness = thickness;
		pitchDiameter = pitchDia;
		outsideDiameter = outsideDia;
		grooveDiameter = grooveDia;
		addConnector("front", Face, Solids.axial(0, 0, 0));
		addConnector("axis", Axis, Solids.axial(0, 0, thickness / 2));
		addConnector("back", Face, Solids.axial(0, 0, thickness));
	}

	/** Rounds to 3 decimal places and trims trailing zeros; avoids printing a binary float's full
	 * imprecise expansion for designations built from non-power-of-two pitches (e.g. MXL's 2.032).
	 */
	static function formatDecimal(value:Float):String {
		var scaled:Int = Math.round(value * 1000);
		var whole:Int = Std.int(scaled / 1000);
		var frac = scaled - whole * 1000;
		if (frac == 0) return Std.string(whole);
		var digits = Std.string(frac);
		while (digits.length < 3) digits = "0" + digits;
		while (digits.length > 1 && digits.charAt(digits.length - 1) == "0") digits = digits.substr(0, digits.length - 1);
		return '$whole.$digits';
	}

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = Solids.prism(profile(), 0, thickness);
		return Solids.cut(body, [Solids.cylinder(boreDiameter / 2, -0.1, thickness + 0.1)]);
	}

	function profile():Array<Vector> {
		var ra = outsideDiameter / 2, rf = grooveDiameter / 2;
		var angleStep = 2 * Math.PI / teeth;
		var halfLand = 0.3 * angleStep / 2, halfGroove = 0.15 * angleStep / 2;
		var points:Array<Vector> = [];
		for (i in 0...teeth) {
			var center = i * angleStep;
			points.push(new Vector(ra * Math.cos(center - halfLand), ra * Math.sin(center - halfLand)));
			points.push(new Vector(rf * Math.cos(center - halfGroove), rf * Math.sin(center - halfGroove)));
			points.push(new Vector(rf * Math.cos(center + halfGroove), rf * Math.sin(center + halfGroove)));
			points.push(new Vector(ra * Math.cos(center + halfLand), ra * Math.sin(center + halfLand)));
		}
		return points;
	}
}
