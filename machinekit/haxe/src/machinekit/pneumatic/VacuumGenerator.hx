package machinekit.pneumatic;

import cadkit.InertiaTensor;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.catalog.Catalog;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.Dimension;
import machinekit.component.PortInterface;
import machinekit.component.PortKind;
import machinekit.component.PortRole;
import machinekit.component.Solids;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;

/** A maker's ejector: its box envelope (silencer included), mass and published performance. */
typedef VacuumGeneratorRow = {
	var maker:String;
	var designation:String;
	var label:String;
	var widthMm:Float;
	var heightMm:Float;
	var envelopeLengthMm:Float;
	var massKg:Float;
	var degreeOfEvacuationPercent:Float;
	var optimalAirPressureBar:Float;
	var suctionRateLMin:Float;
	var airConsumptionLMin:Float;
}

/**
 * An ejector converting compressed air to vacuum: a catalogue product (`model`, its maker's row; an evacuation
 * rating of 85 % becomes 85 kPa against a conservative 100 kPa ambient) or a custom generator.
 */
class VacuumGenerator extends MachineComponent {
	public static inline var CUSTOM = "custom";
	static var table:Null<Catalog<VacuumGeneratorRow>>;

	public static function catalog():Catalog<VacuumGeneratorRow> {
		if (table == null) table = new Catalog("vacuum generator", row -> row.designation, [{
			maker: "Schmalz", designation: "10.02.01.00563", label: "SBP 05 S01 SDA",
			widthMm: 10, heightMm: 28, envelopeLengthMm: 71, massKg: 0.0075,
			degreeOfEvacuationPercent: 85, optimalAirPressureBar: 4.5,
			suctionRateLMin: 8, airConsumptionLMin: 13.5
		}], _ -> ({
			source: "https://www.schmalz.com/en/vacuum-technology-for-automation/vacuum-components/vacuum-generators/basic-ejectors/basic-ejectors-sbp-307660/10.02.01.00563/",
			standard: null, standardEdition: null, dimensionKind: Mixed,
			conformance: NominalEnvelope,
			verifiedFields: ["widthMm", "heightMm", "envelopeLengthMm", "massKg",
				"degreeOfEvacuationPercent", "optimalAirPressureBar",
				"suctionRateLMin", "airConsumptionLMin"]
		}));
		return table;
	}

	/** The catalogue generator `designation`. */
	public static function model(designation:String):VacuumGenerator {
		var row = catalog().get(designation);
		return new VacuumGenerator(row.degreeOfEvacuationPercent, PushIn(4), PushIn(4), row);
	}

	/** The catalogue row it is, or null for a custom generator. */
	public final row:Null<VacuumGeneratorRow>;
	/** Catalog maximum vacuum below ambient in kPa, not a guaranteed cup pressure. */
	public final ratedVacuumKpa:Null<Float>;

	public function new(?ratedVacuumKpa:Float, ?airInterface:PortInterface, ?vacuumInterface:PortInterface, ?row:VacuumGeneratorRow) {
		if (ratedVacuumKpa != null && (!Math.isFinite(ratedVacuumKpa) ||
				ratedVacuumKpa <= 0 || ratedVacuumKpa > 101.325))
			throw "Vacuum generator rating must be between 0 and 101.325 kPa";
		var ratingId = ratedVacuumKpa == null ? "" : '-V${Dimension.format(ratedVacuumKpa)}';
		super(row == null ? 'VACUUM-GENERATOR$ratingId' : row.designation,
			row == null ? "Generic pneumatic vacuum generator" : '${row.maker} ${row.label} ejector', "aluminium 6061", row == null);
		this.row = row;
		this.ratedVacuumKpa = ratedVacuumKpa;
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addPort({name: "air", kind: Pneumatic, role: Consumer,
			iface: airInterface == null ? PushIn(6) : airInterface, required: true});
		addPort({name: "vacuum", kind: Vacuum, role: Supply,
			iface: vacuumInterface == null ? PushIn(6) : vacuumInterface, required: false});
		addConversion("air", "vacuum");
		addFacet(new VacuumActuatorFacet("air"));
		addFacet(new VacuumSourceFacet(ratedVacuumKpa, "vacuum"));
		if (row != null) {
			setMaterial("plastic");
			var x = row.widthMm, y = row.heightMm, z = row.envelopeLengthMm, m = row.massKg;
			declareMass(m, new Vector(0, 0, z / 2), new InertiaTensor(m * (y * y + z * z) / 12, 0, 0, m * (x * x + z * z) / 12, 0, m * (x * x + y * y) / 12));
		}
	}

	/** Its envelope box: width, depth and length along z from the mount. */
	function box():Array<Float> return row == null ? [18.0, 14, 30] : [row.widthMm, row.heightMm, row.envelopeLengthMm];

	static var recipeTypeCache:Null<ComponentType>;

	public static function recipeType():ComponentType {
		if (recipeTypeCache == null) recipeTypeCache = new ComponentType("machinekit.pneumatic.vacuum-generator", [
			ComponentRecipeSupport.choice("model", [CUSTOM].concat(catalog().designations()), CUSTOM), ComponentRecipeSupport.optionalScalar("ratedVacuumKpa")]
			.concat(ComponentRecipeSupport.interfaceParameters("airInterface")).concat(ComponentRecipeSupport.interfaceParameters("vacuumInterface")),
			// A catalogue model's row sets every other input.
			v -> v.token("model") != CUSTOM ? model(v.token("model")) : new VacuumGenerator(v.optionalNumber("ratedVacuumKpa"),
				ComponentRecipeSupport.interfaceFrom(v, "airInterface"), ComponentRecipeSupport.interfaceFrom(v, "vacuumInterface")), true);
		return recipeTypeCache;
	}

	/** Subclasses must declare their own recipe and saved values. */
	override public function componentType():Null<ComponentType>
		return Std.isExactType(this, VacuumGenerator) ? recipeType() : null;

	override public function values():machinekit.component.ComponentValues {
		var values = new machinekit.component.ComponentValues()
			.set("ratedVacuumKpa", ratedVacuumKpa == null ? machinekit.component.ComponentValue.Unset : machinekit.component.ComponentValue.Number(ratedVacuumKpa))
			.setToken("model", row == null ? CUSTOM : row.designation);
		ComponentRecipeSupport.interfaceValues(values, "airInterface", port("air").iface);
		ComponentRecipeSupport.interfaceValues(values, "vacuumInterface", port("vacuum").iface);
		return values.setToken("material", materialSpec());
	}

	override public function hasGeometry():Bool return true;

	/** Venturi body with its ports and the silencer on top (`PneumaticBodies.vacuumGenerator`). The envelope box is also the mass model. */
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var b = box();
		return detail == Envelope ? Part.box(b[0], b[1], b[2]) : PneumaticBodies.vacuumGenerator(b[0], b[1], b[2]);
	}

	override public function massGeometry():Null<Part> {
		var b = box();
		return Part.box(b[0], b[1], b[2]);
	}
}
