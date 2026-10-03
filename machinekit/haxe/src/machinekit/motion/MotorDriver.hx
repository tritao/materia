package machinekit.motion;

import cadkit.modeling.Part;
import machinekit.catalog.Catalog;
import machinekit.catalog.CatalogMetadata.DimensionKind;
import machinekit.catalog.CatalogMetadata.Conformance;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

enum MotorDriverFamily {
	Stepper;
	Servo;
}

enum MotorDriverControl {
	StepDir;
	Bus;
}

typedef MotorDriverRating = {
	var designation:String;
	var family:MotorDriverFamily;
	var control:MotorDriverControl;
	var minimumVoltage:Float;
	var maximumVoltage:Float;
	var maximumCurrent:Float;
	var maximumMicrosteps:Int;
	var maximumStepRate:Float;
	/** Assumed drive position-loop frequency in Hz; zero for stepper drivers. */
	var positionLoopRate:Float;
	var width:Float;
	var height:Float;
	var depth:Float;
}

/** A motor amplifier and its settings. The generic envelopes and ratings are assumptions,
 * not verified product specifications. A stated voltage is used only without a wired supply.
 */
class MotorDriver extends MachineComponent {
	static var table:Null<Catalog<MotorDriverRating>>;
	static var recipe:Null<ComponentType>;
	public final rating:MotorDriverRating;
	public final current:Float;
	public final microsteps:Int;
	public final statedVoltage:Null<Float>;

	public static function catalog():Catalog<MotorDriverRating> {
		if (table == null) table = new Catalog("motor driver", row -> row.designation, [
			{designation: "GENERIC-TMC2209", family: Stepper, control: StepDir,
				minimumVoltage: 5.0, maximumVoltage: 29.0, maximumCurrent: 2.0,
				maximumMicrosteps: 256, maximumStepRate: 250000.0, positionLoopRate: 0.0, width: 20.0, height: 16.0, depth: 12.0},
			{designation: "GENERIC-DM542", family: Stepper, control: StepDir,
				minimumVoltage: 18.0, maximumVoltage: 50.0, maximumCurrent: 3.0,
				maximumMicrosteps: 128, maximumStepRate: 200000.0, positionLoopRate: 0.0, width: 118.0, height: 75.0, depth: 34.0},
			{designation: "GENERIC-SERVO-AMP", family: Servo, control: Bus,
				minimumVoltage: 12.0, maximumVoltage: 60.0, maximumCurrent: 10.0,
				maximumMicrosteps: 1, maximumStepRate: 0.0, positionLoopRate: 4000.0, width: 100.0, height: 60.0, depth: 30.0}
		], row -> ({source: "Assumed generic driver class; ratings and envelope are unverified",
			standard: null, standardEdition: null, dimensionKind: Unverified, conformance: GenericApproximation}));
		return table;
	}

	public function new(designation:String, current:Float, microsteps:Int = 1, ?statedVoltage:Float) {
		var row = catalog().get(designation);
		if (!(current > 0 && current <= row.maximumCurrent) || !Math.isFinite(current))
			throw 'Driver "$designation" current must be positive and within its rating';
		if (microsteps < 1 || microsteps > row.maximumMicrosteps || (microsteps & (microsteps - 1)) != 0)
			throw 'Driver "$designation" needs a supported power-of-two microstep setting';
		if (row.family == Servo && microsteps != 1) throw "A servo driver has no microstep setting";
		if (statedVoltage != null) validateVoltage(row, statedVoltage);
		super('$designation-${Dimension.format(current)}A-M$microsteps' +
			(statedVoltage == null ? "-WIRED" : '-${Dimension.format(statedVoltage)}V'),
			'$designation motor driver, ${Dimension.format(current)} A rms', "aluminium 6061");
		this.rating = row;
		this.current = current;
		this.microsteps = microsteps;
		this.statedVoltage = statedVoltage;
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addPort({name: "power", kind: ElectricalPower, role: Consumer, iface: Unspecified,
			required: statedVoltage == null});
		addPort({name: "motor", kind: ElectricalPower, role: Supply, iface: Unspecified, required: false});
		addPort({name: row.control == StepDir ? "step-dir" : "bus", kind: Signal, role: Consumer,
			iface: Unspecified, required: false});
	}

	public static function validateVoltage(row:MotorDriverRating, voltage:Float):Void {
		if (!Math.isFinite(voltage) || voltage < row.minimumVoltage || voltage > row.maximumVoltage)
			throw 'Driver "${row.designation}" supply is outside ${row.minimumVoltage}–${row.maximumVoltage} V';
	}

	public static function recipeType():ComponentType {
		if (recipe == null) recipe = new ComponentType("machinekit.motion.motor-driver", [
		ComponentRecipeSupport.catalog("designation", catalog(), "GENERIC-TMC2209"),
		ComponentRecipeSupport.scalar("current", 1.68, 0.001),
		ComponentRecipeSupport.count("microsteps", 1),
		ComponentRecipeSupport.flag("wiredSupply", false),
		ComponentRecipeSupport.scalar("voltage", 24, 0.001)
	], v -> new MotorDriver(v.token("designation"), v.number("current"), v.integer("microsteps"),
		v.boolean("wiredSupply") ? null : v.number("voltage")));
		return recipe;
	}

	override public function componentType():Null<ComponentType> return Std.isExactType(this, MotorDriver) ? recipeType() : null;
	override public function values():ComponentValues return new ComponentValues()
		.setToken("designation", rating.designation).setNumber("current", current).setInteger("microsteps", microsteps)
		.setBoolean("wiredSupply", statedVoltage == null).setNumber("voltage", statedVoltage == null ? 24 : statedVoltage)
		.setToken("material", materialSpec());
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part
		return Solids.named(Part.box(rating.width, rating.height, rating.depth), "body");
}
