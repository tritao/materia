package machinekit.motion;

import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport;
import cadkit.threads.ThreadedFace.ThreadStart;

/** Generic SFU1605 screw. Race root and efficiency are assumed reference values.
 * The shaft, support critical speed and transmission use the same screw model as sliding screws.
 */
class BallScrew extends LeadScrew {
	/** Assumed machined bearing journals, shared by support and coupling placement. */
	/** Assumed SFU1605 race root; not a formula for other nominal sizes. */
	public static inline var RACE_ROOT_DIAMETER:Float = 13;
	public static function race():LeadScrewThread return new LeadScrewThread(Ball, 16, 5, 1, RightHand, RACE_ROOT_DIAMETER);
	public static inline var JOURNAL_DIAMETER:Float = 12;
	public static inline var JOURNAL_LENGTH:Float = 25;
	public static inline var INPUT_JOURNAL_LENGTH:Float = 50;
	public final inputJournalLength:Float;
    /** Procurement requirement only; ground does not certify a lead-error grade. */
    public final ground:Bool;
	public function new(length:Float, inputJournalLength:Float = INPUT_JOURNAL_LENGTH, ground:Bool = false) {
		super(race(), length);
		this.inputJournalLength = inputJournalLength;this.ground=ground;
		if (!Math.isFinite(inputJournalLength) || inputJournalLength < JOURNAL_LENGTH) throw "Input journal must accommodate its bearing seat";
		if (length <= inputJournalLength + JOURNAL_LENGTH) throw "Ball screw needs a race between its journals";
		addConnector("retainer", Mount, machinekit.component.Solids.axial(0, 0, BallScrewRetainer.STATION));
	}

	override public function axialRigidityAt(position:Float):Float {
		if (position>=inputJournalLength && position<=totalLength-JOURNAL_LENGTH) return super.axialRigidityAt(position);
		return YOUNGS_MODULUS/1e6*Math.PI*JOURNAL_DIAMETER*JOURNAL_DIAMETER/4.0;
	}

	override public function axialComplianceBetween(from:Float, to:Float):Float {
		if (!Math.isFinite(from) || !Math.isFinite(to) || Math.min(from,to)<0 || Math.max(from,to)>totalLength)
			throw "Axial load path must lie on the ball screw shaft";
		var low = Math.min(from,to), high = Math.max(from,to), compliance = 0.0;
		var boundaries = [0.0,inputJournalLength,totalLength-JOURNAL_LENGTH,totalLength];
		for (index in 0...3) {
			var a = Math.max(low,boundaries[index]), b = Math.min(high,boundaries[index+1]);
			if (b>a) compliance += (b-a)/axialRigidityAt((a+b)*0.5);
		}
		return compliance;
	}

	override public function geometry(detail:machinekit.component.ComponentDetail = Preview):cadkit.modeling.Part {
		var shaft = machinekit.component.Solids.union([
			machinekit.component.Solids.named(detail == Envelope ? cadkit.modeling.Part.cylinderSpan(JOURNAL_DIAMETER / 2, 0, inputJournalLength) : journal(0, inputJournalLength, false), "input-journal"),
			machinekit.component.Solids.named(cadkit.modeling.Part.cylinderSpan(thread.screwDiameter / 2, inputJournalLength, totalLength - JOURNAL_LENGTH), "race"),
			machinekit.component.Solids.named(detail == Envelope ? cadkit.modeling.Part.cylinderSpan(JOURNAL_DIAMETER / 2, totalLength - JOURNAL_LENGTH, totalLength) : journal(totalLength - JOURNAL_LENGTH, totalLength, true), "output-journal")]);
		if (detail == Envelope) return shaft;
		try {
			var result = new cadkit.modeling.Part(cadkit.threads.Threads.attach(
			    shaft.shape, "race:cyl.side", thread.threadSpec(), AxisMinimum));
			shaft.close();
			return result;
		} catch (error:Dynamic) { shaft.close(); throw error; }
	}

	/** Turned reference end details; the stiffness model retains its nominal journal section. */
	static function journal(start:Float, end:Float, floating:Bool):cadkit.modeling.Part {
		var section = floating ? [
			{r:0.0,z:start}, {r:5.7,z:start}, {r:5.7,z:start+0.8}, {r:6.0,z:start+1.2},
			{r:6.0,z:end-4.2}, {r:5.6,z:end-4.2}, {r:5.6,z:end-3.2}, {r:6.0,z:end-3.2},
			{r:6.0,z:end-0.6}, {r:5.4,z:end}, {r:0.0,z:end}
		] : [
			{r:0.0,z:start}, {r:5.4,z:start}, {r:6.0,z:start+0.6},
			{r:6.0,z:end-1.2}, {r:5.7,z:end-0.8}, {r:5.7,z:end}, {r:0.0,z:end}
		];
		return cadkit.modeling.Part.revolve(section);
	}

	public function retainer():BallScrewRetainer return new BallScrewRetainer();

	public static function sfu1605(length:Float):BallScrew return new BallScrew(length);

	static var ballRecipe:Null<ComponentType>;
	public static function ballRecipeType():ComponentType {
		if (ballRecipe == null) ballRecipe = new ComponentType("machinekit.motion.ball-screw",
			[ComponentRecipeSupport.length("length", 400),ComponentRecipeSupport.length("inputJournalLength",INPUT_JOURNAL_LENGTH),ComponentRecipeSupport.flag("ground",false)], v -> new BallScrew(v.number("length"),v.number("inputJournalLength"),v.boolean("ground")));
		return ballRecipe;
	}
	override public function componentType():Null<ComponentType> return ballRecipeType();
	override public function values():ComponentValues return new ComponentValues()
		.setNumber("length", totalLength).setNumber("inputJournalLength",inputJournalLength).setBoolean("ground",ground).setToken("material", materialSpec());
}
