import robotkit.model.RobotModel;
import robotkit.model.Link;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.JointCoupling;
import robotkit.model.Actuator;
import robotkit.model.Transmission;
import robotkit.model.ElasticNetwork;
import robotkit.model.DriveLoads;
import robotkit.model.DriveLoads.AxisLoad;
import robotkit.model.ElasticSolve;

/** Analytic span energies, shared loads and the series reduction limit. */
class BeltElasticityTests {
	public static function run():Void {
		function near(actual:Float, expected:Float, label:String):Void {
			if (Math.abs(actual - expected) > 1e-9) throw '$label: $actual versus $expected';
		}
		function model(ids:Array<String>):RobotModel {
			var value = new RobotModel("belt-span-tests"), base = value.addLink(new Link("base"));
			for (id in ids) value.addJoint(new Joint(id, JointType.Continuous, base, value.addLink(new Link(id + ".body"))));
			return value;
		}
		function hold(value:RobotModel, id:String):Void {
			value.addActuator(new Actuator(id + ".motor", 1, 10, SimpleTransmission(id, 1, 0)));
		}
		var value = model(["a", "b", "motor"]);
		hold(value, "motor");
		var k = 1200.0;
		value.elasticNetworks.push(new ElasticNetwork("loop", [], [
			{stiffness: k, terms: [{joint: "motor", coefficient: 1.0}, {joint: "a", coefficient: -1.0}]},
			{stiffness: k, terms: [{joint: "a", coefficient: 1.0}, {joint: "b", coefficient: -1.0}]},
			{stiffness: k, terms: [{joint: "b", coefficient: 1.0}, {joint: "motor", coefficient: -1.0}]}]));
		var axes = [new AxisLoad("a", false, 1, 0, false, 0), new AxisLoad("b", false, 1, 0, false, 0)];
		var solved = new ElasticSolve(value, axes);
		near(solved.compliance[0][0], 2 / (3 * k), "free-output stiffness is 1.5 k");
		near(solved.compliance[0][1], 1 / (3 * k), "shared span carries cross-output stretch");
		near(solved.compliance[1][0], solved.compliance[0][1], "span energy is symmetric");
		near(solved.compliance[0][0] + solved.compliance[0][1], 1 / k, "equal output loads include shared stretch");
		near(2 * solved.compliance[0][0] + solved.compliance[0][1], 5 / (3 * k), "unequal output loads combine before solving");
		var restored = robotkit.model.RobotModelCodec.decode(robotkit.model.RobotModelCodec.encode(value));
		near(new ElasticSolve(restored, axes).compliance[0][1], 1 / (3 * k), "saved RobotKit network retains shared energy");
		value.elasticNetworks[0].clearances.push({joint: "a", allowance: 0.002});
		var withClearance = new ElasticSolve(value, axes);
		near(withClearance.backlash[0], 0.002, "one contact shifts its adjoining spans together");
		near(withClearance.backlash[1], 0, "a contact allowance is not duplicated at another output");
		var noClearances:Array<robotkit.model.ElasticNetwork.ElasticClearance> = [];
		value.elasticNetworks[0].clearances = noClearances;
		value.elasticNetworks[0].spans.reverse();
		near(new ElasticSolve(value, axes).compliance[0][1], 1 / (3 * k), "span order cannot change compliance");

		var common = model(["z", "a", "b", "motor"]);
		common.elasticNetworks.push(value.elasticNetworks[0]);
		common.addCoupling(new JointCoupling("screw-a", "z", "a", 2, 0));
		common.addCoupling(new JointCoupling("screw-b", "z", "b", 2, 0));
		hold(common, "motor");
		near(new ElasticSolve(common, [new AxisLoad("z", false, 1, 0, false, 0)]).compliance[0][0],
			1 / (8 * k), "two constrained screws combine their actual span energies");

		var series = model(["axis", "shaft", "motor"]);
		var carriage = series.addCoupling(new JointCoupling("carriage", "axis", "shaft", 2, 0));
		carriage.stiffness = 1000;
		series.addCoupling(new JointCoupling("reduction", "shaft", "motor", 3, 0));
		series.elasticNetworks.push(new ElasticNetwork("reduction-belt", ["reduction"], [
			{stiffness: 4000, terms: [{joint: "shaft", coefficient: 1.0}, {joint: "motor", coefficient: -1.0 / 3}]}]));
		hold(series, "motor");
		var load = DriveLoads.forAxis(series, "axis");
		if (load == null) throw "Series belt drive has no axis";
		near(1 / load.stiffness, 1.0 / 1000 + 1.0 / (4000 * 4), "belt springs add in series with squared ratio scaling");
		// The scalar report is deliberately wrong: the shared network owns this spring.
		series.couplings[1].stiffness = 1;
		var changed = DriveLoads.forAxis(series, "axis");
		if (changed == null) throw "Series drive disappeared";
		near(changed.stiffness, load.stiffness, "network ownership prevents double-counting scalar reports");

		var mixed = model(["screw", "shaft", "motor", "orphan", "orphanOutput"]);
		var reduction = mixed.addCoupling(new JointCoupling("reduction", "shaft", "motor", 2, 0));
		mixed.elasticNetworks.push(new ElasticNetwork("driven-belt", ["reduction"], [
			{stiffness: 4000, terms: [{joint: "shaft", coefficient: 1.0}, {joint: "motor", coefficient: -0.5}]}]));
		mixed.elasticNetworks[0].assumptions.push({quantity: "stiffness", label: "belt stiffness with pretension"});
		mixed.elasticNetworks.push(new ElasticNetwork("unbound-belt", [], [
			{stiffness: 4000, terms: [{joint: "orphan", coefficient: 1.0}, {joint: "orphanOutput", coefficient: -1.0}]}]));
		hold(mixed, "screw"); hold(mixed, "motor");
		var mixedLoads = DriveLoads.of(mixed);
		if (mixedLoads.length != 2) throw "Mixed machine needs its screw and belt axes";
		var screw:Null<AxisLoad> = null, shaft:Null<AxisLoad> = null;
		for (item in mixedLoads) if (item.axis == "screw") screw = item else if (item.axis == "shaft") shaft = item;
		if (screw == null || shaft == null || screw.stiffness != 0 || shaft.stiffness <= 0)
			throw "Independent screw must stay rigid beside a shaft-belt network";
		for (assumption in screw.assumptions) if (assumption.label == "belt stiffness with pretension")
			throw "Belt assumptions leaked onto an independent screw";
		var screwOnly = DriveLoads.of(mixed, null, ["screw"]);
		if (screwOnly.length != 1 || screwOnly[0].elastic == null) throw "Partial plan must retain its own compliance";
		near(screwOnly[0].elastic.deflections([10])[0], 0, "held independent screw has no belt deflection");
		var wrongSize = false;
		try screwOnly[0].elastic.deflections([]) catch (_:Dynamic) wrongSize = true;
		if (!wrongSize) throw "Compliance must reject a partial force vector of the wrong size";

		var summed = model(["out", "first", "second"]);
		var first = summed.addCoupling(new JointCoupling("first-path", "first", "out", 1, 0));
		first.backlash = 0.1;
		var second = summed.addCoupling(new JointCoupling("second-path", "second", "out", 1, 0));
		second.backlash = 0.2;
		summed.elasticNetworks.push(new ElasticNetwork("other-belt", [], [
			{stiffness: 4000, terms: [{joint: "first", coefficient: 1.0}, {joint: "second", coefficient: -1.0}]}]));
		hold(summed, "first"); hold(summed, "second");
		near(new ElasticSolve(summed, [new AxisLoad("out", false, 1, 0, false, 0)]).backlash[0], 0.2,
			"parallel coupling lost motion takes its largest bound");

		var partial = model(["x", "y", "motor", "screw", "screwMotor"]);
		partial.addCoupling(new JointCoupling("x-to-motor", "x", "motor", 2, 0));
		partial.addCoupling(new JointCoupling("y-to-motor", "y", "motor", 2, 0));
		partial.elasticNetworks.push(new ElasticNetwork("one-belt", [], [
			{stiffness: 4000, terms: [{joint: "x", coefficient: 1.0}, {joint: "motor", coefficient: -0.5}]}]));
		var screwPath = partial.addCoupling(new JointCoupling("screw-path", "screw", "screwMotor", 2, 0));
		screwPath.stiffness = 1000;
		hold(partial, "motor"); hold(partial, "screwMotor");
		var partialLoads = DriveLoads.of(partial);
		if (partialLoads.length != 3 || partialLoads[0].stiffness != 0 || partialLoads[1].stiffness != 0 ||
			partialLoads[2].stiffness <= 0 || partialLoads[2].elastic == null)
			throw "A partly bound coupled pair must not suppress its independent screw's stiffness";
		var partialErrors = partial.validate();
		if (partialErrors.length == 0 || partialErrors[0].indexOf("under-actuated") < 0)
			throw 'A partly bound coupled pair needs a design diagnostic: $partialErrors';
		if (!(partial.coupledLimits("x").requireEffort() > 0 && partial.coupledLimits("y").requireEffort() > 0))
			throw "Partly bound coupled axes retain the per-axis limit of their bound motor";
	}
}
