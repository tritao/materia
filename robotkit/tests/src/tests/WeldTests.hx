package tests;

import robotkit.tool.ConvexSolid;
import robotkit.tool.GroundedWork;
import robotkit.tool.WeldArcModel.WeldArcConfig;
import robotkit.tool.WeldArcModel;
import robotkit.tool.WeldChannelPolicy;
import robotkit.tool.WeldFault;
import robotkit.tool.WeldSensor.WeldReading;
import robotkit.tool.WeldSensor;

/**
 * The welder's arc model and the geometry it asks, with no physics backend: a steel plate whose top is the plane
 * z = 0, a wire pointing down at it, and the tip's height as the variable.
 */
class WeldTests {
  static var assertions = 0;
  static inline var STEP = 0.01;

  public static function run():Int {
    assertions = 0;
    testSolidDistanceAndRay();
    testHullFromVertices();
    testWorkFollowsItsBody();
    testArcStrikesNearWorkOnly();
    testIgnitionDelay();
    testCurrentAndVoltageFollowTheModel();
    testArcLostWhenPulledAway();
    testTouchWithArcOff();
    testNoArcFault();
    testPowerFollowsEfficiency();
    testArcOffOnCommandAndStuckWire();
    testBurnbackAndSupply();
    testChannelsGoSafeOnStop();
    testSensorFrame();
    Sys.println('RobotKit weld tests passed ($assertions assertions)');
    return assertions;
  }

  static function config(?maxCurrent:Float = 350, ?efficiency:Float = 0.88, ?wire:Float = 1.2, ?stickout:Float = 15):WeldArcConfig
    return {maxCurrentA: maxCurrent, efficiency: efficiency, wireDiameterMm: wire, stickoutMm: stickout};

  /** The plate: 20 cm square, 1 cm thick, its top at z = 0. */
  static function plate():GroundedWork
    return new GroundedWork().addFixed(ConvexSolid.box(-0.1, -0.1, -0.01, 0.1, 0.1, 0.0));

  /** One step of the model with the wire tip `height` metres above the plate's top, pointing down. */
  static function step(model:WeldArcModel, work:GroundedWork, height:Float, arc:Bool, ?wire:Float = 8.0, ?volts:Float = 24.0,
      ?ready:Bool = true, ?x:Float = 0.0, ?dt:Float = STEP):WeldReading {
    return model.step(dt, {arcCommanded: arc, wireSpeed: wire, voltageSet: volts, supplyReady: ready,
      tipDistance: work.distance(x, 0.0, height), wireDistance: work.ray(x, 0.0, height, 0.0, 0.0, -1.0, WeldArcModel.STRIKE_REACH)});
  }

  /** Runs `seconds` of steps at a height and returns the last reading. */
  static function hold(model:WeldArcModel, work:GroundedWork, height:Float, arc:Bool, seconds:Float, ?wire:Float = 8.0,
      ?volts:Float = 24.0):WeldReading {
    var reading = model.reading;
    for (i in 0...Math.round(seconds / STEP)) reading = step(model, work, height, arc, wire, volts);
    return reading;
  }

  static function testSolidDistanceAndRay():Void {
    var box = ConvexSolid.box(-1, -1, -1, 1, 1, 1);
    near(box.distance(0, 0, 3), 2.0, "distance in front of a face", 1e-9);
    near(box.distance(0, 0, 0), -1.0, "depth at the centre", 1e-9);
    near(box.distance(0, 0, 1), 0.0, "zero on the surface", 1e-9);
    near(box.ray(0, 0, 4, 0, 0, -1, 10), 3.0, "a ray straight down meets the top", 1e-9);
    near(box.ray(0, 0, 0, 0, 0, -1, 10), 0.0, "a ray from inside meets it at once", 1e-9);
    check(box.ray(0, 0, 4, 0, 0, -1, 2) == Math.POSITIVE_INFINITY, "a face beyond the range is not met");
    check(box.ray(0, 0, 4, 0, 0, 1, 10) == Math.POSITIVE_INFINITY, "a ray pointing away misses");
    check(box.ray(5, 0, 4, 0, 0, -1, 10) == Math.POSITIVE_INFINITY, "a ray beside it misses");
    var slant = 1 / Math.sqrt(2);
    near(box.ray(2, 0, 2, -slant, 0, -slant, 10), Math.sqrt(2), "a diagonal ray meets the corner edge", 1e-9);
  }

  static function testHullFromVertices():Void {
    // A wedge (a triangular prism): its slanted face is found from the vertices.
    var wedge = new ConvexSolid([0, 0, 0, 2, 0, 0, 0, 2, 0, 0, 0, 1, 2, 0, 1, 0, 2, 1]);
    near(wedge.distance(-1, 0.5, 0.5), 1.0, "distance to the wedge's end", 1e-9);
    check(wedge.distance(0.5, 0.5, 0.5) < 0, "a point inside the wedge is inside");
    check(wedge.distance(1.8, 1.8, 0.5) > 0, "a point past the wedge's slanted face is outside");
    near(wedge.distance(1.5, 1.5, 0.5), Math.sqrt(0.5), "distance to the slant", 1e-9);
    // A hull of more than EXACT_LIMIT vertices is boxed, which only grows it.
    var many:Array<Float> = [];
    for (i in 0...9) for (j in 0...9) for (z in [0.0, 1.0]) many = many.concat([i * 0.1, j * 0.1, z]);
    var big = new ConvexSolid(many);
    check(big.distance(0.4, 0.4, 2.0) > 0.99 && big.distance(0.4, 0.4, 0.5) < 0, "a large hull is its bounding box");
    var flat = false;
    try new ConvexSolid([0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0]) catch (_:Dynamic) flat = true;
    check(flat, "a flat set of vertices is no solid");
  }

  static function testWorkFollowsItsBody():Void {
    var body = {x: 0.0, turn: 0.0};
    var work = new GroundedWork().add(ConvexSolid.box(-1, -1, -1, 1, 1, 1), function() return {position: [body.x, 0.0, 0.0],
      rotation: [0.0, 0.0, Math.sin(body.turn / 2), Math.cos(body.turn / 2)]});
    near(work.distance(3, 0, 0), 2.0, "the work is where its body is", 1e-9);
    body.x = 4;
    near(work.distance(4, 0, 0), -1.0, "the work moved with its body: its centre is now at 4", 1e-9);
    body.x = 0;
    body.turn = Math.PI / 2;
    // A box [-1, 1] x [-2, 2] turned a quarter about Z reaches 2 along X rather than Y.
    work = new GroundedWork().add(ConvexSolid.box(-1, -2, -1, 1, 2, 1), function() return {position: [body.x, 0.0, 0.0],
      rotation: [0.0, 0.0, Math.sin(body.turn / 2), Math.cos(body.turn / 2)]});
    near(work.distance(2, 0, 0), 0.0, "the turned box reaches 2 along X", 1e-9);
    near(work.distance(0, 2, 0), 1.0, "and only 1 along Y", 1e-9);
    near(work.ray(5, 0, 0, -1, 0, 0, 10), 3.0, "a ray meets the turned box", 1e-9);
    check(new GroundedWork().distance(0, 0, 0) == Math.POSITIVE_INFINITY, "no work is infinitely far");
  }

  static function testArcStrikesNearWorkOnly():Void {
    var work = plate();
    var close = new WeldArcModel(config());
    var reading = hold(close, work, 0.002, true, 0.3);
    check(reading.arc && reading.fault == 0, "the arc strikes with the wire tip 2 mm from the work");
    var far = new WeldArcModel(config());
    reading = hold(far, work, 0.05, true, 0.5);
    check(!reading.arc && reading.currentA == 0.0, "no arc in the air, 50 mm from the work");
    var offPlate = new WeldArcModel(config());
    for (i in 0...30) reading = step(offPlate, work, 0.002, true, 8.0, 24.0, true, 0.5);
    check(!reading.arc, "no arc beside the plate: the work is not under the wire");
    var reach = new WeldArcModel(config());
    reading = hold(reach, work, WeldArcModel.STRIKE_REACH + 0.002, true, 0.3);
    check(!reading.arc, "the wire does not reach work farther than the strike reach");
  }

  static function testIgnitionDelay():Void {
    var work = plate();
    var model = new WeldArcModel(config());
    var established = -1.0;
    var time = 0.0;
    for (i in 0...30) {
      var reading = step(model, work, 0.001, true);
      time += STEP;
      if (reading.arc && established < 0) established = time;
    }
    check(established >= WeldArcModel.IGNITION_DELAY - 1e-9 && established <= WeldArcModel.IGNITION_DELAY + STEP + 1e-9,
      'the arc lights after the ignition delay, got $established s');
    // The delay restarts when the wire leaves the work before it is over.
    var restart = new WeldArcModel(config());
    for (i in 0...5) step(restart, work, 0.001, true);
    step(restart, work, 0.1, true);
    var reading = restart.reading;
    for (i in 0...5) reading = step(restart, work, 0.001, true);
    check(!reading.arc, "the ignition delay starts over after the wire left the work");
  }

  static function testCurrentAndVoltageFollowTheModel():Void {
    var work = plate();
    var model = new WeldArcModel(config());
    var reading = hold(model, work, 0.0, true, 0.3, 8.0, 24.0);
    near(reading.currentA, 250.0, "8 m/min of 1.2 mm wire at 15 mm stickout is about 250 A", 1.0);
    near(reading.voltageV, 24.0, "the voltage is the setpoint at the nominal arc length", 1e-9);
    // Burn-off law: the current gives back the wire speed.
    var rate = 0.27 * reading.currentA + 7e-5 * 15 * reading.currentA * reading.currentA;
    near(rate * 60 / 1000, 8.0, "the current burns off the wire that is fed", 1e-6);
    var slow = new WeldArcModel(config());
    var slowReading = hold(slow, work, 0.0, true, 0.3, 4.0, 20.0);
    check(slowReading.currentA < reading.currentA - 50 && slowReading.currentA > 100, 'slower wire, less current, got ${slowReading.currentA}');
    near(slowReading.voltageV, 20.0, "the voltage follows the setpoint", 1e-9);
    var lifted = new WeldArcModel(config());
    var liftedReading = hold(lifted, work, 0.004, true, 0.3, 8.0, 24.0);
    near(liftedReading.voltageV, 24.0 + 1.2 * 4.0, "lifting the torch 4 mm raises the voltage 1.2 V per mm", 1e-6);
    near(liftedReading.currentA, reading.currentA, "the current is the wire speed's", 1e-9);
    var capped = new WeldArcModel(config(200));
    near(hold(capped, work, 0.0, true, 0.3, 8.0, 24.0).currentA, 200.0, "the supply's rating caps the current", 1e-9);
    var thin = new WeldArcModel(config(350, 0.88, 0.8));
    var thinReading = hold(thin, work, 0.0, true, 0.3, 8.0, 24.0);
    check(thinReading.currentA < reading.currentA, 'thinner wire burns off at less current, got ${thinReading.currentA}');
    var long = new WeldArcModel(config(350, 0.88, 1.2, 25));
    check(hold(long, work, 0.0, true, 0.3, 8.0, 24.0).currentA < reading.currentA, "a longer stickout burns off at less current");
    near(new WeldArcModel(config()).currentFor(0.0), 0.0, "no wire, no current", 0);
  }

  static function testArcLostWhenPulledAway():Void {
    var work = plate();
    var model = new WeldArcModel(config());
    var reading = hold(model, work, 0.002, true, 0.3);
    check(reading.arc, "the arc is established");
    reading = hold(model, work, 0.006, true, 0.1);
    check(reading.arc, "the arc stretches to 9 mm and holds");
    reading = hold(model, work, 0.012, true, 0.05);
    check(!reading.arc && reading.fault == WeldFault.ArcLost && reading.currentA == 0.0, "the arc is lost past the longest arc length");
    reading = hold(model, work, 0.0, true, 0.5);
    check(!reading.arc && reading.fault == WeldFault.ArcLost, "the fault holds while the arc is still commanded");
    reading = hold(model, work, 0.002, false, 0.05);
    check(reading.fault == 0, "commanding the arc off clears the fault");
    reading = hold(model, work, 0.002, true, 0.3);
    check(reading.arc, "and it strikes again");
  }

  static function testTouchWithArcOff():Void {
    var work = plate();
    var model = new WeldArcModel(config());
    var reading = step(model, work, 0.0002, false, 0.0, 24.0);
    check(reading.touch && !reading.arc && reading.voltageV == 0.0, "the wire on the work closes the touch and collapses the sensing voltage");
    reading = step(model, work, 0.01, false, 0.0, 24.0);
    check(!reading.touch && reading.voltageV == WeldArcModel.SENSE_VOLTAGE, "10 mm off the work the circuit is open at the sensing voltage");
    reading = step(model, work, -0.002, false, 0.0, 24.0);
    check(reading.touch, "a wire pressed into the work touches it");
    reading = step(model, work, 0.0002, false, 0.0, 24.0, false);
    check(!reading.touch, "with the supply off there is no sensing voltage and no touch");
    reading = step(model, work, 0.0002, false, 0.0, 24.0, true, 0.5);
    check(!reading.touch, "beside the plate there is nothing to touch");
    var striking = new WeldArcModel(config());
    reading = step(striking, work, 0.0, true);
    check(reading.touch && !reading.arc, "touch is closed while the arc is commanded but not yet lit");
    reading = hold(striking, work, 0.0, true, 0.2);
    check(reading.arc && !reading.touch, "and open once the arc burns");
  }

  static function testNoArcFault():Void {
    var work = plate();
    var model = new WeldArcModel(config());
    var reading = hold(model, work, 0.05, true, WeldArcModel.NO_ARC_TIMEOUT - 0.05);
    check(reading.fault == 0, "no fault before the timeout");
    reading = hold(model, work, 0.05, true, 0.1);
    check(reading.fault == WeldFault.NoArc && !reading.arc, "no arc within the timeout is a fault");
    check(WeldSensor.faultMessage(reading.fault) != null, "the fault has words");
    reading = hold(model, work, 0.0, true, 0.5);
    check(!reading.arc && reading.fault == WeldFault.NoArc, "the fault keeps the arc off, though the wire is at the work now");
    reading = hold(model, work, 0.0, false, 0.02);
    check(reading.fault == 0, "arc off clears it");
    reading = hold(model, work, 0.0, true, 0.3);
    check(reading.arc, "and a new attempt strikes");
  }

  static function testPowerFollowsEfficiency():Void {
    var work = plate();
    for (efficiency in [0.88, 0.75, 1.0]) {
      var model = new WeldArcModel(config(350, efficiency));
      var reading = hold(model, work, 0.003, true, 0.3, 8.0, 24.0);
      check(reading.arc, "the arc burns");
      near(reading.powerW, reading.currentA * reading.voltageV / efficiency, "the mains power is I·U/η at η = " + efficiency, 1e-6);
    }
    var idle = new WeldArcModel(config());
    near(hold(idle, work, 0.05, true, 0.2).powerW, 0.0, "no arc draws no arc power", 0);
  }

  static function testArcOffOnCommandAndStuckWire():Void {
    var work = plate();
    var model = new WeldArcModel(config());
    var reading = hold(model, work, 0.002, true, 0.3);
    check(reading.arc, "the arc burns");
    // Stop the wire, then the arc: burnback frees it.
    reading = step(model, work, 0.002, true, 0.0);
    check(!reading.arc && reading.fault == 0, "stopping the wire burns the arc back and out, without a fault");
    reading = step(model, work, 0.002, false, 0.0);
    check(!reading.arc && reading.fault == 0, "so the arc command can follow without sticking the wire");
    // The arc commanded off with the wire still feeding and the tip on the work freezes it in the pool.
    var stuck = new WeldArcModel(config());
    hold(stuck, work, 0.0, true, 0.3);
    reading = step(stuck, work, 0.0, false, 8.0);
    check(!reading.arc && reading.fault == WeldFault.WireStuck && reading.touch, "arc off with the wire feeding on the work sticks the wire");
    reading = step(stuck, work, 0.0, false, 0.0);
    check(reading.fault == WeldFault.WireStuck, "it stays stuck while the torch is on the work");
    reading = step(stuck, work, 0.02, false, 0.0);
    check(reading.fault == 0 && !reading.touch, "and clears once the torch is freed");
    // With the tip away from the work when the arc goes off, nothing sticks.
    var clear = new WeldArcModel(config());
    hold(clear, work, 0.004, true, 0.3);
    reading = step(clear, work, 0.004, false, 8.0);
    check(!reading.arc && reading.fault == 0, "arc off with the wire clear of the work does not stick it");
  }

  static function testBurnbackAndSupply():Void {
    var work = plate();
    var model = new WeldArcModel(config());
    hold(model, work, 0.002, true, 0.3);
    var reading = step(model, work, 0.002, true, 8.0, 24.0, false);
    check(!reading.arc && reading.fault == WeldFault.ArcLost, "the supply dropping out loses the arc");
    var down = new WeldArcModel(config());
    reading = hold(down, work, 0.002, true, 0.3, 0.5);
    check(!reading.arc, "the wire has to feed to strike an arc");
  }

  static function testChannelsGoSafeOnStop():Void {
    check(!WeldChannelPolicy.ARC_KEEPS_ON_STOP, "the arc channel does not keep its output through a stop: it goes safe, off");
    check(!WeldChannelPolicy.WIRE_SPEED_KEEPS_ON_STOP, "the wire stops on a stop");
    check(WeldChannelPolicy.VOLTAGE_KEEPS_ON_STOP, "the voltage setpoint is kept");
    // The model has no output of its own: with the arc channel back at its safe value the arc is off.
    var work = plate();
    var model = new WeldArcModel(config());
    hold(model, work, 0.002, true, 0.3);
    var reading = step(model, work, 0.002, false, 0.0);
    check(!reading.arc && reading.fault == 0 && reading.currentA == 0.0, "the arc goes out when its channel returns to off");
  }

  static function testSensorFrame():Void {
    var work = plate();
    var model = new WeldArcModel(config());
    var reading = hold(model, work, 0.003, true, 0.3);
    var values = WeldSensor.values(reading);
    check(values.length == WeldSensor.COUNT && WeldSensor.valid(values), "a reading is a valid six-value frame");
    check(values[WeldSensor.ARC] == 1.0 && values[WeldSensor.CURRENT] == reading.currentA &&
      values[WeldSensor.VOLTAGE] == reading.voltageV && values[WeldSensor.POWER] == reading.powerW, "the values are in the documented order");
    var back = WeldSensor.reading(values);
    check(back.arc && back.currentA == reading.currentA && back.fault == 0 && !back.touch, "a frame reads back as the reading");
    check(!WeldSensor.valid([1.0, 0.0]) && !WeldSensor.valid([2.0, 0, 0, 0, 0, 0]) && !WeldSensor.valid([0.0, -1, 0, 0, 0, 0]) &&
      !WeldSensor.valid([0.0, 0, 0, 0, 7, 0]), "malformed frames are refused");
  }

  static function near(actual:Float, expected:Float, message:String, tolerance:Float):Void {
    assertions++;
    if (!(Math.abs(actual - expected) <= tolerance)) throw '$message: expected $expected, got $actual';
  }

  static function check(value:Bool, message:String):Void {
    assertions++;
    if (!value) throw message;
  }
}
