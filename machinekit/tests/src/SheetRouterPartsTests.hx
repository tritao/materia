import machinekit.component.*;
import machinekit.catalog.CatalogMetadata.Conformance;
import machinekit.motion.*;
import machinekit.transmission.*;
import machinekit.routing.*;
import machinekit.power.*;
import machinekit.control.*;
import machinekit.structural.LevelingFoot;

/** SR1 tests: engineering identities, assembly interfaces, recipes, and valid owned solids. */
class SheetRouterPartsTests {
  static var assertions = 0;
  static function check(ok:Bool, message:String):Void {
    assertions++;
    if (!ok) throw message;
  }
  static function near(a:Float, b:Float, tol:Float, label:String):Void check(
    Math.isFinite(a) && Math.abs(a - b) <= tol,
    label + ': ' + a + ' vs ' + b
  );
  static function reject(fn:Void -> Void, label:String):Void {
    var failed = false;
    try fn() catch (_:Dynamic) failed = true;
    check(failed, label);
  }
  public static function main():Void {
    var types = [
      PlanetaryReducer.recipeType(),
      CableChain.recipeType(),
      BrakeServo.recipeType(),
      HelicalRack.recipeType(),
      HelicalPinion.recipeType(),
      PreloadedPinionMount.recipeType(),
      AtcSpindle.recipeType(),
      ToolHolder.recipeType(),
      Collet.recipeType(),
      ToolFork.recipeType(),
      ToolRack.recipeType(),
      ToolLengthSetter.recipeType(),
      DustShoe.recipeType(),
      VacuumTable.recipeType(),
      Spoilboard.recipeType(),
      VacuumBlower.recipeType(),
      VacuumZoneValve.recipeType(),
      Vfd.recipeType(),
      BrakeResistor.recipeType(),
      Contactor.recipeType(),
      CircuitBreaker.recipeType(),
      SafetyRelay.recipeType(),
      EmergencyStopButton.recipeType(),
      ControlCabinet.recipeType(),
      LevelingFoot.recipeType()
    ];
    var direct:Array<AssumedCatalogComponent> = [new PlanetaryReducer(),new CableChain(),new BrakeServo(),
      new HelicalRack(),new HelicalPinion(),new PreloadedPinionMount(),new AtcSpindle(),new ToolHolder(),new Collet(),
      new ToolFork(),new ToolRack(),new ToolLengthSetter(),new DustShoe(),new VacuumTable(),new Spoilboard(),
      new VacuumBlower(),new VacuumZoneValve(),new Vfd(),new BrakeResistor(),new Contactor(),new CircuitBreaker(),
      new SafetyRelay(),new EmergencyStopButton(),new ControlCabinet(),new LevelingFoot()];
    for(part in direct) {
      var copy=part.componentType().create(part.values());
      check(part.designation==copy.designation,'direct constructor identity '+part.designation);
    }
    var registry = MachineKitComponents.standard();
    for (type in types) {
      check(registry.has(type.id), 'registered ' + type.id);
      var p = type.create(), q = registry.byId(type.id).create(p.values());
      check(p.bom.partNumber == q.bom.partNumber && p.bom.valuesKey == q.bom.valuesKey, 'BOM rebuild ' + type.id);
      var assumed:AssumedCatalogComponent = cast p;
      check(assumed.ratingsBasis == Assumed, 'assumed ' + type.id);
      check(p.connectors().length > 0, 'frames ' + type.id);
      Sys.println('SR1 geometry: '+type.id);
      var g = p.geometry();
      check(g.valid(), 'valid geometry ' + type.id);
      check(g.volume() > 0, 'nonempty ' + type.id);
      g.close();
      check(p.bom.quantity == 1, 'BOM quantity ' + type.id);
      // An actual assembly decode must reconstruct the same recipe and frames.
      var assembly = new machinekit.assembly.MachineAssembly();
      assembly.addComponent('part', p);
      var rebuilt = machinekit.assembly.MachineAssembly.decode(assembly.encode(), registry);
      check(rebuilt.billOfMaterials().lines().length == 1, 'assembly decode ' + type.id);
    }
    var beta = Math.acos(3 * Math.PI / 10), values = new ComponentValues().setNumber('helixAngle', beta);
    var pinion = new HelicalPinion(values), rack = new HelicalRack(values);
    near(pinion.travelPerRevolution(), 100, 1e-9, '100 mm/rev normal-module reference');
    near(pinion.axialThrust(300), 300 * Math.tan(beta), 1e-9, 'axial thrust');
    var left = new HelicalPinion(values.copy().setToken('hand', 'left'));
    near(left.axialThrust(300), -pinion.axialThrust(300), 1e-9, 'opposite hand thrust');
    near(rack.relation(pinion).ratio * 100, 2 * Math.PI, 1e-9, 'rack transmission');
    check(rack.relation(pinion).basis('stiffness') == Assumed, 'mesh stiffness assumption');
    check(pinion.contactRatio() > 1 && pinion.overlapRatio() > 0, 'continuous tooth contact');
    near(rack.length(), 100, 1e-9, 'rack joins at whole pitch');
    reject(() -> rack.relation(left), 'same-hand mismatch');
    reject(
      () -> rack.relation(new HelicalPinion(new ComponentValues().setNumber('normalModule', 2))),
      'module mismatch'
    );
    reject(() -> new HelicalPinion(new ComponentValues().setNumber('helixAngle', Math.NaN)), 'nonfinite angle');
    reject(() -> new HelicalPinion(new ComponentValues().setNumber('bore', 100)), 'oversize bore');
    var mount = new PreloadedPinionMount();
    near(mount.preload(), 490, 1e-9, 'spring moment balance');
    check(mount.resists(400) && !mount.resists(500), 'separating force bound');
    reject(() -> mount.preload(5), 'spring stroke bound');
    var reducer = new PlanetaryReducer();
    near(reducer.jointTorque(2), 9.5, 1e-9, 'reducer output torque');
    near(reducer.backlashRadians(), Math.PI / 2160, 1e-12, 'arcmin to rad');
    near(reducer.outputDeflection(12), 0.001, 1e-12, 'torsional deflection');
    for (profile in['HGH20CA', 'HGH25CA']) {
      var spec=LinearRailSystem.catalog().get(profile),rating=LinearRailSystem.rating(spec);
      check(rating.source=='Vallder' && !rating.original && rating.staticLoad>0 && spec.ratings.length==3,'sourced rail ratings '+profile);
      var rail = LinearRailSystem.forProfile(profile, 1000, 2);
      check(rail.bom().lines().length == 2, 'rail/block BOM ' + profile);
      for (entry in rail.components()) {
        var g = entry.component.geometry();
        check(g.valid(), 'rail geometry ' + profile);
        g.close();
      }
    }
    var brake = new BrakeServo();
    check(brake.motorBrakeSpec().holdingTorque() > brake.motor.rating.ratedTorque, 'brake holding torque');
    var largeBrake = new BrakeServo(new ComponentValues().setToken('power', '750'));
    check(largeBrake.motorDrive().actuator("motor", "joint", 400, 1).drive == "servo", '750 W drive available');
    var chain = new CableChain();
    var developed = chain.developedLength();
    for (pos in[0.0, 700.0, 1400.0, 2800.0]) {
      var path = chain.path(pos);
      var endpoint = path[path.length - 1];
      var frame = chain.movingFrame(pos);
      var measured = 0.0;
      near(endpoint.y, frame.y, 1e-9, 'chain follows moving end');
      near(endpoint.z, frame.z, 1e-9, 'chain return height');
      for (i in 1...path.length) {
        var a = path[i - 1], b = path[i];
        measured += Math.sqrt(Math.pow(b.y - a.y, 2) + Math.pow(b.z - a.z, 2));
      }
      near(measured, developed, 0.1, 'constant chain length across travel');
    }
    for(position in [0.0,2800.0]) {
      var posed=chain.geometryAt(position);check(posed.valid(),'chain endpoint geometry '+position);posed.close();
    }
    var clearance=chain.geometry(Envelope);check(clearance.valid(),'chain travel envelope');clearance.close();
    reject(() -> chain.path(2801), 'chain travel exceeded');
    reject(() -> new CableChain(new ComponentValues().setNumber('fillArea', 1000)), 'chain fill exceeded');
    near(new ToolHolder().connector('fork').frame.z,-8,1e-9,'fork supports flange underside');
    var holder=new ToolHolder(),collet=new Collet(),h=holder.geometry(),c=collet.geometry();
    var placed=c.translated(new cadkit.modeling.Vector(0,0,holder.connector('collet').frame.z-collet.connector('holder').frame.z));c.close();
    var overlap=h.intersect(placed);check(overlap.isEmpty(),'ER32 collet fits holder seat without interference');
    overlap.close();h.close();placed.close();
    var toolRack = new ToolRack(), setter = new ToolLengthSetter();
    near(toolRack.pocketFrame(7).x - toolRack.pocketFrame(0).x, 770, 1e-9, 'fork pitch frames');
    near(setter.connector('trigger').frame.z, 57.5, 1e-9, 'setter trigger frame');
    reject(() -> toolRack.pocketFrame(8), 'rack pocket bound');
    var spindle = new AtcSpindle();
    check(spindle.releaseForce() > spindle.n('clampForce'), 'pneumatic release capacity');
    var vacuum = new VacuumTable();
    check(vacuum.zoneCount() == 6, 'six isolated zones');
    check(vacuum.zoneValve(0).port('outlet').kind == Vacuum, 'zone isolation valve');
    var parts:Array<{id:String,part:AssumedCatalogComponent}> = [{id:'spindle',part:spindle},{id:'vfd',part:new Vfd()},
      {id:'rack',part:toolRack},{id:'vacuum',part:vacuum},{id:'setter',part:setter}];
    for(i in 0...vacuum.zoneCount()) parts.push({id:'zone'+i,part:vacuum.zoneValve(i)});
    var cabinet=ControlCabinet.forParts(parts);
    check(cabinet.parameterCount('inputs')==23 && cabinet.parameterCount('outputs')==12 && cabinet.parameterCount('analogOutputs')==1,
      'cabinet counts each physical I/O owner once');
    var straightPinion=new SpurGear(2,20,20),straightRack=new Rack(2,20,20);
    near(straightPinion.pitchDiameter,40,1e-9,'existing spur pitch unchanged');
    near(Rack.relation(straightPinion,straightRack,1).ratio,0.05,1e-12,'existing straight-rack relation unchanged');
    for(profile in ['MGN12C','MGN12H','MGN9C','HGH15CA'])
      check(LinearRailSystem.catalog().exists(profile),'existing rail row retained '+profile);
    near(new Spoilboard().permeabilityM2(),1e-12,1e-20,'spoilboard SI permeability');
    reject(()->new AtcSpindle(new ComponentValues().setNumber('releaseBore',32)),'undersized release cylinder');
    reject(()->new PlanetaryReducer(new ComponentValues().setNumber('efficiency',1.1)),'reducer efficiency above one');
    reject(()->new VacuumTable(new ComponentValues().setInteger('columns',0)),'empty vacuum zone layout');
    reject(()->new ToolHolder(new ComponentValues().setNumber('colletBore',50)),'holder collet bore consumes wall');
    reject(() -> ControlCabinet.forParts([{id: 'same', part: spindle}, {id: 'same', part: setter}
    ]), 'duplicate cabinet ids');
    Sys.println('Sheet router SR1 parts: ' + assertions + ' assertions passed');
  }
}
