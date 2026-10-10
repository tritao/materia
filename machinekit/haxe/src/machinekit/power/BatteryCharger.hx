package machinekit.power;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** A floor-mounted charging station with a rated DC output and a CAD docking marker. */
class BatteryCharger extends MachineComponent {
  public static inline var DEFAULT_LENGTH:Float = 300;
  public static inline var DEFAULT_WIDTH:Float = 240;
  public static inline var DEFAULT_HEIGHT:Float = 100;
  public static inline var DEFAULT_MASS_KG:Float = 18;

  public final nominalVolts:Float;
  public final outputWatts:Float;
  public final length:Float;
  public final width:Float;
  public final height:Float;
  public final massKg:Float;
  /** CAD-derived carrier base pose that clears the station footprint while docked. */
  public final dockApproach:AssemblyFrame;
  /** Maximum docking translation and heading error accepted by the charging contacts. */
  public final dockPositionToleranceMm:Float;
  public final dockHeadingToleranceRad:Float;

  public function new(nominalVolts:Float, outputWatts:Float, dockStandoff:Float,
      length:Float = DEFAULT_LENGTH, width:Float = DEFAULT_WIDTH,
      height:Float = DEFAULT_HEIGHT, massKg:Float = DEFAULT_MASS_KG,
      dockPositionToleranceMm:Float = 15, dockHeadingToleranceRad:Float = 0.02) {
    for (value in [nominalVolts, outputWatts, dockStandoff, length, width, height, massKg,
        dockPositionToleranceMm, dockHeadingToleranceRad])
      if (!Math.isFinite(value) || value <= 0)
        throw "Battery charger ratings, docking tolerances, standoff, dimensions and mass must be finite and positive";
    if (dockHeadingToleranceRad >= Math.PI / 2)
      throw "Battery charger heading tolerance must be less than 90 degrees";
    super('CHARGER-${Dimension.format(nominalVolts)}V-${Dimension.format(outputWatts)}W',
      "Floor mounted mobile battery charger", "painted steel", true);
    this.nominalVolts = nominalVolts;
    this.outputWatts = outputWatts;
    this.length = length;
    this.width = width;
    this.height = height;
    this.massKg = massKg;
    this.dockPositionToleranceMm = dockPositionToleranceMm;
    this.dockHeadingToleranceRad = dockHeadingToleranceRad;
    dockApproach = {x: -dockStandoff, y: 0, z: 0, qx: 0, qy: 0,
      qz: Math.sin(Math.PI / 4), qw: Math.cos(Math.PI / 4)};
    addConnector("base", Mount, Solids.axial(0, 0, 0));
    addConnector("dock", Mount, dockApproach);
    addPort({name: "grid", kind: ElectricalPower, role: Consumer, iface: Unspecified, required: false});
    addPort({name: "charge", kind: ElectricalPower, role: Supply, iface: Unspecified, required: false});
    declareMass(massKg, new Vector(0, 0, height / 2));
  }

  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part return Part.box(length, width, height);
}
