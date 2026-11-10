import cadkit.modeling.AssemblyState;
import gearkit.*;
import machinekit.assembly.*;
import machinekit.assembly.AxisBuilder;
import machinekit.component.*;
import machinekit.control.BissMaster;
import machinekit.design.DesignRequirement;
import machinekit.motion.*;
import machinekit.hobbing.*;
import machinekit.power.*;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyDefinition.AssemblyJointType;

class HobberTableWork extends HobberUnit {
  public final table:MachineAssembly;
  public final tableSpec:RotaryAxisSpec;
  public final mandrel:ExpandingMandrel;
  public final workOrigin:AssemblyFrame;
  public function new(spec:GearSpec, ?rotaryAxis:PrecisionRotaryAxis) {
    super("tableWork");
    var installed:machinekit.motion.PrecisionRotaryAxis = rotaryAxis
      == null ? new machinekit.motion.PurchasedRotaryTable() : rotaryAxis;
    table = installed.assembly();
    tableSpec = installed.specification();
    var tableBase = layout.nativeTableFace - tableSpec.number("height"), seat = layout.nativeTableSeat;
    if (tableBase
      < seat) throw "Rotary-axis height exceeds reserved table well; resize foundation seat before installation";
    var tableParent = "bed/tablePlate";
    if (tableBase > seat) {
      externalPart(
        "tableAdapter",
        new machinekit.motion.PurchasedRotaryPart(new ComponentValues().setToken("variant", "adapter").setNumber(
          "adapterHeight",
          tableBase - seat
        ).setNumber(
          "adapterOutside",
          tableSpec.number("housingDiameter")
        )),
        HobberFrame.partAt(0, 0, HobberFrame.PART_DATUM_Z + seat),
        "foundation", "bed-tablePlate", null, true
      );
      tableParent = "tableAdapter";
    }
    include("table", table, HobberFrame.partAt(0, 0, HobberFrame.PART_DATUM_Z + tableBase));
    if (tableParent == "tableAdapter") fixed("table-mount", tableParent, "top", "table/" + tableSpec.mountPart, "mount");
    else externalNativeMate("table-mount", "table/" + tableSpec.mountPart, "mount", "foundation", "bed-tablePlate", true);
    for (name in table.portNames()) {
      var port = table.port(name, "table");
      exposePort("table." + name, port.instanceId, port.portName);
    }
    var diameter = 2 * (spec.rootRadius() - 0.6), workSeat = layout.nativeHobCentreZ - layout.nativeTableFace;
    mandrel = new ExpandingMandrel(new ComponentValues().setNumber("outside", diameter - FlexsplineHobber.LOADING_CLEARANCE).setNumber(
      "radialFitGap",
      FlexsplineHobber.LOADING_CLEARANCE / 2
    ).setNumber(
      "pressedMargin",
      FlexsplineHobber.PRESSED_MARGIN
    ).setNumber(
      "workSeat",
      workSeat
    ).setNumber(
      "height",
      workSeat + Math.max(16, spec.faceWidth + FlexsplineHobber.PRESSED_MARGIN) + 2
    ).setNumber(
      "sleeveStart",
      workSeat
    ).setNumber(
      "sleeveLength",
      Math.max(16, spec.faceWidth + FlexsplineHobber.PRESSED_MARGIN)
    ).setNumber(
      "stopDiameter",
      diameter + 0.6
    ).setBoolean(
      "separateSleeve",
      true
    ).setNumber(
      "baseDiameter",
      Math.max(tableSpec.number("facePcd") + 30, diameter + 10)
    ).setNumber(
      "baseHeight",
      25
    ).setNumber(
      "baseBore",
      40
    ).setNumber(
      "baseBoreDepth",
      20
    ).setNumber(
      "basePilotDiameter",
      tableSpec.number("centeringDiameter")
    ).setNumber(
      "basePilotDepth",
      tableSpec.number("centeringDiameter") > 0 ? 3 : 0
    ).setNumber(
      "baseBoltPcd",
      tableSpec.number("facePcd")
    ));
    fixedPart(
      "mandrel",
      mandrel,
      HobberFrame.partAt(0, 0, HobberFrame.PART_DATUM_Z + layout.nativeTableFace),
      "table/" + tableSpec.workPart,
      F.alongY(-20, 0, HobberFrame.PART_DATUM_Z + layout.nativeTableFace, 0, 0, 1)
    );
    fixedPart(
      "mandrelSleeve",
      new MandrelSleeve(new ComponentValues().setNumber("outside", diameter - FlexsplineHobber.LOADING_CLEARANCE).setNumber(
        "wall",
        mandrel.parameterNumber("sleeveWall")
      ).setNumber(
        "height",
        mandrel.parameterNumber("sleeveLength")
      )),
      HobberFrame.partAt(0, 0, HobberFrame.PART_DATUM_Z + (layout.nativeTableFace + mandrel.parameterNumber("sleeveStart"))),
      "mandrel"
    );
    workOrigin = HobberFrame.partAt(0, 0, heights.hobCentreZ);
    var blank = new FlexsplineCupBlank(new ComponentValues().setNumber("outside", 2 * spec.tipRadius()).setNumber(
      "bore",
      diameter
    ).setNumber(
      "bodyOutside",
      2 * spec.rootRadius()
    ).setNumber(
      "bandWidth",
      spec.faceWidth
    ));
    fixedPart("blank", blank, HobberFrame.partAt(0, 0, heights.hobCentreZ), "mandrel");
      externalPart(
        "chipTray",
        new HobberSaddle(
          "GE-chip-tray",
          new ComponentValues().setNumber("tableFaceDiameter", tableSpec.number("faceDiameter")).setNumber(
            "tableHousingDiameter",
            tableSpec.number("housingDiameter")
          ).setNumber(
            "tableFaceHeight",
            layout.nativeTableFace
          ).setNumber(
            "tableSeatHeight",
            tableBase
          )
        ),
        HobberFrame.partAt(0, 0, HobberFrame.PART_DATUM_Z),
        "foundation", "bed-tablePlate"
      );
    exposeConnector("work", "mandrel", "work");

  }

}
