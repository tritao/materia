import hobber.*;
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

class HobberFoundation extends HobberUnit {
  public final bed:MineralCastBed;
  public final column:MineralCastColumn;
  public function new() {
    super("foundation");
    // The bed and the column size independently (each its own anchors, lifting trials and casting): side by side.
    function sizedBed():machinekit.structural.MineralFoundation return new MineralCastBed();
    function sizedColumn():machinekit.structural.MineralFoundation return new MineralCastColumn();
    var foundations = machinekit.cache.Parallel.all([sizedBed, sizedColumn]);
    bed = cast foundations[0];
    column = cast foundations[1];

    include("bed", bed, heights.foundationBed);
    var footCentres = layout.feet();
    for (i in 0...footCentres.length) {
      group("Leveling feet/Foot " + (i + 1));
      var p = footCentres[i], x = p.x + layout.xNominal().bedCentre, y = p.y;
      var roof = -590 + MineralCastBed.FOOT_ROOF, plateBottom = roof - MineralCastBed.FOOT_PLATE_THICKNESS;
      var plate = machinekit.structural.LevelingFoot.of(Plate(
        MineralCastBed.FOOT_PLATE_WIDTH,
        MineralCastBed.FOOT_PLATE_WIDTH,
        MineralCastBed.FOOT_PLATE_THICKNESS,
        26
      ));
      fixedPart("footSeat" + i, plate, HobberFrame.partAt(-(y), x, HobberFrame.PART_DATUM_Z + plateBottom), "bed/casting", F.alongY(-(y), x, HobberFrame.PART_DATUM_Z + roof, 0, 0, 1));
      var footThread = machinekit.standard.FastenerThreads.metric(24, 3);
      var nut = machinekit.structural.LevelingFoot.of(Nut(Hexagon(36), 19, footThread));
      fixedPart(
        "footNut" + i,
        nut,
        HobberFrame.partAt(-(y), x, HobberFrame.PART_DATUM_Z + plateBottom - 19),
        "footSeat" + i,
        F.alongY(-(y), x, HobberFrame.PART_DATUM_Z + plateBottom, 0, 0, 1)
      );
      fixedPart(
        "footLocknut" + i,
        nut,
        HobberFrame.partAt(-(y), x, HobberFrame.PART_DATUM_Z + plateBottom - 38),
        "footNut" + i,
        F.alongY(-(y), x, HobberFrame.PART_DATUM_Z + plateBottom - 19, 0, 0, 1)
      );
      var foot = machinekit.structural.LevelingFoot.of(Foot(
        100,
        15,
        footThread,
        15,
        47 + MineralCastBed.FOOT_ROOF - 2,
        47 + MineralCastBed.FOOT_ROOF - MineralCastBed.FOOT_PLATE_THICKNESS - 19,
        Nominal,
        NoCollar
      ));
      fixedPart("foot" + i, foot, HobberFrame.partAt(-(y), x, 0), "footNut" + i);
    }
    group("Ungrouped");
    include("column", column, heights.foundationColumn);
    var firstSolePad = layout.columnSolePads()[0];
    externalMate("column-mount", "column/"+firstSolePad.id, "xAxis", "carriage",
      F.alongY(-((firstSolePad.y0+firstSolePad.y1)/2), (firstSolePad.x0+firstSolePad.x1)/2, heights.xCarriageTopZ, 0, 0, 1));
    for (role in ["casting", "tablePlate", "driveSeat", "railLeft", "railRight"]) {
      host("bed-"+role, "bed/"+role, role == "tablePlate" ? "top" : null);
      if (role != "tablePlate") host("column-"+role, "column/"+role);
    }

  }

}
