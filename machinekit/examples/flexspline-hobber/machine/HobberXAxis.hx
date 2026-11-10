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

class HobberXAxis extends HobberUnit {

  public function new(cutDistance:Float, minX:Float) {
    super("xAxis");
    var centre=0.0;
    for (placed in layout.rails("bed", layout.foundationPlacements("bed"))) {
      var insert = "bed/" + placed.insertId;
      externalPart(placed.memberId, placed.rail, HobberFrame.sourcePose(placed.pose), "foundation", "bed-"+placed.insertId, HobberFrame.sourcePose(placed.contact), true);
    }
    // The prismatic joint is the guide block on its rail; the carriage is fixed to that block and carries the others.
    for (side in [-1, 1]) for (row in [layout.xBlockFront, layout.xBlockRear]) {
      var id = "blockX" + (side < 0 ? "Left" : "Right") + (row == layout.xBlockFront ? "" : "Front");
      var block = LinearRailBlock.metric("HGH25CA", LinearRailBlock.SUPPLIER);
      var pose = AxisBuilder.orient(-side * layout.bedRailCentre, centre + row, heights.xGuideOriginZ, [0.0, 0, 1], [0.0, 1, 0]);
      var top = F.alongY(-side * layout.bedRailCentre, centre + row, heights.xCarriageBottomZ, 0, 0, 1);
      if (id == "blockXLeft") {
        addComponent(id, block, pose);
        moving("X", Prismatic, "railXLeft", id, HobberFrame.partAt(0, 0, heights.xGuideOriginZ), [0.0, 1, 0], cutDistance, minX, layout.xRetract);
        fixedPart("Xbridge", new HobberXCarriage(), HobberFrame.partAt(0, centre, HobberFrame.PART_DATUM_Z), id, top);
      } else fixedPart(id, block, pose, "Xbridge", top);
    }
    for (role in HobberCarriageRib.ROLES) {
      var rib = new HobberCarriageRib(new ComponentValues().setToken("role", role));
      var b = rib.band();
      fixedPart("Xrib-" + role, rib, HobberFrame.partAt(0, 0, HobberFrame.PART_DATUM_Z), "Xbridge",
        F.alongY(-((b.low + b.high) / 2), 263, heights.xCarriageBottomZ, 0, 0, 1));
    }
    host("carriage", "Xbridge");
    group("Drive");
    var axis=HobberDrives.x();
      for(support in ["fixed","floating"]) {
        var id=HobberDrives.barMember(axis.id,support),face=HobberPadBolts.face(axis,support);
        externalPart(id,new HobberDriveBar(axis.id,support),HobberFrame.partAt(0, 0, HobberFrame.PART_DATUM_Z),"foundation",axis.id=="X"?"bed-driveSeat":"column-driveSeat",HobberFrame.sourcePose(face.frame),true);
      }
    drive(axis,HobberDrives.barMember("X","fixed"),HobberDrives.barMember("X","floating"),"Xbridge",cutDistance);
    var scaleX=HobberScales.x();
    group("Scale");
    externalPart("XscaleSeat", new HobberScaleSeat("X"), HobberFrame.partAt(0,0,HobberFrame.PART_DATUM_Z), "foundation", "bed-casting", scaleBarContact(scaleX));
    scale(scaleX,"XscaleSeat","Xrib-scale");
    group("Protection");
    for (r in HobberChipCover.installations()) {
      var part=new HobberChipCover(new ComponentValues().setToken("role",r.role));
      var pose=HobberFrame.partAt(0,0,HobberFrame.PART_DATUM_Z), seat=HobberFrame.sourcePose(r.seat);
      if (r.parent=="bed/driveSeat") externalPart(r.id,part,pose,"foundation","bed-driveSeat",seat);
      else fixedPart(r.id,part,pose,r.parent,seat);
    }
    for (p in HobberChipCover.plugs()) fixedPart(p.id,new machinekit.standard.PanelHolePlug(),HobberFrame.partAt(-p.y,p.x,HobberFrame.PART_DATUM_Z+p.z),p.parent,
      F.alongY(-p.y,p.x,HobberFrame.PART_DATUM_Z+p.z,0,0,-1));

  }

}
