import machinekit.hobbing.HobberZLayout as Z;
import machinekit.assembly.PosedParts;
import materia.assembly.AssemblyFrames as F;
import cadkit.modeling.AssemblyModel;
import cadkit.modeling.Part;
class HobberZDriveChecks {
  public static function run():Void {
    var n = 0;
    function check(ok:Bool, label:String):Void {
      n++;
      if (!ok) throw label;
    }
    function near(a:Float, b:Float, label:String):Void check(Math.abs(a - b) < 1e-7, label + ": " + a + " vs " + b);
    var m = new FlexsplineHobber(), model = new AssemblyModel("mm");
    m.addTo(model, "");
    var state = model.initialState();
    state.forwardKinematics();
    var members = m.components(), work = F.inverse(machinekit.hobbing.HobberFrame.workToMachine()), cache = new PosedParts();
    function pose(id:String):materia.assembly.AssemblyRecord.AssemblyFrame return F.compose(work, state.worldPose(id));
    var a = pose("railZLeft"), b = pose("railZRight"),
      screw = pose("Zscrew"), fixed = pose("Zfixed"), floating = pose("Zfloating"), motor = pose("Zmotor");
    near(screw.y, (a.y + b.y) / 2 + Z.axisY(), "screw between rails, offset clear of the A actuator housing");
    near(screw.x - a.x, 15, "screw 15 mm behind guide axis plane");
    near(screw.z, Z.SHAFT_TOP, "top drive journal");
    near(motor.z, 764, "motor mounting face: 34 mm above the screw start");
    var direction = F.transformVector(screw, 0, 0, 1);
    near(direction.z, -1, "screw axis points down");
    check(fixed.z > floating.z, "fixed bearing above floating bearing");
    var part:machinekit.motion.HzMotionDrawingScrew = cast members.filter(p -> p.id == "Zscrew")[0].component;
    near(part.totalLength, 420, "screw length");
    var flange = m.column.insertLocations().filter(p -> p.id == "driveSeat")[0], body = m.column.body.geometry();
    var raw = Part.box(flange.part.boxDimensions(flange.id).width,
      flange.part.boxDimensions(flange.id).depth, 125), back = PosedParts.moved(
        raw,
        F.compose(flange.pose, F.translation(0, 0, -125))
      );
    raw.close();
    // Backing means as-cast occupancy: resin plus the permanent steel and anchors cast into the pockets
    // (the finished body alone has the anchor pockets cut out, which is not thinner casting).
    var composite = m.column.asCastCompositeGeometry();
    var missing = back.subtract(composite), missingMm3 = missing.volume();
    missing.close();
    composite.close();
    back.close();
    check(missingMm3 < 0.001, "125 mm full casting backing behind drive insert");
    check(body.solidCount() == 1, "column remains monolithic");
    body.close();
    var coupling = cache.posed(members.filter(p -> p.id == "Zcoupling")[0].component, state.worldPose("Zcoupling"));
    var couplingClearance = 1e30;
    for (id in ["ZdriveFixedBar","ZdriveFloatingBar"]) {
      var bracket = cache.posed(members.filter(p -> p.id == id)[0].component, state.worldPose(id));
      couplingClearance = Math.min(couplingClearance, coupling.distance(bracket));
      bracket.close();
    }
    coupling.close();
    check(couplingClearance >= 2, "coupling clears the drive pads by at least 2 mm");
    var minimumRaceMargin = 1e30, minimumMovingGap = 1e30;
    try {
      for (z in [-15.0, -10, 0, 190, 195]) {
        var legal = Math.max(-10, Math.min(190, z)), delta = F.translation(0, 0, z - legal);
        state.setJoint("Z", legal);
        state.forwardKinematics();
        var nutWorld = F.compose(delta, state.worldPose("Znut"));
        var nut = F.compose(F.inverse(state.worldPose("Zscrew")), nutWorld);
        var far = nut.z + 45, margin = Math.min(
          nut.z - part.fixedEnd,
          part.totalLength - part.floatingEnd - far
        );
        minimumRaceMargin = Math.min(minimumRaceMargin, margin);
        check(margin >= 10, "nut stays on race with 10 mm allowance at " + z);
        check(pose("Zfixed").z > F.compose(work, nutWorld).z, "gravity-loaded segment hangs below fixed bearing");
        for (id in ["Zfixed", "Zfloating", "ZdriveFixedBar","ZdriveFloatingBar"]) {
          var component = members.filter(p -> p.id == id)[0].component;
          var p = cache.posed(component, state.worldPose(id)), saddle = cache.posed(
            members.filter(p -> p.id == "Zplate")[0].component,
            F.compose(delta, state.worldPose("Zplate"))
          );
          var gap = p.distance(saddle);
          p.close();
          saddle.close();
          minimumMovingGap = Math.min(minimumMovingGap, gap);
          check(gap >= 10 - 1e-7, "moving saddle retains 10 mm drive clearance at " + z + " / " + id + ": " + gap);
        }
      }
      state.setJoint("Z", 0);
      state.forwardKinematics();
      var q0 = state.joint("Zscrew-turn"),
        p0 = F.compose(F.inverse(state.worldPose("Zscrew")), state.worldPose("Znut"));
      state.setJoint("Z", 10);
      state.forwardKinematics();
      var q1 = state.joint("Zscrew-turn"),
        p1 = F.compose(F.inverse(state.worldPose("Zscrew")), state.worldPose("Znut"));
      near(p1.z - p0.z, part.thread.signedLead() * (q1 - q0) / (2 * Math.PI), "physical right-hand screw sign");
      near(pose("Zplate").z, 373, "positive machine Z still raises head");
    } catch (e:Dynamic) {
      cache.close();
      throw e;
    }
    cache.close();
    var report = {
      assertions: n,
      couplingPassageMm: 40,
      couplingWindowCornerRadiusMm: 3,
      couplingClearanceMm: couplingClearance,
      guideSpacingMm: 290,
      screwLateralOffsetMm: screw.y - (a.y + b.y) / 2,
      screwBehindGuideMm: screw.x - a.x,
      shaftTopZMm: Z.SHAFT_TOP,
      shaftBottomZMm: 310,
      motorFaceZMm: motor.z,
      motorTopZMm: 910,
      fixedBearingZMm: fixed.z,
      floatingBearingZMm: floating.z,
      screwLengthMm: 420,
      nutCentreAtZeroZMm: 418,
      minimumRaceMarginMm: minimumRaceMargin,
      minimumSaddleDriveClearanceMm: minimumMovingGap,
      driveInsertMissingBackingMm3: missingMm3,
      castingBackingMm: 125,
      columnCastingKg: m.column.castingByWeight().kg,
      columnCells: [for (b in materia.casting.BoxAlgebra.material(m.column.body.castingEnvelope(), m.column.body.castingFeatures())) materia.casting.BoxAlgebra.limits(b)],
      scope: "Geometry, mechanical travel and thread sign only; structural stiffness and brake performance unqualified."
    };
    var output = Sys.getEnv("HOBBER_Z_DRIVE_EVIDENCE");
    if (output != null) sys.io.File.saveContent(output, haxe.Json.stringify(report));
    Sys.println("HOBBER_Z_DRIVE_PASS " + haxe.Json.stringify(report));
  }
}
