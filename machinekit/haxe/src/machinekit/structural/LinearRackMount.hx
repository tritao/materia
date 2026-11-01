package machinekit.structural;

import machinekit.assembly.*;
import machinekit.assembly.AxisBuilder.AxisSpec;
import machinekit.assembly.RackMount.RackContactLink;
import machinekit.component.*;
import machinekit.transmission.*;
import machinekit.standard.*;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;
import materia.casting.InsertHole;

/** Physical welded-host rack mount, assembled from the shared RM2 steel,
 * replication and source-hole derivation. Dimensions are engineering inputs.
 * Host face is explicit; this factory never guesses a side from a bounding box. */
class LinearRackMount {
  public final mount:RackMount;
  public final members:Array<String>;
  public final receiverFrames:Array<AssemblyFrame>;
  public final connectionScreens:Array<Dynamic>;
  function new(mount:RackMount, members:Array<String>, frames:Array<AssemblyFrame>, screens:Array<Dynamic>) {
    this.mount = mount;
    this.members = members;
    receiverFrames = frames;
    connectionScreens = screens;
  }
  static function hole(id:String, diameter:Float, depth:Float, frame:AssemblyFrame):InsertHole {
    var h = InsertHoles.records(InsertHoles.through(diameter, depth))[0];
    h.id = id;
    h.frame = frame;
    return h;
  }
  static function boltPose(point:AssemblyFrame,
    axis:Array<Float>, up:Array<Float>):AssemblyFrame return AxisBuilder.orient(
      point.x,
      point.y,
      point.z,
      up,
      axis
    );
  public static function build(
    builder:AxisBuilder,
    id:String,
    axis:AxisSpec,
    direction:Array<Float>,
    pinion:MachineComponent,
    pinionPose:AssemblyFrame,
    rack:GearRackRun,
    rackId:String,
    host:String,
    tubeId:String,
    hostFace:AssemblyFrame,
    serviceFace:AssemblyFrame
  ):LinearRackMount {
    RackGeometry.relation(pinion, rack, 1);
    var up = F.transformVector(hostFace, 0, 1, 0), wheel = RackGeometry.pinion(pinion);
    var upArray = [up.x, up.y, up.z];
    var backDistance = wheel.diameter / 2 + rack.section.spec.pitchHeightMm;
    var datum = AxisBuilder.orient(
      pinionPose.x - backDistance * up.x,
      pinionPose.y - backDistance * up.y,
      pinionPose.z - backDistance * up.z,
      upArray,
      direction
    );
    var tube:RectTubeMember = Std.isOfType(builder.component(tubeId),
      RectTubeMember) ? cast builder.component(tubeId) : throw "Rack cleats require an explicit RHS host";
    var tubeDrilling = tube.drilling.copy(), padDrilling:Array<InsertHole> = [];
    var pad:Null<machinekit.gantry.GantryParts.GantryPlate> = null;
    if (host != tubeId) {
      if (!Std.isOfType(builder.component(host),
        machinekit.gantry.GantryParts.GantryPlate)) throw "Rack pad must be explicit plate stock";
      pad = cast builder.component(host);
      if (Std.isOfType(pad, MountDrilledPlate)) padDrilling = cast(pad, MountDrilledPlate).holes();
    }
    var tentative = RackMesh.layout(axis, direction, pinion, pinionPose, datum, rack).pose;
    var tubeInverse = F.inverse(builder.zeroPose(tubeId));
    // The caller owns the accessible side: +Z points into the RHS and +Y
    // along its length. A service face is never inferred from a box.
    var serviceLocal = F.compose(tubeInverse, serviceFace), serviceNormal = F.transformVector(serviceLocal, 0, 0, 1);
    var serviceAlong = F.transformVector(serviceFace, 0, 1, 0);
    if (Math.abs(Math.abs(serviceLocal.x) - tube.profile.width / 2) > 1e-6
      || Math.abs(serviceLocal.y) > 1e-6 || Math.abs(serviceNormal.x + serviceLocal.x / (tube.profile.width / 2)) > 1e-6
      || Math.abs(serviceNormal.y) + Math.abs(serviceNormal.z) > 1e-6 || Math.abs(serviceAlong.x
      - direction[0]) + Math.abs(serviceAlong.y - direction[1]) + Math.abs(serviceAlong.z - direction[2])
      > 1e-6) throw "Rack service face must be the declared neutral-height RHS side, inward and along travel";
    var first = F.transformPoint(F.compose(tubeInverse, tentative), 0, 0, 0);
    var last = F.transformPoint(F.compose(tubeInverse, tentative), 0, 0, rack.length);
    var tubeDirection = F.transformVector(tubeInverse, direction[0], direction[1], direction[2]);
    if (Math.abs(tubeDirection.z - 1) > 1e-9) throw "Rack mount requires host length along positive axis travel";
    // Retain the conservative 29 mm host end margin of the prototype;
    // the outer base bolt is 12 mm inboard from each rack end. Service
    // entry is now at section centre on the declared side face.
    var pitch = Math.PI * wheel.moduleSize;
    var low = Math.ceil((17 - Math.min(first.z, last.z)) / pitch) * pitch;
    var high = Math.floor((tube.length - 17 - Math.max(first.z, last.z)) / pitch) * pitch;
    if (low > high) throw "Rack sections and end anchors cannot fit the declared tube";
    var phaseOffset = Math.max(low, Math.min(0, high));
    var placed = RackMesh.layout(axis, direction, pinion, pinionPose, datum, rack, phaseOffset).pose;
    if (pad != null) {
      var padInverse = F.inverse(builder.zeroPose(host));
      var p0 = F.transformPoint(F.compose(padInverse, placed), 0, 0, 0), p1 = F.transformPoint(
        F.compose(padInverse, placed),
        0,
        0,
        rack.length
      );
      // Extend only mounting stock, preserving its centre/height and every
      // rail datum. Frame tubes are not lengthened to accommodate racks.
      var depth = 5 * Math.ceil(Math.max(pad.depth, 2 * (Math.max(Math.abs(p0.y), Math.abs(p1.y)) + 2)) / 5);
      if (depth > pad.depth) {
        var widened = new machinekit.gantry.GantryParts.GantryPlate(
          pad.drawingNumber,
          "extended rail/rack datum pad",
          pad.width,
          depth,
          pad.height
        );
        widened.setMaterial(pad.materialSpec());
        pad = widened;
      }
    }
    var source = MountedElement.of(rack), length = rack.section.spec.lengthMm;
    // Independent receivers must not butt into a tolerance-dependent
    // load path. The sourced racks retain their 500 mm joint pitch;
    // finished receiver stock stops 1 mm short at each end.
    var receiverLength = length - 2;
    if (rackId == null
      || StringTools.trim(rackId) == "") throw "Rack support requires the installed rack occurrence identity";
    if (length != 500) throw "This five-cleat layout requires 500 mm source sections";
    var washer = FlatWasher.metric("M6"), members:Array<String> = [],
      links:Array<RackContactLink> = [], frames:Array<AssemblyFrame> = [], screens:Array<Dynamic> = [];
    var parents:Map<String, String> = [];
    function attach(name:String, part:MachineComponent, pose:AssemblyFrame, parent:String):Void {
      builder.place(name, part, pose);
      members.push(name);
      parents.set(name, parent);
    }
    var unknown =
      materia.casting.CastingInput.Unresolved("Transfer drill after alignment; measured fabrication error pending");
    var allocation:machinekit.structural.InsertMountPattern.MountHoleErrorAllocation = {
      placementOffsetMm: unknown,
      placementAngleRad: unknown,
      assemblyAdjustmentMm: unknown,
      cureBowMm: unknown,
      machiningErrorMm: unknown,
      pitchAccumulationMm: unknown,
      requiredMarginMm: unknown
    };
    for (sectionIndex in 0...rack.count) {
      var prefix = id + 'Section${sectionIndex+1}', sectionPose = F.compose(
        placed,
        F.translation(0, 0, sectionIndex * length)
      );
      var localRack = F.fromRotationMatrix(
        rack.section.spec.pitchHeightMm - rack.section.spec.holeHeightMm,
        length / 2,
        18 + 2 * washer.thickness + rack.section.spec.faceWidthMm,
        [0, 1, 0, 0, 0, -1, -1, 0, 0]
      );
      var frame = F.compose(sectionPose, F.inverse(localRack));
      frames.push(frame);
      var localRun = F.compose(F.inverse(frame), placed);
      var sectionConnectors = source.connectors.filter(c -> StringTools.startsWith(c.name,
        'section${sectionIndex+1}-'));
      // Section translation is along the receiver's Y axis. Use the first
      // section frame for the common transverse height, avoiding cancellation
      // from a large along-axis translation in later sections.
      var hostLocal = F.transformPoint(
        F.inverse(F.compose(placed, F.inverse(localRack))),
        hostFace.x + 4 * up.x,
        hostFace.y + 4 * up.y,
        hostFace.z + 4 * up.z
      );
      var baseX = hostLocal.x, cleatHeight = 13 - baseX;
      var cleat = new LinearMountCleat(new ComponentValues().setNumber("height", cleatHeight));
      var screen = cleat.connection(1000, 400, 400);
      screens.push(screen);
      if (!screen.pass) throw "Rack cleat connection fails the stated 1000/400/400 N envelope";
      var derived = MountHoleDerivation.derive(
        {x: -13.0, y: -receiverLength / 2, z: 0.0, width: 26.0, depth: receiverLength, height: 18.0},
        C45Bright,
        rack,
        rackId,
        source.fingerprint(rackId),
        localRun,
        sectionConnectors.map(c -> c.name),
        source.thread,
        source.bore.diameter,
        source.bore.reference,
        [TransferAfterCasting],
        allocation
      );
      var holes = derived.holes.copy(), anchors = [-220.0, -125.0, 0.0, 125.0, 220.0];
      var locating:Array<{frame:AssemblyFrame, id:String}> = [];
      if (rack.finishPinDiameter > 0) for (pin in 1...3) {
        var sourceConnector = 'section${sectionIndex+1}-pinPilot$pin';
        var connector = F.compose(localRun, rack.connector(sourceConnector).frame);
        var normal = F.transformVector(connector, 0, 1, 0);
        if (Math.abs(normal.x) + Math.abs(normal.y) > 1e-8
          || Math.abs(normal.z - 1) > 1e-8) throw "Rack pilot is not normal to its receiver";
        var entry = F.fromRotationMatrix(connector.x, connector.y, 18, [1, 0, 0, 0, -1, 0, 0, 0, -1]);
        var h = hole('locating-' + sourceConnector, rack.finishPinDiameter, 11, entry);
        h.extent = Blind;
        holes.push(h);
        locating.push({frame: connector, id: 'LocatingPin$pin'});
      }
      for (i in 0...anchors.length) for (y in [-14.0, 14.0]) {
        var h = InsertHoles.records(InsertHoles.tappedBlind(
          InsertHoles.metricThread(FastenerThreads.coarse(6).mating()),
          18
        ))[0];
        h.id = 'anchor${i+1}-' + y;
        h.frame = F.translation(3, anchors[i] + y, 0);
        holes.push(h);
      }
      // The adjustment bar is machined integrally with the receiver.
      // It stays 5 mm below the rack back; only the push screws touch the
      // rack. It is not a fixed locating lip. No hidden nuts occupy the pad.
      for (y in derived.holes.map(h -> h.frame.y)) {
        var h = InsertHoles.records(InsertHoles.tappedThrough(
          InsertHoles.metricThread(FastenerThreads.coarse(4).mating()),
          6
        ))[0];
        h.id = "push-" + y;
        h.frame = F.fromRotationMatrix(-19, y, 29.1, [0, 0, 1, 0, 1, 0, -1, 0, 0]);
        holes.push(h);
      }
      // Source pilots cannot be silently crowded by the cleat screws.
      // This drove the web pair out to +/-14 mm. Keep at least 4 mm of
      // finished steel between every pair of axial bores/major threads.
      for (a in 0...holes.length) for (b in a + 1...holes.length) {
        var first = holes[a], second = holes[b], fa = F.transformVector(first.frame, 0, 0, 1), fb = F.transformVector(
          second.frame,
          0,
          0,
          1
        );
        if (Math.abs(fa.z) < 0.999 || Math.abs(fb.z) < 0.999) continue;
        var firstThread = first.thread, secondThread = second.thread;
        var ra = (firstThread == null ? first.diameter : firstThread.majorDiameter) / 2;
        var rb = (secondThread == null ? second.diameter : secondThread.majorDiameter) / 2;
        if (Math.sqrt(Math.pow(first.frame.x - second.frame.x, 2) + Math.pow(first.frame.y - second.frame.y,
          2)) < ra + rb + 4) throw "Receiver bores violate the 4 mm net-ligament rule: " + first.id + " / " + second.id;
      }
      var receiver = new LinearMountSteel(LinearMountSteel.receiver(26, receiverLength, 18, derived).values().setNumber(
        "flangeWidth",
        36.6
      ).setNumber(
        "flangeThickness",
        6
      ).setToken(
        "holes",
        InsertHoles.fromNeutral(holes)
      ).setToken("partId", prefix + "Receiver"));
      var receiverId = prefix + "Receiver";
      attach(receiverId, receiver, frame, prefix + "Cleat1");
      // One side-wall window per section. Feed the covered cassettes
      // end-first, turn at the tube's neutral height, slide at most 220 mm
      // and lift with guide rods/magnet. Legs below the tube cannot block
      // this route; no underside socket windows are needed with cages.
      var serviceCentre = F.transformPoint(F.inverse(serviceFace), frame.x, frame.y, frame.z);
      var window = F.compose(tubeInverse, F.compose(serviceFace, F.translation(0, serviceCentre.y, 0)));
      tubeDrilling.push(hole(prefix + "-cassette-entry", 24, tube.profile.wall, window));
      for (pin in locating) {
        // 6 h6 ground C45 pin; match-ream both parts to 6 H7 only after
        // push-screw adjustment and joint-gauge pitch setting. It is a
        // locating fit, not credited with carrying the cutting load.
        var start = pin.frame, axis = F.transformVector(start, 0, -1, 0);
        var length = 30.0, engagement = length - rack.section.spec.faceWidthMm - 2 * washer.thickness;
        if (engagement < 6
          || engagement > 9) throw "Rack locating pin lacks receiver engagement or blind-tip clearance";
        attach(
          prefix + pin.id,
          new machinekit.motion.SteppedShaft([{diameter: 6.0, length: length}]),
          F.compose(frame, AxisBuilder.orient(start.x, start.y, start.z, [0.0, 1, 0], [axis.x, axis.y, axis.z])),
          receiverId
        );
      }
      for (y in derived.holes.map(h -> h.frame.y)) attach(
        prefix + "Push" + y,
        SocketSetScrew.forThread(FastenerThreads.coarse(4), 12),
        F.compose(frame, AxisBuilder.orient(-20, y, 29.1, [0.0, 1, 0], [-1.0, 0, 0])),
        receiverId
      );
      // x in this frame is radial up, z is outward from the receiver.
      for (i in 0...derived.holes.length) {
        var h = derived.holes[i], seat = F.compose(
          localRun,
          F.compose(sectionConnectors[i].frame, F.translation(0, -source.bore.counterboreDepth, 0))
        );
        var screw = MountServiceScrews.select(
          h.id,
          source,
          seat,
          InsertHoles.facet(holes, 26, receiverLength, 18).holes[i],
          h.frame,
          [20, 25, 30]
        );
        var screwFrame = F.compose(frame, F.compose(seat, F.inverse(screw.connector("head").frame)));
        // Two sourced 1.6 mm washers retain the pitch/mesh plane while
        // opening the old 1.1 mm pinion-face clearance to 2.7 mm.
        var spacer = prefix + 'Spacer${i+1}', inner = spacer + "Inner";
        attach(inner, washer, F.compose(frame, F.translation(h.frame.x, h.frame.y, 18)), receiverId);
        attach(spacer, washer, F.compose(frame, F.translation(h.frame.x, h.frame.y, 18 + washer.thickness)), inner);
        attach(prefix + 'RackBolt${i+1}', screw, screwFrame, receiverId);
        links.push({first: "$rack", second: spacer});
        links.push({first: spacer, second: inner});
        links.push({first: inner, second: receiverId});
      }
      for (i in 0...anchors.length) {
        var cleatId = prefix + 'Cleat${i+1}';
        var local = F.fromRotationMatrix(baseX, anchors[i], 0, [0, 0, 1, 0, 1, 0, -1, 0, 0]), cleatFrame = F.compose(
          frame,
          local
        );
        attach(cleatId, cleat, cleatFrame, cleatId + "Replication");
        var layerId = cleatId + "Replication";
        attach(
          layerId,
          new LinearMountReplication(new ComponentValues().setNumber("width", LinearMountCleat.BASE_WIDTH).setNumber(
            "length",
            58
          ).setNumber(
            "thickness",
            4
          )),
          F.compose(cleatFrame, F.translation(LinearMountCleat.BASE_WIDTH / 2, 0, -4)),
          host
        );
        links.push({first: receiverId, second: cleatId});
        links.push({first: cleatId, second: layerId});
        links.push({first: layerId, second: host});
        var cleatElement = new MountedElement(
          cleat,
          [cleat.connector("base--18"), cleat.connector("base-18")],
          {
            diameter: 6.6,
            counterboreDiameter: 13,
            counterboreDepth: 8,
            reference: "LinearMountCleat engineering drawing; recessed M6 base pair"
          },
          FastenerThreads.coarse(6).mating()
        );
        var cassette = LinearMountNutCassette.fromElement(
          cleatElement,
          F.translation(-LinearMountCleat.BOLT_X, 0, 0),
          58,
          18,
          true
        );
        var underside = -(4 + (pad == null ? 0 : pad.height) + tube.profile.wall);
        var cassetteFrame = F.compose(cleatFrame, F.translation(LinearMountCleat.BOLT_X, 0, underside - 10));
        var cassetteId = cleatId + "NutCassette";
        attach(cassetteId, cassette, cassetteFrame, tubeId);
        attach(cassetteId + "Cover", cassette.cover(), F.compose(cassetteFrame, F.translation(0, 0, -2)), cassetteId);
        for (h in cassette.coverThreads()) attach(
          cassetteId + h.id + "Bolt",
          SocketHeadCapScrew.metric("M3", 6),
          F.compose(cassetteFrame, AxisBuilder.orient(h.frame.x, h.frame.y, -2, [0.0, 1, 0], [0.0, 0, -1])),
          cassetteId + "Cover"
        );
        if (Math.sqrt(18 * 18 + 15 * 15) > 24 || Math.sqrt(58 * 58 + 18 * 18)
          > tube.profile.width - 2 * tube.profile.wall || Math.sqrt(58 * 58 + 15 * 15) > tube.profile.height
          - 2 * tube.profile.wall) throw "Captive-nut cassette cannot be inserted and turned inside the host";
        for (y in [-14.0, 14.0]) {
          var screw = SocketHeadCapScrew.metric("M6",
            LinearMountCleat.WEB_BOLT_LENGTH, Steel109), flat = FlatWasher.metric("M6");
          attach(
            cleatId + "WebWasher" + y,
            flat,
            F.compose(
              cleatFrame,
              AxisBuilder.orient(LinearMountCleat.WALL - 1.6, y, cleatHeight - 10, [0.0, 1, 0], [1.0, 0, 0])
            ),
            cleatId
          );
          attach(
            cleatId + "WebBolt" + y,
            screw,
            F.compose(
              cleatFrame,
              AxisBuilder.orient(LinearMountCleat.WALL, y, cleatHeight - 10, [0.0, 1, 0], [1.0, 0, 0])
            ),
            cleatId + "WebWasher" + y
          );
        }
        var layerHoles:Array<InsertHole> = [];
        for (y in [-18.0, 18.0]) {
          var boltId = cleatId + "BaseBolt" + y, seat = F.compose(
            cleatFrame,
            F.translation(LinearMountCleat.BOLT_X, y, LinearMountCleat.BASE - 8 + 1.6)
          );
          var topPoint = F.transformPoint(cleatFrame, LinearMountCleat.BOLT_X, y, -4);
          var top = AxisBuilder.orient(
            topPoint.x,
            topPoint.y,
            topPoint.z,
            [direction[0], direction[1], direction[2]],
            [-up.x, -up.y, -up.z]
          );
          var tubeTop = F.transformPoint(F.inverse(builder.zeroPose(tubeId)), topPoint.x, topPoint.y, topPoint.z);
          // Explicit RHS top/underside in its source coordinates.
          var localTop = F.compose(F.inverse(builder.zeroPose(tubeId)), top);
          localTop.y = tube.profile.height / 2;
          tubeDrilling.push(hole(boltId + "-tube", 6.6, tube.profile.wall, localTop));
          if (pad != null) padDrilling.push(hole(
            boltId + "-pad",
            6.6,
            pad.height,
            F.compose(F.inverse(builder.zeroPose(host)), top)
          ));
          var layerHole = hole(
            boltId + "-compound",
            6.6,
            4,
            F.fromRotationMatrix(
              LinearMountCleat.BOLT_X - LinearMountCleat.BASE_WIDTH / 2,
              y,
              4,
              [1, 0, 0, 0, -1, 0, 0, 0, -1]
            )
          );
          // The layer is compression bedding, not tapped steel. Its drilled
          // clearance ligament is a stated compound manufacturing input.
          layerHole.limits.minWall = 1.5;
          layerHole.limits.overrides.push({name: "minWall", value: 1.5});
          layerHoles.push(layerHole);
          var back = F.compose(builder.zeroPose(tubeId), F.compose(localTop, F.translation(0, 0, tube.profile.wall)));
          var dx = back.x - seat.x, dy = back.y - seat.y, dz = back.z - seat.z;
          var grip = -(dx * up.x + dy * up.y + dz * up.z), nut = HexNut.metric("M6"), flat = FlatWasher.metric("M6");
          var length = 5 * Math.ceil((grip + flat.thickness + nut.height + 2) / 5);
          if (length - grip > 15) throw "Host bolt tip obstructs the neutral-height cassette transfer corridor";
          var screw = SocketHeadCapScrew.metric("M6", length, Steel109);
          if (length - screw.threadLength > grip + flat.thickness) throw "Host bolt plain shank reaches its nut";
          attach(
            boltId + "HeadWasher",
            flat,
            F.compose(cleatFrame, F.translation(LinearMountCleat.BOLT_X, y, LinearMountCleat.BASE - 8)),
            cleatId
          );
          attach(boltId, screw, seat, boltId + "HeadWasher");
          var down = AxisBuilder.orient(back.x, back.y, back.z, direction, [-up.x, -up.y, -up.z]);
          attach(boltId + "Washer", flat, down, tubeId);
          attach(boltId + "Nut", nut, F.compose(down, F.translation(0, 0, flat.thickness)), boltId + "Washer");
        }
        builder.replaceComponent(
          layerId,
          new LinearMountReplication(new ComponentValues().setNumber("width", LinearMountCleat.BASE_WIDTH).setNumber(
            "length",
            58
          ).setNumber(
            "thickness",
            4
          ).setToken(
            "holes",
            InsertHoles.fromNeutral(layerHoles)
          ))
        );
      }
    }
    var drilledTube = new RectTubeMember(
      tube.profile.width,
      tube.profile.height,
      tube.profile.wall,
      tube.length,
      tubeDrilling,
      "RACK-" + tubeId
    );
    builder.replaceComponent(tubeId, drilledTube);
    if (pad != null) builder.replaceComponent(
      host,
      new MountDrilledPlate(new ComponentValues().setNumber("width", pad.width).setNumber("depth", pad.depth).setNumber(
        "height",
        pad.height
      ).setToken(
        "hostMaterial",
        pad.materialSpec()
      ).setToken(
        "holes",
        InsertHoles.fromNeutral(padDrilling)
      ).setToken(
        "partId",
        "RACK-" + host
      ))
    );
    for (name in members) {
      var parent = parents.get(name);
      builder.connect(parent, name);
      builder.addMate(name + "-mount", "fixed", parent, "to-" + name, name, "attach-" + name);
    }
    var localDatum = F.compose(F.inverse(builder.zeroPose(host)), datum);
    return new LinearRackMount(
      new RackMount(host, localDatum, id + "Section1Spacer1", links, phaseOffset),
      members,
      frames,
      screens
    );
  }
}
