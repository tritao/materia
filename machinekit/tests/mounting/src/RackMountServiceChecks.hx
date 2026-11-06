import machinekit.assembly.PosedParts;
import machinekit.assembly.PosedParts.PartBox;
import machinekit.gantry.Gantry;
import machinekit.structural.*;
import machinekit.standard.*;
import materia.assembly.AssemblyFrames as F;
import cadkit.modeling.Part;
import cadkit.modeling.AssemblyState;

/** Occupied service-tool volumes with the carriage parked at the far endpoint.
 * Hex keys are short hardened L keys, not an impossible long straight shaft
 * driven through the fixed rail. Both handle directions are tried explicitly. */
class RackMountServiceChecks {
  static function key(radius:Float, driveTop:Float, maximum:Float, sign:Float, bend:Float):Part {
    var elbow = maximum - radius;
    if (bend < radius
      || elbow - bend < driveTop + 0.01) throw "Short key bend enters the drive head; choose a longer leg";
    var stem = Part.cylinderSpan(radius, driveTop, elbow);
    var raw = Part.cylinder(radius, 80), arm = PosedParts.moved(
      raw,
      machinekit.assembly.AxisBuilder.orient(0, 0, elbow, [1.0, 0, 0], [0.0, sign, 0])
    );
    raw.close();
    // Contains the complete curved bend, including its inner diagonal. No
    // unmodelled material is allowed between two perpendicular cylinders.
    // For bend centreline radius B >= tool radius R, the swept quarter
    // circle has z_min=elbow-B, not elbow-B-R: the cross-section is normal
    // to its changing tangent. Debit the full X/Y envelope conservatively.
    var lowY = -radius, highY = bend + radius, lowZ = elbow - bend;
    var box = Part.box(2 * radius, highY - lowY, maximum - lowZ);
    var corner = box.translated(new cadkit.modeling.Vector(0, sign * (lowY + highY) / 2, lowZ));
    box.close();
    return machinekit.component.Solids.union([stem, arm, corner]);
  }
  /** Purpose-sized thin open spanner envelope; opening follows the same
  * nominal hex profile as the part. Confirm a purchased tool fits these bounds,
  * or cut the stated plate tool before commissioning. */
  static function spanner(af:Float, thickness:Float, z:Float):Part {
    var outer = af / 2 + 4.2;
    var raw = Part.box(10.4, 90, thickness), handle = raw.translated(new cadkit.modeling.Vector(0, -45, z));
    raw.close();
    var stock = machinekit.component.Solids.union([Part.cylinderSpan(outer, z, z + thickness), handle]);
    var mouthRaw = Part.box(af + 0.4, outer + 2, thickness + 2);
    var mouth = mouthRaw.translated(new cadkit.modeling.Vector(0, (outer + 2) / 2, z - 1));
    mouthRaw.close();
    return machinekit.component.Solids.cut(
      stock,
      [Part.prism(cadkit.modeling.Polygon.regular(6, af + 0.4), z - 1, z + thickness + 1), mouth]
    );
  }
  public static function run(machine:Gantry, bed:Float):Void {
    var state = new AssemblyState(machine.definition()),
      rows:Array<Dynamic> = [], selected:Array<Dynamic> = [], spanners:Array<Dynamic> = [], tools = 0, tested = 0;
    var members = machine.components();
    var chainShapes:Map<String, Part> = [], chainBoxes:Map<String, PartBox> = [], chainPoses:Array<Dynamic> = [];
    function closeChains():Void {
      for (shape in chainShapes) shape.close();
    }
    try {
      PosedParts.scope(cache -> {function tool(
        id: String,
        shape: Part,
        pose: materia.assembly.AssemblyRecord.AssemblyFrame,
        axis: String,
        ?movingMember: String
      ): Array<Dynamic> {
        var direction = machine.axisDirection(axis), reference = machine.zeroPose(axis == "x" ? "xCarriage" : "beam");
        var q = (pose.x
          - reference.x) * direction[0] + (pose.y - reference.y) * direction[1] + (pose.z - reference.z) * direction[2];
        var limits = [for (a in machine.axes) if (a.id == axis) a][0];
        var park = movingMember != null
          ? (limits.lower + limits.upper) / 2 : q < (limits.lower + limits.upper) / 2 ? limits.upper : limits.lower;
        for (a in machine.axes) state.setJoint(a.id, a.id == axis ? park : 0);
        state.forwardKinematics();
        var atTool = movingMember == null ? pose : state.worldPose(movingMember);
        var occupied = PosedParts.moved(shape, atTool), box = PosedParts.boxOf(occupied), clashes:Array<Dynamic> = [];
        tools++;
        try {
          for (m in members) {
            var at = state.worldPose(m.id);
            var chain:Part = null, chainBox:PartBox = null;
            for (binding in machine.cableChainBindings) if (binding.member == m.id) {
              var position = state.joint(binding.axis) + binding.positionOffset, key = m.id + ":" + position;
              if (!chainShapes.exists(key)) {
                var routed = binding.chain.geometryAt(position);
                chainShapes.set(key, routed);
                chainBoxes.set(key, PosedParts.boxOf(routed));
                chainPoses.push({member: m.id, axis: binding.axis, positionMm: position});
              }
              chain = chainShapes.get(key);
              chainBox = chainBoxes.get(key);
            }
            var bounds = chain == null ? cache.box(m.component, at) : PosedParts.carried(chainBox, at);
            if (PosedParts.apart(box, bounds)) continue;
            var part = chain == null ? cache.posed(m.component,
              at) : PosedParts.moved(chain, at), common = occupied.intersect(part), volume = common.volume();
            common.close();
            part.close();
            tested++;
            if (volume > 0.001) clashes.push({tool: id,
              other: m.id, axis: axis, parkMm: park, intersectionMm3: volume});
          }
        } catch (e:Dynamic) {
          occupied.close();
          throw e;
        }
        occupied.close();
        return clashes;
      }
      for (m in members) if (Std.isOfType(m.component, RectTubeMember)) {
        var tube:RectTubeMember = cast m.component;
        for (h in tube.drilling) if (StringTools.endsWith(h.id, "-cassette-entry")) {
          var raw = Part.cylinderSpan(11.82, -100, 0.2);
          try rows = rows.concat(tool(
            h.id,
            raw,
            F.compose(machine.zeroPose(m.id), h.frame),
            StringTools.startsWith(h.id, "rackMountX") ? "x" : "y"
          )) catch (e : Dynamic) {
            raw.close();
            throw e;
          }
          raw.close();
        }
      }
      for (m in members) {
        if (!StringTools.startsWith(m.id, "rackMount") && !StringTools.startsWith(m.id, "pinion")) continue;
        var af:Float, driveTop:Float, maximum:Float;
        if (Std.isOfType(m.component, SocketHeadCapScrew)) {
          var screw:SocketHeadCapScrew = cast m.component;
          if (screw.diameter == 3) continue; // cassette cover is serviced on the bench
          af = screw.spec.socketSize;
          driveTop = screw.spec.headHeight;
          maximum = driveTop - screw.spec.socketDepth + (m.id.indexOf("WebBolt") >= 0 ? 12 : 16);
        } else if (Std.isOfType(m.component, SocketSetScrew)) {
          var screw:SocketSetScrew = cast m.component;
          af = screw.socketSize;
          driveTop = 0;
          maximum = -screw.socketDepth + (af <= 2 ? 10 : 12);
        } else continue;
        var axis = StringTools.startsWith(m.id, "rackMountX") || StringTools.startsWith(m.id, "pinionX") ? "x" : "y";
        var moving = StringTools.startsWith(m.id, "pinion") ? m.id : null;
        var radius = af / Math.sqrt(3) + 0.1, bend = Math.max(af, radius + 1), first = key(
          radius,
          driveTop,
          maximum,
          1,
          bend
        ), positive:Array<Dynamic>;
        try positive = tool(m.id + "-key-positive", first, machine.zeroPose(m.id), axis, moving) catch (e:Dynamic) {
          first.close();
          throw e;
        }
        first.close();
        if (positive.length == 0) {
          selected.push({
            id: m.id,
            handleDirection: 1,
            afMm: af,
            maximumProjectionMm: maximum,
            maximumBendCentrelineRadiusMm: bend
          });
          continue;
        }
        var second = key(radius, driveTop, maximum, -1, bend), negative:Array<Dynamic>;
        try negative = tool(m.id + "-key-negative", second, machine.zeroPose(m.id), axis, moving) catch (e:Dynamic) {
          second.close();
          throw e;
        }
        second.close();
        if (negative.length == 0) selected.push({
          id: m.id,
          handleDirection: -1,
          afMm: af,
          maximumProjectionMm: maximum,
          maximumBendCentrelineRadiusMm: bend
        });
        else {
          rows = rows.concat(positive);
          rows = rows.concat(negative);
        }
      }
      for (m in members) if (StringTools.endsWith(m.id, "SpringAdjuster") || StringTools.endsWith(
        m.id,
        "SpringAdjusterNut"
      )) {
        var adjuster = StringTools.endsWith(m.id, "SpringAdjuster");
        var af = adjuster ? machinekit.transmission.PinionPreloadHardware.PinionPreloadAdjuster.WRENCH_AF : cast(
          m.component,
          HexNut
        ).spec.acrossFlats;
        var height = adjuster ? machinekit.transmission.PinionPreloadHardware.PinionPreloadAdjuster.HEAD_HEIGHT : cast(
          m.component,
          HexNut
        ).spec.height;
        var thickness = adjuster ? 3.0 : 5.0, z = (height - thickness) / 2, axis = StringTools.startsWith(
          m.id,
          "pinionX"
        ) ? "x" : "y";
        var raw = spanner(af, thickness, z), all:Array<Dynamic> = [], passed = false;
        for (turn in 0...4) {
          var angle = turn * Math.PI / 2, c = Math.cos(angle), v = Math.sin(angle);
          var shape = PosedParts.moved(raw,
            F.fromRotationMatrix(0, 0, 0, [c, -v, 0, v, c, 0, 0, 0, 1])), clashes:Array<Dynamic>;
          try clashes = tool(m.id + "-spanner-" + turn, shape, machine.zeroPose(m.id), axis, m.id) catch (e:Dynamic) {
            shape.close();
            raw.close();
            throw e;
          }
          shape.close();
          if (clashes.length == 0) {
            spanners.push({
              id: m.id,
              quarterTurn: turn,
              afMm: af,
              thicknessMm: thickness,
              outerHeadRadiusMm: af / 2 + 4.2,
              handleLengthMm: 90
            });
            passed = true;
            break;
          }
          all = all.concat(clashes);
        }
        raw.close();
        if (!passed) rows = rows.concat(all);
      }
      });
    } catch (e:Dynamic) {
      closeChains();
      throw e;
    }
    closeChains();
    sys.io.File.saveContent(
      'build/router-mount-implementation/service-' + Std.int(bed) + '.json',
      haxe.Json.stringify(
        {
          bedMm: bed,
          toolTrials: tools,
          nativePairs: tested,
          selectedKeys: selected,
          selectedSpanners: spanners,
          routedChainPoses: chainPoses,
          failures: rows,
          status: rows.length
      == 0 ? "PASS" : "FAIL",
      envelopes : {
        cassetteApproachRadiusMm : 11.82,
        cassetteApproachLengthMm : 100,
        hexKeyHandleLengthMm : 80,
        hexKeyRadialAllowanceMm : 0.1
      },
      assumptions : [
        "Hardened L keys with at most the stated centreline bend radius and stated outside projection; verify actual tool dimensions",
        "One side window per section, magnetic/guide-rod transfer at neutral height, no underside wrench access",
        "Cassette cover screws are serviced after removal; preload adjuster/locknut use the recorded thin spanner envelopes",
        "Cable chains use geometryAt at the actual parked joint coordinate plus binding offset, not the static zero-position display mesh"
      ]
    },
      null,
      "  "
    )
    );
    Sys.println('Rack mount service ' + bed + ': ' + tools + ' tool trials, ' + rows.length + ' clashes');
    Sys.stdout().flush();
    if (rows.length > 0) throw "Mount service access fails; see report";
  }
}
