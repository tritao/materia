package materia.tooling.casting;

import materia.assembly.AssemblyFrames as F;
import materia.tooling.casting.CastingMoldPlan;
import materia.casting.CastingRegion;
import materia.tooling.panel.PanelContours;
import materia.tooling.panel.PanelJoints;
import materia.tooling.panel.PanelCodec;

/** Proud raw steel sits in a routed shell-panel rebate. The matched cured closure fills
 * the cutter relief and clearance, so no resin-side gap is silently ignored. */
class MoldPanelReceivers {
  /** Cure the actual clearance/relief around a dry-fitted datum. Its drilled
	 * holes remain dry access passages, not compound-filled receiver volume. */
  public static function datum(
    plan:CastingMoldPlan,
    insertId:String,
    bed:materia.tooling.panel.PanelAssemblyPlan.PanelPlacement,
    volume:MoldVolume,
    pocketId:String
  ):Void {
    var operations = bed.panel.operations.filter(operation -> operation.id == pocketId);
    if (operations.length != 1 || operations[0].through || operations[0].side != materia
      .tooling.ToolingEnums.MachiningSide.PositiveZ) throw "Datum closure needs its actual blind receiver: " + pocketId;
    var operation = operations[0], gross = MoldRegions.MoldVolumeRecords.copy(volume);
    gross.cuts = [];
    var sealId = materia.tooling.ToolingIds.derive("receiver-seal-", volume.id);
    plan.seals.push({
      panelId: bed.panel.id,
      insertId: insertId,
      materialSpec: plan.document.recipe.hardware.sealMaterial,
      volume: {
        id: sealId,
        material: SealantStock,
        pieces: [],
        cuts: [],
        regions: [],
        profiles: [{
          panelId: bed.panel.id,
          operationId: pocketId,
          contour: operation.contour,
          depth: operation.depth,
          pose: F.compose(
            bed.pose,
            F.translation(
              0,
              0,
              bed.panel.stock.measuredThickness - operation.depth
            )
          ),
          subtract: [MoldRegions.tooling(gross)]
        }]
      }
    });
    var openings = plan.openings.filter(opening -> opening.panelId == bed.panel.id && opening.operationId == pocketId);
    if (openings.length != 1) throw "Datum closure needs one receiver opening: " + pocketId;
    openings[0].sealingIds.push(sealId);
  }

  public static function add(plan:CastingMoldPlan):Void {
    var d = plan.document, r = d.recipe;
    for (panel in plan.panels.members) {
      var t = panel.panel.stock.measuredThickness;
      var sign = switch panel.panel.contactFace {
        case materia.tooling.ToolingEnums.ContactFace.PositiveZ:
          -1.0;
        case materia.tooling.ToolingEnums.ContactFace.NegativeZ:
          1.0;
        case materia.tooling.ToolingEnums.ContactFace.None:
          0.0;
        default:
          throw "Unknown receiver panel contact face: " + panel.panel.id;
      };
      if (sign == 0) continue;
      var surface = sign < 0 ? t : 0.0;
      for (insert in d
        .input.inserts) for (end in materia.casting.InsertSpec.InsertBodyCodec.planarEndFrames(insert.body)) {
        var pose = F.compose(d.setup.sourceToMold, insert.pose);
        var face = F.compose(pose, end);
        var local = F.compose(F.inverse(panel.pose), face), normal = F.transformVector(local, 0, 0, 1);
        var depth = (local.z - surface) * sign;
        if (depth <= 1e-7) continue;
        if (normal.z * sign
          < 1 - 1e-7) throw "Proud steel needs an aligned shell receiver: " + panel.panel.id + "/" + insert.id;
        if (depth + r.minimumLigament
          > t) throw "Proud steel consumes the panel's remaining ligament: " + panel.panel.id + "/" + insert.id;
        // A blind proud-steel receiver cannot slide vertically past
        // its insert. Release this panel along its outward normal;
        // the exact graph then requires the bed to leave its slot first.
        var outward = F.transformVector(panel.pose, 0, 0, sign);
        panel.withdrawal = {x: outward.x, y: outward.y, z: outward.z};
        var clearance = r.jointClearance;
        var contour = switch materia.casting.InsertSpec.InsertBodyCodec.mouth(insert.body) {
          case Box(width, height, _):
            PanelContours.relievedRectangle(0, 0, width + 2 * clearance, height + 2 * clearance, r.routerDiameter / 2);
          case Cylinder(diameter, _):
            PanelContours.circle(0, 0, diameter / 2 + clearance);
        };
        var u = F.transformVector(local, 1, 0, 0);
        contour = PanelContours.placed(contour, Math.atan2(u.y, u.x), local.x, local.y);
        var id = materia.tooling.ToolingIds.derive("steel-rebate-", insert.id, "-" + panel.panel.id);
        var side = sign < 0
          ? materia.tooling.ToolingEnums.MachiningSide.PositiveZ : materia.tooling.ToolingEnums.MachiningSide.NegativeZ;
        var operation = PanelJoints.operation(
          id,
          Pocket,
          contour,
          depth,
          r.routerDiameter,
          side,
          false,
          r.routerDiameter
        );
        panel.panel.operations.push(operation);
        panel.panel.contactOpenings.push(id);
        var sealId = materia.tooling.ToolingIds.derive("steel-seal-", insert.id, "-" + panel.panel.id);
        plan.seals.push({
          panelId: panel.panel.id,
          insertId: insert.id,
          materialSpec: r.hardware.sealMaterial,
          volume: {
            id: sealId,
            material: SealantStock,
            pieces: [],
            cuts: [],
            regions: [],
            profiles: [{
              panelId: panel.panel.id,
              operationId: id,
              contour: operation.contour,
              depth: depth,
              pose: F.compose(
                panel.pose,
                F.translation(
                  0,
                  0,
                  sign < 0 ? t - depth : 0
                )
              ),
              subtract : [MoldRegions.insert(
                d,
                insert.id
              )]
            }]
          }
        });
        plan.openings.push({
          panelId: panel.panel.id,
          operationId: id,
          kind: ToolingOpening,
          sealingIds: [
            materia.tooling.ToolingIds.derive(
              "insert-",
              insert.id
            ),
            sealId
          ],
          protection: null,
          sealSpec: r.hardware.sealMaterial
        });
      }
      placeLabel(panel.panel);
      panel.panel = PanelCodec.copy(panel.panel);
    }
  }
  /** Choose a dry label rectangle from exact cut-boundary candidates. */
  static function placeLabel(panel:materia.tooling.panel.PanelPlan):Void {
    var halfW = PanelCodec.LABEL_WIDTH_MM / 2, halfH = PanelCodec.LABEL_HEIGHT_MM / 2;
    var outline = panel.operations.filter(op -> op.kind == OutsideProfile)[0].contour;
    var bounds = materia.casting.CastingProfileGeometry.bounds(outline);
    var xs = [panel.labelX, bounds.x + halfW, bounds.x + bounds.width - halfW];
    var ys = [panel.labelY, bounds.y + halfH, bounds.y + bounds.height - halfH];
    for (op in panel.operations) if (op.kind != OutsideProfile) {
      var b = materia.casting.CastingProfileGeometry.bounds(op.contour);
      xs.push(b.x - halfW);
      xs.push(b.x + b.width + halfW);
      ys.push(b.y - halfH);
      ys.push(b.y + b.height + halfH);
    }
    for (y in ys) for (x in xs) {
      if (!materia.casting.CastingProfileGeometry.containsRect(outline,
        x - halfW, y - halfH, 2 * halfW, 2 * halfH)) continue;
      var clear = true;
      for (op in panel.operations) if (op.kind != OutsideProfile) {
        var b = materia.casting.CastingProfileGeometry.bounds(op.contour);
        if (b.x < x + halfW && b.x + b.width > x - halfW && b.y < y + halfH && b.y + b.height > y - halfH) {
          clear = false;
          break;
        }
      }
      if (clear) {
        panel.labelX = x;
        panel.labelY = y;
        return;
      }
    }
    throw "No dry label rectangle on receiver panel: " + panel.id;
  }

}
