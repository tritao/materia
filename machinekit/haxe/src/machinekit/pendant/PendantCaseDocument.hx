package machinekit.pendant;

import cadkit.modeling.Footprint;
import cadkit.modeling.FootprintGeometry;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.parametric.Document;
import cadkit.parametric.Feature;
import cadkit.parametric.recording.DocumentBuilder;
import machinekit.component.PanelBoss;
import machinekit.hmi.PanelFacePlate;
import machinekit.hmi.PanelRecorder;

/**
 * The pendant's shell, plate and board envelope recorded as parametric documents, one labelled and grouped feature per
 * operation, in the order `PendantCase` builds them. The planner's placements are explicit feature inputs; only the
 * case's own size is named, since that is the design intent.
 */
class PendantCaseDocument {
  public final document:Document;

  function new(document:Document) {
    this.document = document;
  }

  /** A new solid for the document's result; the caller closes it. Its faces keep the document's face numbering. */
  public function part():Part return new Part(document.result().cloneShape());

  /** `sizeEditable`: the recording is the plate's recipe, whose inputs the size is; otherwise the size follows it. */
  public static function recordShell(b:DocumentBuilder, pendantCase:PendantCase, sizeEditable:Bool = false):Feature {
    var face = pendantCase.face;
    var depth = pendantCase.outerDepth();
    var r = new PanelRecorder(b);
    var width = b.length("width", face.width), height = b.length("height", face.height);
    if (!sizeEditable) for (named in [width, height]) named.derivedBy("the pendant's size: its face plate's recipe");
    var corner = PanelFacePlate.CORNER_RADIUS;
    var body:Feature = null;

    b.group("Body", function() {
      var outer = r.rounded("Outer shape", width, height, corner, -depth, 0.0);
      var cavity = r.rounded(
        "Cavity",
        face.width - 2 * face.wall,
        face.height - 2 * face.wall,
        corner - face.wall,
        -face.caseInnerDepth,
        1.0
      );
      body = r.cut("Hollow the case", outer, cavity);
    });

    b.group("Cable and strap", function() {
      var receptacle = r.cylinderTurned(
        "Receptacle hole tool",
        PendantCase.RECEPTACLE_HOLE / 2,
        face.wall + 2.0,
        new Vector(1, 0, 0),
        -Math.PI / 2,
        pendantCase.connectorX,
        -face.height / 2 - 1.0,
        pendantCase.connectorZ
      );
      body = r.cut("Cable receptacle hole", body, receptacle);
      for (s in pendantCase.strapSlots) {
        var slot = r.boxAt(
          "Strap slot tool",
          s.x - PendantCase.STRAP_SLOT_WIDTH / 2,
          s.y - PendantCase.STRAP_SLOT_HEIGHT / 2,
          -depth - 1.0,
          PendantCase.STRAP_SLOT_WIDTH,
          PendantCase.STRAP_SLOT_HEIGHT,
          depth - face.caseInnerDepth + 2.0
        );
        body = r.cut("Strap slot", body, slot);
      }
      if (pendantCase.enabling != null) {
        var side = pendantCase.enabling.x < 0 ? -1.0 : 1.0;
        var tool = r.cylinderTurned(
          "Enabling switch hole tool",
          PendantCase.ENABLING_HOLE / 2,
          face.wall + 2.0,
          new Vector(0, 1, 0),
          -side * Math.PI / 2,
          side * (face.width / 2 + 1.0),
          pendantCase.enabling.y,
          PendantCase.enablingZ(face)
        );
        body = r.cut("Enabling switch opening", body, tool);
      }
    });

    b.group("Fasteners", function() {
      for (p in pendantCase.posts) {
        var post = r.cylinderZ("Post", PendantCase.POST_DIAMETER / 2, -face.caseInnerDepth - 0.5, 0.0, p.x, p.y);
        var pilot = r.cylinderZ("Post pilot", PendantCase.PILOT_DIAMETER / 2, -PendantCase.PILOT_DEPTH, 1.0, p.x, p.y);
        body = r.cut("Post pilot hole", r.fuse("Screw post", body, post), pilot);
      }
    });

    b.group("Reinforcement", function() {
      var top = PendantCase.ribTop(pendantCase.boardTop);
      for (rib in pendantCase.ribs) body = r.fuse(
        "Rib",
        body,
        r.boxAt(
          "Rib",
          rib.x - rib.w / 2,
          rib.y - rib.h / 2,
          -face.caseInnerDepth - 0.5,
          rib.w,
          rib.h,
          top + face.caseInnerDepth + 0.5
        )
      );
    });
    return body;
  }

  public static function shell(pendantCase:PendantCase):PendantCaseDocument {
    return new PendantCaseDocument(DocumentBuilder.build(b -> b.output(recordShell(b, pendantCase))));
  }

  /** The face plate: bosses on its back, every control's cutout and holes, the screw holes, and the perimeter tongue. */
  /** `sizeEditable`: the recording is the plate's recipe, whose inputs the size is; otherwise the size follows it. */
  public static function recordPlate(b:DocumentBuilder, pendantCase:PendantCase, sizeEditable:Bool = false):Feature {
    var face = pendantCase.face;
    var r = new PanelRecorder(b);
    var width = b.length("width", face.width), height = b.length("height", face.height);
    if (!sizeEditable) for (named in [width, height]) named.derivedBy("the pendant's size: its face plate's recipe");
    var corner = PanelFacePlate.CORNER_RADIUS;
    var body:Feature = null;

    b.group("Plate", function() {
      body = r.rounded("Face plate", width, height, corner, 0.0, face.thickness);
    });

    var bosses:Array<PanelBoss> = pendantCase.bosses();
    for (c in face.controls) for (boss in c.bosses()) bosses.push({
      x: c.x + boss.x,
      y: c.y + boss.y,
      diameter: boss.diameter,
      height: boss.height,
      pilotDiameter: boss.pilotDiameter,
      pilotDepth: boss.pilotDepth
    });
    b.group("Bosses", function() {
      for (boss in bosses) body = r.fuse(
        "Boss",
        body,
        r.cylinderZ("Boss", boss.diameter / 2, -boss.height, 0.5, boss.x, boss.y)
      );
      for (boss in bosses) body = r.cut(
        "Boss pilot hole",
        body,
        r.cylinderZ(
          "Boss pilot",
          boss.pilotDiameter / 2,
          -boss.height - 1.0,
          -boss.height + boss.pilotDepth,
          boss.x,
          boss.y
        )
      );
    });

    b.group("Control cutouts", function() {
      for (c in face.controls) {
        body = r.cut("Cutout for " + c.id, body, r.footprint("Cutout tool", c.cutout(), c.x, c.y, -1.0, face.thickness + 1.0));
        for (hole in c.holes()) body = r.cut(
          "Mounting hole for " + c.id,
          body,
          r.footprint("Hole tool", Circle(hole.diameter), c.x + hole.x, c.y + hole.y, -1.0, face.thickness + 1.0)
        );
      }
    });

    b.group("Fasteners", function() {
      for (p in pendantCase.posts) {
        body = r.cut(
          "Screw hole",
          body,
          r.cylinderZ("Screw hole tool", PanelFacePlate.SCREW_HOLE / 2, -1.0, face.thickness + 1.0, p.x, p.y)
        );
        body = r.cut(
          "Counterbore",
          body,
          r.cylinderZ(
            "Counterbore tool",
            PanelFacePlate.COUNTERBORE / 2,
            face.thickness - PanelFacePlate.COUNTERBORE_DEPTH,
            face.thickness + 1.0,
            p.x,
            p.y
          )
        );
      }
    });

    b.group("Perimeter tongue", function() {
      var inset = face.wall + PendantCase.TONGUE_CLEARANCE;
      var radius = Math.max(corner - inset, 1.0);
      var tongue = r.rounded(
        "Tongue outline",
        face.width - 2 * inset,
        face.height - 2 * inset,
        radius,
        -PendantCase.TONGUE_HEIGHT,
        0.0
      );
      var inner = r.rounded(
        "Tongue inner",
        face.width - 2 * (inset + PendantCase.TONGUE_WIDTH),
        face.height - 2 * (inset + PendantCase.TONGUE_WIDTH),
        Math.max(radius - PendantCase.TONGUE_WIDTH, 0.5),
        -PendantCase.TONGUE_HEIGHT - 1.0,
        1.0
      );
      tongue = r.cut("Hollow the tongue", tongue, inner);
      for (c in face.controls) tongue = r.cut(
        "Tongue relief for " + c.id,
        tongue,
        r.footprint(
          "Relief tool",
          FootprintGeometry.grow(c.body(), 1.0),
          c.x,
          c.y,
          -PendantCase.TONGUE_HEIGHT - 1.0,
          1.0
        )
      );
      for (p in pendantCase.posts) tongue = r.cut(
        "Tongue relief for post",
        tongue,
        r.footprint(
          "Relief tool",
          Circle(PendantCase.POST_DIAMETER + 2.0),
          p.x,
          p.y,
          -PendantCase.TONGUE_HEIGHT - 1.0,
          1.0
        )
      );
      body = r.fuse("Join the tongue", body, tongue);
    });
    return body;
  }

  public static function plate(pendantCase:PendantCase):PendantCaseDocument {
    return new PendantCaseDocument(DocumentBuilder.build(b -> b.output(recordPlate(b, pendantCase))));
  }

  /** The board envelope: its outline, every notch and the mounting holes. */
  /** `sizeEditable`: the recording is the plate's recipe, whose inputs the size is; otherwise the size follows it. */
  public static function recordBoard(b:DocumentBuilder, pendantCase:PendantCase, sizeEditable:Bool = false):Feature {
    var top = pendantCase.boardTop;
    var r = new PanelRecorder(b);
    var half = PendantCase.boardHalf(pendantCase.face);
    var body:Feature = null;
    b.group("Board", function() {
      body = r.rounded("Board outline", half.x * 2, half.y * 2, 3.0, top - PendantCase.BOARD_THICKNESS, top);
      for (n in pendantCase.notches()) body = r.cut(
        "Notch",
        body,
        r.footprint("Notch tool", n.f, n.x, n.y, top - PendantCase.BOARD_THICKNESS - 1.0, top + 1.0)
      );
      for (s in pendantCase.standoffs) body = r.cut(
        "Mounting hole",
        body,
        r.cylinderZ("Mounting hole tool", PendantCase.BOARD_HOLE / 2, top - PendantCase.BOARD_THICKNESS - 1.0, top + 1.0, s.x, s.y)
      );
    });
    return body;
  }

  public static function board(pendantCase:PendantCase):PendantCaseDocument {
    return new PendantCaseDocument(DocumentBuilder.build(b -> b.output(recordBoard(b, pendantCase))));
  }
}
