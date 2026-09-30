package materia.assembly;

import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinition.AssemblyJointType;

/**
 * A rigid body of an assembly: the occurrences that fixed joints hold together. Moving joints
 * (revolute, continuous, prismatic) separate bodies, so a body is what actually moves as one piece.
 */
typedef AssemblyBody = {
  /** The occurrence at the body's root; it names the body. */
  var id:String;
  /** Every occurrence in the body: the root first, then the rest in definition order. */
  var occurrences:Array<String>;
  /** The body this one hangs from, or null for a body that no joint carries. */
  var parent:Null<String>;
  /** The moving joint that carries this body, or null when there is none. */
  var joint:Null<String>;
  var jointType:Null<AssemblyJointType>;
}

/** Groups an assembly's occurrences into rigid bodies, for hierarchy views and simulation links. */
class AssemblyBodies {
  /** Bodies in the order their first occurrence appears; only tree joints connect occurrences. */
  public static function of(definition:AssemblyDefinition):Array<AssemblyBody> {
    var incoming = new Map<String, KinematicJoint>();
    for (joint in definition.joints) if (joint.role == AssemblyJointRole.Tree) incoming.set(joint.child, joint);
    var limit = definition.occurrences.length;
    function rootOf(id:String):String {
      var current = id;
      for (_ in 0...limit + 1) {
        var joint = incoming.get(current);
        if (joint == null || joint.type != AssemblyJointType.Fixed) return current;
        current = joint.parent;
      }
      throw 'Assembly joints form a cycle at "$id"';
    }
    var bodies:Array<AssemblyBody> = [];
    var byRoot = new Map<String, AssemblyBody>();
    for (occurrence in definition.occurrences) {
      var root = rootOf(occurrence.id);
      var body = byRoot.get(root);
      if (body == null) {
        var carrier = incoming.get(root);
        body = {id: root, occurrences: [root], parent: carrier == null ? null : rootOf(carrier.parent),
          joint: carrier == null ? null : carrier.id, jointType: carrier == null ? null : carrier.type};
        byRoot.set(root, body);
        bodies.push(body);
      }
      if (occurrence.id != root) body.occurrences.push(occurrence.id);
    }
    return bodies;
  }

  /**
   * A readable name for an occurrence id: each path segment is split at capitals and digits and
   * capitalised, and segments are joined with " › " (`tool/suctionCup` reads `Tool › Suction cup`).
   */
  public static function displayName(id:String):String {
    var parts:Array<String> = [];
    for (segment in id.split("/")) if (segment.length > 0) parts.push(words(segment));
    return parts.length == 0 ? id : parts.join(" › ");
  }

  static function isDigit(c:String):Bool {
    if (c.length != 1) return false;
    var code = StringTools.fastCodeAt(c, 0);
    return code >= 48 && code <= 57;
  }

  static function isUpper(c:String):Bool return c != c.toLowerCase();

  /** Splits at a capital after a lower-case letter and at every letter/digit change. */
  static function words(segment:String):String {
    var out = new StringBuf();
    var previous = "";
    for (index in 0...segment.length) {
      var c = segment.charAt(index);
      if (c == "-" || c == "_") {
        out.add(" ");
        previous = " ";
        continue;
      }
      var boundary = previous != "" && previous != " " &&
        ((isUpper(c) && !isUpper(previous)) || isDigit(c) != isDigit(previous));
      if (boundary) out.add(" ");
      out.add(c.toLowerCase());
      previous = c;
    }
    var text = out.toString();
    return text.charAt(0).toUpperCase() + text.substr(1);
  }
}
