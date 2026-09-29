package humankit;

/** Resolves a portable job spec into world-space HumanKit actions. */
class HumanJobBuilder {
  public static function build(spec:HumanJobSpec, targets:HumanJobTargets, body:HumanBody):HumanJobBuildResult {
    var job = new HumanJob(body);
    var holds:Array<HumanJobHold> = [];
    var heldByHand:Map<Int, String> = new Map();
    var actionSteps:Array<Int> = [];
    for (stepIndex in 0...spec.steps.length) {
      var step = spec.steps[stepIndex];
      var action:String = Reflect.field(step, "action");
      switch action {
        case "walkTo":
          var target:Dynamic = Reflect.field(step, "target");
          var id:Null<String> = Reflect.field(target, "object");
          var point:Array<Float> = id == null ? Reflect.field(target, "point") : stopAt(box(targets, id), body);
          var via:Null<Array < Array < Float>>> = Reflect.field(step, "via");
          if (via == null || via.length == 0) job.add(new WalkTo(point, 1.0));
          else job.add(WalkTo.along(via.concat([point]), 1.0));
        case "pick":
          var id:String = Reflect.field(step, "object");
          var b = box(targets, id);
          var point = [b.center[0], b.center[1], b.center[2] + b.halfExtents[2] + 0.01];
          var hands = limbs(Reflect.field(step, "hand"));
          job.add(new ApproachFor(point, hands[0], 1.0, hands.length == 2));
          var pick = new Pick(point, hands);
          job.add(pick);
          for (hand in hands) holds.push({action: pick, objectId: id, grasp: point.copy(), hand: hand}
          );
          for (hand in hands) heldByHand.set(hand, id);
        case "place":
          var heldHands = limbs(Reflect.field(step, "hand"));
          var heldId = heldByHand.get(heldHands[0]);
          if (heldId == null) throw "Place needs a preceding pick";
          var supportId:String = Reflect.field(step, "onto");
          var support = box(targets, supportId);
          var held = box(targets, heldId);
          var offset:Null<Array < Float>> = Reflect.field(step, "offset");
          if (offset == null) offset = [0.0, 0.0];
          var c = Math.cos(support.yaw), s = Math.sin(support.yaw);
          var point = [
            support.center[0] + c * offset[0] - s * offset[1],
            support.center[1] + s * offset[0] + c * offset[1],
            support.center[2] + support.halfExtents[2] + held.halfExtents[2]
          ];
          job.add(new ApproachFor(point, heldHands[0], 1.0, heldHands.length == 2));
          var place = new Place(point, heldHands);
          job.add(place);
          for (hand in heldHands) holds.push({action: place, objectId: heldId, grasp: point.copy(), hand: hand}
          );
          var root = body.rootTransform();
          var dx = root[12] - point[0], dy = root[13] - point[1];
          var length = Math.sqrt(dx * dx + dy * dy);
          if (length < 0.000001) {
            dx = -root[0];
            dy = -root[1];
            length = 1.0;
          }
          job.add(new WalkTo([point[0] + dx / length * 0.8, point[1] + dy / length * 0.8], 1.0));
          for (hand in heldHands) heldByHand.remove(hand);
        case "press":
          var target:Dynamic = Reflect.field(step, "target");
          var id:Null<String> = Reflect.field(target, "object");
          var point:Array<Float>;
          if (id == null) {
            var xy:Array<Float> = Reflect.field(target, "point");
            point = [xy[0], xy[1], 1.0];
          } else point = anchor(box(targets, id), Reflect.field(target, "anchor"), body);
          var hands = limbs(Reflect.field(step, "hand"));
          job.add(new ApproachFor(point, hands[0], 1.0, hands.length == 2));
          job.add(new Press(point, hands[0]));
        case "wait":
          job.add(new Wait(Reflect.field(step, "seconds")));
        case "playClip":
          job.add(new PlayClip(Reflect.field(step, "clip"), Reflect.field(step, "seconds")));
      }
      while (actionSteps.length < job.orderedActions().length) actionSteps.push(stepIndex);
    }
    return {job: job, holds: holds, actionSteps: actionSteps};
  }

  static function box(targets:HumanJobTargets, id:String):HumanTargetBox {
    var result = targets.box(id);
    if (result == null) throw 'Unknown job object "$id"';
    return result;
  }

  static function limbs(hand:String):Array < HumanLimb > return hand == "both" ?[ArmL,
    ArmR] : hand == "left" ?[ArmL] :[ArmR];

  /** Nearest point outside the oriented footprint, half a metre clear. */
  static function stopAt(box:HumanTargetBox, body:HumanBody):Array < Float > {
    var root = body.rootTransform();
    var c = Math.cos(box.yaw), s = Math.sin(box.yaw);
    var dx = root[12] - box.center[0], dy = root[13] - box.center[1];
    var lx = c * dx + s * dy, ly = -s * dx + c * dy;
    var px = Math.max(-box.halfExtents[0], Math.min(box.halfExtents[0], lx));
    var py = Math.max(-box.halfExtents[1], Math.min(box.halfExtents[1], ly));
    var nx = lx - px, ny = ly - py;
    var length = Math.sqrt(nx * nx + ny * ny);
    if (length < 0.000001) {
      nx = lx >= 0 ? 1.0 : -1.0;
      ny = 0.0;
      length = 1.0;
    }
    px += nx / length * 0.5;
    py += ny / length * 0.5;
    return [box.center[0] + c * px - s * py, box.center[1] + s * px + c * py];
  }

  static function anchor(box:HumanTargetBox, name:Null<String>, body:HumanBody):Array < Float> {
    if (name == "top") return [box.center[0], box.center[1], box.center[2] + box.halfExtents[2]];
    if (name == "front") {
      var root = body.rootTransform();
      var dx = root[12] - box.center[0], dy = root[13] - box.center[1];
      var c = Math.cos(box.yaw), s = Math.sin(box.yaw);
      var lx = c * dx + s * dy, ly = -s * dx + c * dy;
      if (Math.abs(lx / box.halfExtents[0]) >= Math.abs(ly / box.halfExtents[1])) return [
        box.center[0] + c * box.halfExtents[0] *(lx >= 0 ? 1 : -1),
        box.center[1] + s * box.halfExtents[0] *(lx >= 0 ? 1 : -1),
        box.center[2]
      ];
      return [
        box.center[0] - s * box.halfExtents[1] *(ly >= 0 ? 1 : -1),
        box.center[1] + c * box.halfExtents[1] *(ly >= 0 ? 1 : -1),
        box.center[2]
      ];
    }
    return box.center.copy();
  }
}
