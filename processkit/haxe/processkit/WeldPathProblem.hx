package processkit;

import motionkit.kinematics.IkTolerance;
import motionkit.kinematics.PathRequest;
import motionkit.kinematics.Pose3;
import motionkit.path.OrientationPolicy;
import motionkit.path.PoseLine;
import motionkit.path.PosePath;
import motionkit.path.PosePrimitive;
import motionkit.path.PoseWaypoint;
import motionkit.robot.ProgramCompiler;
import processkit.WeldCorner.WristLimits;
import processkit.skill.WeldPlan;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;

/** Authored geometry for one weld alternative. Selection and timing are callers'
 * responsibilities; this builder performs no IK, collision queries or retries. */
class WeldPathProblem {
  public final plan:WeldPlan;
  final styles:Array<Int>;
  final wrist:WristLimits;
  final approachSpeed:Float;
  public final seam:PosePath;
  public final path:PosePath;
  public final sections:Array<PosePath>;
  public final seamOffset:Float;
  public final seamLength:Float;
  public final approach:Pose3;
  public final retreat:Pose3;
  final contactSegments:Array<{from:Vec3,to:Vec3}>;

  public function new(plan:WeldPlan,wrist:WristLimits,styles:Array<Int>,frame:String,
      approachSpeed:Float=0.05) {
    if(plan==null || wrist==null || styles==null || styles.length!=plan.segments.length ||
        frame==null || frame=="" || !Math.isFinite(approachSpeed) || approachSpeed<=0)
      throw "Weld path problem requires a plan, corner styles, task frame and approach speed";
    for(style in styles)if(style<0 || style>=WeldCorner.STYLES)throw "Unknown weld corner style";
    this.plan=plan;this.styles=styles.copy();
    this.wrist={angularSpeed:wrist.angularSpeed,angularAcceleration:wrist.angularAcceleration};
    this.approachSpeed=approachSpeed;
    seam=new PosePath(frame,WeldingPlanRunner.pathOf(plan,plan.parameters.travelSpeed,wrist,styles));
    seamLength=seam.length();
    var first=plan.start(),last=plan.stop();
    var start=pose(first),stop=pose(last);
    approach=displaced(first,-plan.parameters.approach);
    retreat=displaced(last,-plan.parameters.approach);
    var primitives:Array<PosePrimitive> = [line(approach,start,approachSpeed)];
    seamOffset=primitives[0].length();
    for(primitive in seam.primitives)primitives.push(primitive);
    // Burnback lift is a process boundary on this same geometric retreat line;
    // engagement outputs/dwells belong to the compiler's process schedule.
    primitives.push(line(stop,retreat,approachSpeed));
    path=new PosePath(frame,primitives);
    sections=[for(section in ProgramCompiler.timingSections(path))section.path];
    contactSegments=[for(segment in plan.segments){
      from:new Vec3(segment.start.translation.x,segment.start.translation.y,segment.start.translation.z),
      to:new Vec3(segment.stop.translation.x,segment.stop.translation.y,segment.stop.translation.z)
    }];
  }

  /** The opposite travel alternative, preserving each physical corner style. */
  public function reversed():WeldPathProblem {
    var reversedStyles=[WeldCorner.AROUND];
    for(index in 1...styles.length)reversedStyles.push(styles[styles.length-index]);
    return new WeldPathProblem(plan.reversed(),wrist,reversedStyles,path.frameId,approachSpeed);
  }

  /** Author a corner alternative as geometry, without enumerating roll/entry
   * solves or compiling candidate programs. */
  public function withCornerStyles(alternative:Array<Int>):WeldPathProblem
    return new WeldPathProblem(plan,wrist,alternative,path.frameId,approachSpeed);

  /** Geometric contact permission, evaluated at actual TCP poses in the task
   * frame. It also covers the nearby approach and burnback lift. */
  public function contact(pose:Pose3):Bool {
    var point=new Vec3(pose.x,pose.y,pose.z);
    for(segment in contactSegments){
      var direction=segment.to.sub(segment.from),length2=direction.dot(direction);
      var fraction=length2==0 ? 0.0 : Math.max(0.0,Math.min(1.0,point.sub(segment.from).dot(direction)/length2));
      if(point.sub(segment.from.add(direction.scale(fraction))).norm()<=WeldPathPlanner.CONTACT_ZONE)return true;
    }
    return false;
  }

  /** One global sample grid, retaining every primitive boundary and timing stop. */
  public function request(start:Array<Float>,tolerance:IkTolerance,jump:Array<Float>,velocity:Array<Float>,
      resolution:Float=WeldPathPlanner.STEP):PathRequest {
    if(!Math.isFinite(resolution) || resolution<=0)throw "Weld sample resolution must be finite and positive";
    var distances:Array<Float> = [],poses:Array<Pose3> = [],freedoms:Array<OrientationPolicy> = [];
    var offset=0.0;
    for(section in sections){
      var local=0.0;
      for(primitive in section.primitives){
        // The outgoing primitive owns the common geometric knot.
        if(distances.length>0){distances.pop();poses.pop();freedoms.pop();}
        var length=primitive.length(),pieces=Std.int(Math.max(1,Math.ceil(length/resolution)));
        for(piece in 0...pieces+1){
          var at=piece==pieces?length:length*piece/pieces;
          distances.push(offset+local+at);poses.push(primitive.waypointAt(at).pose);
          freedoms.push(primitive.orientationPolicy());
        }
        local+=length;
      }
      distances[distances.length-1]=offset+section.length();
      offset+=section.length();
    }
    return new PathRequest(distances,poses,start,tolerance,jump,velocity,48,freedoms);
  }

  static function pose(frame:Transform3):Pose3
    return new Pose3(frame.translation.x,frame.translation.y,frame.translation.z,
      frame.rotation.x,frame.rotation.y,frame.rotation.z,frame.rotation.w);

  static function displaced(frame:Transform3,distance:Float):Pose3 {
    var point=frame.translation.add(frame.rotation.rotate(new Vec3(0,0,1)).scale(distance));
    return new Pose3(point.x,point.y,point.z,frame.rotation.x,frame.rotation.y,frame.rotation.z,frame.rotation.w);
  }

  static function line(from:Pose3,to:Pose3,speed:Float):PoseLine
    return new PoseLine(new PoseWaypoint(from,WeldingPlanRunner.PATH_TOLERANCE,0.01),
      new PoseWaypoint(to,WeldingPlanRunner.PATH_TOLERANCE,0.01),OrientationPolicy.FreeAboutTool,0.1,speed);
}
