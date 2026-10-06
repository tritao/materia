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
import motionkit.robot.StructuredJointPathPlanner;
import motionkit.robot.ManipulatorKinematics;
import motionkit.robot.CandidateProblem.CandidateSamplingOptions;
import motionkit.planner.JointPathSamples;
import robotkit.manipulation.KinematicGroup;
import robotkit.manipulation.ArmClearance;
import processkit.WeldCorner.WristLimits;
import processkit.skill.WeldPlan;
import robotkit.spatial.Transform3;
import robotkit.spatial.Vec3;

/** Authored geometry for one weld alternative. Construction performs no IK or
 * collision queries. select delegates the complete problem to the shared ladder;
 * timing and process engagement remain the caller's responsibilities. */
class WeldPathProblem {
  public final plan:WeldPlan;
  final styles:Array<Int>;
  final wrist:WristLimits;
  final approachSpeed:Float;
  final airClearance:Float;
  public final seam:PosePath;
  public final path:PosePath;
  public final sections:Array<PosePath>;
  public final phases:Array<WeldPathPhase>;
  public final seamOffset:Float;
  public final seamLength:Float;
  /** Distance in the original seam where this execution begins. */
  public final startDistance:Float;
  public final fullSeamLength:Float;
  public final seamEnd:Float;
  public final burnbackEnd:Float;
  public final lift:Pose3;
  public final approach:Pose3;
  public final retreat:Pose3;
  final contactSegments:Array<{from:Vec3,to:Vec3}>;

  public function new(plan:WeldPlan,wrist:WristLimits,styles:Array<Int>,frame:String,
      approachSpeed:Float=0.05,startDistance:Float=0.0,airClearance:Float=0.0) {
    if(plan==null || wrist==null || styles==null || styles.length!=plan.segments.length ||
        frame==null || frame=="" || !Math.isFinite(approachSpeed) || approachSpeed<=0 ||
        !Math.isFinite(airClearance) || airClearance<0)
      throw "Weld path problem requires a plan, corner styles, task frame and approach speed";
    for(style in styles)if(style<0 || style>=WeldCorner.STYLES)throw "Unknown weld corner style";
    this.plan=plan;this.styles=styles.copy();
    this.wrist={angularSpeed:wrist.angularSpeed,angularAcceleration:wrist.angularAcceleration};
    this.approachSpeed=approachSpeed;this.airClearance=airClearance;
    var authored=new PosePath(frame,WeldingPlanRunner.pathOf(plan,plan.parameters.travelSpeed,wrist,styles));
    if(!Math.isFinite(startDistance) || startDistance<0 || startDistance>=authored.length())
      throw "Weld restart distance lies outside the authored seam";
    this.startDistance=startDistance;fullSeamLength=authored.length();
    seam=startDistance==0 ? authored : ProcessPathSlice.from(authored,startDistance);
    seamLength=seam.length();
    var first=plan.start(),last=plan.stop();
    var start=seam.poseAt(0),stop=pose(last);
    var restartFrame=new Transform3(new Vec3(start.x,start.y,start.z),
      new robotkit.spatial.Quat(start.qx,start.qy,start.qz,start.qw));
    var nearEntry=displaced(restartFrame,-plan.parameters.approach);
    var nearExit=displaced(last,-plan.parameters.approach);
    approach=clearAir(nearEntry,startDistance==0 ? plan.segments[0].open : openAt(start),airClearance);
    retreat=clearAir(nearExit,plan.segments[plan.segments.length-1].open,airClearance);
    var entryLines:Array<PosePrimitive> = [];
    if(airClearance>0)entryLines.push(line(approach,nearEntry,approachSpeed));
    entryLines.push(line(nearEntry,start,approachSpeed));
    lift=displaced(last,-WeldPathPlanner.LIFT);
    var groups:Array<{phase:WeldPathPhase,path:PosePath}> = [
      {phase:Approach,path:new PosePath(frame,entryLines)},
      {phase:Weld,path:seam},
      {phase:Burnback,path:new PosePath(frame,[line(stop,lift,
        Math.max(WeldPathPlanner.LIFT/plan.parameters.burnback,0.01))])}
    ];
    seamOffset=groups[0].path.length();
    seamEnd=seamOffset+seamLength;
    burnbackEnd=seamEnd+groups[2].path.length();
    // Exact process stops remain even if neighboring tangents/speeds agree.
    // A short approach may place the retreat inside the burnback lift: preserve
    // the resulting reverse travel rather than replacing it with one line.
    if(motionkit.path.PoseMath.distance(lift,retreat)>1e-12){
      var exitLines:Array<PosePrimitive> = [];
      if(airClearance>0){
        if(motionkit.path.PoseMath.distance(lift,nearExit)>1e-12)exitLines.push(line(lift,nearExit,approachSpeed));
        exitLines.push(line(nearExit,retreat,approachSpeed));
      }else exitLines.push(line(lift,retreat,approachSpeed));
      groups.push({phase:Retreat,path:new PosePath(frame,exitLines)});
    }
    var primitives:Array<PosePrimitive> = [];
    sections=[];phases=[];
    for(group in groups){
      for(primitive in group.path.primitives)primitives.push(primitive);
      for(section in ProgramCompiler.timingSections(group.path)){
        sections.push(section.path);phases.push(group.phase);
      }
    }
    path=new PosePath(frame,primitives);
    contactSegments=[for(segment in plan.segments){
      from:new Vec3(segment.start.translation.x,segment.start.translation.y,segment.start.translation.z),
      to:new Vec3(segment.stop.translation.x,segment.stop.translation.y,segment.stop.translation.z)
    }];
  }

  public function cornerStyles():Array<Int> return styles.copy();

  /** Explicit geometric air route along the CAD seam's open side. Selection
   * must still check the complete entry, route and retreat against the cell. */
  public function withAirClearance(distance:Float):WeldPathProblem
    return new WeldPathProblem(plan,wrist,styles,path.frameId,approachSpeed,startDistance,distance);

  function openAt(pose:Pose3):Null<Vec3> {
    var point=new Vec3(pose.x,pose.y,pose.z),nearest=Math.POSITIVE_INFINITY,result:Null<Vec3> = null;
    for(segment in plan.segments){
      var from=segment.start.translation,direction=segment.stop.translation.sub(from),length2=direction.dot(direction);
      var fraction=length2==0 ? 0.0 : Math.max(0.0,Math.min(1.0,point.sub(from).dot(direction)/length2));
      var distance=point.sub(from.add(direction.scale(fraction))).norm();
      if(distance<=nearest){nearest=distance;result=segment.open;}
    }
    return result;
  }

  static function clearAir(pose:Pose3,open:Null<Vec3>,distance:Float):Pose3 {
    if(distance==0)return pose;
    if(open==null || open.norm()<1e-12)throw "Air entry requires a declared seam open side";
    var offset=open.normalized().scale(distance);
    return new Pose3(pose.x+offset.x,pose.y+offset.y,pose.z+offset.z,pose.qx,pose.qy,pose.qz,pose.qw);
  }

  /** Resume the same authored geometry; corner turns and weave phase stay at
   * their original coordinates rather than being regenerated from a shorter plan. */
  public function recovery(distance:Float):WeldPathProblem
    return new WeldPathProblem(plan,wrist,styles,path.frameId,approachSpeed,distance,airClearance);

  /** The opposite travel alternative, preserving each physical corner style. */
  public function reversed():WeldPathProblem {
    var reversedStyles=[WeldCorner.AROUND];
    for(index in 1...styles.length)reversedStyles.push(styles[styles.length-index]);
    return new WeldPathProblem(plan.reversed(),wrist,reversedStyles,path.frameId,approachSpeed,0,airClearance);
  }

  /** Author a corner alternative as geometry, without enumerating roll/entry
   * solves or compiling candidate programs. */
  public function withCornerStyles(alternative:Array<Int>):WeldPathProblem
    return new WeldPathProblem(plan,wrist,alternative,path.frameId,approachSpeed,0,airClearance);

  /** Select this complete geometry from the measured robot state. The default
   * leaves the approach start free; external lattice/rules and motion weights
   * remain explicit inputs from the cell/process. No candidate is timed. */
  public function select(group:KinematicGroup,request:PathRequest,?sampling:CandidateSamplingOptions,
      ?clearance:ArmClearance,
      ?entryCheck:(Array<Float>,Array<Float>)->Null<ArmClearance.ClearanceViolation>,
      ?exitCheck:Array<Float>->Null<ArmClearance.ClearanceViolation>,
      ?retreatJoints:Array<Float>,?weights:Array<Float>,rollWeight:Float=0,
      ?preferences:ManipulatorKinematics):Array<JointPathSamples> {
    return selectWithCost(group,request,sampling,clearance,entryCheck,exitCheck,
      retreatJoints,weights,rollWeight,preferences).curves;
  }

  public function selectWithCost(group:KinematicGroup,request:PathRequest,?sampling:CandidateSamplingOptions,
      ?clearance:ArmClearance,
      ?entryCheck:(Array<Float>,Array<Float>)->Null<ArmClearance.ClearanceViolation>,
      ?exitCheck:Array<Float>->Null<ArmClearance.ClearanceViolation>,
      ?retreatJoints:Array<Float>,?weights:Array<Float>,rollWeight:Float=0,
      ?preferences:ManipulatorKinematics):WeldPathSelection {
    var options=sampling==null ? new CandidateSamplingOptions(8,3,8,false) : sampling;
    var source=preferences;
    if(source==null){source=new ManipulatorKinematics(group);source.preferTargetOrientation=true;}
    var planner=new StructuredJointPathPlanner(group,options,null,clearance,8,false,
      null,retreatJoints,weights,rollWeight,source,contact,contactNeighborhood);
    var curves=planner.planSections(sections,request,null,entryCheck,exitCheck);
    return new WeldPathSelection(this,curves,cast(planner.selectionCost,Float));
  }

  /** Geometric contact permission, evaluated at actual TCP poses in the task
   * frame. It also covers the nearby approach and burnback lift. */
  public function contact(pose:Pose3):Bool return contactNeighborhood(pose,0.0);

  /** Distance to a fixed seam is 1-Lipschitz in TCP position. */
  public function contactNeighborhood(pose:Pose3,displacement:Float):Bool {
    if(!Math.isFinite(displacement) || displacement<0 || displacement>WeldPathPlanner.CONTACT_ZONE)return false;
    var point=new Vec3(pose.x,pose.y,pose.z);
    for(segment in contactSegments){
      var direction=segment.to.sub(segment.from),length2=direction.dot(direction);
      var fraction=length2==0 ? 0.0 : Math.max(0.0,Math.min(1.0,point.sub(segment.from).dot(direction)/length2));
      if(point.sub(segment.from.add(direction.scale(fraction))).norm()<=WeldPathPlanner.CONTACT_ZONE-displacement)return true;
    }
    return false;
  }

  /** One global sample grid, retaining every primitive boundary and timing stop. */
  public function request(start:Array<Float>,tolerance:IkTolerance,jump:Array<Float>,velocity:Array<Float>,
      resolution:Float=WeldPathPlanner.STEP,?acceleration:Array<Float>):PathRequest {
    if(!Math.isFinite(resolution) || resolution<=0)throw "Weld sample resolution must be finite and positive";
    var distances:Array<Float> = [],poses:Array<Pose3> = [],freedoms:Array<OrientationPolicy> = [],feeds:Array<Float> = [];
    var offset=0.0;
    for(sectionIndex in 0...sections.length){
      var section=sections[sectionIndex],local=0.0;
      for(primitive in section.primitives){
        // The outgoing primitive owns the common geometric knot.
        if(distances.length>0){distances.pop();poses.pop();freedoms.pop();feeds.pop();}
        var length=primitive.length(),pieces=motionkit.path.PoseSampling.pieces(primitive,resolution);
        for(piece in 0...pieces+1){
          var at=piece==pieces?length:length*piece/pieces;
          distances.push(offset+local+at);poses.push(primitive.waypointAt(at).pose);
          freedoms.push(primitive.orientationPolicy());
          feeds.push(Math.min(primitive.speedLimit(),phases[sectionIndex]==Weld?plan.parameters.travelSpeed:primitive.speedLimit()));
        }
        local+=length;
      }
      distances[distances.length-1]=offset+section.length();
      offset+=section.length();
    }
    return new PathRequest(distances,poses,start,tolerance,jump,velocity,48,freedoms,
      acceleration==null?null:new motionkit.kinematics.PathDriveLimits(acceleration,feeds));
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

/** Engagement phase for each stopped timing section of a joined weld problem. */
enum WeldPathPhase {
  Approach;
  Weld;
  Burnback;
  Retreat;
}

/** One complete geometric choice and its ladder objective; no timing artifact. */
class WeldPathSelection {
  public final problem:WeldPathProblem;
  public final curves:Array<JointPathSamples>;
  public final cost:Float;
  public function new(problem:WeldPathProblem,curves:Array<JointPathSamples>,cost:Float){
    if(problem==null || curves==null || !Math.isFinite(cost))throw "Weld selection requires a finite complete route";
    this.problem=problem;this.curves=curves;this.cost=cost;
  }
}
