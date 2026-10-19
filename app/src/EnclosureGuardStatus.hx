package app;

/** Transport and initialization are separate from the guard's mechanical phase. */
enum abstract EnclosureRuntimeState(String) {
  var Initializing = "initializing";
  var Paused = "paused";
  /** Running with startup feedback confirmed; the guard phase still governs each action. */
  var Ready = "ready";
  var Faulted = "faulted";
}

typedef EnclosureGuardObservation = {
  var phase:String;
  var message:String;
  var position:Float;
  var closed:Bool; var open:Bool; var locked:Bool;
  var spindleStandstill:Bool; var axesStandstill:Bool;
  var canStart:Bool; var allowCut:Bool; var fault:Bool; var interrupted:Bool;
  var feedbackSampled:Bool; var initialized:Bool; var doorMoving:Bool;
}

/** Immutable published controller image, including presentation state and permitted operator actions. */
class EnclosureGuardStatus {
  final observation:EnclosureGuardObservation;
  public final state:EnclosureRuntimeState;
  public final running:Bool;
  public final phase:String;
  public final message:String;
  public final doorMessage:String;
  public final motionMessage:String;
  public final position:Float;
  public final closed:Bool; public final open:Bool; public final locked:Bool;
  public final spindleStandstill:Bool; public final axesStandstill:Bool;
  /** Guard readiness independent of playback, also useful to deterministic stepping clients. */
  public final canStart:Bool;
  public final allowCut:Bool; public final fault:Bool; public final interrupted:Bool;
  public final feedbackSampled:Bool; public final initialized:Bool;
  public final canOpen:Bool; public final canClose:Bool; public final canStartCut:Bool;
  public final canReset:Bool; public final canResume:Bool; public final canResetSimulation:Bool;

  public function new(o:EnclosureGuardObservation,running:Bool) {
    observation={phase:o.phase,message:o.message,position:o.position,closed:o.closed,open:o.open,locked:o.locked,
      spindleStandstill:o.spindleStandstill,axesStandstill:o.axesStandstill,canStart:o.canStart,allowCut:o.allowCut,
      fault:o.fault,interrupted:o.interrupted,feedbackSampled:o.feedbackSampled,initialized:o.initialized,doorMoving:o.doorMoving};
    this.running=running;phase=o.phase;position=o.position;
    closed=o.closed;open=o.open;locked=o.locked;
    spindleStandstill=o.spindleStandstill;axesStandstill=o.axesStandstill;
    canStart=o.canStart;allowCut=o.allowCut;fault=o.fault;interrupted=o.interrupted;
    feedbackSampled=o.feedbackSampled;initialized=o.initialized;
    state=fault?Faulted:!running?Paused:!initialized?Initializing:Ready;
    message=fault?o.message:!running?"Simulation paused. Resume to operate the doors.":
      !initialized?"Initializing guard: waiting for measured feedback and standstill.":o.message;
    doorMessage=!feedbackSampled?"Door and lock feedback have not been sampled yet.":
      'Door ${open?"open":closed?"closed":running&&o.doorMoving?"moving":"between end switches"} · ${locked?"locked":"unlocked"}';
    motionMessage=!feedbackSampled?"Spindle and axis feedback have not been sampled yet.":
      'Spindle ${spindleStandstill?"stopped":"stop unconfirmed"} · axes ${axesStandstill?"stopped":"stop unconfirmed"}';
    canOpen=state==Ready&&(phase=="Secured"||phase=="Idle");
    canClose=state==Ready&&phase=="Open";
    canStartCut=state==Ready&&canStart;
    canReset=running&&fault&&closed&&locked&&axesStandstill&&spindleStandstill;
    canResume=!running;canResetSimulation=interrupted||fault;
  }

  /** Publish a transport transition without reading worker-owned mutable controller inputs. */
  public function withRunning(value:Bool):EnclosureGuardStatus return new EnclosureGuardStatus(observation,value);
}
