package robotkit.runtime;

import haxe.Int64;

/** Optional in-process RKD6 device and simulated UART for Simulation.addRobot. */
class VirtualDeviceOptions {
  /** Board-defined profile data; process owners supply its meaning. */
  public var peripheralKind:Int = 0;
  public var peripheralParameters:Array<Float> = [];
  public var profile:Int = 1; // 1 full, 2 minimal
  public var tickHz:Int64 = Int64.ofInt(1000000);
  public var stepTickHz:Int = 40000;
  public var offsetTicks:Int64 = Int64.ofInt(50000);
  public var driftPpm:Int = 0;
  public var baud:Int = 921600;
  public var latencyNs:Int64 = Int64.ofInt(100000);
  public var jitterNs:Int64 = Int64.ofInt(0);
  public var frameDropRate:Float = 0.0;
  public var corruptionRate:Float = 0.0;
  public var seed:Int64 = Int64.ofInt(1);
  public var stepsPerUnit:Array<Float> = [];
  public var actuators:Array<VirtualActuatorOptions> = [];
  /** The virtual board's unique id, 32 hex digits, which a deployment names; "Virtual-Device-1" by default. */
  public var controller:String = "5669727475616c2d4465766963652d31";
  public var targetError:Float = 0.00001;
  public var clockBoundNs:Int64 = Int64.ofInt(5000000);
  public var linkLossTimeoutNs:Int64 = Int64.ofInt(500000000);

  public function new() {}
}
