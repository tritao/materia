package robotkit.runtime;

import haxe.Int64;

/** Optional in-process RKD6 device and simulated UART for Simulation.addRobot. */
class VirtualDeviceOptions {
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
  public var fingerprint:String = "00000000000000000000000000000000";
  public var targetError:Float = 0.00001;
  public var clockBoundNs:Int64 = Int64.ofInt(5000000);
  public var linkLossTimeoutNs:Int64 = Int64.ofInt(500000000);

  public function new() {}
}
