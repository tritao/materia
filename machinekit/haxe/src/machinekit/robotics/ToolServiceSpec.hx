package machinekit.robotics;

/** A proposed harness contract, not a vendor connector pinout or a robot OEM tool-I/O rating. */
class ToolServiceSpec {
  public final voltage:Float;
  public final totalCurrent:Float;
  public final branchCurrent:Float;
  public final protocol:String;
  public final connectorKey:String;
  public final pins:Int;
  public final pinoutVerified:Bool;
  public function new(voltage:Float = 24,totalCurrent:Float = 5,branchCurrent:Float = 2,
      protocol:String = "Modbus RTU / RS485",connectorKey:String = "provisional-tool-service-v1",pins:Int = 8) {
    for(v in [voltage,totalCurrent,branchCurrent]) if(!Math.isFinite(v) || v<=0) throw "Tool service needs positive finite electrical ratings";
    if(branchCurrent>totalCurrent || protocol==null || protocol.length==0 || connectorKey==null || connectorKey.length==0 || pins<4)
      throw "Invalid tool-service contract";
    this.voltage=voltage;this.totalCurrent=totalCurrent;this.branchCurrent=branchCurrent;
    this.protocol=protocol;this.connectorKey=connectorKey;this.pins=pins;
    pinoutVerified=false;
  }
  public function key():String return connectorKey+'-$voltage-V-$protocol';
  /** Validate a proposed complete circuit, including simultaneous peaks and unique bus addresses.
      This deliberately does not claim pinout verification, cable sizing or a commissioned bus. */
  public function checkLoads(sourceVoltage:Float,sourceCurrent:Float,loads:Array<ToolServiceLoad>):Float {
    if(!Math.isFinite(sourceVoltage) || Math.abs(sourceVoltage-voltage)>0.001 || !Math.isFinite(sourceCurrent) || sourceCurrent<=0)
      throw "Tool-service source voltage/current mismatch";
    if(loads==null || loads.length==0) throw "Tool service needs at least one load";
    var total=0.0,addresses=new Map<Int,Bool>();
    for(load in loads) {
      if(load==null || load.id==null || load.id.length==0 || !Math.isFinite(load.voltage) || Math.abs(load.voltage-voltage)>0.001 || load.protocol!=protocol)
        throw "Tool-service load voltage/protocol mismatch";
      if(!Math.isFinite(load.peakCurrent) || load.peakCurrent<=0 || load.peakCurrent>branchCurrent)
        throw "Tool-service branch peak current exceeds its design rating";
      if(load.address<1 || load.address>247 || addresses.exists(load.address)) throw "Tool-service bus addresses must be unique in 1..247";
      addresses.set(load.address,true);total+=load.peakCurrent;
    }
    if(total>totalCurrent+1e-9 || total>sourceCurrent+1e-9) throw "Simultaneous tool peak current exceeds the shared supply/harness rating";
    return total;
  }
  public static function pgcPair():Array<ToolServiceLoad> return [for(i in 0...2)
    {id:i==0 ? "gripper-a" : "gripper-b",voltage:24.0,peakCurrent:1.2,protocol:"Modbus RTU / RS485",address:i+1}];
}
typedef ToolServiceLoad = {var id:String;var voltage:Float;var peakCurrent:Float;var protocol:String;var address:Int;}
