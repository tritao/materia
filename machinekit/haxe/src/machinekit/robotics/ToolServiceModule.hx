package machinekit.robotics;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentDetail;
import machinekit.component.Solids;
import materia.assembly.AssemblyFrames;

/** Passive power/data passage. Two logical services share each physical connector.
    Socket dimensions are planning envelopes; this is not an M8/M12 or DH pinout. */
class ToolServiceModule extends MachineComponent {
  public final spec:ToolServiceSpec;
  public final outlets:Int;
  public final outletCurrent:Float;
  public final width:Float;
  public final depth:Float;
  public final height:Float = 18;
  public function new(spec:ToolServiceSpec,outlets:Int = 1) {
    if(spec==null || outlets<1 || outlets>2) throw "Tool service module needs a contract and one or two outlets";
    super('TOOL-SERVICE-${spec.key()}-${spec.pins}PIN-${spec.totalCurrent}A-${spec.branchCurrent}A-$outlets',
      outlets==1 ? "Provisional wrist power/data socket" : "Provisional passive dual-gripper junction", "aluminium 6061",true);
    this.spec=spec;this.outlets=outlets;outletCurrent=outlets==1 ? spec.totalCurrent : spec.branchCurrent;width=outlets==1 ? 32 : 48;depth=outlets==1 ? 28 : 44;
    addConnector("mount",Mount,AssemblyFrames.identity());
    addConnector("inlet",Mount,AssemblyFrames.fromRotationMatrix(-width/2,0,10,[0,0,-1,1,0,0,0,-1,0]));
    addPort({name:"powerIn",kind:ElectricalPower,role:Consumer,iface:Plug(spec.key(),spec.pins),required:true,connector:"inlet"});
    addPort({name:"signalIn",kind:Signal,role:Consumer,iface:Plug(spec.key(),spec.pins),required:true,connector:"inlet"});
    for(i in 1...outlets+1) {
      var y=outlets==1 ? 0.0 : (i==1 ? -11.0 : 11.0);
      addConnector('outlet$i',Mount,AssemblyFrames.fromRotationMatrix(width/2,y,10,[0,0,1,-1,0,0,0,-1,0]));
      addPort({name:'power$i',kind:ElectricalPower,role:Supply,iface:Plug(spec.key(),spec.pins),required:false,connector:'outlet$i'});
      addPort({name:'signal$i',kind:Signal,role:Supply,iface:Plug(spec.key(),spec.pins),required:false,connector:'outlet$i'});
      addBridge("powerIn",'power$i');addBridge("signalIn",'signal$i');
    }
  }
  /** Explicit circuit budget check; the generic port graph alone does not check current. */
  public function checkLoads(sourceVoltage:Float,sourceCurrent:Float,loads:Array<ToolServiceSpec.ToolServiceLoad>):Float {
    if(loads==null || loads.length>outlets) throw "More tool loads than module outlets";
    return new ToolServiceSpec(spec.voltage,spec.totalCurrent,outletCurrent,spec.protocol,spec.connectorKey,spec.pins).checkLoads(sourceVoltage,sourceCurrent,loads);
  }
  /** Budget the single wrist outlet feeding downstream devices through a passive junction.
      Addresses belong to leaf devices; the junction itself has no Modbus address. */
  public function checkPassThrough(sourceVoltage:Float,sourceCurrent:Float,loads:Array<ToolServiceSpec.ToolServiceLoad>):Float {
    if(outlets!=1) throw "Pass-through budget is for the single wrist outlet";
    return new ToolServiceSpec(spec.voltage,spec.totalCurrent,outletCurrent,spec.protocol,spec.connectorKey,spec.pins).checkLoads(sourceVoltage,sourceCurrent,loads);
  }
  /** Check the complete downstream load list against the actual wired source and
      every service-module shared rating on the power path. External boundaries
      are intentionally unrated until a real ElectricalSource is connected. */
  public function checkWiredLoads(assembly:machinekit.assembly.MachineAssembly,instance:String,loads:Array<ToolServiceSpec.ToolServiceLoad>):Float {
    var upstream=assembly.upstream(instance,"powerIn");
    if(upstream.external) throw "Tool circuit has an unrated external power boundary";
    var source:machinekit.motion.ElectricalSource=null;
    for(entry in assembly.components()) if(entry.id==upstream.port.instanceId) source=entry.component.electricalSource();
    if(source==null) throw "Tool circuit needs a rated electrical source";
    var rating=source.outputRating(upstream.port.portName);
    var current=checkLoads(rating.voltage.volts(),rating.current.amps(),loads);
    var path=assembly.upstreamChain(instance,"powerIn");
    for(entry in assembly.components()) if(Std.isOfType(entry.component,ToolServiceModule)) {
      var used=false;for(node in path) if(node==entry.id+"/powerIn") used=true;
      if(!used || entry.id==instance) continue;
      var module:ToolServiceModule=cast entry.component;
      if(module.spec.key()!=spec.key() || module.spec.pins!=spec.pins) throw "Tool service path has incompatible electrical contracts";
      if(module.outlets!=1) throw "Nested branched tool junctions need an explicit branch load model";
      module.checkPassThrough(rating.voltage.volts(),rating.current.amps(),loads);
    }
    return current;
  }
  static var recipe:Null<machinekit.component.ComponentType>;
  public static function recipeType():machinekit.component.ComponentType {
    if(recipe==null) recipe=new machinekit.component.ComponentType("machinekit.robotics.tool-service-module",[
      machinekit.component.ComponentRecipeSupport.scalar("voltage",24,0.001),
      machinekit.component.ComponentRecipeSupport.scalar("totalCurrent",5,0.001),
      machinekit.component.ComponentRecipeSupport.scalar("branchCurrent",2,0.001),
      machinekit.component.ComponentRecipeSupport.text("protocol","Modbus RTU / RS485"),
      machinekit.component.ComponentRecipeSupport.text("connectorKey","provisional-tool-service-v1"),
      machinekit.component.ComponentRecipeSupport.count("pins",8),
      machinekit.component.ComponentRecipeSupport.count("outlets",1)
    ],v -> new ToolServiceModule(new ToolServiceSpec(v.number("voltage"),v.number("totalCurrent"),v.number("branchCurrent"),
      v.token("protocol"),v.token("connectorKey"),v.integer("pins")),v.integer("outlets")),true);
    return recipe;
  }
  override public function componentType():Null<machinekit.component.ComponentType> return recipeType();
  override public function values():machinekit.component.ComponentValues return new machinekit.component.ComponentValues()
    .setNumber("voltage",spec.voltage).setNumber("totalCurrent",spec.totalCurrent).setNumber("branchCurrent",spec.branchCurrent)
    .setToken("protocol",spec.protocol).setToken("connectorKey",spec.connectorKey).setInteger("pins",spec.pins)
    .setInteger("outlets",outlets).setToken("material",materialSpec());
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part {
    var parts=[Part.box(width,depth,height)];
    parts.push(Part.cylinderAlong(6,new Vector(-width/2-8,0,10),Vector.X(),9));
    for(i in 1...outlets+1) parts.push(Part.cylinderAlong(6,new Vector(width/2-1,outlets==1 ? 0 : (i==1 ? -11 : 11),10),Vector.X(),9));
    var tools:Array<Part> = [];
    for(x in [-10.0,10.0]) for(y in [-9.0,9.0]) {
      tools.push(Part.cylinderSpan(1.7,-1,height+1,x,y));
      tools.push(Part.cylinderSpan(3,14,height+1,x,y));
    }
    // Visible connector bores distinguish socket envelopes from solid cable ends.
    tools.push(Part.cylinderAlong(4,new Vector(-width/2-8.1,0,10),Vector.X(),5));
    for(i in 1...outlets+1) tools.push(Part.cylinderAlong(4,new Vector(width/2+3,outlets==1 ? 0 : (i==1 ? -11 : 11),10),Vector.X(),5.1));
    return Solids.cut(Solids.union(parts),tools);
  }
}
