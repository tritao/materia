import collisionkit.CollisionPose;
import collisionkit.CollisionGeometry;
import collisionkit.CollisionPairRule;
import collisionkit.CollisionPairStatus;
import collisionkit.CollisionMargins;
import collisionkit.native.NativeCollisionWorld;
import cadkit.modeling.AssemblyState;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import robotkit.runtime.Simulation;
import nativekit.sim.SimObject;

/** The authored mill safety field, sampled from physical links and physically gripped stock. */
class TendingElectricZone {
  final world=new NativeCollisionWorld();
  final simulation:Simulation;
  final zone:Int;
  final links:Map<Int,Int> = [];
  final stock:Map<Int,{object:SimObject,body:Int}> = [];
  public function new(scene:TendingElectricSimScene,simulation:Simulation,fixtures:AssemblyState,objects:Map<Int,SimObject>) {
    this.simulation=simulation;
    var sensors=fixtures.definition.sensors;
    if(sensors==null)throw "Physical PLC needs the mill's authored robot zone";
    var selected=[for(sensor in sensors)if(sensor.kind=="robot_zone")sensor];
    if(selected.length!=1)throw "Physical PLC requires one authored mill robot zone";
    var sensor=selected[0],frame=fixtures.worldConnector(sensor.occurrence,sensor.connector),boxes:Array<Float> = cast sensor.zoneBoxes;
    zone=world.addBody(0);world.setBodyStatic(zone,true);world.setBodyPose(zone,pose(frame));
    for(i in 0...Std.int(boxes.length/6))world.add(zone,new CollisionPose(boxes[i*6]*0.001,boxes[i*6+1]*0.001,boxes[i*6+2]*0.001,0,0,0,1),
      Box(boxes[i*6+3]*0.001,boxes[i*6+4]*0.001,boxes[i*6+5]*0.001));
    var bodies:Array<Int> = [];
    for(shape in scene.blueprint.linkCollisionShapes) {
      if(!links.exists(shape.link)){var body=world.addBody(1);links.set(shape.link,body);bodies.push(body);}
      var body:Int=cast links.get(shape.link),object=world.add(body,CollisionPose.identity(),Convex(shape.shape.convexVertices()));
      world.setInflation(object,shape.shape.inflation);
    }
    for(slot in objects.keys()) {
      var object:SimObject=cast objects.get(slot),body=world.addBody(1);bodies.push(body);
      world.add(body,CollisionPose.identity(),Convex(object.shape.convexVertices()));stock.set(slot,{object:object,body:body});
      world.setBodyRule(zone,body,Allow,Allowed);
    }
    for(i in 0...bodies.length)for(j in i+1...bodies.length)world.setBodyRule(bodies[i],bodies[j],Allow,Rigid);
  }
  static function pose(f:AssemblyFrame):CollisionPose return new CollisionPose(f.x*0.001,f.y*0.001,f.z*0.001,f.qx,f.qy,f.qz,f.qw);
  public function clear(held:Array<Int>):Bool {
    for(link in links.keys()) {
      var p=simulation.linkPose(0,link);world.setBodyPose(cast links.get(link),new CollisionPose(p.position[0],p.position[1],p.position[2],p.rotation[0],p.rotation[1],p.rotation[2],p.rotation[3]));
    }
    for(slot in stock.keys()) {
      var value:{object:SimObject,body:Int}=cast stock.get(slot),p=simulation.objectPose(value.object);
      world.setBodyPose(value.body,new CollisionPose(p.x,p.y,p.z,p.qx,p.qy,p.qz,p.qw));
      world.setBodyRule(zone,value.body,held.indexOf(value.object.handle.rawValue())>=0 ? Check : Allow,Allowed);
    }
    return world.violation(new CollisionMargins(2,0.001))==null;
  }
  public function dispose():Void world.dispose();
}
