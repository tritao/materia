package app.examples;

import app.ApplicationSimulation;
import app.ScriptedSetup;
import app.SensorConfiguration;
import app.SetupScript;
import robotkit.model.Actuator;
import robotkit.model.Frame;
import robotkit.model.Joint;
import robotkit.model.JointType;
import robotkit.model.Link;

/** Example used by Materia and the headless scripted-setup runner. */
class TwoRobotSetupScript implements SetupScript {
  public static inline var REFERENCE="materia.examples.two-robot";
  public function new() {}
  public function reference():String return REFERENCE;
  public function version():Int return 1;
  public function evaluate(output:ScriptedSetup):Void {
    var configuration=new SensorConfiguration();
    configureRobot(configuration,0.25);
    configuration.setRobotPose("materia/robot",[-1.0,0.0,0.0],[0.0,0.0,0.0,1.0]);
    configuration.selectRobot("materia/robot-b");configureRobot(configuration,0.4);
    configuration.setRobotPose("materia/robot-b",[1.0,0.0,0.0],[0.0,0.0,0.0,1.0]);
    configuration.selectRobot("materia/robot");configuration.document.markSaved();
    output.define(reference(),version(),configuration,[{
      id:"moving-obstacle",label:"Moving obstacle",type:"rectangle",x:0.0,y:1.5,z:1.0,
      width:0.8,height:0.8,depth:0.8,collisionEnabled:true,dynamicBody:true,mass:2.0,
      red:0.9,green:0.35,blue:0.2,visible:true
    }],ApplicationSimulation.MUJOCO,0.01);
  }
  static function configureRobot(configuration:SensorConfiguration,mountX:Float):Void {
    var model=configuration.model,base=model.links[0],arm=model.addLink(new Link("Arm","arm"));
    var joint=new Joint("Arm joint",JointType.Revolute,base,arm,"joint/arm");
    joint.limits.lower=-1.2;joint.limits.upper=1.2;joint.limits.velocity=2.0;joint.limits.effort=5.0;
    joint.drive=new Actuator("Arm drive",5.0,2.0);model.addJoint(joint);
    var mount=model.addFrame(new Frame("Arm sensors",arm,"arm/sensors"));mount.position=[mountX,0.0,0.0];
    var lidar=model.sensors[0];lidar.frame=mount;lidar.updateRate=20.0;lidar.rayCount=32;lidar.maxRange=12.0;
    var imu=configuration.add("imu");imu.frame=mount;imu.updateRate=100.0;imu.noiseStddev=0.002;
    var encoder=configuration.add("joint_encoder");encoder.frame=mount;encoder.updateRate=100.0;
  }
}
