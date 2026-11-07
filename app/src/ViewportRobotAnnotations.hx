package app;

import app.ApplicationSimulation.SimulationRobotVisual;
import app.ApplicationSimulation.SimulationPoseVisual;
import robotkit.core.SensorFrame;
import robotkit.spatial.Transform3;
import robotkit.kinematics.RobotKinematics;
import kinematicskit.KinematicState;
import kinematicskit.KinematicSnapshot;

typedef ViewportSensorAnnotation = {
  var id:String;
  var kind:String;
  var linkId:String;
  var position:Array<Float>;
  var rotation:Array<Float>;

  /** Observations are absent in design preview; never manufacture sensor data. */
  var reading:Null<SensorFrame>;
}

typedef ViewportRobotAnnotation = {
  var id:String;
  var position:Array<Float>;
  var rotation:Array<Float>;
  var links:Array<SimulationPoseVisual>;
  var sensors:Array<ViewportSensorAnnotation>;
}

/** Adapts measured poses or authored mounts to the same inspection renderer. */
class ViewportRobotAnnotations {
  public static function measured(robots:Array<SimulationRobotVisual>, ?authored:Array<ViewportRobotAnnotation>):Array<ViewportRobotAnnotation> {
    var result:Array<ViewportRobotAnnotation> = [];
    for (robot in robots) {
      var sensors:Array<ViewportSensorAnnotation> = [
        for (sensor in robot.sensors)
          {
            id: sensor.sensorId,
            kind: sensor.kind,
            linkId: sensor.linkId,
            position: sensor.mountPosition.toArray(),
            rotation: sensor.mountRotation.toArray(),
            reading: sensor
          }
      ];
      // A mount exists before its first observation; use measured link poses to
      // place configured mounts without synthesizing a measurement.
      if (authored != null)
        for (definition in authored)
          if (definition.id == robot.id)
            for (sensor in definition.sensors) {
              var found = false;
              for (measured in sensors)
                if (measured.id == sensor.id)
                  found = true;
              if (!found)
                sensors.push(sensor);
            }
      result.push({
        id: robot.id,
        position: robot.position,
        rotation: robot.rotation,
        links: robot.links,
        sensors: sensors
      });
    }
    return result;
  }

  public static function authored(configuration:SensorConfiguration,
    ?assembly:materia.assembly.AssemblyDefinition.AssemblyStateRecord, metresPerUnit:Float = 1.0):Array<ViewportRobotAnnotation> {
    var result:Array<ViewportRobotAnnotation> = [];
    for (robot in configuration.robotModels()) {
      var model = RobotKinematics.compile(robot.model);
      var state = new KinematicState(model);
      if (assembly != null) for (dof in 0...model.dofCount()) {
        var joint = robot.model.joints[model.dofJoint[dof]];
        for (coordinate in assembly.jointCoordinates) if (coordinate.joint == joint.id)
          state.q[dof] = coordinate.value * (joint.type == robotkit.model.JointType.Prismatic ? metresPerUnit : 1.0);
      }
      var base = Transform3.fromArrays(robot.position, robot.rotation);
      for (body in 0...model.bodyCount())
        if (model.bodyParentJoint[body] < 0)
          state.setRootPose(body, RobotKinematics.toTransform(base));
      var snapshot = KinematicSnapshot.of(state);
      var links:Array<SimulationPoseVisual> = [
        for (body in 0...model.bodyCount()) {
          var pose = snapshot.bodyPose(body);
          {id: model.bodyIds[body], position: [pose.x, pose.y, pose.z], rotation: [pose.qx, pose.qy, pose.qz, pose.qw]};
        }
      ];
      result.push({
        id: robot.id,
        position: robot.position,
        rotation: robot.rotation,
        links: links,
        sensors: [
          for (sensor in robot.model.sensors)
            {
              id: sensor.id,
              kind: sensor.kind,
              linkId: sensor.frame == null ? "" : sensor.frame.link.id,
              position: sensor.frame == null ? [0.0, 0.0, 0.0] : sensor.frame.position.copy(),
              rotation: sensor.frame == null ? [0.0, 0.0, 0.0, 1.0] : sensor.frame.rotation.copy(),
              reading: null
            }
        ]
      });
    }
    return result;
  }
}
