package materia.automation.mission;

enum MissionStatus {
  Pending;
  Running;
  Succeeded;
  Failed(message:String);
  Cancelled;
}
