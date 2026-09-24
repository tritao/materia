package materia.automation.task;

enum abstract TaskKind(String) from String to String {
  var Transport = "transport";
  var Pick = "pick";
  var Place = "place";
  var Charge = "charge";
}
