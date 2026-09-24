package robotkit.skill;

/** Shared lifecycle state for a composable robot skill. */
enum SkillStatus {
  Idle;
  Running;
  Succeeded;
  Cancelled;
  Failed(message:String);
}
