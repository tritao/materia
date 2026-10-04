package robotkit.skill;

/** Terminal skill outcome returned for application logging and orchestration. */
class SkillResult {
  public final status:SkillStatus;
  public final message:String;

  public function new(status:SkillStatus, message:String) {
    if (status == null || message == null)
      throw "Skill result requires a status and message";
    this.status = status;
    this.message = message;
  }
}
