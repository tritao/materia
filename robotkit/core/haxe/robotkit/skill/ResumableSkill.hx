package robotkit.skill;

/** A cancelled skill whose next start can continue its retained measured operation. */
interface ResumableSkill extends Skill {
  function canResume():Bool;
}
