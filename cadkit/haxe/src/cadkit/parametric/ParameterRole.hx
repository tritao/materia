package cadkit.parametric;

/**
 * Who sets a named parameter's value. Authoring metadata, saved with the document; it does not change evaluation.
 * - `Editable`: an input a person may change (an editor override, a recipe input).
 * - `Derived`: computed from other values by a rule (`NamedParameter.note`); shown, never edited directly. An expression
 *   parameter is always derived.
 * - `Fixed`: set by something outside the design: bought stock, a catalogue row, a standard (`note` names it).
 */
class ParameterRole {
  public static inline final Editable = "editable";
  public static inline final Derived = "derived";
  public static inline final Fixed = "fixed";

  public static function validate(role:String):String {
    if (role != Editable && role != Derived && role != Fixed) throw new ParametricError("unknown parameter role: " + role);
    return role;
  }
}
