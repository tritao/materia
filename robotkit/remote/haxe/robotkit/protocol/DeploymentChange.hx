package robotkit.protocol;
/** Select an operator-configured deployment; callers cannot supply filesystem paths. */
@:wire
class DeploymentChange {
  @:id(1) public var name:String;
  public function new(?name:String = "") this.name = name;
}
