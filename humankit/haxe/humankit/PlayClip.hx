package humankit;

/** Crossfades to a named clip and plays it for a fixed duration. */
class PlayClip extends HumanActionBase {
	public final name:String;
	public final seconds:Float;
	public final fade:Float;
	var elapsed:Float = 0.0;

	public function new(name:String, seconds:Float, fade:Float = 0.2) {
		super();
		this.name = name;
		this.seconds = seconds;
		this.fade = fade;
	}

	override public function start(worker:HumanBody):Void {
		super.start(worker);
		if (seconds < 0.0 || fade < 0.0) { fail("Clip timing cannot be negative"); return; }
		var clip = worker.character.asset.clipIndex(name);
		if (clip < 0) { fail('The character needs a "$name" clip'); return; }
		worker.character.player.play(clip, fade);
		if (seconds == 0.0) done = true;
	}

	override public function advance(seconds:Float):Void {
		elapsed += seconds;
		if (elapsed >= this.seconds) done = true;
	}
}
