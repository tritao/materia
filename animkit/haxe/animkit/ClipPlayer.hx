package animkit;

/**
 * Plays one clip at a time on an instance and crossfades between clips.
 * The outgoing clip keeps advancing while its weight fades to zero.
 */
class ClipPlayer {
	public final instance:AnimationInstance;
	/** Playback rate multiplier applied to both clips. */
	public var speed:Float = 1.0;

	var clip:Int = -1;
	var time:Float = 0.0;
	var loop:Bool = true;
	var previousClip:Int = -1;
	var previousTime:Float = 0.0;
	var previousLoop:Bool = true;
	var fadeDuration:Float = 0.0;
	var fadeElapsed:Float = 0.0;

	public function new(instance:AnimationInstance) {
		this.instance = instance;
	}

	public function currentClip():Int
		return clip;

	public function currentTime():Float
		return time;

	/** Starts a clip by name; returns false when the asset has no such clip. */
	public function playNamed(name:String, fadeSeconds:Float = 0.2, loop:Bool = true):Bool {
		var index = instance.asset.clipIndex(name);
		if (index < 0)
			return false;
		play(index, fadeSeconds, loop);
		return true;
	}

	/** Starts a clip from its beginning, fading from the current one. Replaying the current clip is a no-op. */
	public function play(index:Int, fadeSeconds:Float = 0.2, loop:Bool = true):Void {
		if (index < 0 || index >= instance.asset.clipNames.length)
			throw 'Clip index $index is out of range';
		if (index == clip)
			return;
		if (clip >= 0 && fadeSeconds > 0.0) {
			previousClip = clip;
			previousTime = time;
			previousLoop = this.loop;
			fadeDuration = fadeSeconds;
			fadeElapsed = 0.0;
		} else {
			previousClip = -1;
		}
		clip = index;
		time = 0.0;
		this.loop = loop;
	}

	/** Advances playback, updates the instance layers, and evaluates the pose. */
	public function advance(seconds:Float):Void {
		var step = seconds * speed;
		time += step;
		var weight = 1.0;
		if (previousClip >= 0) {
			previousTime += step;
			fadeElapsed += seconds;
			if (fadeElapsed >= fadeDuration)
				previousClip = -1;
			else
				weight = fadeElapsed / fadeDuration;
		}
		instance.setLayer(0, clip, time, weight, loop);
		if (previousClip >= 0)
			instance.setLayer(1, previousClip, previousTime, 1.0 - weight, previousLoop);
		else
			instance.setLayer(1, -1, 0.0, 0.0);
		instance.evaluate();
	}
}
