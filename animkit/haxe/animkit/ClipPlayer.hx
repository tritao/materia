package animkit;

/**
 * Plays one clip at a time on an instance and crossfades between clips.
 * The outgoing clips keep advancing while their weights fade to zero.
 */
class ClipPlayer {
	public final instance:AnimationInstance;
	/** Playback rate multiplier applied to both clips. */
	public var speed:Float = 1.0;

	/** Layers the instance holds besides the current clip's. */
	static inline var MAX_OUTGOING:Int = AnimationInstance.MAX_LAYERS - 1;

	var clip:Int = -1;
	var time:Float = 0.0;
	var loop:Bool = true;
	/**
	 * Clips fading out, each with the share of the outgoing blend it holds (the shares sum to one).
	 * A clip started while another fade is still running keeps the earlier outgoing clips, so the
	 * pose continues from the blend it was in instead of snapping to the current clip alone.
	 */
	final outgoing:Array<{clip:Int, time:Float, loop:Bool, share:Float}> = [];
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
			// The blend now is the current clip at its fade weight over the outgoing ones at the rest.
			var incoming = currentWeight();
			for (entry in outgoing) entry.share *= 1.0 - incoming;
			outgoing.push({clip: clip, time: time, loop: this.loop, share: incoming});
			dropWeakest();
			fadeDuration = fadeSeconds;
			fadeElapsed = 0.0;
		} else {
			outgoing.resize(0);
		}
		clip = index;
		time = 0.0;
		this.loop = loop;
	}

	/**
	 * Starts a clip from its beginning with no fade, even when it is already the
	 * current clip, dropping any crossfade in progress and returning the rate to
	 * normal. This puts the player back as if the clip had just been started.
	 */
	public function restart(index:Int, loop:Bool = true):Void {
		if (index < 0 || index >= instance.asset.clipNames.length)
			throw 'Clip index $index is out of range';
		outgoing.resize(0);
		fadeDuration = 0.0;
		fadeElapsed = 0.0;
		clip = index;
		time = 0.0;
		this.loop = loop;
		speed = 1.0;
	}

	/** The current clip's weight: how far its fade-in has run, or one when nothing is fading out. */
	function currentWeight():Float
		return outgoing.length == 0 || fadeDuration <= 0.0 ? 1.0 : Math.min(1.0, fadeElapsed / fadeDuration);

	/** Keeps the instance's spare layers enough: the weakest outgoing clip goes, and the rest rescale. */
	function dropWeakest():Void {
		while (outgoing.length > MAX_OUTGOING) {
			var weakest = 0;
			for (index in 1...outgoing.length) if (outgoing[index].share < outgoing[weakest].share) weakest = index;
			outgoing.splice(weakest, 1);
			var total = 0.0;
			for (entry in outgoing) total += entry.share;
			if (total > 0.0) for (entry in outgoing) entry.share /= total;
		}
	}

	/** Advances playback, updates the instance layers, and evaluates the pose. */
	public function advance(seconds:Float):Void {
		var step = seconds * speed;
		time += step;
		if (outgoing.length > 0) {
			fadeElapsed += seconds;
			for (entry in outgoing) entry.time += step;
			if (fadeElapsed >= fadeDuration) outgoing.resize(0);
		}
		var weight = currentWeight();
		instance.setLayer(0, clip, time, weight, loop);
		for (index in 0...MAX_OUTGOING) {
			if (index < outgoing.length) {
				var entry = outgoing[index];
				instance.setLayer(index + 1, entry.clip, entry.time, entry.share * (1.0 - weight), entry.loop);
			} else
				instance.setLayer(index + 1, -1, 0.0, 0.0);
		}
		instance.evaluate();
	}
}
