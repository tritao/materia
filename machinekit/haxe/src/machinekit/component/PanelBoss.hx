package machinekit.component;

/**
 * A round boss a part asks for on the back of its panel, with a blind pilot hole in its end for a heat-set insert. Offsets
 * are from the part's centre; the boss stands `height` behind the panel's back face and the hole runs `pilotDepth` into it.
 */
typedef PanelBoss = {x:Float, y:Float, diameter:Float, height:Float, pilotDiameter:Float, pilotDepth:Float};
