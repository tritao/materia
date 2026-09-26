package machinekit.transmission;

/** Belt family and pitch-line geometry for a simplified timing pulley.
 * Custom requires an explicit family, pitch and pitch-line differential in millimetres.
 */
enum TimingBeltProfile {
	GT2;
	HTD3M;
	HTD5M;
	HTD8M;
	HTD14M;
	T5;
	XL;
	Custom(family:String, pitch:Float, pitchLineDifferential:Float);
}
