package machinekit.component;

/**
 * Assumed effective densities (kg/m³) of purchased parts whose envelope is a housing around air, electronics and
 * terminals: datasheet mass over envelope volume for reference products of the class (2026-10-21). They replace
 * the mass of a solid block of the housing material, which overstates such parts five to ten times. A catalogue
 * row's datasheet mass, where one is known, overrides them (`MachineComponent.declareMass`).
 */
class EffectiveDensities {
	/**
	 * DIN-rail contactors and miniature circuit breakers, 1040–1460: Schneider TeSys D LC1D09 (45×77×95 mm, 0.48 kg,
	 * https://www.farnell.com/datasheets/3449660.pdf), Schneider Acti9 iC60N 3P (54×85×78.5 mm, 0.375 kg,
	 * https://123elec.com/media/wysiwyg/telechargement/document/Fiche-technique-disjoncteur-Schneider-Acti9-iC60-A9F74316.pdf),
	 * ABB S203 (52.5×88×69 mm, 0.375 kg).
	 */
	public static inline var DinRailSwitchgear:Float = 1200;

	/**
	 * DIN-rail electronic modules such as safety relays: Pilz PNOZ s4 (22.5×98×120 mm, 185 g,
	 * https://www.pilz.com/download/open/OM_PNOZ_s4_21396-EN-23.pdf), about 700.
	 */
	public static inline var DinRailElectronics:Float = 700;

	/**
	 * Enclosed switch-mode DC power supplies and rectifiers, 910–1150: Mean Well LRS-150-24, LRS-350-24 and SDR-240-24
	 * (https://www.meanwell.com/Upload/PDF/LRS-350/LRS-350-SPEC.PDF and the LRS-150 and SDR-240 sheets).
	 */
	public static inline var PowerSupply:Float = 1000;

	/**
	 * Variable-frequency drives of 1.5–5.5 kW, 490–810: Delta MS300 frames B3 to D2
	 * (https://www.farnell.com/datasheets/3171459.pdf), ABB ACS580-01 frames R1 and R2 (hardware manual
	 * 3AXD50000044794). Book-style drives sit near 750, wall-mount drives with cable boxes near 500.
	 */
	public static inline var Drive:Float = 650;

	/**
	 * Aluminium-housed braking resistors, 1400–1950: Delta BR080W200 to BR1K0W020
	 * (https://www.mouser.lt/datasheet/3/1208/1/Delta_Brake_Resistors_Data_Sheet.pdf).
	 */
	public static inline var BrakeResistor:Float = 1800;

	/**
	 * Equipped robot and machine controller cabinets, 230–750: ABB IRC5 single cabinet and IRC5 Compact
	 * (https://library.e.abb.com/public/c13e1c5490c61230c125796000515137/IRC5%20datasheet%20PR10258%20EN_R13.pdf),
	 * Yaskawa YRC1000micro, Universal Robots UR5e control box. Floor cabinets near 300, compact controllers near 450.
	 */
	public static inline var ControlCabinet:Float = 350;
}
