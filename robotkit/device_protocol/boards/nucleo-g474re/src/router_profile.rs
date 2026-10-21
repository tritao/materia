//! SR15 proposed carrier pin profile. Pure constants are compiled on host and ARM.
//! DS12288 Rev 6 tables 12/13. Hardware output adapter remains SR18.
#[derive(Clone, Copy, Debug)]
pub struct Pin {
    pub signal: &'static str,
    pub pin: &'static str,
    pub af: i8,
}
pub const PINS: &[Pin] = &[
    Pin {
        signal: "driveX.step",
        pin: "PA2",
        af: 9,
    },
    Pin {
        signal: "driveX.dir",
        pin: "PC8",
        af: -1,
    },
    Pin {
        signal: "driveX.encoderA",
        pin: "PA0",
        af: 1,
    },
    Pin {
        signal: "driveX.encoderB",
        pin: "PA1",
        af: 1,
    },
    Pin {
        signal: "driveYLeft.step",
        pin: "PA6",
        af: 1,
    },
    Pin {
        signal: "driveYLeft.dir",
        pin: "PC9",
        af: -1,
    },
    Pin {
        signal: "driveYLeft.encoderA",
        pin: "PB4",
        af: 2,
    },
    Pin {
        signal: "driveYLeft.encoderB",
        pin: "PB5",
        af: 2,
    },
    Pin {
        signal: "driveYRight.step",
        pin: "PA7",
        af: 1,
    },
    Pin {
        signal: "driveYRight.dir",
        pin: "PC10",
        af: -1,
    },
    Pin {
        signal: "driveYRight.encoderA",
        pin: "PA11",
        af: 10,
    },
    Pin {
        signal: "driveYRight.encoderB",
        pin: "PA12",
        af: 10,
    },
    Pin {
        signal: "driveZ.step",
        pin: "PB2",
        af: 3,
    },
    Pin {
        signal: "driveZ.dir",
        pin: "PC11",
        af: -1,
    },
    Pin {
        signal: "driveZ.encoderA",
        pin: "PC6",
        af: 4,
    },
    Pin {
        signal: "driveZ.encoderB",
        pin: "PC7",
        af: 4,
    },
    Pin {
        signal: "driveSpare.step",
        pin: "PC12",
        af: 1,
    },
    Pin {
        signal: "driveSpare.dir",
        pin: "PB0",
        af: -1,
    },
    Pin {
        signal: "driveSpare.encoderA",
        pin: "PA8",
        af: 6,
    },
    Pin {
        signal: "driveSpare.encoderB",
        pin: "PA9",
        af: 6,
    },
    Pin {
        signal: "setter.trigger",
        pin: "PC0",
        af: -1,
    },
    Pin {
        signal: "switchXhome",
        pin: "PC1",
        af: -1,
    },
    Pin {
        signal: "switchXnegativeLimit",
        pin: "PC2",
        af: -1,
    },
    Pin {
        signal: "switchXpositiveLimit",
        pin: "PC3",
        af: -1,
    },
    Pin {
        signal: "switchYLefthome",
        pin: "PA5",
        af: -1,
    },
    Pin {
        signal: "switchYLeftnegativeLimit",
        pin: "PB6",
        af: -1,
    },
    Pin {
        signal: "switchYLeftpositiveLimit",
        pin: "PB7",
        af: -1,
    },
    Pin {
        signal: "switchYRighthome",
        pin: "PB8",
        af: -1,
    },
    Pin {
        signal: "switchYRightnegativeLimit",
        pin: "PB9",
        af: -1,
    },
    Pin {
        signal: "switchYRightpositiveLimit",
        pin: "PB10",
        af: -1,
    },
    Pin {
        signal: "switchZhome",
        pin: "PB11",
        af: -1,
    },
    Pin {
        signal: "switchZnegativeLimit",
        pin: "PB12",
        af: -1,
    },
    Pin {
        signal: "switchZpositiveLimit",
        pin: "PC14",
        af: -1,
    },
    Pin {
        signal: "probe.trigger",
        pin: "PC15",
        af: -1,
    },
    Pin {
        signal: "serial.tx",
        pin: "PC4",
        af: 7,
    },
    Pin {
        signal: "serial.rx",
        pin: "PC5",
        af: 7,
    },
    Pin {
        signal: "expander.sck",
        pin: "PB13",
        af: 5,
    },
    Pin {
        signal: "expander.miso",
        pin: "PB14",
        af: 5,
    },
    Pin {
        signal: "expander.mosi",
        pin: "PB15",
        af: 5,
    },
    Pin {
        signal: "expander.cs",
        pin: "PA3",
        af: -1,
    },
    Pin {
        signal: "spindle.speed",
        pin: "PA4",
        af: -1,
    },
    Pin {
        signal: "watchdog.heartbeat",
        pin: "PB1",
        af: -1,
    },
    Pin {
        signal: "outputs.permission",
        pin: "PB3",
        af: -1,
    },
    Pin {
        signal: "debug.swdio",
        pin: "PA13",
        af: 0,
    },
    Pin {
        signal: "debug.swclk",
        pin: "PA14",
        af: 0,
    },
];
pub const TIMER_CLOCK_HZ: u32 = 170_000_000;
pub const TIMER_PRESCALER: u16 = 16;
pub const IWDG_PRESCALER: u16 = 4;
pub const IWDG_RELOAD: u16 = 159;

/// 10000 feedback edges / 5000 commanded steps per motor turn, assumed.
pub const ENCODER_COUNTS_PER_STEP: [i64; 5] = [2; 5];
/// 0.10 mm following error: 4 um XY pulses, 2 um Z pulses.
pub const FOLLOWING_ERROR_COUNTS: [i64; 5] = [50, 50, 50, 100, 50];
