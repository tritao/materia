//! The host's compiled operator profile, decoded into the fixed tables `Authority` runs on, and the
//! digest both sides compute over it. The host builds it with `PendantCompiler` (machinekit.operator).

use crate::authority::{Config, JogRule, UNUSED};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProfileError {
    Wire(crate::WireError),
    /// A rule slot at or beyond `rule_count` is not zero.
    Rule,
    /// The spindle key is not F1 to F3 or "none", or disagrees with `has_spindle`.
    SpindleKey,
}

impl From<crate::WireError> for ProfileError {
    fn from(e: crate::WireError) -> Self { ProfileError::Wire(e) }
}

/// A `PendantProfile` record from the host, decoded into the fixed tables `Authority` runs on.
/// The host builds it with `PendantCompiler` (machinekit.operator); `digest` lets both sides
/// confirm they hold the same profile before the pendant is accepted.
#[derive(Clone, Copy, Debug)]
pub struct CompiledProfile {
    pendant_axes: [u8; 11],
    rules: [JogRule; 8],
    rule_count: usize,
    has_spindle: bool,
    spindle_key: Option<u8>,
    step_units: [i32; 3],
    timeout_us: u64,
    host_timeout_us: u64,
    reasons: [[u8; 21]; 8],
}

impl CompiledProfile {
    pub fn from_record(input: &[u8]) -> Result<Self, ProfileError> {
        let r = crate::operator_wire::PendantProfile::decode(input)?;
        let count = r.rule_count as usize;
        let mut rules = [JogRule { axis: 0, condition: 0, reason: 0 }; 8];
        for i in 0..8 {
            if i < count {
                rules[i] = JogRule { axis: r.rule_axis[i], condition: r.rule_condition[i], reason: r.rule_reason[i] };
            } else if r.rule_axis[i] != 0 || r.rule_condition[i] != 0 || r.rule_reason[i] != 0 {
                return Err(ProfileError::Rule);
            }
        }
        let has_spindle = r.has_spindle == 1;
        let spindle_key = match r.spindle_key {
            UNUSED => None,
            k @ 0..=2 if has_spindle => Some(k),
            _ => return Err(ProfileError::SpindleKey),
        };
        Ok(CompiledProfile {
            pendant_axes: r.axes, rules, rule_count: count, has_spindle, spindle_key, step_units: r.step_units,
            timeout_us: r.timeout_ms as u64 * 1000, host_timeout_us: r.host_timeout_ms as u64 * 1000,
            reasons: [[0; 21]; 8],
        })
    }

    /// Store one `PendantReasonText` record; the status line for `Refusal::Rule(id)`.
    pub fn set_reason(&mut self, input: &[u8]) -> Result<(), ProfileError> {
        let t = crate::operator_wire::PendantReasonText::decode(input)?;
        self.reasons[t.index as usize] = t.text;
        Ok(())
    }

    /// The status text for a rule reason id, up to its NUL; empty when none was stored.
    pub fn reason_text(&self, id: u8) -> &[u8] {
        match self.reasons.get(id as usize) {
            Some(text) => crate::label(text),
            None => &[],
        }
    }

    /// The stored text for a rule reason id as its raw 21 bytes (NUL padded); empty when the id is out of range.
    pub fn reason_text_bytes(&self, id: u8) -> &[u8] {
        match self.reasons.get(id as usize) {
            Some(text) => &text[..],
            None => &[],
        }
    }

    /// How long an OperatorState stays fresh; give it to `HostLink::new`.
    pub fn host_timeout_us(&self) -> u64 { self.host_timeout_us }

    pub fn config(&self) -> Config<'_> {
        Config {
            pendant_axes: self.pendant_axes, rules: &self.rules[..self.rule_count], has_spindle: self.has_spindle,
            spindle_key: self.spindle_key, step_units: self.step_units, timeout_us: self.timeout_us,
        }
    }
}

/// 32-bit FNV-1a over the records in order: the profile, its reason texts, then its labels.
pub fn digest(records: &[&[u8]]) -> u32 {
    let mut h: u32 = 0x811C_9DC5;
    for record in records {
        for &b in *record {
            h ^= b as u32;
            h = h.wrapping_mul(0x0100_0193);
        }
    }
    h
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::authority::{Authority, Move, Refusal};
    use crate::testing::{same, vector, Bench, DRILL};

    #[test]
    fn compiled_profiles_match_the_hand_written_configs() {
        // The scenarios above run on DRILL and HOBBER, which mirror the Haxe tests; the host-compiled
        // vectors must decode to exactly those configs.
        let drill = CompiledProfile::from_record(&vector("drill-tap").profile).unwrap();
        same(&drill.config(), &DRILL);
        let hobber = CompiledProfile::from_record(&vector("hobber").profile).unwrap();
        // The hobber profile indexes its axes c b x y z a, so positions differ from HOBBER's x y z a b c.
        let c = hobber.config();
        assert_eq!(c.pendant_axes[..6], [2, 3, 4, 5, 1, 0]);
        assert_eq!(c.rules, [JogRule { axis: 1, condition: 0, reason: 0 }, JogRule { axis: 0, condition: 0, reason: 0 }]);
        assert_eq!((c.has_spindle, c.spindle_key, c.step_units, c.timeout_us), (true, Some(1), [1, 10, 100], 50_000));
    }

    #[test]
    fn compiled_hobber_behaves_like_the_scenarios() {
        let v = vector("hobber");
        let mut p = CompiledProfile::from_record(&v.profile).unwrap();
        for r in &v.reasons { p.set_reason(r).unwrap(); }
        assert_eq!(p.reason_text(0), b"Gearbox engaged");
        let mut a = Authority::new(p.config());
        let mut b = Bench::new();
        let f = b.frame(5, 0, true, 0, 1, true); // position 5 is B
        a.accept(&f, 0, true, 1);
        let f = b.frame(5, 3, true, 0, 1, true);
        let out = a.accept(&f, b.nx(10), true, 1).unwrap();
        assert_eq!((out.movement, out.refusal), (None, Refusal::Rule(0)));
        let f = b.frame(3, 3, true, 0, 1, true); // position 3 is Z, index 4
        assert_eq!(a.accept(&f, b.nx(10), true, 1).unwrap().movement, Some(Move { axis: 4, distance: 30 }));
        let f = b.frame(5, 3, true, 0, 1, true);
        assert_eq!(a.accept(&f, b.nx(10), true, 0).unwrap().movement, Some(Move { axis: 1, distance: 30 }));
    }

    #[test]
    fn digest_matches_the_host() {
        for name in ["drill-tap", "hobber"] {
            let v = vector(name);
            let mut records: std::vec::Vec<&[u8]> = std::vec![&v.profile[..]];
            records.extend(v.reasons.iter().map(|r| &r[..]));
            records.extend(v.labels.iter().map(|r| &r[..]));
            assert_eq!(digest(&records), v.digest, "{name}");
        }
        assert_eq!(digest(&[b"a"]), 0xE40C_292C); // FNV-1a reference value
    }

    #[test]
    fn rejects_inconsistent_records() {
        let good = vector("hobber").profile;
        let mut bad = good.clone();
        bad[11] = 9; // rule_count above 8
        assert_eq!(CompiledProfile::from_record(&bad).unwrap_err(), ProfileError::Wire(crate::WireError::Constraint("PendantProfile.rule_count")));
        let mut bad = good.clone();
        bad[12 + 5] = 1; // a rule slot beyond rule_count
        assert_eq!(CompiledProfile::from_record(&bad).unwrap_err(), ProfileError::Rule);
        let mut bad = good.clone();
        bad[37] = 3; // spindle key F4
        assert_eq!(CompiledProfile::from_record(&bad).unwrap_err(), ProfileError::SpindleKey);
        let mut bad = good.clone();
        bad[36] = 0; // spindle key without a spindle
        assert_eq!(CompiledProfile::from_record(&bad).unwrap_err(), ProfileError::SpindleKey);
        assert!(CompiledProfile::from_record(&good[..51]).is_err());
    }
}
