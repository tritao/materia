use robotkit_device_protocol::ScheduledSegment;

#[test]
fn f32_evaluator_matches_motionkit_core_vectors() {
    let segment = ScheduledSegment::new(1, 0, 1_000, 5,
        [[0.1, 1.25, -0.3, 0.2, -0.05, 0.005]], false).unwrap();
    for line in include_str!("segment_vectors.tsv").lines() {
        let mut fields = line.split('\t');
        let tick: u64 = fields.next().unwrap().parse().unwrap();
        let position: f64 = fields.next().unwrap().parse().unwrap();
        let velocity: f64 = fields.next().unwrap().parse().unwrap();
        let (q, v) = segment.evaluate(tick, 1_000);
        assert!((q[0] as f64 - position).abs() < 2e-6);
        assert!((v[0] as f64 - velocity).abs() < 2e-6);
    }
}
