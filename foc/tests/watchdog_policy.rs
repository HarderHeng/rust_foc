use foc::watchdog::FeedPolicy;

#[test]
fn adc_interrupts_cannot_keep_a_stalled_executor_alive() {
    let mut policy = FeedPolicy::new(0, 0, 0, 50);
    // Even calling the supervisor policy cannot conceal a stalled encoder task.
    for ms in 1..=2_000 {
        assert!(!policy.should_feed(ms * 20, 0, ms));
    }
}

#[test]
fn task_progress_cannot_conceal_a_missing_current_irq() {
    let mut policy = FeedPolicy::new(400, 7, 0, 50);
    for ms in 1..=2_000 {
        assert!(!policy.should_feed(400, 7 + ms, ms));
    }
}

#[test]
fn every_feed_requires_new_work_from_both_workers() {
    let mut policy = FeedPolicy::new(0, 0, 0, 50);
    assert!(policy.should_feed(1_000, 100, 50));
    assert!(!policy.should_feed(1_000, 100, 100));
    assert!(!policy.should_feed(2_000, 100, 101));
    assert!(!policy.should_feed(1_000, 200, 102));
    // Rejected observations must not consume either worker's progress.
    assert!(policy.should_feed(2_000, 200, 103));
    assert!(!policy.should_feed(2_000, 200, 153));
}

#[test]
fn feeds_require_elapsed_wall_time() {
    let mut policy = FeedPolicy::new(0, 0, 10, 50);
    assert!(!policy.should_feed(100, 1, 10));
    assert!(!policy.should_feed(500, 5, 59));
    assert!(policy.should_feed(500, 5, 60));
    assert!(!policy.should_feed(501, 6, 60));
    assert!(!policy.should_feed(900, 10, 109));
    assert!(policy.should_feed(1_500, 15, 110));
}

#[test]
fn idle_maintenance_uses_timeout_margin_and_requires_progress_afterwards() {
    let mut policy = FeedPolicy::new(0, 0, 0, 50);
    assert!(policy.should_feed(1_000, 100, 50));
    // During a blocking flash save or calibration the supervisor does not run.
    // A 200 ms pause needs no special feed or Idle exemption.
    assert!(!policy.should_feed(1_000, 100, 250));
    assert!(!policy.should_feed(1_001, 100, 251));
    assert!(policy.should_feed(1_002, 101, 252));
}

#[test]
fn worker_sequences_and_millisecond_clock_can_wrap() {
    let mut policy = FeedPolicy::new(u32::MAX, u32::MAX, u32::MAX - 24, 50);
    assert!(!policy.should_feed(0, 0, 24));
    assert!(policy.should_feed(0, 0, 25));
    assert!(!policy.should_feed(1, 0, 75));
    assert!(policy.should_feed(1, 1, 75));
}
