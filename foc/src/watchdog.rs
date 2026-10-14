//! Independent-watchdog feed policy. Only the supervisor evaluates it.
//!
//! Both the ADC ISR and encoder task must complete new work since the last
//! feed. Calling this policy itself establishes supervisor progress; neither
//! worker can keep the hardware watchdog alive on its own.

pub struct FeedPolicy {
    adc_sequence: u32,
    encoder_sequence: u32,
    last_feed_ms: u32,
    interval_ms: u32,
}

impl FeedPolicy {
    pub const fn new(
        adc_sequence: u32,
        encoder_sequence: u32,
        now_ms: u32,
        interval_ms: u32,
    ) -> Self {
        Self {
            adc_sequence,
            encoder_sequence,
            last_feed_ms: now_ms,
            interval_ms,
        }
    }

    /// Consume progress only when a feed is permitted. Sequences and time wrap.
    /// There is deliberately no Idle or maintenance bypass: short blocking
    /// operations use the hardware timeout allowance, then must show progress.
    pub fn should_feed(&mut self, adc: u32, encoder: u32, now_ms: u32) -> bool {
        if now_ms.wrapping_sub(self.last_feed_ms) < self.interval_ms
            || adc == self.adc_sequence
            || encoder == self.encoder_sequence
        {
            return false;
        }
        self.adc_sequence = adc;
        self.encoder_sequence = encoder;
        self.last_feed_ms = now_ms;
        true
    }
}
