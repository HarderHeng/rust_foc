//! Exclusive controller ownership with short interrupt-masked transfers.
//!
//! Computation owns `T` on its stack. Reentrant calls cannot borrow that value;
//! reset/gain requests enter a mailbox and invalidate the outstanding result.

use core::cell::RefCell;

use cortex_m::interrupt::Mutex;

pub trait ControllerState {
    fn reset(&mut self);
    fn set_gains(&mut self, kp: f32, ki: f32);
}

struct Slot<T> {
    state: Option<T>,
    initialized: bool,
    revision: u32,
    reset: bool,
    gains: Option<(f32, f32)>,
}

impl<T: ControllerState> Slot<T> {
    fn apply_requests(&mut self, state: &mut T) {
        if self.reset {
            state.reset();
            self.reset = false;
        }
        if let Some((kp, ki)) = self.gains.take() {
            state.set_gains(kp, ki);
        }
    }
}

pub struct Controller<T>(Mutex<RefCell<Slot<T>>>);

impl<T: ControllerState> Default for Controller<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: ControllerState> Controller<T> {
    pub const fn new() -> Self {
        Self(Mutex::new(RefCell::new(Slot {
            state: None,
            initialized: false,
            revision: 0,
            reset: false,
            gains: None,
        })))
    }

    pub fn init(&self, mut state: T) {
        cortex_m::interrupt::free(|cs| {
            let cell = self.0.borrow(cs);
            let mut slot = cell.borrow_mut();
            assert!(!slot.initialized, "controller already initialized");
            slot.apply_requests(&mut state);
            slot.state = Some(state);
            slot.initialized = true;
        });
    }

    pub fn reset(&self) {
        self.request(|slot| slot.reset = true);
    }

    pub fn set_gains(&self, kp: f32, ki: f32) {
        self.request(|slot| slot.gains = Some((kp, ki)));
    }

    fn request(&self, change: impl FnOnce(&mut Slot<T>)) {
        cortex_m::interrupt::free(|cs| {
            let cell = self.0.borrow(cs);
            let mut slot = cell.borrow_mut();
            slot.revision = slot.revision.wrapping_add(1);
            change(&mut slot);
            if let Some(mut state) = slot.state.take() {
                slot.apply_requests(&mut state);
                slot.state = Some(state);
            }
        });
    }

    /// Run compute with the caller's interrupt state unchanged. Commit runs in
    /// a short critical section only if no reset/gain request arrived meanwhile;
    /// it must also validate the control-mode token before publishing outputs.
    /// Reentrant computation returns false without aliasing checked-out state.
    pub fn compute<R>(
        &self,
        compute: impl FnOnce(&mut T) -> R,
        commit: impl FnOnce(R) -> bool,
    ) -> bool {
        let owned = cortex_m::interrupt::free(|cs| {
            let cell = self.0.borrow(cs);
            let mut slot = cell.borrow_mut();
            slot.state.take().map(|state| (state, slot.revision))
        });
        let Some((mut state, revision)) = owned else {
            return false;
        };
        let result = compute(&mut state);
        cortex_m::interrupt::free(|cs| {
            let current = {
                let cell = self.0.borrow(cs);
                let mut slot = cell.borrow_mut();
                let current = revision == slot.revision;
                slot.apply_requests(&mut state);
                slot.state = Some(state);
                current
            }; // Commit can request a reset; release the RefCell first.
            current && commit(result)
        })
    }
}
