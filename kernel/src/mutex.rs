use core::{cell::UnsafeCell, ops::{Deref, DerefMut}, ptr, sync::atomic::{AtomicBool, Ordering}};
use crate::inter::CS;

/// Mutex based on a spinlock
pub struct Spin<T> {
    /// Whether the mutex has been acquired or not
    atomic: AtomicBool,
    /// Protected data
    inner: UnsafeCell<T>
}

/// Mutex guard for `Spin<T>`
pub struct SpinGuard<'a, T> {
    /// Mutex this is the guard for
    mutex: &'a Spin<T>
}

/// Base address of the hardware spinlocks
const SPINLOCK_ADDR: usize = 0x100 + 0xd0000000;

/// Forces hardware spinlock to unlock
/// # Safety
/// Must be called when no lock has been acquired
pub unsafe fn force_spinlock_unlock() {
    unsafe {
        unlock_spinlock();
    }
}
    
/// Locks a hardware spinlock
fn lock_spinlock() {
    let lock: *const u32 = ptr::with_exposed_provenance(SPINLOCK_ADDR);
    loop {
        // SAFETY
        // This is reading from the spinlock address
        unsafe {
            let val = lock.read_volatile();
            // if non-zero, acquired the spinlock
            if val != 0 {
                break;
            }
        }
    }
}

/// unlocks a hardware spinlock
/// # Safety
/// This must be called after calling `lock_spinlock`
unsafe fn unlock_spinlock() {
    let lock: *mut u32 = ptr::with_exposed_provenance_mut(SPINLOCK_ADDR);
    // SAFETY
    // This is writing to the spinlock address
    unsafe {
        lock.write_volatile(1);
    }
}

impl<T> Spin<T> {
    /// Creates a spinlock
    /// `inner` is the data being protected by the spinlock
    pub const fn new(inner: T) -> Self {
        Self {
            inner: UnsafeCell::new(inner),
            atomic: AtomicBool::new(false)
        }
    }

    /// Locks the spinlock
    pub fn lock(&self) -> SpinGuard<'_, T> {
        loop {
            lock_spinlock();
            // Can update atomic as have locked spinlock
            if self.atomic.load(Ordering::Acquire) == false {
                self.atomic.store(true, Ordering::Release);
                // Safe as acquired spinlock
                unsafe {
                    unlock_spinlock();
                }
                break;
            } else {
                // Safe as acquired spinlock
                unsafe {
                    unlock_spinlock();
                }
            }
        }
        SpinGuard { mutex: self }
    }
}

unsafe impl<T: Send> Send for Spin<T> {}
unsafe impl<T: Sync> Sync for Spin<T> {}

impl<T> Drop for SpinGuard<'_, T> {
    /// Frees the spinlock when its guard goes out of scope
    fn drop(&mut self) {
        // nothing else should be writing to the atomic as is true
        self.mutex.atomic.store(false, Ordering::Release);
    }
}

impl<T> Deref for SpinGuard<'_, T> {
    type Target = T;

    /// Provides access to the protected data
    fn deref(&self) -> &Self::Target {
        // SAFETY
        // Safe to dereference as have locked mutex
        unsafe {
            & *self.mutex.inner.get()
        }
    }
}

impl<T> DerefMut for SpinGuard<'_, T> {
    /// Provides access to the protected data
    fn deref_mut(&mut self) -> &mut Self::Target {
        // SAFETY
        // Safe to dereference as have locked mutex
        unsafe {
            &mut *self.mutex.inner.get()
        }
    }
}

/// Mutex based on no interrupts
pub struct IRQMutex<T> {
    /// Protected data
    inner: UnsafeCell<T>
}

/// Mutex guard for `IRQMutex<T>`
pub struct IRQGuard<'a, 'b, T> {
    /// Mutex this is the guard for
    mutex: &'a IRQMutex<T>,
    /// The promise interrupts can't happen
    cs: &'b CS
}

impl<T> IRQMutex<T> {
    /// Creates an `IRQMutex<T>`  
    /// `inner` is the data being protected by the mutex
    /// # Safety
    /// It must be safe to access `inner` from multiple cores
    pub const unsafe fn new(inner: T) -> Self {
        Self { inner: UnsafeCell::new(inner) }
    }

    /// Locks the mutex
    pub fn lock<'a, 'b>(&'a self, cs: &'b CS) -> IRQGuard<'a, 'b, T> {
        IRQGuard { mutex: self, cs }
    }
}

unsafe impl<T: Send> Send for IRQMutex<T> {}
unsafe impl<T: Sync> Sync for IRQMutex<T> {}

impl<T> Deref for IRQGuard<'_, '_, T> {
    type Target = T;

    /// Provides access to the protected data
    fn deref(&self) -> &Self::Target {
        unsafe {
            & *self.mutex.inner.get()
        }
    }
}

impl<T> DerefMut for IRQGuard<'_, '_, T> {
    /// Provides access to the protected data
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe {
            &mut *self.mutex.inner.get()
        }
    }
}

/// Mutex based on no interrupts and with a spinlock
pub struct SpinIRQ<T> {
    /// Whether the mutex has been acquired or not
    atomic: AtomicBool,
    /// Protected data
    inner: UnsafeCell<T>
}

pub struct SpinIRQGuard<'a, 'b, T> {
    /// Mutex this is the guard for
    mutex: &'a SpinIRQ<T>,
    /// The promise interrupts can't happen
    cs: &'b CS
}

impl<T> SpinIRQ<T> {
    /// Creates a `SpinIRQ<T>`
    /// `inner` is the data being protected by the mutex
    pub const fn new(inner: T) -> Self {
        Self {
            inner: UnsafeCell::new(inner),
            atomic: AtomicBool::new(false)
        }
    }

    /// Locks the mutex
    /// `cs` is the critical section token
    pub fn lock<'a, 'b>(&'a self, cs: &'b CS) -> SpinIRQGuard<'a, 'b, T> {
        loop {
            lock_spinlock();
            // Can update atomic as have locked spinlock
            if self.atomic.load(Ordering::Acquire) == false {
                self.atomic.store(true, Ordering::Release);
                // Safe as acquired spinlock
                unsafe {
                    unlock_spinlock();
                }
                break;
            } else {
                // Safe as acquired spinlock
                unsafe {
                    unlock_spinlock();
                }
            }
        }
        SpinIRQGuard { mutex: self, cs }
    }
}

unsafe impl<T: Send> Send for SpinIRQ<T> {}
unsafe impl<T: Sync> Sync for SpinIRQ<T> {}

impl<T> Drop for SpinIRQGuard<'_, '_, T> {
    /// Frees the mutex when its guard goes out of scope
    fn drop(&mut self) {
        // nothing else should be writing to the atomic as is true
        self.mutex.atomic.store(false, Ordering::Release);
    }
}

impl<T> Deref for SpinIRQGuard<'_, '_, T> {
    type Target = T;

    /// Provides access to the protected data
    fn deref(&self) -> &Self::Target {
        // SAFETY
        // Safe to dereference as have locked mutex
        unsafe {
            & *self.mutex.inner.get()
        }
    }
}

impl<T> DerefMut for SpinIRQGuard<'_, '_, T> {
    /// Provides access to the protected data
    fn deref_mut(&mut self) -> &mut Self::Target {
        // SAFETY
        // Safe to dereference as have locked mutex
        unsafe {
            &mut *self.mutex.inner.get()
        }
    }
}
