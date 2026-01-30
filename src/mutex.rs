use core::{cell::UnsafeCell, ops::{Deref, DerefMut}, ptr, sync::atomic::{AtomicBool, Ordering}};
use crate::inter::CS;

pub struct Spin<T> {
    atomic: AtomicBool,
    inner: UnsafeCell<T>
}

pub struct SpinGuard<'a, T> {
    mutex: &'a Spin<T>
}

const SPINLOCK_ADDR: usize = 0x100 + 0xd0000000;

/// Forces hardware spinlock to unlock
/// SAFETY
/// Must be called when no lock has been acquired
pub unsafe fn force_spinlock_unlock() {
    unlock_spinlock();
}
    
fn lock_spinlock() {
        let lock: *const u32 = ptr::with_exposed_provenance(SPINLOCK_ADDR);
        loop {
            // SAFETY
            // This is reading from the spinlock address
            unsafe {
                let val = lock.read_volatile();
                if val != 0 {
                    break;
                }
            }
        }
    }

fn unlock_spinlock() {
    let lock: *mut u32 = ptr::with_exposed_provenance_mut(SPINLOCK_ADDR);
    // SAFETY
    // This is writing to the spinlock address
    unsafe {
        lock.write_volatile(1);
    }
}

impl<T> Spin<T> {
    pub const fn new(inner: T) -> Self {
        Self {
            inner: UnsafeCell::new(inner),
            atomic: AtomicBool::new(false)
        }
    }


    pub fn lock(&self) -> SpinGuard<'_, T> {
        loop {
            lock_spinlock();
            // Can update atomic as have locked spinlock
            if self.atomic.load(Ordering::Acquire) == false {
                self.atomic.store(true, Ordering::Release);
                unlock_spinlock();
                break;
            } else {
                unlock_spinlock();
            }
        }
        SpinGuard { mutex: self }
    }
}

unsafe impl<T: Send> Send for Spin<T> {}
unsafe impl<T: Sync> Sync for Spin<T> {}

impl<T> Drop for SpinGuard<'_, T> {
    fn drop(&mut self) {
        // nothing else should be writing to the atomic as is true
        self.mutex.atomic.store(false, Ordering::Release);
    }
}

impl<T> Deref for SpinGuard<'_, T> {
    type Target = T;

    // SAFETY
    // Safe to dereference as have locked mutex
    fn deref(&self) -> &Self::Target {
        unsafe {
            & *self.mutex.inner.get()
        }
    }
}

impl<T> DerefMut for SpinGuard<'_, T> {
    // SAFETY
    // Safe to dereference as have locked mutex
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe {
            &mut *self.mutex.inner.get()
        }
    }
}

pub struct IRQMutex<T> {
    inner: UnsafeCell<T>
}

pub struct IRQGuard<'a, 'b, T> {
    mutex: &'a IRQMutex<T>,
    cs: &'b CS
}

impl<T> IRQMutex<T> {
    pub const unsafe fn new(inner: T) -> Self {
        Self { inner: UnsafeCell::new(inner) }
    }

    pub fn lock<'a, 'b>(&'a self, cs: &'b CS) -> IRQGuard<'a, 'b, T> {
        IRQGuard { mutex: self, cs }
    }
}

unsafe impl<T: Send> Send for IRQMutex<T> {}
unsafe impl<T: Sync> Sync for IRQMutex<T> {}

impl<T> Deref for IRQGuard<'_, '_, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        unsafe {
            & *self.mutex.inner.get()
        }
    }
}

impl<T> DerefMut for IRQGuard<'_, '_, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe {
            &mut *self.mutex.inner.get()
        }
    }
}

pub struct SpinIRQ<T> {
    atomic: AtomicBool,
    inner: UnsafeCell<T>
}

pub struct SpinIRQGuard<'a, 'b, T> {
    mutex: &'a SpinIRQ<T>,
    cs: &'b CS
}

impl<T> SpinIRQ<T> {
    pub const fn new(inner: T) -> Self {
        Self {
            inner: UnsafeCell::new(inner),
            atomic: AtomicBool::new(false)
        }
    }


    pub fn lock<'a, 'b>(&'a self, cs: &'b CS) -> SpinIRQGuard<'a, 'b, T> {
        loop {
            lock_spinlock();
            // Can update atomic as have locked spinlock
            if self.atomic.load(Ordering::Acquire) == false {
                self.atomic.store(true, Ordering::Release);
                unlock_spinlock();
                break;
            } else {
                unlock_spinlock();
            }
        }
        SpinIRQGuard { mutex: self, cs }
    }
}

unsafe impl<T: Send> Send for SpinIRQ<T> {}
unsafe impl<T: Sync> Sync for SpinIRQ<T> {}

impl<T> Drop for SpinIRQGuard<'_, '_, T> {
    fn drop(&mut self) {
        // nothing else should be writing to the atomic as is true
        self.mutex.atomic.store(false, Ordering::Release);
    }
}

impl<T> Deref for SpinIRQGuard<'_, '_, T> {
    type Target = T;

    // SAFETY
    // Safe to dereference as have locked mutex
    fn deref(&self) -> &Self::Target {
        unsafe {
            & *self.mutex.inner.get()
        }
    }
}

impl<T> DerefMut for SpinIRQGuard<'_, '_, T> {
    // SAFETY
    // Safe to dereference as have locked mutex
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe {
            &mut *self.mutex.inner.get()
        }
    }
}
