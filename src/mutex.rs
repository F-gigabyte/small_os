use core::{cell::UnsafeCell, ops::{Deref, DerefMut}, ptr, sync::atomic::{AtomicBool, Ordering}};

pub struct Mutex<T> {
    atomic: AtomicBool,
    inner: UnsafeCell<T>
}

pub struct MutexGuard<'a, T> {
    mutex: &'a Mutex<T>
}

const SPINLOCK_ADDR: usize = 0x100 + 0xd0000000;

impl<T> Mutex<T> {
    pub const fn new(inner: T) -> Self {
        Self {
            inner: UnsafeCell::new(inner),
            atomic: AtomicBool::new(false)
        }
    }

    fn lock_spinlock() {
        let lock: *const u32 = ptr::with_exposed_provenance(SPINLOCK_ADDR);
        loop {
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
        unsafe {
            lock.write_volatile(1);
        }
    }

    pub fn lock(&self) -> MutexGuard<'_, T> {
        /*loop {
            Self::lock_spinlock();
            if self.atomic.load(Ordering::Acquire) == false {
                self.atomic.store(true, Ordering::Release);
                Self::unlock_spinlock();
                break;
            } else {
                Self::unlock_spinlock();
            }
        }*/
        MutexGuard { mutex: self }
    }
}

unsafe impl<T: Send> Send for Mutex<T> {}
unsafe impl<T: Sync> Sync for Mutex<T> {}

impl<T> Drop for MutexGuard<'_, T> {
    fn drop(&mut self) {
        // nothing else should be writing to the atomic as is true
        self.mutex.atomic.store(false, Ordering::Release);
    }
}

impl<T> Deref for MutexGuard<'_, T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        unsafe {
            & *self.mutex.inner.get()
        }
    }
}

impl<T> DerefMut for MutexGuard<'_, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe {
            &mut *self.mutex.inner.get()
        }
    }
}
