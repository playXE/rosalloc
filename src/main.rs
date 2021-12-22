#![feature(allocator_api)]
use std::{
    alloc::{Allocator, Layout},
    ops::{Deref, DerefMut},
    ptr::NonNull,
};

use rosalloc::{
    allocator::RosallocAllocator,
    defs::{GB, NUM_THREAD_LOCAL_SIZE_BRACKETS},
    BRACKET_SIZES, NUM_OF_PAGES, NUM_OF_SLOTS,
};

pub struct UniquePtr<T> {
    alloc: RosallocAllocator,
    ptr: NonNull<T>,
}
impl<T> UniquePtr<T> {
    pub fn new(alloc: RosallocAllocator, value: T) -> Self {
        let ptr = alloc
            .allocate(Layout::new::<T>())
            .unwrap()
            .as_ptr()
            .cast::<T>();
        unsafe {
            ptr.write(value);
            Self {
                alloc,
                ptr: NonNull::new_unchecked(ptr),
            }
        }
    }
}

impl<T> Drop for UniquePtr<T> {
    fn drop(&mut self) {
        unsafe {
            self.alloc.deallocate(self.ptr.cast(), Layout::new::<T>());
        }
    }
}

impl<T> Deref for UniquePtr<T> {
    type Target = T;
    fn deref(&self) -> &Self::Target {
        unsafe { &*self.ptr.as_ptr() }
    }
}

impl<T> DerefMut for UniquePtr<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { &mut *self.ptr.as_ptr() }
    }
}

impl<T> std::fmt::Pointer for UniquePtr<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:p}", self.ptr)
    }
}

fn main() {
    println!(
        "{:?}\n{:?}\n{:?}",
        &BRACKET_SIZES[0..NUM_THREAD_LOCAL_SIZE_BRACKETS],
        &NUM_OF_PAGES[0..NUM_THREAD_LOCAL_SIZE_BRACKETS],
        &NUM_OF_SLOTS[0..NUM_THREAD_LOCAL_SIZE_BRACKETS]
    );
    let rosalloc = RosallocAllocator::new(2 * GB);
    let x = UniquePtr::new(rosalloc, 48);

    println!("{:p}", x);
    drop(x);
    unsafe {
        rosalloc.force_merge_freelists_tls();
    }
    let x = UniquePtr::new(rosalloc, 48);
    println!("{:p}", x);
}
