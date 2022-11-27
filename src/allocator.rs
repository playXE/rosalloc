use std::{
    alloc::{AllocError, Allocator},
    cell::UnsafeCell,
    ptr::{null_mut, NonNull},
};

use crate::{
    dedicated_full_run,
    defs::{NUM_THREAD_LOCAL_SIZE_BRACKETS, PAGE_SIZE},
    mmap::Mmap,
    round_up, Rosalloc, Run,
};

#[derive(Clone, Copy)]
pub struct RosallocAllocator {
    inner: *mut Inner,
}

struct TLSRuns {
    alloc: *mut Rosalloc,
    runs: [*mut Run; NUM_THREAD_LOCAL_SIZE_BRACKETS],
}

unsafe impl Send for TLSRuns {}

impl Default for TLSRuns {
    fn default() -> Self {
        Self {
            alloc: null_mut(),
            runs: [dedicated_full_run(); NUM_THREAD_LOCAL_SIZE_BRACKETS],
        }
    }
}
struct Inner {
    rosalloc: *mut Rosalloc,
    map: Mmap,
    tls: thread_local::ThreadLocal<UnsafeCell<TLSRuns>>,
}

unsafe impl Send for RosallocAllocator {}
unsafe impl Sync for RosallocAllocator {}

impl RosallocAllocator {
    pub fn new(capacity: usize) -> Self {
        let map = Mmap::new(round_up(capacity as _, PAGE_SIZE as _) as _);

        let rosalloc = Rosalloc::new(
            map.start(),
            map.size(),
            map.size(),
            crate::defs::PageReleaseMode::SizeAndEnd,
            4096,
        );
        Self {
            inner: Box::into_raw(Box::new(Inner {
                rosalloc,
                map,
                tls: thread_local::ThreadLocal::new(),
            })),
        }
    }

    pub fn dispose(alloc: Self) {
        unsafe {
            let mut alloc = Box::from_raw(alloc.inner);
            let rosalloc = alloc.rosalloc;
            let map = std::mem::replace(&mut alloc.map, Mmap::uninit());
            drop(alloc);
            drop(Box::from_raw(rosalloc));
            drop(map);
        }
    }
    unsafe fn get_tls(&self) -> &mut TLSRuns {
        let inner = &mut *self.inner;
        let rosalloc = inner.rosalloc;
        &mut *inner
            .tls
            .get_or(|| {
                let mut tls = TLSRuns::default();
                tls.alloc = rosalloc;
                UnsafeCell::new(tls)
            })
            .get()
    }
    unsafe fn allocate_local(&self, size: usize) -> *mut u8 {
        let inner = &mut *self.inner;

        (*inner.rosalloc).alloc::<true>(&mut self.get_tls().runs, size, &mut 0, &mut 0, &mut 0)
    }

    unsafe fn alloc_impl(&self, size: usize) -> Result<NonNull<[u8]>, AllocError> {
        let ptr = self.allocate_local(size);
        let ptr = NonNull::new(ptr).ok_or(AllocError)?;
        Ok(NonNull::slice_from_raw_parts(ptr, size))
    }
    unsafe fn free_impl(&self, ptr: *mut u8) {
        let inner = &mut *self.inner;
        (*inner.rosalloc).free(ptr);
    }

    pub unsafe fn force_merge_freelists_tls(&self) {
        self.get_tls().runs.iter().for_each(|run| {
            let run = *run;
            (*run).merge_thread_local_free_list_to_free_list(&mut false);
        });
    }

    pub unsafe fn usable_size(&self, ptr: *mut u8) -> usize {
        let inner = &mut *self.inner;
        (*inner.rosalloc).usable_size(ptr)
    }

    pub unsafe fn block_start(&self, ptr: *const u8) -> *mut u8 {
        let inner = &mut *self.inner;
        (*inner.rosalloc).block_start(ptr as _)
    }
}
unsafe impl Allocator for RosallocAllocator {
    fn allocate(
        &self,
        layout: std::alloc::Layout,
    ) -> Result<std::ptr::NonNull<[u8]>, std::alloc::AllocError> {
        let size = if layout.size() < 8 { 8 } else { layout.size() };
        unsafe { self.alloc_impl(size) }
    }

    unsafe fn deallocate(&self, ptr: NonNull<u8>, _layout: std::alloc::Layout) {
        self.free_impl(ptr.as_ptr().cast());
    }
}

