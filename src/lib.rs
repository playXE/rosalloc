#![cfg_attr(
    feature = "allocator-api",
    feature(allocator_api, nonnull_slice_from_raw_parts)
)]
#![allow(clippy::identity_op, clippy::if_same_then_else)]
#[cfg(feature = "allocator-api")]
pub mod allocator;
pub mod defs;
pub mod mmap;
use defs::*;
use indexmap::IndexSet;
use mmap::Mmap;
use parking_lot::{lock_api::RawMutex, lock_api::RawRwLock, RawMutex as Lock, RawRwLock as RwLock};
use std::{
    collections::HashSet,
    hash::{BuildHasherDefault, Hasher},
    mem::size_of,
    ptr::null_mut,
    sync::atomic::AtomicBool,
};


pub const TRACE_ROSALLOC: bool = true;

#[repr(C)]
pub struct FreePageRun {
    pub magic_num: u8,
}

static mut DEDICATED_FULL_RUN_STORAGE: [usize; PAGE_SIZE / size_of::<usize>()] =
    [0; PAGE_SIZE / size_of::<usize>()];
static mut DEDICATED_FULL_RUN: *mut Run = null_mut();
static INITIALIZED: AtomicBool = AtomicBool::new(false);

unsafe fn initialize() {
    
    INITIALIZED.store(true, std::sync::atomic::Ordering::Release);
    #[cfg(not(miri))]
    {
        DEDICATED_FULL_RUN = DEDICATED_FULL_RUN_STORAGE.as_mut_ptr().cast();
    }
    #[cfg(miri)]
    {
        DEDICATED_FULL_RUN = libc::malloc(size_of::<usize>() * (PAGE_SIZE / size_of::<usize>())) as *mut Run;
        core::ptr::write_bytes(DEDICATED_FULL_RUN as *mut u8, 0, size_of::<usize>() * (PAGE_SIZE / size_of::<usize>()));
    }
    if cfg!(debug_assertions) {
        (*DEDICATED_FULL_RUN).magic_num = MAGIC_NUM;
    }
    (*DEDICATED_FULL_RUN).size_bracket_idx = 0;
    (*DEDICATED_FULL_RUN).is_thread_local = 1;
}

impl FreePageRun {
    pub fn is_free(&self) -> bool {
        !cfg!(debug_assertions) || self.magic_num == MAGIC_NUM_FREE
    }

    pub fn begin(&self) -> *mut u8 {
        self as *const Self as *mut u8
    }
    #[inline]
    pub unsafe fn byte_size(&self, rosalloc: &Rosalloc) -> usize {
        let fpr_base = self as *const Self as *mut u8;

        let pm_idx = rosalloc.to_page_map_index(fpr_base);

        *rosalloc.free_page_run_size_map.get_unchecked(pm_idx)
    }

    pub unsafe fn set_byte_size(&self, rosalloc: &mut Rosalloc, byte_size: usize) {
        let fpr_base = self as *const Self as *mut u8;

        let pm_idx = rosalloc.to_page_map_index(fpr_base);
        rosalloc.free_page_run_size_map[pm_idx] = byte_size;
    }

    pub fn end(&self, rosalloc: &Rosalloc) -> *mut u8 {
        let fpr_base = self as *const Self as *mut u8;

        unsafe { fpr_base.add(self.byte_size(rosalloc)) }
    }

    pub unsafe fn is_larger_than_page_release_threshold(&self, rosalloc: &Rosalloc) -> bool {
        self.byte_size(rosalloc) >= rosalloc.page_release_size_threshold
    }

    pub unsafe fn is_at_end_of_space(&self, rosalloc: &Rosalloc) -> bool {
        self as *const Self as usize + self.byte_size(rosalloc)
            == rosalloc.base as usize + rosalloc.footprint
    }

    pub unsafe fn should_release_pages(&self, rosalloc: &Rosalloc) -> bool {
        match rosalloc.page_release_mode {
            PageReleaseMode::None => false,
            PageReleaseMode::End => self.is_at_end_of_space(rosalloc),
            PageReleaseMode::Size => self.is_larger_than_page_release_threshold(rosalloc),
            PageReleaseMode::SizeAndEnd => {
                self.is_larger_than_page_release_threshold(rosalloc)
                    && self.is_at_end_of_space(rosalloc)
            }
            PageReleaseMode::All => true,
        }
    }

    pub unsafe fn release_pages(&self, rosalloc: &mut Rosalloc) {
        let start = self as *const Self as *mut u8;
        let byte_size = self.byte_size(rosalloc);
        if self.should_release_pages(rosalloc) {
            rosalloc.release_page_range(start, (start as usize + byte_size) as _);
        }
    }
}

pub struct Slot {
    pub next: *mut Self,
}

impl Slot {
    #[inline]
    pub fn next(&self) -> *mut Self {
        self.next
    }
    #[inline]
    pub fn set_next(&mut self, next: *mut Self) {
        self.next = next;
    }
    #[inline]
    pub fn left(&self, bracket_size: usize) -> *mut Self {
        let this = self as *const Self as usize;
        (this - bracket_size) as _
    }
    #[inline]
    pub fn clear(&mut self) {
        self.next = null_mut();
    }
}

// We use the tail (`USE_TAIL` == true) for the bulk or thread-local free lists to avoid the need to
// traverse the list from the head to the tail when merging free lists.
// We don't use the tail (`USE_TAIL` == false) for the free list to avoid the need to manage the
// tail in the allocation fast path for a performance reason.
#[repr(C)]
pub struct SlotFreeList<const USE_TAIL: bool> {
    /// A pointer (Slot*) to the head of the list. Always 8 bytes so that we will have the same
    /// layout between 32 bit and 64 bit, which is not strictly necessary, but we do so for 1)
    /// uniformity, 2) we won't need to change this code if we move to a non-low 4G heap in the
    /// future, and 3) the space savings by using 32 bit fields in 32 bit would be lost in noise
    /// (won't open up enough space to cause an extra slot to be available).
    head: u64,
    /// A pointer (Slot*) to the tail of the list. Always 8 bytes so that we will have the same
    /// layout between 32 bit and 64 bit. The tail is stored to speed up merging of lists.
    /// Unused if USE_TAIL is false.
    tail: u64,
    /// The number of slots in the list. This is used to make it fast to check if a free list is all
    /// free without traversing the whole free list.
    size: u32,
    #[allow(dead_code)]
    padding: u32,
}

impl<const USE_TAIL: bool> SlotFreeList<USE_TAIL> {
    pub const fn new() -> Self {
        Self {
            head: 0,
            tail: 0,
            size: 0,
            padding: 0,
        }
    }
    #[inline]
    pub fn head(&self) -> *mut Slot {
        self.head as *mut _
    }
    #[inline]
    pub fn tail(&self) -> *mut Slot {
        assert!(USE_TAIL);
        self.tail as *mut _
    }
    #[inline]
    pub const fn size(&self) -> usize {
        self.size as _
    }
    #[inline]
    pub fn remove(&mut self) -> *mut Slot {
        let slot;
        if cfg!(debug_assertions) {
            self.verify();
        }
        unsafe {
            let headp = &mut self.head as *mut u64 as *mut *mut Slot;
            let tailp = if USE_TAIL {
                &mut self.tail as *mut u64 as *mut *mut Slot
            } else {
                null_mut()
            };
            let old_head = *headp;
            if old_head.is_null() {
                if USE_TAIL {
                    debug_assert!(tailp.read().is_null());
                }
                return null_mut();
            } else {
                if USE_TAIL {
                    debug_assert!(!tailp.read().is_null());
                }
                let old_head_next = (*old_head).next();
                slot = old_head;
                headp.write(old_head_next);
                if USE_TAIL && old_head_next.is_null() {
                    tailp.write(null_mut());
                }
            }
            (*slot).clear();
            self.size -= 1;
            if cfg!(debug_assertions) {
                self.verify();
            }
            slot
        }
    }
    #[inline]
    pub fn add(&mut self, slot: *mut Slot) {
        if cfg!(debug_assertions) {
            self.verify();
        }
        debug_assert!(!slot.is_null());
        unsafe {
            debug_assert!((*slot).next().is_null());
            let headp = &mut self.head as *mut u64 as *mut *mut Slot;
            let tailp = if USE_TAIL {
                &mut self.tail as *mut u64 as *mut *mut Slot
            } else {
                null_mut()
            };
            let old_head = *headp;

            if old_head.is_null() {
                if USE_TAIL {
                    debug_assert!(tailp.read().is_null());
                }
                headp.write(slot);
                if USE_TAIL {
                    tailp.write(slot);
                }
            } else {
                if USE_TAIL {
                    debug_assert!(!tailp.read().is_null());
                }
                headp.write(slot);
                (*slot).set_next(old_head);
            }
            self.size += 1;
            if cfg!(debug_assertions) {
                self.verify();
            }
        }
    }
    /// Merge the given list into this list. Empty the given list.
    /// Deliberately support only a kUseTail == true SlotFreeList parameter because 1) we don't
    /// currently have a situation where we need a kUseTail == false SlotFreeList parameter, and 2)
    /// supporting the kUseTail == false parameter would require a O(n) linked list traversal to do
    /// the merge if 'this' SlotFreeList has kUseTail == false, which we'd like to avoid.
    pub fn merge(&mut self, list: &mut SlotFreeList<true>) {
        if cfg!(debug_assertions) {
            self.verify();
            list.verify();
        }

        if list.size() == 0 {
            return;
        }
        let headp = &mut self.head as *mut u64 as *mut *mut Slot;
        let tailp = if USE_TAIL {
            &mut self.tail as *mut u64 as *mut *mut Slot
        } else {
            null_mut()
        };
        unsafe {
            let old_head = headp.read();
            if old_head.is_null() {
                headp.write(list.head());
                if USE_TAIL {
                    tailp.write(list.tail());
                }
                self.size = list.size() as _;
            } else {
                headp.write(list.head());
                (*list.tail()).set_next(old_head);
                self.size += list.size() as u32;
            }
            list.reset();
            if cfg!(debug_assertions) {
                self.verify();
            }
        }
    }

    pub fn reset(&mut self) {
        self.head = 0;
        if USE_TAIL {
            self.tail = 0;
        }
        self.size = 0;
    }
    fn verify(&self) {
        unsafe {
            let head = self.head();
            let tail = if USE_TAIL { self.tail() } else { null_mut() };

            if self.size == 0 {
                assert!(head.is_null());
                if USE_TAIL {
                    assert!(tail.is_null());
                }
            } else {
                assert!(!head.is_null());
                if USE_TAIL {
                    assert!(!tail.is_null());
                }

                let mut count = 0;
                let mut slot = head;
                while !slot.is_null() {
                    count += 1;
                    if USE_TAIL && (*slot).next().is_null() {
                        assert_eq!(slot, tail);
                    }
                    slot = (*slot).next();
                }
                assert_eq!(count, self.size);
            }
        }
    }
}

// Represents a run of memory slots of the same size.
//
// A run's memory layout:
//
// +-------------------+
// | magic_num         |
// +-------------------+
// | size_bracket_idx  |
// +-------------------+
// | is_thread_local   |
// +-------------------+
// | to_be_bulk_freed  |
// +-------------------+
// |                   |
// | free list         |
// |                   |
// +-------------------+
// |                   |
// | bulk free list    |
// |                   |
// +-------------------+
// |                   |
// | thread-local free |
// | list              |
// |                   |
// +-------------------+
// | padding due to    |
// | alignment         |
// +-------------------+
// | slot 0            |
// +-------------------+
// | slot 1            |
// +-------------------+
// | slot 2            |
// +-------------------+
// ...
// +-------------------+
// | last slot         |
// +-------------------+
//
#[repr(C)]
pub struct Run {
    pub magic_num: u8,
    pub size_bracket_idx: u8,
    pub is_thread_local: u8,
    pub to_be_bulk_freed: bool,
    #[allow(dead_code)]
    pub padding: u32,
    pub free_list: SlotFreeList<false>,
    pub bulk_free_list: SlotFreeList<true>,
    pub thread_local_free_list: SlotFreeList<true>,
}

impl Run {
    #[inline]
    pub fn alloc_slot(&mut self) -> *mut Slot {
        let slot = self.free_list.remove();
        #[cfg(feature="trace")]
        if !slot.is_null() {
            log::info!(
                "RosAlloc::Run::alloc_slot() : {:p}, bracket_size={}, slot_idx={}",
                slot,
                BRACKET_SIZES[self.size_bracket_idx as usize],
                unsafe { self.slot_index(slot as _) }
            );
        }
        slot 
    }
    #[inline]
    pub fn is_full(&self) -> bool {
        self.free_list.size() == 0
    }
    #[inline]
    pub fn is_all_free(&self) -> bool {
        self.free_list.size() == NUM_OF_SLOTS[self.size_bracket_idx as usize]
    }
    #[inline]
    pub unsafe fn free_slot(&mut self, ptr: *mut u8) {
        let idx = self.size_bracket_idx as usize;
        let bracket_size = *BRACKET_SIZES.get_unchecked(idx);
        let slot = self.to_slot(ptr);

        core::ptr::write_bytes(slot.cast::<u8>(), 0, bracket_size);
        //memx::memset(std::slice::from_raw_parts_mut(slot.cast(), bracket_size), 0);

        self.free_list.add(slot);
    }
    #[inline]
    pub fn merge_thread_local_free_list_to_free_list(
        &mut self,
        is_all_free_after_out: &mut bool,
    ) -> bool {
        debug_assert!(self.is_thread_local == 1);
        let idx = self.size_bracket_idx as usize;
        let size_before = self.free_list.size();
        let mut list = std::mem::replace(&mut self.thread_local_free_list, SlotFreeList::new());
        self.free_list.merge(&mut list);
        self.thread_local_free_list = list;
        let size_after = self.free_list.size();
        *is_all_free_after_out = self.free_list.size() == NUM_OF_SLOTS[idx];
        size_before < size_after
    }

    #[inline]
    pub fn merge_bulk_free_list_to_free_list(&mut self) {
        debug_assert!(self.is_thread_local == 0);
        let mut list = std::mem::replace(&mut self.bulk_free_list, SlotFreeList::new());
        self.free_list.merge(&mut list);
        self.bulk_free_list = list;
    }

    #[inline]
    pub fn merge_bulk_free_list_to_thread_local_free_list(&mut self) {
        debug_assert!(self.is_thread_local == 0);
        let mut list = std::mem::replace(&mut self.bulk_free_list, SlotFreeList::new());
        self.thread_local_free_list.merge(&mut list);
        self.bulk_free_list = list;
    }

    #[inline]
    pub unsafe fn add_to_free_list_shared(
        &mut self,
        ptr: *mut u8,
        free_list: &mut SlotFreeList<true>,
    ) -> usize {
        let idx = self.size_bracket_idx as usize;
        let bracket_size = *BRACKET_SIZES.get_unchecked(idx);

        let slot = self.to_slot(ptr);
        core::ptr::write_bytes(slot.cast::<u8>(), 0, bracket_size);
        (*free_list).add(slot);

        bracket_size
    }

    #[inline]
    pub unsafe fn add_to_bulk_free_list(&mut self, ptr: *mut u8) -> usize {
        
        //self.add_to_free_list_shared(ptr, list)

        let idx = self.size_bracket_idx as usize;
        let bracket_size = *BRACKET_SIZES.get_unchecked(idx);

        let slot = self.to_slot(ptr);
        let list = &mut self.bulk_free_list;
        core::ptr::write_bytes(slot.cast::<u8>(), 0, bracket_size);
        list.add(slot);

        bracket_size
    }

    #[inline]
    pub unsafe fn add_to_thread_local_free_list(&mut self, ptr: *mut u8) {
        /*let list = &mut self.thread_local_free_list;
        self.add_to_free_list_shared(ptr, list);*/

        let idx = self.size_bracket_idx as usize;
        let bracket_size = *BRACKET_SIZES.get_unchecked(idx);

        let slot = self.to_slot(ptr);
        let list = &mut self.thread_local_free_list;
        core::ptr::write_bytes(slot.cast::<u8>(), 0, bracket_size);
        list.add(slot);

    }

    #[inline]
    pub fn zero_header_and_slot_headers(&mut self) {
        let idx = self.size_bracket_idx as usize;
        let mut slot = self.free_list.head();
        while !slot.is_null() {
            unsafe {
                let next_slot = (*slot).next();
                (*slot).clear();
                slot = next_slot;
            }
        }

        unsafe {
            /*memx::memset(
                std::slice::from_raw_parts_mut(self as *mut Self as *mut u8, HEADER_SIZES[idx]),
                0,
            );*/

            core::ptr::write_bytes(self as *const Self as *mut u8, 0, HEADER_SIZES[idx]);

            if cfg!(debug_assertions) {
                let size = NUM_OF_PAGES[idx] * PAGE_SIZE;
                let word_ptr = self as *mut Self as *mut usize;
                for i in 0..(size / size_of::<usize>()) {
                    assert_eq!(word_ptr.add(i).read(), 0);
                }
            }
        }
    }

    #[inline]
    pub unsafe fn zero_data(&mut self) {
        let idx = self.size_bracket_idx as usize;
        let slot_begin = self.first_slot();

        /*memx::memset(
            std::slice::from_raw_parts_mut(
                slot_begin.cast(),
                *NUM_OF_SLOTS.get_unchecked(idx) * *BRACKET_SIZES.get_unchecked(idx),
            ),
            0,
        );*/
        core::ptr::write_bytes(slot_begin.cast::<u8>(), 0, *NUM_OF_SLOTS.get_unchecked(idx) * *BRACKET_SIZES.get_unchecked(idx));
    }

    pub const fn fixed_header_size() -> usize {
        size_of::<Self>()
    }
    #[inline]
    pub unsafe fn first_slot(&self) -> *mut Slot {
        let idx = self.size_bracket_idx as usize;
        (self as *const Self as usize + *HEADER_SIZES.get_unchecked(idx)) as _
    }
    #[inline]
    pub unsafe fn last_slot(&self) -> *mut Slot {
        let idx = self.size_bracket_idx as usize;
        let bracket_size = *BRACKET_SIZES.get_unchecked(idx);
        let end = self.end();
        let last_slot = end.sub(bracket_size).cast::<Slot>();
        debug_assert!(self.first_slot() <= last_slot);
        last_slot
    }

    pub unsafe fn init_free_list(&mut self) {
        let idx = self.size_bracket_idx as usize;
        let bracket_size = BRACKET_SIZES[idx];
        let first_slot = self.first_slot();
        let mut slot = self.last_slot();
        while slot >= first_slot {
            self.free_list.add(slot);
            slot = (*slot).left(bracket_size);
        }
    }
    #[inline]
    pub fn number_of_free_slots(&self) -> usize {
        self.free_list.size()
    }
    #[inline]
    pub fn is_bulk_free_list_empty(&self) -> bool {
        self.bulk_free_list.size() == 0
    }
    #[inline]
    pub fn is_thread_local_free_list_empty(&self) -> bool {
        self.thread_local_free_list.size() == 0
    }

    #[inline]
    pub unsafe fn slot_from_ptr(&self, ptr: *const u8) -> *mut Slot {
        let idx = self.size_bracket_idx as usize;
        let bracket_size = *BRACKET_SIZES.get_unchecked(idx);
        let offset_from_slot_base = ptr as usize - self.first_slot() as usize;
        let slot_idx = offset_from_slot_base / bracket_size;

        self.first_slot().add(slot_idx * bracket_size)
    }

    #[inline]
    pub unsafe fn to_slot(&self, ptr: *const u8) -> *mut Slot {
        let idx = self.size_bracket_idx as usize;
        let bracket_size = *BRACKET_SIZES.get_unchecked(idx);
        let offset_from_slot_base = ptr as usize - self.first_slot() as usize;
        debug_assert_eq!(offset_from_slot_base % bracket_size, 0);
        let slot_idx = offset_from_slot_base / bracket_size;
        debug_assert!(slot_idx < NUM_OF_SLOTS[idx]);
        ptr as _
    }
    #[inline]
    pub unsafe fn slot_index(&self, ptr: *const u8) -> usize {
        let idx = self.size_bracket_idx as usize;
        let bracket_size = *BRACKET_SIZES.get_unchecked(idx);
        let offset_from_slot_base = ptr as usize - self.first_slot() as usize;
        debug_assert_eq!(offset_from_slot_base % bracket_size, 0);
        let slot_idx = offset_from_slot_base / bracket_size;
        debug_assert!(slot_idx < NUM_OF_SLOTS[idx]);
        slot_idx
    }
    #[inline]
    pub fn end(&self) -> *mut u8 {
        (self as *const Self as usize + PAGE_SIZE * NUM_OF_PAGES[self.size_bracket_idx as usize])
            as _
    }
}

pub struct Rosalloc {
    base: *mut u8,
    footprint: usize,
    capacity: usize,
    max_capacity: usize,

    full_runs: [*mut HashSet<*mut Run, BuildNoopHasher>; NUM_OF_SIZE_BRACKETS],
    current_runs: [*mut Run; NUM_OF_SIZE_BRACKETS],
    size_bracket_locks: [*mut Lock; NUM_OF_SIZE_BRACKETS],
    non_full_runs: [*mut IndexSet<*mut Run, BuildNoopHasher>; NUM_OF_SIZE_BRACKETS],
    free_page_runs: Box<IndexSet<*mut FreePageRun, BuildNoopHasher>>,
    page_map: *mut u8,
    page_map_size: usize,
    max_page_map_size: usize,
    page_map_mem_map: Mmap,
    pub lock: Lock,
    bulk_free_lock: RwLock,

    page_release_mode: PageReleaseMode,
    page_release_size_threshold: usize,

    free_page_run_size_map: Vec<usize>,

    morecore: Option<extern "C" fn(rosalloc: *mut Rosalloc, increment: isize, data: *mut u8)>,
    morecore_data: *mut u8,
}
impl Drop for Rosalloc {
    fn drop(&mut self) {
        unsafe {
            for i in 0..NUM_OF_SIZE_BRACKETS {
                let _ = Box::from_raw(self.full_runs[i]);
                let _ = Box::from_raw(self.non_full_runs[i]);
                let _ = Box::from_raw(self.size_bracket_locks[i]);
            }
        }
    }
}
impl Rosalloc {
    pub fn set_footprint_limit(&mut self, new_capacity: usize) {
        self.lock.lock();

        if self.capacity < new_capacity {
            debug_assert!(new_capacity <= self.max_capacity);
            self.capacity = new_capacity;
        }
        unsafe {
            self.lock.unlock();
        }
    }

    pub fn footprint(&self) -> usize {
        self.lock.lock();
        let f = self.footprint;
        unsafe {
            self.lock.unlock();
        }
        f
    }

    pub fn footprint_limit(&self) -> usize {
        self.lock.lock();
        let f = self.capacity;
        unsafe {
            self.lock.unlock();
        }
        f
    }

    pub fn block_start(&self, ptr: *const u8) -> *mut u8 {
        unsafe {
            let mut pm_idx = self.round_down_to_page_map_index(ptr);

            self.lock.lock();

            match self.page_map.add(pm_idx).cast::<PageMapKind>().read() {
                PageMapKind::Empty | PageMapKind::Released => {
                    self.lock.unlock();
                    return null_mut();
                }

                PageMapKind::LargeObject => {
                    self.lock.unlock();
                    return self.base.add(pm_idx * PAGE_SIZE);
                }

                PageMapKind::LargeObjectPart => {
                    while self.page_map.add(pm_idx).cast::<PageMapKind>().read()
                        != PageMapKind::LargeObject
                    {
                        pm_idx -= 1;
                    }
                    self.lock.unlock();
                    return self.base.add(pm_idx * PAGE_SIZE);
                }

                PageMapKind::Run => {
                    let run = self.base.add(pm_idx * PAGE_SIZE).cast::<Run>();
                    let slot = (*run).slot_from_ptr(ptr);
                    self.lock.unlock();
                    return slot as _;
                }

                PageMapKind::RunPart => {
                    while self.page_map.add(pm_idx).cast::<PageMapKind>().read()
                        != PageMapKind::Run
                    {
                        pm_idx -= 1;
                    }
                    let run = self.base.add(pm_idx * PAGE_SIZE).cast::<Run>();
                    let slot = (*run).slot_from_ptr(ptr);
                    self.lock.unlock();
                    return slot as _;
                }
            }
        }
    }

    pub fn usable_size(&self, ptr: *mut u8) -> usize {
        unsafe {
            let mut pm_idx = self.round_down_to_page_map_index(ptr);
            self.lock.lock();

            match self.page_map.add(pm_idx).cast::<PageMapKind>().read() {
                PageMapKind::Released | PageMapKind::Empty | PageMapKind::LargeObjectPart => {
                    unreachable!("Unreachable: pm_idx={}, ptr={:p}", pm_idx, ptr);
                }
                PageMapKind::LargeObject => {
                    let mut num_pages = 1;
                    let mut idx = pm_idx + 1;
                    let end = self.page_map_size;
                    while idx < end
                        && self.page_map.add(idx).cast::<PageMapKind>().read()
                            == PageMapKind::LargeObjectPart
                    {
                        num_pages += 1;
                        idx += 1;
                    }
                    return num_pages * PAGE_SIZE;
                }
                PageMapKind::Run | PageMapKind::RunPart => {
                    while self.page_map.add(pm_idx).cast::<PageMapKind>().read() != PageMapKind::Run
                    {
                        pm_idx -= 1;
                    }

                    let run = self.base.add(pm_idx * PAGE_SIZE).cast::<Run>();

                    let idx = (*run).size_bracket_idx;
                    Self::index_to_bracket_size(idx as _)
                }
            }
        }
    }

    pub unsafe fn trim(&mut self) -> bool {
        self.lock.lock();

        let last_free_page_run;
        let it = self.free_page_runs.last();
        if let Some(it) = it.copied() {
            last_free_page_run = it;
            if (*last_free_page_run).end(self) == self.base.add(self.footprint) {
                self.free_page_runs.remove(&last_free_page_run);
                let decrement = (*last_free_page_run).byte_size(self);
                let new_footprint = self.footprint - decrement;
                let new_num_of_pages = new_footprint / PAGE_SIZE;
                self.page_map_size = new_num_of_pages;
                self.free_page_run_size_map.resize(new_num_of_pages, 0);

                if let Some(morecore) = self.morecore {
                    morecore(self, -(decrement as isize), self.morecore_data);
                }
                self.footprint = new_footprint;
                self.lock.unlock();
                return true;
            }
        }
        self.lock.unlock();
        false
    }

    pub fn set_morecore(
        &mut self,
        morecore: extern "C" fn(*mut Self, isize, *mut u8),
        data: *mut u8,
    ) {
        self.morecore = Some(morecore);
        self.morecore_data = data;
    }
    pub fn new(
        base: *mut u8,
        capacity: usize,
        max_capacity: usize,
        page_release_mode: PageReleaseMode,
        page_release_size_threshold: usize,
    ) -> *mut Self {
        let mut this = Self {
            base,
            capacity,
            max_capacity,
            footprint: capacity,
            bulk_free_lock: RwLock::INIT,
            lock: Lock::INIT,
            size_bracket_locks: [null_mut(); NUM_OF_SIZE_BRACKETS],
            page_release_mode,
            page_release_size_threshold,
            free_page_run_size_map: Vec::new(),
            free_page_runs: Box::new(IndexSet::with_hasher(BuildNoopHasher::default())),
            full_runs: [null_mut(); NUM_OF_SIZE_BRACKETS],
            current_runs: [null_mut(); NUM_OF_SIZE_BRACKETS],
            non_full_runs: [null_mut(); NUM_OF_SIZE_BRACKETS],
            page_map: null_mut(),
            page_map_mem_map: Mmap::uninit(),
            page_map_size: 0,
            max_page_map_size: 0,
            morecore: None,
            morecore_data: null_mut(),
        };

        

        if INITIALIZED
            .compare_exchange(
                false,
                true,
                std::sync::atomic::Ordering::SeqCst,
                std::sync::atomic::Ordering::Relaxed,
            )
            .is_ok()
        {
            unsafe {
                initialize();
            }
        }

        unsafe {
            for i in 0..NUM_OF_SIZE_BRACKETS {
                this.size_bracket_locks[i] = Box::into_raw(Box::new(Lock::INIT));
                this.current_runs[i] = DEDICATED_FULL_RUN;
            }

            let num_of_pages = this.footprint / PAGE_SIZE;
            let max_num_of_pages = max_capacity / PAGE_SIZE;

            this.page_map_mem_map = Mmap::new(round_up(max_num_of_pages as _, PAGE_SIZE as _) as _);
            this.page_map = this.page_map_mem_map.start();
            this.page_map_size = num_of_pages;
            this.max_page_map_size = max_num_of_pages;

            this.free_page_run_size_map.resize(num_of_pages, 0);
            let mut free_pages = this.base.cast::<FreePageRun>();
            if cfg!(debug_assertions) {
                (*free_pages).magic_num = MAGIC_NUM_FREE;
            }

            (*free_pages).set_byte_size(&mut this, capacity);

            (*free_pages).release_pages(&mut this);
            this.free_page_runs.insert(free_pages);
            for i in 0..NUM_OF_SIZE_BRACKETS {
                this.non_full_runs[i] =
                    Box::into_raw(Box::new(IndexSet::with_hasher(Default::default())));
                this.full_runs[i] =
                    Box::into_raw(Box::new(HashSet::with_hasher(Default::default())));
            }
        }

        Box::into_raw(Box::new(this))
    }

    pub unsafe fn bulk_free(&mut self, pointers: &[*mut u8]) -> usize {
        let mut freed_bytes = 0;
        if false {
            for ptr in pointers.iter().copied() {
                freed_bytes += self.free_internal(ptr);
            }
            return freed_bytes;
        }

        self.bulk_free_lock.lock_exclusive();
        let mut runs = IndexSet::with_hasher(BuildNoopHasher::default());
        for ptr in pointers.iter().copied() {
            let pm_idx = self.round_down_to_page_map_index(ptr);
            let run;
            let page_map_entry = self.page_map.add(pm_idx).cast::<PageMapKind>().read();

            if page_map_entry == PageMapKind::Run {
                run = self.base.add(pm_idx * PAGE_SIZE).cast::<Run>();
            } else if page_map_entry == PageMapKind::RunPart {
                let mut pi = pm_idx;
                // find the beginning of the run
                while {
                    pi -= 1;
                    self.page_map.add(pi).cast::<PageMapKind>().read() != PageMapKind::Run
                } {}
                run = self.base.add(pi * PAGE_SIZE).cast::<Run>();
            } else if page_map_entry == PageMapKind::LargeObject {
                self.lock.lock();
                freed_bytes += self.free_pages(ptr, false);
                self.lock.unlock();
                continue;
            } else {
                unreachable!("Unreachable: page map type: {:?}", page_map_entry);
            }

            freed_bytes += (*run).add_to_bulk_free_list(ptr);
            runs.insert(run);
        }

        for run in runs {
            let idx = (*run).size_bracket_idx as usize;
            (**self.size_bracket_locks.get_unchecked(idx)).lock();
            if (*run).is_thread_local != 0 {
                (*run).merge_bulk_free_list_to_thread_local_free_list();
            } else {
                let run_was_full = (*run).is_full();
                (*run).merge_bulk_free_list_to_free_list();

                let non_full_runs = &mut *self.non_full_runs[idx];
                let full_runs = if cfg!(debug_assertions) {
                    self.full_runs[idx]
                } else {
                    null_mut()
                };
                if (*run).is_all_free() {
                    let run_was_current = run == *self.current_runs.get_unchecked(idx);
                    if run_was_full {
                        if cfg!(debug_assertions) {
                            debug_assert!((*full_runs).remove(&run));
                        }
                    } else {
                        (*non_full_runs).remove(&run);
                    }

                    if !run_was_current {
                        (*run).zero_header_and_slot_headers();
                        self.lock.lock();
                        self.free_pages(run.cast(), true);
                        self.lock.unlock();
                    }
                } else {
                    if run == *self.current_runs.get_unchecked(idx) {
                        debug_assert!(!non_full_runs.contains(&run));
                    } else if run_was_full {
                        if cfg!(debug_assertions) {
                            debug_assert!((*full_runs).remove(&run));
                        }

                        non_full_runs.insert(run);
                    } else {
                        debug_assert!(non_full_runs.contains(&run));
                    }
                }
            }
            (**self.size_bracket_locks.get_unchecked(idx)).unlock();
        }

        self.bulk_free_lock.unlock_exclusive();
        freed_bytes
    }

    pub unsafe fn alloc_pages(&mut self, num_pages: usize, page_map_type: u8) -> *mut u8 {
        debug_assert!(self.lock.is_locked());

        let mut res = null_mut();
        let req_byte_size = num_pages * PAGE_SIZE;

        for i in 0..self.free_page_runs.len() {
            let fpr = *self.free_page_runs.get_index(i).unwrap_unchecked();

            let fpr_byte_size = (*fpr).byte_size(self);
            if req_byte_size <= fpr_byte_size {
                self.free_page_runs.remove(&fpr);

                #[cfg(feature="trace")]
                {
                    log::info!(
                        "RosAlloc::alloc_pages() : Erased run {:p} from free_page_runs",
                        fpr  
                    );
                }

                if req_byte_size < fpr_byte_size {
                    let remainder = fpr.cast::<u8>().add(req_byte_size).cast::<FreePageRun>();
                    if cfg!(debug_assertions) {
                        (*remainder).magic_num = MAGIC_NUM_FREE;
                    }
                    (*remainder).set_byte_size(self, fpr_byte_size - req_byte_size);
                    self.free_page_runs.insert(remainder);
                    #[cfg(feature="trace")]
                    {
                        log::info!(
                            "RosAlloc::alloc_pages() : Inserted run {:p} into free_page_runs",
                            remainder
                        );
                    }
                    (*fpr).set_byte_size(self, req_byte_size);
                }
                res = fpr;
                break;
            }
        }

        if res.is_null() && self.capacity > self.footprint {
            let mut last_free_page_run = null_mut();
            let mut last_free_page_run_size = 0;
            let mut it = self.free_page_runs.last();
            if it.is_some() {
                last_free_page_run = *it.unwrap_unchecked();
                if (*last_free_page_run).end(self) == self.base.add(self.footprint) {
                    last_free_page_run_size = (*last_free_page_run).byte_size(self);
                }
            } else {
                last_free_page_run_size = 0;
            }

            if self.capacity - self.footprint + last_free_page_run_size >= req_byte_size {
                let increment = (2 * MB)
                    .max(req_byte_size - last_free_page_run_size)
                    .min(self.capacity - self.footprint);

                let new_footprint = self.footprint + increment;
                let new_num_of_pages = new_footprint / PAGE_SIZE;

                self.page_map_size = new_num_of_pages;
                self.free_page_run_size_map.resize(new_num_of_pages, 0);
                if let Some(morecore) = self.morecore {
                    morecore(self, increment as _, self.morecore_data);
                } else {
                    return null_mut();
                }
                if last_free_page_run_size > 0 {
                    (*last_free_page_run).set_byte_size(self, last_free_page_run_size + increment);
                } else {
                    let new_free_page_run = self.base.add(self.footprint).cast::<FreePageRun>();
                    if cfg!(debug_assertions) {
                        (*new_free_page_run).magic_num = MAGIC_NUM_FREE;
                    }
                    (*new_free_page_run).set_byte_size(self, increment);
                    self.free_page_runs.insert(new_free_page_run);
                }

                self.footprint = new_footprint;

                it = self.free_page_runs.last();
                let fpr = *it.unwrap_unchecked();

                if cfg!(debug_assertions) && last_free_page_run_size > 0 {
                    debug_assert!(!last_free_page_run.is_null());
                    debug_assert_eq!(last_free_page_run, fpr);
                }

                let fpr_byte_size = (*fpr).byte_size(self);

                self.free_page_runs.remove(&fpr);

                if req_byte_size < fpr_byte_size {
                    let remainder = fpr.cast::<u8>().add(req_byte_size).cast::<FreePageRun>();
                    if cfg!(debug_assertions) {
                        (*remainder).magic_num = MAGIC_NUM_FREE;
                    }
                    (*remainder).set_byte_size(self, fpr_byte_size - req_byte_size);
                    self.free_page_runs.insert(remainder);

                    (*fpr).set_byte_size(self, req_byte_size);
                }
                res = fpr;
            }
        }

        if !res.is_null() {
            let page_map_idx = self.to_page_map_index(res.cast());
            for i in 0..num_pages {
                debug_assert!(self.is_free_page(page_map_idx + i));
            }

            match page_map_type {
                x if x as u8 == PageMapKind::Run as u8 => {
                    self.page_map.add(page_map_idx).write(PageMapKind::Run as _);
                    for i in 1..num_pages {
                        self.page_map
                            .add(page_map_idx + i)
                            .write(PageMapKind::RunPart as _);
                    }
                }
                x if x as u8 == PageMapKind::LargeObject as u8 => {
                    self.page_map
                        .add(page_map_idx)
                        .write(PageMapKind::LargeObject as _);
                    for i in 1..num_pages {
                        self.page_map
                            .add(page_map_idx + i)
                            .write(PageMapKind::LargeObjectPart as _);
                    }
                }
                _ => {
                    unreachable!()
                }
            }

            if cfg!(debug_assertions) {
                core::ptr::write_bytes(res.cast::<u8>(), 0, PAGE_SIZE);
                //memx::memset(std::slice::from_raw_parts_mut(res.cast(), PAGE_SIZE), 0);
            }

            return res.cast();
        }
        null_mut()
    }
    pub unsafe fn free_pages(&mut self, ptr: *mut u8, already_zero: bool) -> usize {
        let pm_idx = self.to_page_map_index(ptr);
        let pm_type = self.page_map.add(pm_idx).read();
        let pm_part_type;
        match pm_type {
            x if x == PageMapKind::Run as u8 => pm_part_type = PageMapKind::RunPart as u8,
            x if x == PageMapKind::LargeObject as u8 => {
                pm_part_type = PageMapKind::LargeObjectPart as u8
            }
            _ => unreachable!(),
        }

        let mut num_pages = 1;
        self.page_map.add(pm_idx).write(PageMapKind::Empty as _);
        let mut idx = pm_idx + 1;
        let end = self.page_map_size;
        while idx < end && self.page_map.add(idx).read() == pm_part_type {
            self.page_map.add(idx).write(PageMapKind::Empty as _);
            num_pages += 1;
            idx += 1;
        }

        let byte_size = num_pages * PAGE_SIZE;
        if !already_zero && self.page_release_mode != PageReleaseMode::All {
            core::ptr::write_bytes(ptr, 0, byte_size);
            //memx::memset(std::slice::from_raw_parts_mut(ptr, byte_size), 0);
        }

        let mut fpr = ptr.cast::<FreePageRun>();

        if cfg!(debug_assertions) {
            (*fpr).magic_num = MAGIC_NUM_FREE;
        }

        (*fpr).set_byte_size(self, byte_size);

        if !self.free_page_runs.is_empty() {
            let mut i = 0;
            while i < self.free_page_runs.len() {
                let item = &self.free_page_runs[i];
                let it = *item;
                if it < fpr {
                    i += 1;
                    continue;
                }

                let h = it;
                if (*fpr).end(self) == (*h).begin() {
                    if cfg!(debug_assertions) {
                        (*h).magic_num = 0;
                    }
                    self.free_page_runs.remove(&it);
                    (*fpr).set_byte_size(self, (*fpr).byte_size(self) + (*h).byte_size(self));
                } else {
                    break;
                }
                i += 1;
            }
            let mut i = self.free_page_runs.len() as isize - 1;
            while i > 0 {
                let item = &self.free_page_runs[i as usize];
                let it = *item;
                if it > fpr {
                    i -= 1;
                    continue;
                }
                let l = it;

                if (*l).end(self) == (*fpr).begin() {
                    self.free_page_runs.remove(&it);

                    (*l).set_byte_size(self, (*l).byte_size(self) + (*fpr).byte_size(self));
                    if cfg!(debug_assertions) {
                        (*fpr).magic_num = 0;
                    }
                    fpr = l;
                } else {
                    break;
                }
                i -= 1;
            }
        }
        (*fpr).release_pages(self);
        self.free_page_runs.insert(fpr);
        byte_size
    }

    pub unsafe fn alloc_large_object(
        &mut self,
        size: usize,
        bytes_allocated: &mut usize,
        usable_size: &mut usize,
        bytes_tl_bulk_allocated: &mut usize,
    ) -> *mut u8 {
        let num_pages = round_up(size as _, PAGE_SIZE as _) as usize / PAGE_SIZE;
        let r;
        {
            self.lock.lock();
            r = self.alloc_pages(num_pages, PageMapKind::LargeObject as _);
            self.lock.unlock();
        }

        let total_bytes = num_pages * PAGE_SIZE;
        *bytes_allocated = total_bytes;
        *usable_size = total_bytes;
        *bytes_tl_bulk_allocated = total_bytes;
        r
    }

    unsafe fn free_internal(&mut self, ptr: *mut u8) -> usize {
        let mut pm_idx = self.round_down_to_page_map_index(ptr);
        let run;
        {
            self.lock.lock();

            match self.page_map.add(pm_idx).cast::<PageMapKind>().read() {
                PageMapKind::LargeObject => {
                    
                    let bytes = self.free_pages(ptr, false);
                    self.lock.unlock();
                    return bytes;
                }
                PageMapKind::LargeObjectPart => {
                    unreachable!("Unreachable: trying to free large object part at {:p}", ptr);
                }
                PageMapKind::RunPart => {
                    // find the beginning of the run
                    while {
                        pm_idx -= 1;
                        self.page_map.add(pm_idx).cast::<PageMapKind>().read() != PageMapKind::Run
                    } {}
                    run = self.base.add(pm_idx * PAGE_SIZE);
                }
                PageMapKind::Run => {
                    run = self.base.add(pm_idx * PAGE_SIZE);
                }
                _ => unreachable!("Trying to free unallocated memory at {:p}", ptr),
            }
            self.lock.unlock();
        }
        self.free_from_run(ptr, run.cast())
    }

    pub unsafe fn alloc_run(&mut self, idx: usize) -> *mut Run {
        let mut new_run;
        {
            self.lock.lock();
            new_run = self
                .alloc_pages(NUM_OF_PAGES[idx], PageMapKind::Run as _)
                .cast::<Run>();
            self.lock.unlock();
        }

        if !new_run.is_null() {
            if cfg!(debug_assertions) {
                (*new_run).magic_num = MAGIC_NUM;
            }
            (*new_run).size_bracket_idx = idx as _;
            (*new_run).init_free_list();
        }

        new_run
    }

    pub unsafe fn refill_run(&mut self, idx: usize) -> *mut Run {
        let bt = self.non_full_runs[idx];
        if !(*bt).is_empty() {
            let it = (*bt).first().unwrap_unchecked();
            let non_full_run = *it;
            (*bt).remove(&non_full_run);
            return non_full_run;
        }
        self.alloc_run(idx)
    }

    pub unsafe fn alloc_from_current_run_unlocked(&mut self, idx: usize) -> *mut u8 {
        let mut current_run = self.current_runs[idx];
        let mut slot_addr = (*current_run).alloc_slot();
        if !slot_addr.is_null() {
            return slot_addr.cast();
        }

        if cfg!(debug_assertions) && current_run != DEDICATED_FULL_RUN {
            (*self.full_runs[idx]).insert(current_run);
        }

        current_run = self.refill_run(idx);
        if current_run.is_null() {
            self.current_runs[idx] = DEDICATED_FULL_RUN;
            return null_mut();
        }
        (*current_run).is_thread_local = 0;
        self.current_runs[idx] = current_run;
        slot_addr = (*current_run).alloc_slot();
        slot_addr.cast()
    }
    pub unsafe fn alloc_from_run_thread_unsafe(
        &mut self,
        size: usize,
        bytes_allocated: &mut usize,
        usable_size: &mut usize,
        bytes_tl_bulk_allocated: &mut usize,
    ) -> *mut u8 {
        let (idx, bracket_size) = Self::size_to_index_and_bracket_size(size);
        let slot_addr = self.alloc_from_current_run_unlocked(idx);
        if !slot_addr.is_null() {
            *bytes_allocated = bracket_size;
            *usable_size = bracket_size;
            *bytes_tl_bulk_allocated = bracket_size;
        }
        slot_addr
    }

    unsafe fn revoke_run(&mut self, idx: usize, run: *mut Run) {
        if (*run).is_full() {
            if cfg!(debug_assertions) {
                (*self.full_runs[idx]).insert(run);
            }
        } else if (*run).is_all_free() {
            (*run).zero_header_and_slot_headers();
            self.lock.lock();
            self.free_pages(run.cast(), true);
            self.lock.unlock();
        } else {
            (*self.non_full_runs[idx]).insert(run);
        }
    }

    pub fn collect(&mut self, tls_runs: &mut [*mut Run; NUM_THREAD_LOCAL_SIZE_BRACKETS]) {
        unsafe {
            self.revoke_thread_unsafe_current_runs();
            self.revoke_thread_local_runs(tls_runs);
        } 
    }

    /// Revoke the current runs which share the same idx as thread local runs.
    pub unsafe fn revoke_thread_unsafe_current_runs(&mut self) {
        for idx in 0..NUM_THREAD_LOCAL_SIZE_BRACKETS {
            (*self.size_bracket_locks[idx]).lock();
            if self.current_runs[idx] != DEDICATED_FULL_RUN {
                self.revoke_run(idx, self.current_runs[idx]);
                self.current_runs[idx] = DEDICATED_FULL_RUN;
            }
            (*self.size_bracket_locks[idx]).unlock();
        }
    }

    pub unsafe fn revoke_thread_local_runs(
        &mut self,
        tls_runs: &mut [*mut Run; NUM_THREAD_LOCAL_SIZE_BRACKETS],
    ) -> usize {
        let mut free_bytes = 0;
        for idx in 0..NUM_THREAD_LOCAL_SIZE_BRACKETS {
            (*self.size_bracket_locks[idx]).lock();
            let thread_local_run = tls_runs[idx];
            if thread_local_run != DEDICATED_FULL_RUN {
                tls_runs[idx] = DEDICATED_FULL_RUN;
                let num_free_slots = (*thread_local_run).number_of_free_slots();
                free_bytes += num_free_slots * BRACKET_SIZES[idx];

                (*thread_local_run).merge_thread_local_free_list_to_free_list(&mut false);
                (*thread_local_run).is_thread_local = 0;
                self.revoke_run(idx, thread_local_run);
            }
            (*self.size_bracket_locks[idx]).unlock();
        }
        free_bytes
    }

    pub unsafe fn revoke_all_thread_local_runs<'a>(&mut self, tls_runs: impl Iterator<Item = &'a mut [*mut Run; NUM_THREAD_LOCAL_SIZE_BRACKETS]>) -> usize {
        let mut free_bytes = 0;
        for tls_runs in tls_runs {
            free_bytes += self.revoke_thread_local_runs(tls_runs);
        }

        self.revoke_thread_unsafe_current_runs();

        free_bytes
    }

    #[inline]
    unsafe fn alloc_from_run(
        &mut self,
        tls_run: &mut [*mut Run; NUM_THREAD_LOCAL_SIZE_BRACKETS],
        size: usize,
        bytes_allocated: &mut usize,
        usable_size: &mut usize,
        bytes_tl_bulk_allocated: &mut usize,
    ) -> *mut u8 {
        let (idx, bracket_size) = Self::size_to_index_and_bracket_size(size);

        let mut slot_addr;
        if idx < NUM_THREAD_LOCAL_SIZE_BRACKETS {
            let mut thread_local_run = tls_run[idx];
            slot_addr = (*thread_local_run).alloc_slot().cast::<u8>();
            if slot_addr.is_null() {
                (**self.size_bracket_locks.get_unchecked(idx)).lock();
                let mut is_all_free_after_merge = false;

                if (*thread_local_run)
                    .merge_thread_local_free_list_to_free_list(&mut is_all_free_after_merge)
                {
                    debug_assert_ne!(thread_local_run, dedicated_full_run());
                    debug_assert!(!(*thread_local_run).is_full());
                    debug_assert_eq!(is_all_free_after_merge, (*thread_local_run).is_all_free());
                } else {
                    if thread_local_run != DEDICATED_FULL_RUN {
                        (*thread_local_run).is_thread_local = 0;
                        if cfg!(debug_assertions) {
                            (*self.full_runs[idx]).insert(thread_local_run);
                        }
                    }
                    thread_local_run = self.refill_run(idx);

                    if thread_local_run.is_null() {
                        tls_run[idx] = DEDICATED_FULL_RUN;
                        (**self.size_bracket_locks.get_unchecked(idx)).unlock();
                        return null_mut();
                    }

                    (*thread_local_run).is_thread_local = 1;
                    tls_run[idx] = thread_local_run;
                }

                *bytes_tl_bulk_allocated =
                    (*thread_local_run).number_of_free_slots() * bracket_size;
                slot_addr = (*thread_local_run).alloc_slot().cast();

                debug_assert!(!slot_addr.is_null());
                (**self.size_bracket_locks.get_unchecked(idx)).unlock();
            } else {
                *bytes_tl_bulk_allocated = 0;
            }
            *bytes_allocated = bracket_size;
            *usable_size = bracket_size;
        } else {
            (**self.size_bracket_locks.get_unchecked(idx)).lock();
            slot_addr = self.alloc_from_current_run_unlocked(idx);
            (**self.size_bracket_locks.get_unchecked(idx)).unlock();
            if !slot_addr.is_null() {
                *bytes_allocated = bracket_size;
                *usable_size = bracket_size;
                *bytes_tl_bulk_allocated = bracket_size;
            }
        }
        slot_addr
    }
    /// Allocates `size` bytes in rosalloc space using global runs. This operation is expensive in case multiple
    /// threads are allocating into the same run. Returns null if no memory to satisfy allocation `size` of bytes is left.
    #[inline]
    pub unsafe fn alloc_global(
        &mut self,
        size: usize,
        bytes_allocated: &mut usize,
        usable_size: &mut usize,
        bytes_tl_bulk_allocated: &mut usize,
    ) -> *mut u8 {
        if size > Self::LARGE_SIZE_THRESHOLD {
            return self.alloc_large_object(
                size,
                bytes_allocated,
                usable_size,
                bytes_tl_bulk_allocated,
            );
        }
        let (idx, bracket_size) = Self::size_to_index_and_bracket_size(size);
        (**self.size_bracket_locks.get_unchecked(idx)).lock();
        let slot_addr = self.alloc_from_current_run_unlocked(idx);
        (**self.size_bracket_locks.get_unchecked(idx)).unlock();
        if !slot_addr.is_null() {
            *bytes_allocated = bracket_size;
            *usable_size = bracket_size;
            *bytes_tl_bulk_allocated = bracket_size;
        }
        slot_addr
    }

    /// Attempts to allocate a block of memory. If there is no enough memory `null` is returned and `bytes_allocated `with `usable_size` are set to zero.
    ///
    /// # Safety
    /// - `tls_run` slice must be filled with `dedicated_full_run()` return value or with already populated runs (they are populated by calls to this functions automatically).  
    /// - if `THREAD_SAFE` is false user must be sure that no other threads invoke [Rosalloc::alloc](Rosalloc::alloc) and only one single thread does allocation.
    #[inline(always)]
    pub unsafe fn alloc<const THREAD_SAFE: bool>(
        &mut self,
        tls_run: &mut [*mut Run; NUM_THREAD_LOCAL_SIZE_BRACKETS],
        size: usize,
        bytes_allocated: &mut usize,
        usable_size: &mut usize,
        bytes_tl_bulk_allocated: &mut usize,
    ) -> *mut u8 {
        if size > Self::LARGE_SIZE_THRESHOLD {
            return self.alloc_large_object(
                size,
                bytes_allocated,
                usable_size,
                bytes_tl_bulk_allocated,
            );
        }

        let m;
        if THREAD_SAFE {
            m = self.alloc_from_run(
                tls_run,
                size,
                bytes_allocated,
                usable_size,
                bytes_tl_bulk_allocated,
            )
        } else {
            m = self.alloc_from_run_thread_unsafe(
                size,
                bytes_allocated,
                usable_size,
                bytes_tl_bulk_allocated,
            )
        }
        m
    }

    unsafe fn free_from_run(&mut self, ptr: *mut u8, run: *mut Run) -> usize {
        let idx = (*run).size_bracket_idx as usize;
        let bracket_size = *BRACKET_SIZES.get_unchecked(idx as usize);

        (*self.size_bracket_locks[idx]).lock();

        if (*run).is_thread_local != 0 {
            (*run).add_to_thread_local_free_list(ptr);
            (*self.size_bracket_locks[idx]).unlock();
            return bracket_size;
        }

        (*run).free_slot(ptr);
        let non_full_runs = self.non_full_runs[idx as usize];
        if (*run).is_all_free() {
            (*non_full_runs).remove(&run);

            if run == self.current_runs[idx] {
                self.current_runs[idx] = DEDICATED_FULL_RUN;
            }

            (*run).zero_header_and_slot_headers();
            {
                self.lock.lock();
                self.free_pages(run.cast(), true);
                self.lock.unlock();
            }
        } else {
            // It is not completely free. If it wasn't the current run or
            // already in the non-full run set (i.e., it was full) insert it
            // into the non-full run set.
            if run != self.current_runs[idx] {
                let full_runs = if cfg!(debug_assertions) {
                    self.full_runs[idx]
                } else {
                    null_mut()
                };
                if !(*non_full_runs).contains(&run) {
                    if cfg!(debug_assertions) {
                        (*full_runs).remove(&run);
                    }
                    (*non_full_runs).insert(run);
                    debug_assert!(!(*run).is_full());
                }
            }
        }
        (*self.size_bracket_locks[idx]).unlock();
        bracket_size
    }
    /// Deallocates the memory referenced by `ptr`.
    ///
    ///
    /// # Safety
    /// - `ptr` must be a block of memory allocated by this instance of [Rosalloc].
    ///
    /// # Panics
    /// Panics if `ptr` points to unallocated pages managed by [Rosalloc].
    pub unsafe fn free(&mut self, ptr: *mut u8) -> usize {
        self.bulk_free_lock.lock_shared();
        let bytes = self.free_internal(ptr);
        self.bulk_free_lock.unlock_shared();
        bytes
    }

    #[inline]
    fn is_free_page(&self, idx: usize) -> bool {
        unsafe {
            let pm_type = self.page_map.add(idx).read();
            pm_type == PageMapKind::Released as u8 || pm_type == PageMapKind::Empty as u8
        }
    }
    pub fn begin(&self) -> *mut u8 {
        self.base
    }

    pub fn end(&self) -> *mut u8 {
        unsafe { self.base.add(self.capacity) }
    }

    #[inline]
    pub const fn index_to_bracket_size(idx: usize) -> usize {
        BRACKET_SIZES[idx]
    }
    #[inline]
    pub const fn bracket_size_to_index(size: usize) -> usize {
        let idx;
        if size == 1 * KB {
            idx = NUM_OF_SIZE_BRACKETS - 2;
        } else if size == 2 * KB {
            idx = NUM_OF_SIZE_BRACKETS - 1;
        } else if size <= MAX_THREAD_LOCAL_BRACKET_SIZE {
            idx = size / THREAD_LOCAL_BRACKET_QUANTUM_SIZE - 1;
        } else {
            idx = ((size - MAX_THREAD_LOCAL_BRACKET_SIZE) / BRACKET_QUANTUM_SIZE - 1)
                + NUM_THREAD_LOCAL_SIZE_BRACKETS;
        }

        idx
    }
    #[inline]
    pub const fn is_size_for_thread_local(size: usize) -> bool {
        size <= MAX_THREAD_LOCAL_BRACKET_SIZE
    }
    #[inline]
    pub const fn round_to_bracket_size(size: usize) -> usize {
        if size <= MAX_THREAD_LOCAL_BRACKET_SIZE {
            round_up(size as _, THREAD_LOCAL_BRACKET_QUANTUM_SIZE as _) as _
        } else if size <= MAX_REGULAR_BRACKET_SIZE {
            round_up(size as _, BRACKET_QUANTUM_SIZE as _) as _
        } else if size <= 1 * KB {
            1 * KB
        } else {
            2 * KB
        }
    }

    #[inline]
    pub const fn size_to_index(size: usize) -> usize {
        if size <= MAX_THREAD_LOCAL_BRACKET_SIZE {
            round_up(size as _, THREAD_LOCAL_BRACKET_QUANTUM_SIZE as _) as usize
                / THREAD_LOCAL_BRACKET_QUANTUM_SIZE
                - 1
        } else if size <= MAX_REGULAR_BRACKET_SIZE {
            (round_up(size as _, BRACKET_QUANTUM_SIZE as _) as usize
                - MAX_THREAD_LOCAL_BRACKET_SIZE)
                / BRACKET_QUANTUM_SIZE
                - 1
                + NUM_THREAD_LOCAL_SIZE_BRACKETS
        } else if size <= 1 * KB {
            NUM_OF_SIZE_BRACKETS - 2
        } else {
            NUM_OF_SIZE_BRACKETS - 1
        }
    }

    #[inline]
    pub const fn size_to_index_and_bracket_size(size: usize) -> (usize, usize) {
        (Self::size_to_index(size), Self::round_to_bracket_size(size))
    }

    #[inline]
    pub fn to_page_map_index(&self, addr: *const u8) -> usize {
        let byte_offset = addr as usize - self.base as usize;
        byte_offset / PAGE_SIZE
    }

    #[inline]
    pub fn round_down_to_page_map_index(&self, addr: *const u8) -> usize {
        (addr as usize - self.base as usize) / PAGE_SIZE
    }

    unsafe fn release_page_range(&mut self, mut start: *mut u8, end: *mut u8) -> usize {
        if cfg!(debug_assertions) {
            // In the debug build, the first page of a free page run
            // contains a magic number for debugging. Exclude it.
            start = start.add(PAGE_SIZE);

            if start == end {
                return 0;
            }
        }

        /*memx::memset(
            std::slice::from_raw_parts_mut(start, end.offset_from(start) as _),
            0,
        );*/
        core::ptr::write_bytes(start, 0, end as usize - start as usize);
        let mut pm_idx = self.to_page_map_index(start);
        let mut reclaimed_bytes = 0;
        let max_idx = pm_idx + (end as usize - start as usize) / PAGE_SIZE;
        while pm_idx < max_idx {
            debug_assert!(self.is_free_page(pm_idx));
            if self.page_map.add(pm_idx).read() == PageMapKind::Empty as u8 {
                // Mark the page as released and update how many bytes we released.
                reclaimed_bytes += PAGE_SIZE;
                self.page_map.add(pm_idx).write(PageMapKind::Released as _);
            }
            pm_idx += 1;
        }
        reclaimed_bytes
    }

    pub const LARGE_SIZE_THRESHOLD: usize = 2048;
}
pub const BRACKET_SIZES: [usize; NUM_OF_SIZE_BRACKETS] = {
    let mut bracket_sizes: [usize; NUM_OF_SIZE_BRACKETS] = [0; NUM_OF_SIZE_BRACKETS];
    let mut i = 0;
    while i < NUM_OF_SIZE_BRACKETS {
        if i < NUM_THREAD_LOCAL_SIZE_BRACKETS {
            bracket_sizes[i] = THREAD_LOCAL_BRACKET_QUANTUM_SIZE * (i + 1);
        } else if i < NUM_REGULAR_SIZE_BRACKETS {
            bracket_sizes[i] = BRACKET_QUANTUM_SIZE * (i - NUM_THREAD_LOCAL_SIZE_BRACKETS + 1)
                + (THREAD_LOCAL_BRACKET_QUANTUM_SIZE * NUM_THREAD_LOCAL_SIZE_BRACKETS);
        } else if i == NUM_OF_SIZE_BRACKETS - 2 {
            bracket_sizes[i] = 1 * KB;
        } else {
            bracket_sizes[i] = 2 * KB;
        }
        i += 1;
    }
    bracket_sizes
};

pub const NUM_OF_PAGES: [usize; NUM_OF_SIZE_BRACKETS] = {
    let mut num_of_pages: [usize; NUM_OF_SIZE_BRACKETS] = [0; NUM_OF_SIZE_BRACKETS];
    let mut i = 0;
    while i < NUM_OF_SIZE_BRACKETS {
        if i < NUM_THREAD_LOCAL_SIZE_BRACKETS {
            num_of_pages[i] = 1;
        } else if i < ((NUM_THREAD_LOCAL_SIZE_BRACKETS + NUM_REGULAR_SIZE_BRACKETS) / 2) {
            num_of_pages[i] = 1;
        } else if i < NUM_REGULAR_SIZE_BRACKETS {
            num_of_pages[i] = 1;
        } else if i == NUM_OF_SIZE_BRACKETS - 2 {
            num_of_pages[i] = 2;
        } else {
            num_of_pages[i] = 4;
        }
        i += 1;
    }
    num_of_pages
};
#[inline(always)]
pub const fn round_down(x: u64, n: u64) -> u64 {
    let x = x as i64;
    let n = n as i64;
    (x & -n) as u64
}

#[inline(always)]
pub const fn round_up(x: u64, n: u64) -> u64 {
    round_down(x.wrapping_add(n).wrapping_sub(1), n)
    //round_down(x + n - 1, n)
}

pub const NUM_OF_SLOTS: [usize; NUM_OF_SIZE_BRACKETS] = {
    let mut num_of_slotsc: [usize; NUM_OF_SIZE_BRACKETS] = [0; NUM_OF_SIZE_BRACKETS];
    let mut header_sizes = [0; NUM_OF_SIZE_BRACKETS];
    let mut i = 0;
    while i < NUM_OF_SIZE_BRACKETS {
        let bracket_size = BRACKET_SIZES[i];
        let run_size = PAGE_SIZE * NUM_OF_PAGES[i];
        let max_num_of_slots = run_size / bracket_size;

        let fixed_header_size =
            round_up(Run::fixed_header_size() as _, size_of::<u64>() as _) as usize;
        let mut header_size = 0;
        let mut num_of_slots = 0;

        let mut s = max_num_of_slots as isize;

        while s >= 0 {
            let tmp_slots_size = bracket_size * s as usize;
            let tmp_unaligned_header_size = fixed_header_size;

            let tmp_header_size = if tmp_unaligned_header_size % bracket_size == 0 {
                tmp_unaligned_header_size
            } else {
                tmp_unaligned_header_size
                    + (bracket_size - tmp_unaligned_header_size % bracket_size)
            };

            if tmp_slots_size + tmp_header_size <= run_size {
                num_of_slots = s as _;
                header_size = tmp_header_size;
                break;
            }
            s -= 1;
        }
        header_size += run_size % bracket_size;
        num_of_slotsc[i] = num_of_slots;
        header_sizes[i] = header_size;
        i += 1;
    }
    let _ = header_sizes;
    num_of_slotsc
};

pub const HEADER_SIZES: [usize; NUM_OF_SIZE_BRACKETS] = {
    let mut num_of_slotsc: [usize; NUM_OF_SIZE_BRACKETS] = [0; NUM_OF_SIZE_BRACKETS];
    let mut header_sizes = [0; NUM_OF_SIZE_BRACKETS];
    let mut i = 0;
    while i < NUM_OF_SIZE_BRACKETS {
        let bracket_size = BRACKET_SIZES[i];
        let run_size = PAGE_SIZE * NUM_OF_PAGES[i];
        let max_num_of_slots = run_size / bracket_size;

        let fixed_header_size =
            round_up(Run::fixed_header_size() as _, size_of::<u64>() as _) as usize;
        let mut header_size = 0;
        let mut num_of_slots = 0;

        let mut s = max_num_of_slots as isize;

        while s >= 0 {
            let tmp_slots_size = bracket_size * s as usize;
            let tmp_unaligned_header_size = fixed_header_size;

            let tmp_header_size = if tmp_unaligned_header_size % bracket_size == 0 {
                tmp_unaligned_header_size
            } else {
                tmp_unaligned_header_size
                    + (bracket_size - tmp_unaligned_header_size % bracket_size)
            };

            if tmp_slots_size + tmp_header_size <= run_size {
                num_of_slots = s as _;
                header_size = tmp_header_size;
                break;
            }
            s -= 1;
        }
        header_size += run_size % bracket_size;
        num_of_slotsc[i] = num_of_slots;
        header_sizes[i] = header_size;
        i += 1;
    }
    let _ = num_of_slotsc;
    header_sizes
};

#[derive(Clone, Copy, Debug, Default)]
struct PtrHasher(u64);

impl Hasher for PtrHasher {
    fn write(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            self.0 = (self.0 << 8) ^ (byte as u64);
        }
    }

    fn write_u64(&mut self, value: u64) {
        self.0 ^= value;
    }
    fn finish(&self) -> u64 {
        self.0
    }
}

type BuildNoopHasher = BuildHasherDefault<PtrHasher>;

/// Returns dedicated full run so rosalloc knows when to create new run for allocation.
#[inline]
pub fn dedicated_full_run() -> *mut Run {
    unsafe { DEDICATED_FULL_RUN }
}

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum VisitPointerResult {
    Free,
    Keep,
}

pub trait WalkPages {
    fn visit_pointer(&mut self, ptr: *mut u8, size: usize, page: *mut u8, page_kind: PageMapKind) -> VisitPointerResult;
}