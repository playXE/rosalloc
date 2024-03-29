#[cfg(all(not(miri), windows))]
pub mod _win {

    use core::{ptr::null_mut, usize};
    use winapi::um::{
        memoryapi::{VirtualAlloc, VirtualFree},
        winnt::{MEM_COMMIT, MEM_DECOMMIT, MEM_RELEASE, MEM_RESERVE, PAGE_READWRITE},
    };
    pub struct Mmap {
        start: *mut u8,
        end: *mut u8,
        size: usize,
    }
    impl Mmap {
        pub const fn uninit() -> Self {
            Self {
                start: null_mut(),
                end: null_mut(),
                size: 0,
            }
        }
        pub fn new(size: usize) -> Self {
            unsafe {
                let mem = VirtualAlloc(null_mut(), size, MEM_RESERVE, PAGE_READWRITE);
                let mem = mem as *mut u8;

                let end = mem.add(size);

                Self {
                    start: mem,
                    end,
                    size,
                }
            }
        }
        /// Return aligned pointer
        pub fn aligned(&self, align: usize) -> *mut u8 {
            let offset = align - (self.start as usize) % align;
            unsafe { self.start.add(offset) as *mut u8 }
        }

        pub fn start(&self) -> *mut u8 {
            self.start
        }
        pub fn end(&self) -> *mut u8 {
            self.end
        }

        pub fn dontneed(&self, _page: *mut u8, _size: usize) {
            /*unsafe {
                //DiscardVirtualMemory(page.cast(), size as _);
                //VirtualFree(page.cast(), size, MEM_DECOMMIT);
            }*/
        }

        pub fn decommit(&self, page: *mut u8, size: usize) {
            unsafe {
                //DiscardVirtualMemory(page.cast(), size as _);
                VirtualFree(page.cast(), size, MEM_DECOMMIT);
            }
        }

        pub fn commit(&self, page: *mut u8, size: usize) {
            unsafe {
                VirtualAlloc(page.cast(), size, MEM_COMMIT, PAGE_READWRITE);
            }
        }
        pub const fn size(&self) -> usize {
            self.size
        }
    }

    impl Drop for Mmap {
        fn drop(&mut self) {
            unsafe {
                VirtualFree(self.start.cast(), self.size, MEM_RELEASE);
            }
        }
    }
}

#[cfg(all(unix, not(miri)))]
pub mod _unix {

    use std::ptr::null_mut;

    pub struct Mmap {
        start: *mut u8,
        end: *mut u8,
        size: usize,
    }

    impl Mmap {
        pub const fn size(&self) -> usize {
            self.size
        }
        pub const fn uninit() -> Self {
            Self {
                start: null_mut(),
                end: null_mut(),
                size: 0,
            }
        }
        pub fn new(size: usize) -> Self {
            unsafe {
                let map = libc::mmap(
                    core::ptr::null_mut(),
                    size as _,
                    libc::PROT_READ | libc::PROT_WRITE,
                    libc::MAP_PRIVATE | libc::MAP_ANON,
                    -1,
                    0,
                );
                libc::madvise(map, size, libc::MADV_SEQUENTIAL);
                if map == libc::MAP_FAILED {
                    panic!("mmap failed");
                }
                Self {
                    start: map as *mut u8,
                    end: (map as usize + size) as *mut u8,
                    size,
                }
            }
        }
        /// Return aligned pointer
        pub fn aligned(&self, align: usize) -> *mut u8 {
            let offset = align - (self.start as usize) % align;
            unsafe { self.start.add(offset) as *mut u8 }
        }

        pub fn start(&self) -> *mut u8 {
            self.start
        }
        pub fn end(&self) -> *mut u8 {
            self.end
        }

        pub fn dontneed(&self, page: *mut u8, size: usize) {
            unsafe {
                libc::madvise(page as *mut _, size as _, libc::MADV_DONTNEED);
            }
        }

        pub fn decommit(&self, page: *mut u8, size: usize) {
            unsafe {
                libc::madvise(page as *mut _, size as _, libc::MADV_DONTNEED);
            }
        }

        pub fn commit(&self, page: *mut u8, size: usize) {
            unsafe {
                libc::madvise(
                    page as *mut _,
                    size as _,
                    libc::MADV_WILLNEED | libc::MADV_SEQUENTIAL,
                );
            }
        }
    }

    impl Drop for Mmap {
        fn drop(&mut self) {
            unsafe {
                libc::munmap(self.start() as *mut _, self.size as _);
            }
        }
    }
}

#[cfg(miri)]
pub mod _miri {
    use std::{mem::size_of, ptr::null_mut};

    pub struct Mmap {
        vec: Vec<usize>,
        start: *mut u8,
        end: *mut u8,
        size: usize,
    }

    impl Mmap {
        pub const fn uninit() -> Self {
            Self {
                start: null_mut(),
                end: null_mut(),
                size: 0,
                vec: Vec::new(),
            }
        }

        pub fn new(size: usize) -> Self {
            let mem = unsafe { libc::malloc(size) as *mut u8 }; //vec![0usize; size / size_of::<usize>() + 32];
            unsafe {
                core::ptr::write_bytes(mem, 0, size);
            }
            let start = mem;
            let end = unsafe { start.add(size) };
            Self {
                vec: Vec::new(),
                start,
                end,
                size,
            }
        }

        pub fn aligned(&self, align: usize) -> *mut u8 {
            let offset = align - (self.start as usize) % align;
            unsafe { self.start.add(offset) as *mut u8 }
        }

        pub const fn start(&self) -> *mut u8 {
            self.start
        }
        pub const fn end(&self) -> *mut u8 {
            self.end
        }

        pub const fn dontneed(&self, _page: *mut u8, _size: usize) {}

        pub const fn decommit(&self, _page: *mut u8, _size: usize) {}

        pub const fn commit(&self, _page: *mut u8, _size: usize) {}
        pub const fn size(&self) -> usize {
            self.size
        }
    }
    impl Drop for Mmap {
        fn drop(&mut self) {
            unsafe { libc::free(self.start as _); }
        }
    }
}

#[cfg(all(not(miri), unix))]
pub use _unix::*;
#[cfg(all(not(miri), windows))]
pub use _win::*;

impl Mmap {
    pub fn dontneed_and_zero(&self, page: *mut u8, size: usize) {
        unsafe {
            std::ptr::write_bytes(page, 0, size);
        }
        self.dontneed(page, size);
    }
}

#[cfg(miri)]
pub use _miri::*;
