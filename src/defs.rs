/// Different page release modes
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub enum PageReleaseMode {
    /// Release no empty pages
    None,
    /// Release empty pages at the end of the space
    End,
    /// Release empty pages that are larger than the threshold
    Size,
    /// Release empty pages that are larger than the threshold or at the end of the space
    SizeAndEnd,
    /// Release all empty pages
    All,
}

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
#[repr(u8)]
pub enum PageMapKind {
    Released,
    Empty,
    Run,
    RunPart,
    LargeObject,
    LargeObjectPart,
}

pub const KB: usize = 1024;
pub const MB: usize = KB * KB;
pub const GB: usize = 1024 * MB;

pub const DEFAULT_PAGE_RELEASE_THRESHOLD: usize = 4 * MB;
pub const NUM_THREAD_LOCAL_SIZE_BRACKETS: usize = 16;
pub const MAX_THREAD_LOCAL_BRACKET_SIZE: usize = 128;
pub const NUM_REGULAR_SIZE_BRACKETS: usize = 40;
pub const MAX_REGULAR_BRACKET_SIZE: usize = 512;
pub const THREAD_LOCAL_BRACKET_QUANTUM_SIZE: usize = 16;
pub const BRACKET_QUANTUM_SIZE: usize = 16;
pub const BRACKET_QUANTUM_SIZE_SHIFT: usize = 4;
pub const MAGIC_NUM: u8 = 42;
pub const MAGIC_NUM_FREE: u8 = 43;
pub const NUM_OF_SIZE_BRACKETS: usize = 42;

cfg_if::cfg_if! {
    if #[cfg(all(any(target_os="macos", target_os = "ios"), target_arch="aarch64"))] {
        pub const PAGE_SIZE: usize = 16 * KB;
    } else {
        pub const PAGE_SIZE: usize = 4 * KB;
    }

}