pub struct MswebrtcImpl {
    pub addr: u32,
}

#[no_mangle]
pub extern "C" fn create_mswebrtc_impl() -> *mut MswebrtcImpl {
    let out = Box::new(MswebrtcImpl { addr: 0 });
    Box::into_raw(out)
}

#[no_mangle]
pub extern "C" fn destroy_mswebrtc_impl(obj: *mut MswebrtcImpl) {
    unsafe { drop(Box::from_raw(obj)) };
}
