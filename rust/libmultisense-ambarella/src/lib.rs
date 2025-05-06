use std::sync::Arc;
use webrtc::peer_connection::RTCPeerConnection;
use wrtc::new_peer_connection;

pub struct MswebrtcImpl {
    rt: tokio::runtime::Runtime,
    pc: Arc<RTCPeerConnection>,
}

#[no_mangle]
pub extern "C" fn create_mswebrtc_impl() -> *mut MswebrtcImpl {
    let rt = tokio::runtime::Runtime::new().expect("Failed to create async runtime");
    let peer_connection = rt
        .block_on(new_peer_connection())
        .expect("Failed to create wrtc peer connection");
    let out = Box::new(MswebrtcImpl {
        rt: rt,
        pc: peer_connection,
    });
    Box::into_raw(out)
}

#[no_mangle]
pub extern "C" fn destroy_mswebrtc_impl(obj: *mut MswebrtcImpl) {
    drop(unsafe { Box::from_raw(obj) });
}
