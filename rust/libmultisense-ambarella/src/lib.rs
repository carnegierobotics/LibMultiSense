use std::sync::Arc;
use webrtc::{
    api::media_engine::MediaEngine, api::APIBuilder, interceptor::registry::Registry,
    peer_connection::configuration::RTCConfiguration, peer_connection::RTCPeerConnection,
};

pub struct MswebrtcImpl {
    rt: tokio::runtime::Runtime,
    pc: Arc<RTCPeerConnection>,
}

#[no_mangle]
pub extern "C" fn create_mswebrtc_impl() -> *mut MswebrtcImpl {
    let rt = tokio::runtime::Runtime::new().expect("Failed to create async runtime");
    let registry = Registry::new();
    let m = MediaEngine::default();
    let api = APIBuilder::new()
        .with_media_engine(m)
        .with_interceptor_registry(registry)
        .build();
    let config = RTCConfiguration::default();
    let pc = Arc::new(
        rt.block_on(api.new_peer_connection(config))
            .expect("failed to make new peer connection"),
    );
    let out = Box::new(MswebrtcImpl { rt: rt, pc: pc });
    Box::into_raw(out)
}

#[no_mangle]
pub extern "C" fn destroy_mswebrtc_impl(obj: *mut MswebrtcImpl) {
    drop(unsafe { Box::from_raw(obj) });
}
