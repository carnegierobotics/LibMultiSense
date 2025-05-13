use std::sync::Arc;
use webrtc::{
    api::media_engine::MediaEngine,
    api::APIBuilder,
    interceptor::registry::Registry,
    peer_connection::{
        configuration::RTCConfiguration, sdp::session_description::RTCSessionDescription,
        RTCPeerConnection,
    },
};

pub struct MswebrtcImpl {
    rt: tokio::runtime::Runtime,
    pc: Arc<RTCPeerConnection>,
}

impl MswebrtcImpl {
    fn connect(&self, host: String) -> bool {
        let client = reqwest::blocking::Client::builder()
            .danger_accept_invalid_certs(true)
            .build()
            .unwrap();
        let offer = match client.get(host.clone() + "/offer").send() {
            Ok(response) => {
                let text = response.text().unwrap();
                Some(text)
            }
            Err(e) => None,
        }
        .expect("Failed to get offer");

        let answer = self.rt.block_on(async {
            match RTCSessionDescription::offer(offer) {
                Ok(offer) => {
                    // set the remote description
                    //let asdf = offer;
                    //asdf.sdp;
                    //dbg!(&offer);
                    self.pc.set_remote_description(offer).await.unwrap();

                    // Create an answer
                    let answer = self.pc.create_answer(None).await.unwrap();

                    // Sets the LocalDescription, and starts our UDP listeners
                    self.pc.set_local_description(answer.clone()).await.unwrap();

                    // Create channel that is blocked until ICE Gathering is complete
                    let mut gather_complete = self.pc.gathering_complete_promise().await;

                    let _ = gather_complete.recv().await;
                    //Ok(answer)

                    if let Some(local_desc) = self.pc.local_description().await {
                        Ok(local_desc)
                    } else {
                        println!("failed to generate local description");
                        Err(String::from("failed to generate local description"))
                    }
                }
                Err(e) => Err(String::from("failed to parse offer")),
            }
        });

        let answer = serde_json::to_string(&answer).expect("Failed to serialize answer");
        client
            .post(host + "/answer")
            .header("content-type", "application/json")
            .body(answer)
            .send()
            .is_ok()
    }
    fn disconnect(&self) {
        println!("DISCONNECT");
    }
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
pub extern "C" fn connect_mswebrtc_impl(
    obj: *mut MswebrtcImpl,
    addr: *const std::ffi::c_char,
) -> bool {
    let (r, addr) = unsafe {
        (
            &*obj,
            std::ffi::CStr::from_ptr(addr as *const i8)
                .to_str()
                .unwrap()
                .to_string(),
        )
    };
    r.connect(addr)
}

#[no_mangle]
pub extern "C" fn disconnect_mswebrtc_impl(obj: *mut MswebrtcImpl) {
    let r = unsafe { &*obj };
    r.disconnect()
}

#[no_mangle]
pub extern "C" fn destroy_mswebrtc_impl(obj: *mut MswebrtcImpl) {
    drop(unsafe { Box::from_raw(obj) });
}
