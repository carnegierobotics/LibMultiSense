use std::net::UdpSocket;
use str0m::{change::SdpOffer, Candidate, Rtc};
use systemstat::{Platform, System};

pub struct MswebrtcImpl {
    pc: Rtc,
    sockets: Vec<UdpSocket>,
}

#[allow(dead_code)]
#[derive(Debug)]
pub struct MswebrtcError {
    what: String,
    err: Option<Box<dyn std::error::Error>>,
}

impl MswebrtcImpl {
    fn connect_internal(&mut self, host: String) -> Result<(), MswebrtcError> {
        let client = reqwest::blocking::Client::builder()
            .danger_accept_invalid_certs(true)
            .build()
            .unwrap();
        let offer = SdpOffer::from_sdp_string(
            client
                .get(host.clone() + "/offer")
                .send()
                .map_err(|e| MswebrtcError {
                    what: "Failed to request offer".to_owned(),
                    err: Some(Box::new(e)),
                })?
                .text()
                .map_err(|e| MswebrtcError {
                    what: "Failed to extract offer from HTTP response".to_owned(),
                    err: Some(Box::new(e)),
                })?
                .as_str(),
        )
        .map_err(|e| MswebrtcError {
            what: "Failed to parse offer".to_owned(),
            err: Some(Box::new(e)),
        })?;

        //enumerate local ip addresses to enumerate ice candidates
        {
            let system = System::new();
            let networks = system.networks().map_err(|e| MswebrtcError {
                what: "Failed to enumarate networks".to_owned(),
                err: Some(Box::new(e)),
            })?;

            let mut found = false;
            for net in networks.values() {
                for n in &net.addrs {
                    if let systemstat::IpAddr::V4(v) = n.addr {
                        if !v.is_loopback() && !v.is_link_local() && !v.is_broadcast() {
                            let socket =
                                UdpSocket::bind(format!("{v}:0")).map_err(|e| MswebrtcError {
                                    what: "Failed to bind socket".to_owned(),
                                    err: Some(Box::new(e)),
                                })?;
                            let candidate = Candidate::host(
                                socket.local_addr().map_err(|e| MswebrtcError {
                                    what: "failed to get address for socket".to_owned(),
                                    err: Some(Box::new(e)),
                                })?,
                                "udp",
                            )
                            .map_err(|e| MswebrtcError {
                                what: "Failed to create ICE candidate".to_owned(),
                                err: Some(Box::new(e)),
                            })?;
                            if self.pc.add_local_candidate(candidate).is_some() {
                                found = true;
                                self.sockets.push(socket);
                            }
                        }
                    }
                }
            }
            if !found {
                Err(MswebrtcError {
		    what: "Failed to find valid local ICE candidate. Do we have any network interfaces?".to_owned(),
		    err: None
		})?
            }
        }
        let answer = self
            .pc
            .sdp_api()
            .accept_offer(offer)
            .map_err(|e| MswebrtcError {
                what: "Failed to accept RTC offer".to_owned(),
                err: Some(Box::new(e)),
            })?;

        let answer = serde_json::to_string(&answer).map_err(|e| MswebrtcError {
            what: "Failed to serialize answer".to_owned(),
            err: Some(Box::new(e)),
        })?;
        client
            .post(host + "/answer")
            .header("content-type", "application/json")
            .body(answer)
            .send()
            .map(|_| Ok(()))
            .map_err(|e| MswebrtcError {
                what: "Failed to post answer to server".to_owned(),
                err: Some(Box::new(e)),
            })?
    }
    fn connect(&mut self, host: String) -> bool {
        match self.connect_internal(host) {
            Ok(_) => true,
            Err(e) => {
                println!("ERROR CONNECTING: {:?}", e);
                false
            }
        }
    }
    fn disconnect(&self) {
        println!("DISCONNECT");
    }
}

#[no_mangle]
pub extern "C" fn create_mswebrtc_impl() -> *mut MswebrtcImpl {
    let pc = Rtc::new();
    let out = Box::new(MswebrtcImpl {
        pc: pc,
        sockets: Vec::new(),
    });
    Box::into_raw(out)
}

#[no_mangle]
pub extern "C" fn connect_mswebrtc_impl(
    obj: *mut MswebrtcImpl,
    addr: *const std::ffi::c_char,
) -> bool {
    let (r, addr) = unsafe {
        (
            &mut *obj,
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
