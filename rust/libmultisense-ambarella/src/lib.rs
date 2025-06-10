use std::net::UdpSocket;
use str0m::{change::SdpOffer, Candidate, Rtc};
use systemstat::{Platform, System};

pub struct MswebrtcImpl {
    host: String,
    pc: Rtc,
    sockets: Vec<UdpSocket>,
}

struct Info {
    builddate: String,
}

#[allow(dead_code)]
#[derive(Debug)]
pub struct MswebrtcError {
    what: String,
    err: Option<Box<dyn std::error::Error>>,
}

impl MswebrtcImpl {
    fn connect(&mut self) -> Result<(), MswebrtcError> {
        let client = reqwest::blocking::Client::builder()
            .danger_accept_invalid_certs(true)
            .build()
            .unwrap();
        let offer = SdpOffer::from_sdp_string(
            client
                .get(self.host.clone() + "/offer")
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
            .post(self.host.clone() + "/answer")
            .header("content-type", "application/json")
            .body(answer)
            .send()
            .map(|_| Ok(()))
            .map_err(|e| MswebrtcError {
                what: "Failed to post answer to server".to_owned(),
                err: Some(Box::new(e)),
            })?
    }
    fn disconnect(&self) {
        println!("DISCONNECT");
    }

    fn get_info(&self) -> Result<Info, MswebrtcError> {
        let client = reqwest::blocking::Client::builder()
            .danger_accept_invalid_certs(true)
            .build()
            .unwrap();
        let j = client
            .get(self.host.clone() + "/info.json")
            .send()
            .map_err(|e| MswebrtcError {
                what: "Failed to request offer".to_owned(),
                err: Some(Box::new(e)),
            })?;
        let j = j.text().map_err(|e| MswebrtcError {
            what: "Failed to extract offer from HTTP response".to_owned(),
            err: Some(Box::new(e)),
        })?;
        let j: serde_json::Value = serde_json::from_str(j.as_str()).map_err(|e| MswebrtcError {
            what: "Failed to parse info.json".to_owned(),
            err: Some(Box::new(e)),
        })?;
        Ok(Info {
            builddate: j["build-time"].to_string(),
        })
    }
}

#[no_mangle]
pub extern "C" fn create_mswebrtc_impl(host: *const std::ffi::c_char) -> *mut MswebrtcImpl {
    let pc = Rtc::new();
    let out = Box::new(MswebrtcImpl {
        host: unsafe { std::ffi::CStr::from_ptr(host) }
            .to_str()
            .unwrap()
            .to_string(),
        pc: pc,
        sockets: Vec::new(),
    });
    Box::into_raw(out)
}

#[no_mangle]
pub extern "C" fn connect_mswebrtc_impl(obj: *mut MswebrtcImpl) -> bool {
    let r = unsafe { &mut *obj };
    match r.connect() {
        Ok(_) => true,
        Err(e) => {
            println!("ERROR CONNECTING: {:?}", e);
            false
        }
    }
}

#[no_mangle]
pub extern "C" fn get_info_mswebrtc_impl(
    obj: *mut MswebrtcImpl,
    mut buildtime: *mut std::ffi::c_char,
) -> bool {
    let r = unsafe { &*obj };
    let i = r.get_info();
    match i {
        Ok(i) => {
            for (i, c) in i.builddate.chars().enumerate() {
                unsafe { *buildtime = c as std::ffi::c_char };
                buildtime = buildtime.wrapping_add(1);
                if i >= 31 {
                    break;
                }
            }
            true
        }
        Err(e) => {
            println!("ERROR Getting Info: {:?}", e);
            false
        }
    }
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
