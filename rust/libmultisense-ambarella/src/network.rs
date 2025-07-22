use serde::{Deserialize, Serialize};
use std::{net::Ipv4Addr, str::FromStr};

#[derive(Debug, Deserialize, Serialize, Clone)]
pub struct Network {
    #[serde(rename(serialize = "ip-address", deserialize = "ip-address"))]
    pub addr: String,
    #[serde(rename(serialize = "netmask", deserialize = "netmask"))]
    pub mask: u8,
    pub gateway: Option<String>,
}

fn addr_to_cidr(addr: &str) -> Option<u8> {
    let a = Ipv4Addr::from_str(addr);
    match a {
        Ok(a) => {
            let b = a.to_bits();
            let mut out = 32;
            let mut i = 0;
            while i < 32 {
                if (b & (1 << i)) != 0 {
                    break;
                }
                out -= 1;
                i += 1;
            }
            while i < 32 {
                if (b & (1 << i)) == 0 {
                    return None;
                }
                i += 1;
            }
            Some(out)
        }
        Err(_) => None,
    }
}

impl Network {
    pub fn from_strings(addr: &str, mask: &str, gateway: &str) -> Option<Self> {
        Some(Network {
            addr: addr.to_string(),
            mask: addr_to_cidr(mask)?,
            gateway: if gateway.is_empty() {
                None
            } else {
                Some(gateway.to_string())
            },
        })
    }
}
