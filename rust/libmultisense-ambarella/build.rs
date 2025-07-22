extern crate cbindgen;

use uuid::Uuid;

fn main() {
    let mut output_file = std::env::current_dir().unwrap();
    output_file.push(std::env::var("CARGO_TARGET_DIR").unwrap());
    output_file.push("mswebrtc.h");

    let b = cbindgen::Builder::new()
        .with_include_guard(
            "MSWEBRTC_H".to_owned()
                + &Uuid::new_v4()
                    .to_string()
                    .to_uppercase()
                    .split("-") //split and fold here because join is experimental
                    .fold(String::new(), |a, b| a + "_" + b),
        )
        .with_crate(std::env::current_dir().unwrap());
    b.generate()
        .expect("Failed to generate bindings")
        .write_to_file(output_file);
}
