use wasm_bindgen::JsCast;
use wasm_bindgen_futures::JsFuture;
use web_sys::{window, Response};

/// Read a file into a string. File needs to be hosted on the site.
pub async fn read_web_file(file_path: &str) -> String {
    let window = window().expect("no global `window` exists");
    let resp_value = JsFuture::from(window.fetch_with_str(file_path))
        .await
        .unwrap();
    let resp: Response = resp_value.dyn_into().unwrap();
    let buf = JsFuture::from(resp.text().unwrap()).await.unwrap();
    let buf = buf.as_string().unwrap();

    // // Supposed to be faster to do the decoding in Rust
    // let buf = JsFuture::from(resp.array_buffer().unwrap()).await.unwrap();
    // // Convert ArrayBuffer -> Uint8Array
    // let uint8_array = js_sys::Uint8Array::new(&buf);

    // // Decode using TextDecoder
    // let decoder = TextDecoder::new().expect("failed to create TextDecoder");
    // let buf = decoder
    //     .decode_with_js_u8_array(&uint8_array)
    //     .expect("text decode failed");

    buf
}

#[macro_export]
macro_rules! toJsFloat32Array {
    ($q:expr) => {
        Float32Array::from(
            $q.iter()
                .map(|qi| *qi as f32)
                .collect::<Vec<f32>>()
                .as_slice(),
        )
    };
}

#[macro_export]
macro_rules! toJsUint32Array {
    ($q:expr) => {
        Uint32Array::from(
            $q.iter()
                .map(|qi| *qi as u32)
                .collect::<Vec<u32>>()
                .as_slice(),
        )
    };
}
