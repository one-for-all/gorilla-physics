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

    buf
}
