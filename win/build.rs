fn main() {
    // Embeds the app icon in the .exe (File Explorer, taskbar, shortcuts).
    if std::env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("windows") {
        embed_resource::compile("app.rc", embed_resource::NONE);
    }
}
