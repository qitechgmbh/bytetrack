 cargo install rustdoc-md --git https://github.com/qitechgmbh/rustdoc-md --rev 6dc34f9c9d878799d9c1b4f2d2bcbb1f2757ede7
 RUSTC_BOOTSTRAP=1 RUSTDOCFLAGS="-Z unstable-options --output-format json" cargo doc --no-deps
 rustdoc-md --path ../target/doc/bytetrack.json --output rustdoc.md