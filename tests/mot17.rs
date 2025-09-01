use anyhow::Error;
use std::path::Path;

pub async fn download_and_unzip(url: &str, dest: &Path) -> Result<(), Error> {
    let response = reqwest::get(url).await?;
    let bytes = response.bytes().await?;
    let cursor = std::io::Cursor::new(bytes);
    let mut zip = zip::ZipArchive::new(cursor)?;

    for i in 0..zip.len() {
        let mut file = zip.by_index(i)?;
        let outpath = dest.join(file.sanitized_name());

        if (*file.name()).ends_with('/') {
            std::fs::create_dir_all(&outpath)?;
        } else {
            if let Some(p) = outpath.parent() {
                if !p.exists() {
                    std::fs::create_dir_all(&p)?;
                }
            }
            let mut outfile = std::fs::File::create(&outpath)?;
            std::io::copy(&mut file, &mut outfile)?;
        }
    }
    Ok(())
}

pub async fn download_mot17() -> Result<(), Error> {
    // download and unzip the MOT17 dataset to datasets/MOT17
    // https://motchallenge.net/data/MOT17Labels.zip
    let url = "https://motchallenge.net/data/MOT17Labels.zip";
    let dest = Path::new("datasets/MOT17");
    download_and_unzip(url, dest).await
}

// enum for train or test
enum DatasetType {
    Train,
    Test,
}

enum DetectorType {
    DPM,
    FRCNN,
    SDP,
}

async fn parse_mot(
    path: &Path,
    dataset_type: DatasetType,
    detector_type: Option<DetectorType>,
) -> Result<polars::prelude::DataFrame, Error> {
    // assign column names
    let det_columns = &[
        "frame",
        "id",
        "bb_left",
        "bb_top",
        "bb_width",
        "bb_height",
        "conf",
        "x",
        "y",
        "z",
    ];

    Ok(df)
}

#[tokio::test]
async fn test_mot() {
    download_mot17().await.unwrap();
}
