//! An example of opening an image.
use std::env;
use std::path::Path;

fn main() {
    let file = if env::args().count() == 2 {
        env::args().nth(1).unwrap()
    } else {
        panic!("Please enter a file")
    };
    let scene = ferret::Scene::new(Path::new(&file));
    let start = [23.0f64, 11.0];
    let goal = [45.0f64, 50.0];
    let path = ferret::calculate_path(&scene, &start, &goal);
    println!("path is {:?}", path);
}
