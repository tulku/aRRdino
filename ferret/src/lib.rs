use image::{self, DynamicImage};
use rand::distributions::{Distribution, Uniform};

use std::path::Path;

use image::GenericImageView;

pub struct Scene {
    map: DynamicImage,
    resolution: (f64, f64),
}

impl Scene {
    pub fn new(filename: &Path) -> Self {
        let algo = Self {
            map: image::open(filename).unwrap(),
            resolution: (0.1, 0.1),
        };
        let dimensions = algo.map.dimensions();
        println!("{:?}", dimensions);
        println!(
            "The first pixel value is: {}",
            algo.map.get_pixel(0, 0).0[0]
        );
        algo
    }

    fn to_pixel(&self, point: &[f64]) -> (u32, u32) {
        let max_y = self.map.dimensions().1;
        let x = (point[0] / self.resolution.0).floor() as u32;
        let y = (point[1] / self.resolution.1).floor() as u32;
        let y = max_y - y;
        (x, y)
    }

    fn to_map(&self, x: u32, y: u32) -> (f64, f64) {
        let max_y = self.map.dimensions().1;
        let x = x as f64 * self.resolution.0;
        let y = max_y - y;
        let y = y as f64 * self.resolution.1;
        (x, y)
    }

    fn is_feasible(&self, point: &[f64]) -> bool {
        let (x, y) = self.to_pixel(point);
        let value = self.map.get_pixel(x, y).0[0];
        // println!("p: {:?} [({}, {})] -> {}", point, x, y, value);
        value > 250
    }

    fn random_sample(&self) -> Vec<f64> {
        let dimensions = self.map.dimensions();
        let x_range = Uniform::new(0, dimensions.0);
        let y_range = Uniform::new(0, dimensions.1);
        let mut rng = rand::thread_rng();
        let (x, y) = self.to_map(x_range.sample(&mut rng), y_range.sample(&mut rng));
        vec![x, y]
    }
}

pub fn calculate_path(scene: &Scene, start: &[f64], goal: &[f64]) -> Vec<Vec<f64>> {
    let mut path = rrt::dual_rrt_connect(
        &start,
        &goal,
        |x: &[f64]| scene.is_feasible(x),
        || scene.random_sample(),
        0.025,
        1e+7 as usize,
    )
    .unwrap();
    rrt::smooth_path(&mut path, |x: &[f64]| scene.is_feasible(x), 0.01, 100);
    path
}
