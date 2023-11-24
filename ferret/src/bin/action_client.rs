use futures::executor::LocalPool;
use futures::stream::StreamExt;
use futures::task::LocalSpawnExt;

use std::sync::{Arc, Mutex};

use r2r::builtin_interfaces::msg::Time;
use r2r::geometry_msgs::msg::PointStamped;
use r2r::geometry_msgs::msg::Pose;
use r2r::geometry_msgs::msg::PoseStamped;
use r2r::geometry_msgs::msg::Quaternion;
use r2r::nav_msgs::msg::Path;
use r2r::std_msgs::msg::Header;
use r2r::QosProfile;

use std::time::{SystemTime, UNIX_EPOCH};
// use r2r::ferret_msgs::action::GetPath;

#[derive(Debug)]
struct Goal {
    origin: Option<PoseStamped>,
    destination: Option<PoseStamped>,
    sent: bool,
}

impl Goal {
    fn new() -> Self {
        Self {
            origin: None,
            destination: None,
            sent: false,
        }
    }

    fn is_ready(&self) -> bool {
        self.origin.is_some() && self.destination.is_some() && !self.sent
    }

    fn set_origin(&mut self, origin: PointStamped) {
        self.origin = Some(PoseStamped {
            header: origin.header,
            pose: Pose {
                position: origin.point,
                orientation: Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            },
        });
        self.sent = false;
    }

    fn set_destination(&mut self, destination: PoseStamped) {
        self.destination = Some(destination);
        self.sent = false;
    }

    fn sent(&mut self) {
        self.sent = true;
    }
}

fn get_path_from_poses(origin: PoseStamped, destination: PoseStamped) -> Path {
    let start = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("Time went backwards");
    let secs = start.as_secs();
    let nanos = start.subsec_nanos();
    Path {
        header: Header {
            stamp: Time {
                sec: secs as i32,
                nanosec: nanos,
            },
            frame_id: "map".to_string(),
        },
        poses: vec![origin, destination],
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "ferret_2d", "")?;

    let mut clicked_point_sub =
        node.subscribe::<PointStamped>("/clicked_point", QosProfile::default())?;
    let mut goal_pose_sub = node.subscribe::<PoseStamped>("/goal_pose", QosProfile::default())?;

    // Share state
    // let done = Arc::new(Mutex::new(false));
    let goal = Arc::new(Mutex::new(Goal::new()));

    let mut pool = LocalPool::new();
    let spawner = pool.spawner();

    let goal_click_cb = goal.clone();
    // task that loads the clicked point to the starting pose variable;
    spawner.spawn_local(async move {
        println!("I got to points click");
        loop {
            match clicked_point_sub.next().await {
                Some(msg) => {
                    goal_click_cb.lock().unwrap().set_origin(msg);
                    println!("{:?}", goal_click_cb.lock().unwrap().origin);
                }
                None => break,
            }
        }
    })?;

    let goal_pose_cb = goal.clone();
    // task that loads the clicked point to the starting pose variable;
    spawner.spawn_local(async move {
        println!("I got to move base");
        loop {
            match goal_pose_sub.next().await {
                Some(msg) => {
                    goal_pose_cb.lock().unwrap().set_destination(msg);
                    println!("{:?}", goal_pose_cb.lock().unwrap().destination);
                }
                None => break,
            }
        }
    })?;

    let publisher = node.create_publisher::<Path>("/get_path", QosProfile::default())?;

    loop {
        node.spin_once(std::time::Duration::from_millis(100));
        pool.run_until_stalled();
        if goal.lock().unwrap().is_ready() {
            let mut goal = goal.lock().unwrap();
            let origin = goal.origin.clone().unwrap();
            let destination = goal.destination.clone().unwrap();
            let path = get_path_from_poses(origin, destination);
            publisher.publish(&path)?;
            println!("New path sent: {:?}", path);
            goal.sent();
        }
    }
}
