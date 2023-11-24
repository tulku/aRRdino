use futures::executor::LocalPool;
use futures::stream::StreamExt;
use futures::task::LocalSpawnExt;

use std::sync::{Arc, Mutex};

use r2r::QosProfile;
use r2r::geometry_msgs::msg::PoseStamped;
// use r2r::ferret_msgs::action::GetPath;

#[derive(Debug)]
struct Goal {
    origin: Option<r2r::geometry_msgs::msg::PoseStamped>,
    destination: Option<r2r::geometry_msgs::msg::PoseStamped>,
}

impl Goal {
    fn new() -> Self {
        Self {
            origin: None,
            destination: None,
        }
    }

    fn is_ready(&self) -> bool {
        self.origin.is_some() && self.destination.is_some()
    }
    
    // fn to_goal(&self) -> r2r::ferret_msgs::action::GetPath::Goal {
    //     let mut goal = r2r::ferret_msgs::action::GetPath::Goal::default();
    //     goal.origin = self.origin.clone();
    //     goal.destination = self.destination.clone();
    //     goal
    // }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "action_client", "")?;

    let mut clicked_point_sub =
        node.subscribe::<PoseStamped>("/clicked_point", QosProfile::default())?;
    let mut move_base_simple_sub =
        node.subscribe::<PoseStamped>("/move_base_simple", QosProfile::default())?;


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
                    goal_click_cb.lock().unwrap().origin = Some(msg);
                    // println!("{:?}", goal_click_cb.lock().unwrap().origin);
                }
                None => break,
            }
        }
    })?;

    let goal_movebase_cb = goal.clone();
    // task that loads the clicked point to the starting pose variable;
    spawner.spawn_local(async move {
        println!("I got to move base");
        loop {
            match move_base_simple_sub.next().await {
                Some(msg) => {
                    goal_movebase_cb.lock().unwrap().destination = Some(msg);
                    // println!("{:?}", goal_movebase_cb.lock().unwrap().destination);
                }
                None => break,
            }
        }
    })?;

    let publisher = node.create_publisher::<PoseStamped>("/get_path", QosProfile::default())?;

    loop {
        node.spin_once(std::time::Duration::from_millis(100));
        pool.run_until_stalled();
        if goal.lock().unwrap().is_ready() {
            let goal = goal.lock().unwrap().origin.clone().unwrap();
            publisher.publish(&goal)?;
            println!("Goal sent: {:?}", goal);
        }
    }
}