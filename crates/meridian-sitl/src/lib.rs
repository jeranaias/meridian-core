//! Meridian SITL — Software-In-The-Loop simulation.
//!
//! Physics engine, sensor simulation, and deterministic replay.
//! Always runs in std mode.

pub mod physics;
pub mod physics_fixedwing;
pub mod physics_rover;
pub mod physics_sub;
pub mod physics_jetboat;
pub mod sensors;
pub mod scheduler;
pub mod hover_test;
pub mod sim_runner;
pub mod sim_fixedwing;
pub mod sim_rover;
pub mod test_scenarios;
pub mod test_safety;
pub mod test_edge_cases;

pub use physics::{PhysicsState, VehicleParams, GRAVITY, PHYSICS_HZ};
pub use sensors::{SensorSim, NoiseParams, BaroGroundCal};
pub use scheduler::{Scheduler, RateGroupId, TaskBudget};
pub use hover_test::{run_hover_simulation, HoverResult};
pub use sim_runner::{run_simulation, SimConfig, SimResult, Target, WindModel};
