#![no_std]

//! Navigation algorithms: waypoint management, L1 guidance, path planning.

pub mod waypoint;
pub mod l1;
pub mod smartrtl;
pub mod rally;
pub mod scurve;

pub use waypoint::{Waypoint, WaypointNav, WaypointStatus, AltFrame};
pub use l1::L1Controller;
pub use smartrtl::SmartRTL;
pub use rally::{RallyPoint, RallyManager};
pub use scurve::{SCurveSegment, SplineSegment};

pub mod current_estimator;
pub use current_estimator::{CurrentEstimator, CurrentEstimatorConfig, CurrentEstimate};
