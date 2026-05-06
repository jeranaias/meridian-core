//! `meridian-dds` — CDR serialization, ROS2 message types, and Meridian↔ROS2 conversion.
//!
//! This crate is `no_std` by default so that the CDR codec and type definitions can be
//! reused on embedded targets with Ethernet (future dust-dds integration).  Enable the
//! `std` feature for the companion-side bridge binary.
//!
//! # Architecture
//!
//! ```text
//! meridian-dds (this crate)
//!   ├── cdr        — Zero-alloc CDR (XCDR1) serializer/deserializer
//!   ├── ros2_msgs  — ROS2 standard message structs (sensor_msgs, geometry_msgs, etc.)
//!   ├── convert    — Meridian internal types ↔ ROS2 types (NED↔ENU)
//!   └── topics     — Topic name constants and QoS profile definitions
//! ```

#![no_std]

pub mod cdr;
pub mod ros2_msgs;
pub mod convert;
pub mod topics;
