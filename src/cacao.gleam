//// Cacao - Physics add-on for Tiramisu.
////
//// Provides Rapier physics integration as a hybrid library:
//// declarative physics attributes on meshes in the view, imperative core for
//// world management and stepping. No web component registration needed.
////
//// ## Usage
////
//// ```gleam
//// import cacao
////
//// // In init — create the physics world
//// fn init(_) {
////   #(Model(physics: None), cacao.init(vec3.Vec3(0.0, -9.81, 0.0), PhysicsReady))
//// }
////
//// // In update — step physics on each tick
//// fn update(model, msg) {
////   case msg {
////     PhysicsReady(world) -> #(Model(physics: Some(world)), effect.none())
////     Tick(_) -> {
////       let world = cacao.step(model.physics)
////       #(Model(..model, physics: world), effect.none())
////     }
////   }
//// }
////
//// // In view — add physics attributes to meshes
//// fn view(_model) {
////   renderer.renderer([...], [
////     mesh.mesh("player", [
////       mesh.geometry_box(vec3.Vec3(1.0, 1.0, 1.0)),
////       cacao.body_type(cacao.Dynamic),
////       cacao.collider(cacao.Cuboid(0.5, 0.5, 0.5)),
////     ], []),
////   ])
//// }
//// ```

// IMPORTS ---------------------------------------------------------------------

import estoque
import gleam/javascript/promise

import gleam/bool
import gleam/float
import gleam/int
import gleam/list
import gleam/option.{type Option, Some}

import lustre/attribute.{type Attribute}
import lustre/effect.{type Effect}

import quaternion
import vec/vec3

// CORE TYPES ------------------------------------------------------------------

/// Opaque physics state held in the user's Lustre model.
///
/// Created with `init()`, updated with `step()`.
/// Registries (body→handle, collider→mesh) are stored as JS Maps on the
/// underlying Rapier world object for zero-cost per-frame access.
pub type PhysicsWorld {
  PhysicsWorld(world: estoque.World, collision_events: List(CollisionEvent))
}

/// A collision event from the physics simulation.
///
/// Populated during `step()` and read via `collision_events()` or
/// `get_collisions_for()`.
pub type CollisionEvent {
  CollisionEvent(
    mesh_id_a: Option(String),
    mesh_id_b: Option(String),
    collision_type: CollisionEventType,
    is_sensor: Bool,
  )
}

/// Whether a collision started or stopped.
pub type CollisionEventType {
  CollisionStarted
  CollisionStopped
}

/// The type of a physics body.
pub type BodyType {
  /// Affected by forces, gravity, and collisions.
  Dynamic
  /// Immovable, not affected by forces (walls, floors).
  Fixed
  /// Moved by user code via position. Affects dynamic bodies.
  KinematicPosition
  /// Moved by user code via velocity. Affects dynamic bodies.
  KinematicVelocity
}

/// The shape of a physics collider.
pub type ColliderShape {
  Cuboid(hx: Float, hy: Float, hz: Float)
  Ball(radius: Float)
  Capsule(half_height: Float, radius: Float)
  Cylinder(half_height: Float, radius: Float)
  Cone(half_height: Float, radius: Float)
}

/// Result of a raycast hit against the physics world.
pub type RayHit {
  RayHit(
    /// World-space hit point.
    point: vec3.Vec3(Float),
    /// Surface normal at the hit point.
    normal: vec3.Vec3(Float),
    /// Distance from the ray origin to the hit point.
    distance: Float,
    /// Mesh ID of the hit body, if it's registered in the body registry.
    mesh_id: Option(String),
  )
}

// INITIALIZATION --------------------------------------------------------------

/// Initialize Rapier WASM and create a physics world with the given gravity.
///
/// The world is delivered asynchronously via the provided message constructor
/// once Rapier WASM finishes loading.
///
pub fn init(
  gravity: vec3.Vec3(Float),
  on_ready: fn(PhysicsWorld) -> msg,
) -> Effect(msg) {
  effect.from(fn(dispatch) {
    do_init(vec3.to_tuple(gravity), fn(pw) { dispatch(on_ready(pw)) })
    Nil
  })
}

@external(javascript, "./cacao.ffi.mjs", "init")
fn do_init(
  gravity: #(Float, Float, Float),
  callback: fn(PhysicsWorld) -> Nil,
) -> promise.Promise(Nil)

// STEP ------------------------------------------------------------------------

/// Step the physics simulation.
///
/// On each call:
/// 1. Auto-discovers meshes with `data-physics-body` attributes in the DOM
/// 2. Creates/removes Rapier bodies to match
/// 3. Steps the Rapier simulation
/// 4. Drains collision events
/// 5. Syncs body transforms back to Three.js
///
/// Returns the updated PhysicsWorld with fresh collision events.
///
pub fn step(pw: PhysicsWorld) -> PhysicsWorld {
  do_step(pw)
}

@external(javascript, "./cacao.ffi.mjs", "step")
fn do_step(pw: PhysicsWorld) -> PhysicsWorld

// BODY RESOLUTION -------------------------------------------------------------

/// Resolve a mesh ID to a RigidBody from the PhysicsWorld.
///
/// Used internally by force, body, collision, and raycast functions.
///
@external(javascript, "./cacao.ffi.mjs", "resolveBody")
pub fn resolve_body(
  pw: PhysicsWorld,
  mesh_id: String,
) -> Result(estoque.RigidBody, Nil)

/// Resolve a collider handle to a mesh ID.
///
@external(javascript, "./cacao.ffi.mjs", "resolveColliderToMesh")
pub fn resolve_collider_to_mesh(
  pw: PhysicsWorld,
  collider_handle: Int,
) -> Result(String, Nil)

// HELPERS ---------------------------------------------------------------------

/// Run a function on the body identified by `mesh_id`, inside a Lustre effect.
///
/// If the body cannot be found (invalid mesh ID or not yet registered),
/// the function is silently skipped.
///
fn with_body(
  pw: PhysicsWorld,
  mesh_id: String,
  f: fn(estoque.RigidBody) -> Nil,
) -> Effect(msg) {
  effect.from(fn(_dispatch) {
    case resolve_body(pw, mesh_id) {
      Ok(body) -> f(body)
      Error(_) -> Nil
    }
  })
}

// VIEW ATTRIBUTES -------------------------------------------------------------

/// Mark a mesh as a physics body with the given type.
///
/// Produces: `data-physics-body="dynamic|fixed|kinematic-position|kinematic-velocity"`
///
pub fn body_type(bt: BodyType) -> Attribute(msg) {
  let value = case bt {
    Dynamic -> "dynamic"
    Fixed -> "fixed"
    KinematicPosition -> "kinematic-position"
    KinematicVelocity -> "kinematic-velocity"
  }
  attribute.attribute("data-physics-body", value)
}

/// Set the collider shape for a physics body.
///
/// Produces: `data-physics-collider="cuboid:hx,hy,hz"` or `"ball:r"` etc.
///
pub fn collider(shape: ColliderShape) -> Attribute(msg) {
  let value = case shape {
    Cuboid(hx, hy, hz) ->
      "cuboid:"
      <> float.to_string(hx)
      <> ","
      <> float.to_string(hy)
      <> ","
      <> float.to_string(hz)
    Ball(radius) -> "ball:" <> float.to_string(radius)
    Capsule(half_height, radius) ->
      "capsule:"
      <> float.to_string(half_height)
      <> ","
      <> float.to_string(radius)
    Cylinder(half_height, radius) ->
      "cylinder:"
      <> float.to_string(half_height)
      <> ","
      <> float.to_string(radius)
    Cone(half_height, radius) ->
      "cone:" <> float.to_string(half_height) <> "," <> float.to_string(radius)
  }
  attribute.attribute("data-physics-collider", value)
}

/// Set the friction coefficient.
pub fn friction(value: Float) -> Attribute(msg) {
  attribute.attribute("data-physics-friction", float.to_string(value))
}

/// Set the restitution (bounciness).
pub fn restitution(value: Float) -> Attribute(msg) {
  attribute.attribute("data-physics-restitution", float.to_string(value))
}

/// Set the density.
pub fn density(value: Float) -> Attribute(msg) {
  attribute.attribute("data-physics-density", float.to_string(value))
}

/// Mark as a sensor (trigger volume, no physical collision).
pub fn sensor(is_sensor: Bool) -> Attribute(msg) {
  attribute.attribute("data-physics-sensor", bool.to_string(is_sensor))
}

/// Enable continuous collision detection to prevent tunneling.
pub fn ccd(enabled: Bool) -> Attribute(msg) {
  attribute.attribute("data-physics-ccd", bool.to_string(enabled))
}

/// Set linear damping (slows down linear velocity over time).
pub fn linear_damping(value: Float) -> Attribute(msg) {
  attribute.attribute("data-physics-linear-damping", float.to_string(value))
}

/// Set angular damping (slows down angular velocity over time).
pub fn angular_damping(value: Float) -> Attribute(msg) {
  attribute.attribute("data-physics-angular-damping", float.to_string(value))
}

/// Lock rotations around specific axes.
///
/// `True` means that axis is locked (cannot rotate around it).
///
pub fn lock_rotations(x x: Bool, y y: Bool, z z: Bool) -> Attribute(msg) {
  attribute.attribute(
    "data-physics-lock-rotations",
    bool.to_string(x) <> "," <> bool.to_string(y) <> "," <> bool.to_string(z),
  )
}

/// Set collision groups for filtering.
///
/// `membership`: Which groups this body belongs to (bitmask).
/// `filter`: Which groups this body can collide with (bitmask).
///
pub fn collision_group(membership m: Int, filter f: Int) -> Attribute(msg) {
  attribute.attribute(
    "data-physics-collision-group",
    int.to_string(m) <> "," <> int.to_string(f),
  )
}

// FORCES & IMPULSES -----------------------------------------------------------

/// Apply a force to a body at its center of mass.
///
/// Forces accumulate and are applied during the next physics step.
/// Use for continuous forces like thrust, wind, or movement.
///
pub fn apply_force(
  pw: PhysicsWorld,
  mesh_id: String,
  force: vec3.Vec3(Float),
) -> Effect(msg) {
  with_body(pw, mesh_id, estoque.apply_force(_, force.x, force.y, force.z))
}

/// Apply an impulse to a body at its center of mass.
///
/// Impulses cause instant velocity changes. Use for one-time effects
/// like jumps, explosions, or knockback.
///
pub fn apply_impulse(
  pw: PhysicsWorld,
  mesh_id: String,
  impulse: vec3.Vec3(Float),
) -> Effect(msg) {
  with_body(pw, mesh_id, estoque.apply_impulse(
    _,
    impulse.x,
    impulse.y,
    impulse.z,
  ))
}

/// Apply a torque (rotational force) to a body.
///
/// Torque causes angular acceleration. Use for continuous spinning forces.
///
pub fn apply_torque(
  pw: PhysicsWorld,
  mesh_id: String,
  torque: vec3.Vec3(Float),
) -> Effect(msg) {
  with_body(pw, mesh_id, estoque.apply_torque(_, torque.x, torque.y, torque.z))
}

/// Apply a torque impulse (instant angular velocity change) to a body.
///
pub fn apply_torque_impulse(
  pw: PhysicsWorld,
  mesh_id: String,
  torque_impulse: vec3.Vec3(Float),
) -> Effect(msg) {
  with_body(pw, mesh_id, estoque.apply_torque_impulse(
    _,
    torque_impulse.x,
    torque_impulse.y,
    torque_impulse.z,
  ))
}

/// Apply a force at a specific world-space point on the body.
///
/// This creates both linear and angular acceleration (torque from offset).
///
pub fn apply_force_at_point(
  pw: PhysicsWorld,
  mesh_id: String,
  force force: vec3.Vec3(Float),
  point point: vec3.Vec3(Float),
) -> Effect(msg) {
  with_body(pw, mesh_id, fn(body) {
    estoque.apply_force_at_point(
      body,
      force.x,
      force.y,
      force.z,
      point.x,
      point.y,
      point.z,
    )
  })
}

/// Apply an impulse at a specific world-space point on the body.
///
pub fn apply_impulse_at_point(
  pw: PhysicsWorld,
  mesh_id: String,
  impulse impulse: vec3.Vec3(Float),
  point point: vec3.Vec3(Float),
) -> Effect(msg) {
  with_body(pw, mesh_id, fn(body) {
    estoque.apply_impulse_at_point(
      body,
      impulse.x,
      impulse.y,
      impulse.z,
      point.x,
      point.y,
      point.z,
    )
  })
}

// VELOCITY CONTROL ------------------------------------------------------------

/// Set the linear velocity of a body directly.
///
/// Overrides the current velocity. Useful for capping speed or
/// implementing tight movement controls.
///
pub fn set_linear_velocity(
  pw: PhysicsWorld,
  mesh_id: String,
  linear_velocity: vec3.Vec3(Float),
) -> Effect(msg) {
  with_body(pw, mesh_id, fn(body) {
    estoque.set_linear_velocity(
      body,
      linear_velocity.x,
      linear_velocity.y,
      linear_velocity.z,
    )
  })
}

/// Set the angular velocity of a body directly.
///
pub fn set_angular_velocity(
  pw: PhysicsWorld,
  mesh_id: String,
  angular_velocity: vec3.Vec3(Float),
) -> Effect(msg) {
  with_body(pw, mesh_id, fn(body) {
    estoque.set_angular_velocity(
      body,
      angular_velocity.x,
      angular_velocity.y,
      angular_velocity.z,
    )
  })
}

// BODY STATE QUERIES ----------------------------------------------------------

/// Get the world-space position of a body.
///
/// Dispatches the position as `#(Float, Float, Float)` to the provided
/// message constructor. Silently skipped if the body is not found.
///
pub fn get_position(
  pw: PhysicsWorld,
  mesh_id: String,
  to_msg: fn(vec3.Vec3(Float)) -> msg,
) -> Effect(msg) {
  effect.from(fn(dispatch) {
    case resolve_body(pw, mesh_id) {
      Ok(body) ->
        body
        |> estoque.get_translation
        |> vec3.from_tuple
        |> to_msg
        |> dispatch
      Error(_) -> Nil
    }
  })
}

/// Get the rotation of a body as a quaternion (x, y, z, w).
///
pub fn get_rotation(
  pw: PhysicsWorld,
  mesh_id: String,
  to_msg: fn(quaternion.Quaternion) -> msg,
) -> Effect(msg) {
  effect.from(fn(dispatch) {
    case resolve_body(pw, mesh_id) {
      Ok(body) ->
        body
        |> estoque.get_rotation
        |> quaternion.from_tuple
        |> to_msg
        |> dispatch
      Error(_) -> Nil
    }
  })
}

/// Get the linear velocity of a body.
///
pub fn get_linear_velocity(
  pw: PhysicsWorld,
  mesh_id: String,
  to_msg: fn(vec3.Vec3(Float)) -> msg,
) -> Effect(msg) {
  effect.from(fn(dispatch) {
    case resolve_body(pw, mesh_id) {
      Ok(body) ->
        body
        |> estoque.get_linear_velocity
        |> vec3.from_tuple
        |> to_msg
        |> dispatch
      Error(_) -> Nil
    }
  })
}

/// Get the angular velocity of a body.
///
pub fn get_angular_velocity(
  pw: PhysicsWorld,
  mesh_id: String,
  to_msg: fn(vec3.Vec3(Float)) -> msg,
) -> Effect(msg) {
  effect.from(fn(dispatch) {
    case resolve_body(pw, mesh_id) {
      Ok(body) ->
        body
        |> estoque.get_angular_velocity
        |> vec3.from_tuple
        |> to_msg
        |> dispatch
      Error(_) -> Nil
    }
  })
}

/// Check if a body is sleeping (not being simulated).
///
pub fn is_sleeping(
  pw: PhysicsWorld,
  mesh_id: String,
  to_msg: fn(Bool) -> msg,
) -> Effect(msg) {
  effect.from(fn(dispatch) {
    case resolve_body(pw, mesh_id) {
      Ok(body) ->
        body
        |> estoque.is_sleeping
        |> to_msg
        |> dispatch
      Error(_) -> Nil
    }
  })
}

// BODY STATE WRITES -----------------------------------------------------------

/// Teleport a body to an exact world-space position.
///
/// For dynamic bodies, this bypasses physics — prefer forces/impulses
/// for realistic movement.
///
pub fn teleport(
  pw: PhysicsWorld,
  mesh_id: String,
  position: vec3.Vec3(Float),
) -> Effect(msg) {
  with_body(pw, mesh_id, fn(body) {
    estoque.set_translation(body, position.x, position.y, position.z)
  })
}

/// Set the rotation of a body as a quaternion (x, y, z, w).
///
pub fn set_rotation(
  pw: PhysicsWorld,
  mesh_id: String,
  rotation: quaternion.Quaternion,
) -> Effect(msg) {
  with_body(pw, mesh_id, estoque.set_rotation(
    _,
    rotation.x,
    rotation.y,
    rotation.z,
    rotation.w,
  ))
}

/// Wake up a sleeping body so it resumes simulation.
///
pub fn wake_up(pw: PhysicsWorld, mesh_id: String) -> Effect(msg) {
  with_body(pw, mesh_id, estoque.wake_up)
}

/// Put a body to sleep, pausing its simulation.
///
/// Sleeping bodies skip physics updates until woken by a collision or
/// explicit `wake_up` call.
///
pub fn sleep(pw: PhysicsWorld, mesh_id: String) -> Effect(msg) {
  with_body(pw, mesh_id, estoque.sleep)
}

// COLLISION HELPERS -----------------------------------------------------------

/// Get collision events involving a specific mesh.
///
/// Convenience wrapper that filters collision events from the last step.
///
pub fn get_collisions_for(
  pw: PhysicsWorld,
  mesh_id: String,
) -> List(CollisionEvent) {
  pw.collision_events
  |> list.filter(involves(_, mesh_id))
}

fn involves(info: CollisionEvent, mesh_id: String) -> Bool {
  info.mesh_id_a == Some(mesh_id) || info.mesh_id_b == Some(mesh_id)
}

// RAYCASTING ------------------------------------------------------------------

/// Cast a ray into the physics world and return the closest hit.
///
/// The ray starts at `origin` and travels in `direction` up to
/// `max_distance` units.
///
///
pub fn cast_ray(
  pw: PhysicsWorld,
  to_msg: fn(Result(RayHit, Nil)) -> msg,
  origin origin: vec3.Vec3(Float),
  direction direction: vec3.Vec3(Float),
  max_distance max_distance: Float,
) -> Effect(msg) {
  effect.from(fn(dispatch) {
    let result =
      do_cast_ray(
        pw,
        origin.x,
        origin.y,
        origin.z,
        direction.x,
        direction.y,
        direction.z,
        max_distance,
      )
    dispatch(to_msg(result))
  })
}

@external(javascript, "./cacao.ffi.mjs", "castRay")
fn do_cast_ray(
  pw: PhysicsWorld,
  origin_x: Float,
  origin_y: Float,
  origin_z: Float,
  dir_x: Float,
  dir_y: Float,
  dir_z: Float,
  max_distance: Float,
) -> Result(RayHit, Nil)
