import estoque
import gleam/bool
import gleam/dict
import gleam/float
import gleam/int
import gleam/list
import gleam/option.{type Option, Some}
import gleam/result
import gleam/string
import lustre/attribute.{type Attribute}
import lustre/effect.{type Effect}
import quaternion
import savoiardi.{type Object3D}
import tiramisu/extension
import vec/vec3

// CORE TYPES ------------------------------------------------------------------

/// Opaque physics state held in the user's Lustre model.
///
/// Created with `init/3`, updated with `step/1`.
/// Registries (body→handle, collider→mesh, Object3D refs) are stored as JS
/// Maps on the underlying Rapier world object for zero-cost per-frame access.
///
pub type PhysicsWorld {
  PhysicsWorld(world: estoque.World, collision_events: List(CollisionEvent))
}

/// A collision event from the physics simulation.
///
/// Populated during `step/1` and read via `collision_events` or
/// `get_collisions_for/2`.
///
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
    /// Mesh ID of the hit body, if registered in the body registry.
    mesh_id: Option(String),
  )
}

// INITIALIZATION --------------------------------------------------------------

/// Initialize Rapier WASM and create a physics world with the given gravity.
///
/// The world is stored in `holder` so the tiramisu extension hooks can create
/// and remove bodies as the scene changes. The `on_ready` message is dispatched
/// once the world is fully initialized.
///
pub fn init(
  gravity: vec3.Vec3(Float),
  cacao_app: CacaoApp,
  on_ready to_msg: fn(PhysicsWorld) -> msg,
) -> Effect(msg) {
  use dispatch <- effect.from
  use physics_world <- do_init(vec3.to_tuple(gravity))

  do_set_world(cacao_app, physics_world)
  physics_world
  |> to_msg
  |> dispatch
}

@external(javascript, "./cacao.ffi.mjs", "newWorldHolder")
pub fn cacao_app() -> CacaoApp

@external(javascript, "./cacao.ffi.mjs", "init")
fn do_init(
  gravity: #(Float, Float, Float),
  callback: fn(PhysicsWorld) -> Nil,
) -> Nil

// EXTENSION -------------------------------------------------------------------

/// Create the tiramisu extension that drives physics body lifecycle.
///
/// Register it with `tiramisu.register/1`. The same `WorldHolder` must be
/// passed to `init/3` later to connect the hooks to the Rapier world.
///
pub fn extension(holder: CacaoApp) -> extension.Extension {
  extension.AttributeExtension(
    extension.Attribute(
      observed_attributes: [
        "physics-body",
        "physics-collider",
        "physics-friction",
        "physics-restitution",
        "physics-density",
        "physics-sensor",
        "physics-ccd",
        "physics-linear-damping",
        "physics-angular-damping",
        "physics-lock-rotations",
        "physics-collision-group",
      ],
      on_create: fn(_, id, object, attrs) {
        dict.get(attrs, "physics-body")
        |> result.map(create_body(holder, id, object, _, attrs))
        |> result.unwrap(Nil)
      },
      on_update: fn(_, id, object, attrs) {
        dict.get(attrs, "physics-body")
        |> result.map_error(fn(_) { handle_remove(holder, id) })
        |> result.map(fn(body) {
          use <- bool.guard(is_body_id_registered(holder, id), Nil)
          create_body(holder, id, object, body, attrs)
        })
        |> result.unwrap(Nil)
      },
      on_remove: fn(_, id) { handle_remove(holder, id) },
      on_object_resolved: fn(_, id, object) {
        handle_resolve(holder, id, object)
      },
    ),
  )
}

// STEP ------------------------------------------------------------------------

/// Step the physics simulation.
///
/// On each call:
/// 1. Steps the Rapier simulation
/// 2. Drains collision events
/// 3. Syncs body transforms back to Three.js via stored object references
///
/// Physics bodies are managed automatically by the tiramisu extension hooks —
/// no DOM scanning needed.
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
/// Produces: `physics-body="dynamic|fixed|kinematic-position|kinematic-velocity"`
///
pub fn body_type(bt: BodyType) -> Attribute(msg) {
  let value = case bt {
    Dynamic -> "dynamic"
    Fixed -> "fixed"
    KinematicPosition -> "kinematic-position"
    KinematicVelocity -> "kinematic-velocity"
  }
  attribute.attribute("physics-body", value)
}

/// Set the collider shape for a physics body.
///
/// Produces: `physics-collider="cuboid:hx,hy,hz"` or `"ball:r"` etc.
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
  attribute.attribute("physics-collider", value)
}

/// Set the friction coefficient.
pub fn friction(value: Float) -> Attribute(msg) {
  attribute.attribute("physics-friction", float.to_string(value))
}

/// Set the restitution (bounciness).
pub fn restitution(value: Float) -> Attribute(msg) {
  attribute.attribute("physics-restitution", float.to_string(value))
}

/// Set the density.
pub fn density(value: Float) -> Attribute(msg) {
  attribute.attribute("physics-density", float.to_string(value))
}

/// Mark as a sensor (trigger volume, no physical collision).
pub fn sensor(is_sensor: Bool) -> Attribute(msg) {
  attribute.attribute("physics-sensor", bool.to_string(is_sensor))
}

/// Enable continuous collision detection to prevent tunneling.
pub fn ccd(enabled: Bool) -> Attribute(msg) {
  attribute.attribute("physics-ccd", bool.to_string(enabled))
}

/// Set linear damping (slows down linear velocity over time).
pub fn linear_damping(value: Float) -> Attribute(msg) {
  attribute.attribute("physics-linear-damping", float.to_string(value))
}

/// Set angular damping (slows down angular velocity over time).
pub fn angular_damping(value: Float) -> Attribute(msg) {
  attribute.attribute("physics-angular-damping", float.to_string(value))
}

/// Lock rotations around specific axes.
///
/// `True` means that axis is locked (cannot rotate around it).
///
pub fn lock_rotations(x x: Bool, y y: Bool, z z: Bool) -> Attribute(msg) {
  attribute.attribute(
    "physics-lock-rotations",
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
    "physics-collision-group",
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
  estoque.apply_impulse(_, impulse.x, impulse.y, impulse.z)
  |> with_body(pw, mesh_id, _)
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
  estoque.apply_torque(_, torque.x, torque.y, torque.z)
  |> with_body(pw, mesh_id, _)
}

/// Apply a torque impulse (instant angular velocity change) to a body.
///
pub fn apply_torque_impulse(
  pw: PhysicsWorld,
  mesh_id: String,
  torque_impulse: vec3.Vec3(Float),
) -> Effect(msg) {
  estoque.apply_torque_impulse(
    _,
    torque_impulse.x,
    torque_impulse.y,
    torque_impulse.z,
  )
  |> with_body(pw, mesh_id, _)
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
  estoque.apply_force_at_point(
    _,
    force.x,
    force.y,
    force.z,
    point.x,
    point.y,
    point.z,
  )
  |> with_body(pw, mesh_id, _)
}

/// Apply an impulse at a specific world-space point on the body.
///
pub fn apply_impulse_at_point(
  pw: PhysicsWorld,
  mesh_id: String,
  impulse impulse: vec3.Vec3(Float),
  point point: vec3.Vec3(Float),
) -> Effect(msg) {
  estoque.apply_impulse_at_point(
    _,
    impulse.x,
    impulse.y,
    impulse.z,
    point.x,
    point.y,
    point.z,
  )
  |> with_body(pw, mesh_id, _)
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
  estoque.set_linear_velocity(
    _,
    linear_velocity.x,
    linear_velocity.y,
    linear_velocity.z,
  )
  |> with_body(pw, mesh_id, _)
}

/// Set the angular velocity of a body directly.
///
pub fn set_angular_velocity(
  pw: PhysicsWorld,
  mesh_id: String,
  angular_velocity: vec3.Vec3(Float),
) -> Effect(msg) {
  estoque.set_angular_velocity(
    _,
    angular_velocity.x,
    angular_velocity.y,
    angular_velocity.z,
  )
  |> with_body(pw, mesh_id, _)
}

// BODY STATE QUERIES ----------------------------------------------------------

/// Get the world-space position of a body.
///
/// Dispatches the position to the provided message constructor.
/// Silently skipped if the body is not found.
///
pub fn get_position(
  pw: PhysicsWorld,
  mesh_id: String,
  to_msg: fn(vec3.Vec3(Float)) -> msg,
) -> Effect(msg) {
  use dispatch <- effect.from
  case resolve_body(pw, mesh_id) {
    Ok(body) ->
      body
      |> estoque.get_translation
      |> vec3.from_tuple
      |> to_msg
      |> dispatch
    Error(_) -> Nil
  }
}

/// Get the rotation of a body as a quaternion (x, y, z, w).
///
pub fn get_rotation(
  pw: PhysicsWorld,
  mesh_id: String,
  to_msg: fn(quaternion.Quaternion) -> msg,
) -> Effect(msg) {
  use dispatch <- effect.from
  case resolve_body(pw, mesh_id) {
    Ok(body) ->
      body
      |> estoque.get_rotation
      |> quaternion.from_tuple
      |> to_msg
      |> dispatch
    Error(_) -> Nil
  }
}

/// Get the linear velocity of a body.
///
pub fn get_linear_velocity(
  pw: PhysicsWorld,
  mesh_id: String,
  to_msg: fn(vec3.Vec3(Float)) -> msg,
) -> Effect(msg) {
  use dispatch <- effect.from
  case resolve_body(pw, mesh_id) {
    Ok(body) ->
      body
      |> estoque.get_linear_velocity
      |> vec3.from_tuple
      |> to_msg
      |> dispatch
    Error(_) -> Nil
  }
}

/// Get the angular velocity of a body.
///
pub fn get_angular_velocity(
  pw: PhysicsWorld,
  mesh_id: String,
  to_msg: fn(vec3.Vec3(Float)) -> msg,
) -> Effect(msg) {
  use dispatch <- effect.from
  case resolve_body(pw, mesh_id) {
    Ok(body) ->
      body
      |> estoque.get_angular_velocity
      |> vec3.from_tuple
      |> to_msg
      |> dispatch
    Error(_) -> Nil
  }
}

/// Check if a body is sleeping (not being simulated).
///
pub fn is_sleeping(
  pw: PhysicsWorld,
  mesh_id: String,
  to_msg: fn(Bool) -> msg,
) -> Effect(msg) {
  use dispatch <- effect.from
  case resolve_body(pw, mesh_id) {
    Ok(body) ->
      body
      |> estoque.is_sleeping
      |> to_msg
      |> dispatch
    Error(_) -> Nil
  }
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
  estoque.set_translation(_, position.x, position.y, position.z)
  |> with_body(pw, mesh_id, _)
}

/// Set the rotation of a body as a quaternion (x, y, z, w).
///
pub fn set_rotation(
  pw: PhysicsWorld,
  mesh_id: String,
  rotation: quaternion.Quaternion,
) -> Effect(msg) {
  estoque.set_rotation(_, rotation.x, rotation.y, rotation.z, rotation.w)
  |> with_body(pw, mesh_id, _)
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
pub fn cast_ray(
  pw: PhysicsWorld,
  to_msg: fn(Result(RayHit, Nil)) -> msg,
  origin origin: vec3.Vec3(Float),
  direction direction: vec3.Vec3(Float),
  max_distance max_distance: Float,
) -> Effect(msg) {
  use dispatch <- effect.from
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
  |> to_msg
  |> dispatch
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

// WORLD HOLDER ----------------------------------------------------------------

/// Mutable holder that connects the tiramisu extension hooks to the Rapier
/// physics world.
///
/// Create with `world/0`, register via `extension/1`, then pass to `init/3`.
/// Also forwarded to the Lustre app as flags so `init` can call `cacao.init`.
///
pub type CacaoApp

@external(javascript, "./cacao.ffi.mjs", "setWorld")
fn do_set_world(holder: CacaoApp, pw: PhysicsWorld) -> Nil

@external(javascript, "./cacao.ffi.mjs", "handleCreate")
fn do_handle_create(
  holder: CacaoApp,
  id: String,
  object: Option(Object3D),
  body_type: String,
  collider_type: String,
  collider_p0: Float,
  collider_p1: Float,
  collider_p2: Float,
  friction: Float,
  restitution: Float,
  density: Float,
  is_sensor: Bool,
  ccd: Bool,
  linear_damping: Float,
  angular_damping: Float,
  lock_x: Bool,
  lock_y: Bool,
  lock_z: Bool,
  has_collision_group: Bool,
  collision_membership: Int,
  collision_filter: Int,
) -> Nil

@external(javascript, "./cacao.ffi.mjs", "handleRemove")
fn handle_remove(holder: CacaoApp, id: String) -> Nil

@external(javascript, "./cacao.ffi.mjs", "handleResolved")
fn handle_resolve(holder: CacaoApp, id: String, object: Object3D) -> Nil

@external(javascript, "./cacao.ffi.mjs", "bodyExists")
fn is_body_id_registered(holder: CacaoApp, id: String) -> Bool

/// Parse attrs dict and call the FFI body creation function.
fn create_body(
  holder: CacaoApp,
  id: String,
  object: Option(Object3D),
  body_type: String,
  attrs: dict.Dict(String, String),
) -> Nil {
  let collider =
    dict.get(attrs, "physics-collider")
    |> result.unwrap("cuboid:0.5,0.5,0.5")
  let #(collider_type, p0, p1, p2) = parse_collider(collider)

  let #(lock_x, lock_y, lock_z) =
    dict.get(attrs, "physics-lock-rotations")
    |> result.unwrap("False,False,False")
    |> parse_lock_rotations

  let #(has_cg, cg_membership, cg_filter) =
    dict.get(attrs, "physics-collision-group")
    |> option.from_result
    |> parse_collision_group

  do_handle_create(
    holder,
    id,
    object,
    body_type,
    collider_type,
    p0,
    p1,
    p2,
    dict.get(attrs, "physics-friction")
      |> result.try(float.parse)
      |> result.unwrap(0.5),
    dict.get(attrs, "physics-restitution")
      |> result.try(float.parse)
      |> result.unwrap(0.0),
    dict.get(attrs, "physics-density")
      |> result.try(float.parse)
      |> result.unwrap(1.0),
    dict.get(attrs, "physics-sensor")
    |> result.unwrap("False")
      == "True",
    dict.get(attrs, "physics-ccd")
    |> result.unwrap("False")
      == "True",
    dict.get(attrs, "physics-linear-damping")
      |> result.try(float.parse)
      |> result.unwrap(0.0),
    dict.get(attrs, "physics-angular-damping")
      |> result.try(float.parse)
      |> result.unwrap(0.0),
    lock_x,
    lock_y,
    lock_z,
    has_cg,
    cg_membership,
    cg_filter,
  )
}

/// Parse a collider string like "cuboid:0.5,0.5,0.5" or "ball:0.5".
fn parse_collider(s: String) -> #(String, Float, Float, Float) {
  case string.split(s, ":") {
    [kind, params] ->
      case string.split(params, ",") {
        [p0] -> #(kind, float.parse(p0) |> result.unwrap(0.5), 0.0, 0.0)
        [p0, p1] -> #(
          kind,
          float.parse(p0) |> result.unwrap(0.5),
          float.parse(p1) |> result.unwrap(0.5),
          0.0,
        )
        [p0, p1, p2, ..] -> #(
          kind,
          float.parse(p0) |> result.unwrap(0.5),
          float.parse(p1) |> result.unwrap(0.5),
          float.parse(p2) |> result.unwrap(0.5),
        )
        _ -> #("cuboid", 0.5, 0.5, 0.5)
      }
    _ -> #("cuboid", 0.5, 0.5, 0.5)
  }
}

/// Parse a lock-rotations string like "True,False,True".
fn parse_lock_rotations(s: String) -> #(Bool, Bool, Bool) {
  case string.split(s, ",") {
    [x, y, z] -> #(x == "True", y == "True", z == "True")
    _ -> #(False, False, False)
  }
}

/// Parse an optional collision group string like "4,7".
fn parse_collision_group(s: Option(String)) -> #(Bool, Int, Int) {
  case s {
    option.None -> #(False, 0, 0)
    option.Some(str) ->
      case string.split(str, ",") {
        [m, f] -> #(
          True,
          int.parse(m) |> result.unwrap(0),
          int.parse(f) |> result.unwrap(0),
        )
        _ -> #(False, 0, 0)
      }
  }
}
