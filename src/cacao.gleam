import gleam/bool
import gleam/dict
import gleam/float
import gleam/int
import gleam/json
import gleam/list
import gleam/option.{type Option}
import gleam/result
import gleam/string

import lustre/attribute.{type Attribute}
import lustre/effect.{type Effect}

import estoque
import quaternion
import savoiardi.{type Object3D}
import tiramisu/extension
import vec/vec3

pub type CacaoApp

pub type World =
  estoque.World

pub type CollisionEventType {
  CollisionStarted
  CollisionStopped
}

pub type BodyType {
  Dynamic
  Fixed
  KinematicPosition
  KinematicVelocity
}

pub type CollisionShape {
  Cuboid(Float, Float, Float)
  Ball(Float)
  Capsule(Float, Float)
  Cylinder(Float, Float)
  Cone(Float, Float)
}

pub type PhysicsWorld {
  PhysicsWorld(
    world: World,
    app: CacaoApp,
    collision_events: List(CollisionEvent),
  )
}

pub type CollisionEvent {
  CollisionEvent(
    mesh_id_a: Option(String),
    mesh_id_b: Option(String),
    collision_type: CollisionEventType,
    is_sensor: Bool,
  )
}

pub type RayHit {
  RayHit(point: vec3.Vec3(Float), distance: Float, mesh_id: Option(String))
}

type BodyConfig {
  BodyConfig(
    body_type: estoque.BodyType,
    shape: estoque.ColliderShape,
    friction: Float,
    restitution: Float,
    density: Float,
    is_sensor: Bool,
    ccd_enabled: Bool,
    linear_damping: Float,
    angular_damping: Float,
    lock_x: Bool,
    lock_y: Bool,
    lock_z: Bool,
    collision_group: Option(#(Int, Int)),
  )
}

pub fn init(
  gravity: vec3.Vec3(Float),
  cacao_app: CacaoApp,
  on_ready to_msg: fn(PhysicsWorld) -> msg,
) -> Effect(msg) {
  use dispatch <- effect.from
  use world <- do_init_world(#(gravity.x, gravity.y, gravity.z))
  let physics_world = PhysicsWorld(world, cacao_app, [])
  do_set_world(cacao_app, world)
  physics_world |> to_msg |> dispatch
}

@external(javascript, "./cacao.ffi.mjs", "newWorldHolder")
pub fn app() -> CacaoApp

@external(javascript, "./cacao.ffi.mjs", "doInitWorld")
fn do_init_world(
  gravity: #(Float, Float, Float),
  callback: fn(World) -> Nil,
) -> Nil

pub fn extension(app: CacaoApp) -> extension.Extension {
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
        "physics-lock-rotation-x",
        "physics-lock-rotation-y",
        "physics-lock-rotation-z",
        "physics-collision-group",
      ],
      on_create: fn(_, id, object, attrs) {
        dict.get(attrs, "physics-body")
        |> result.map(fn(body_type_str) {
          let config = parse_body_config(body_type_str, attrs)
          let create_fn = fn(world: World, mesh_id: String, obj: Object3D) -> Nil {
            build_body(app, world, mesh_id, obj, config)
          }
          do_queue_body_create(app, id, object, create_fn)
        })
        |> result.unwrap(Nil)
      },
      on_update: fn(_, id, object, attrs) {
        dict.get(attrs, "physics-body")
        |> result.map_error(fn(_) { handle_remove(app, id) })
        |> result.map(fn(body_type_str) {
          use <- bool.guard(do_body_exists(app, id), Nil)
          let config = parse_body_config(body_type_str, attrs)
          let create_fn = fn(world: World, mesh_id: String, obj: Object3D) -> Nil {
            build_body(app, world, mesh_id, obj, config)
          }
          do_queue_body_create(app, id, object, create_fn)
        })
        |> result.unwrap(Nil)
      },
      on_remove: fn(_, id) { handle_remove(app, id) },
      on_object_resolved: fn(_, id, object) {
        do_on_object_resolved(app, id, object)
      },
    ),
  )
}

fn build_body(
  app: CacaoApp,
  world: World,
  id: String,
  obj: Object3D,
  config: BodyConfig,
) -> Nil {
  let pos = savoiardi.get_object_position(obj)
  let rot = savoiardi.get_object_quaternion(obj)

  let body =
    estoque.rigid_body(config.body_type)
    |> estoque.translation(pos)
    |> estoque.rotation(rot)
    |> estoque.linear_damping(config.linear_damping)
    |> estoque.angular_damping(config.angular_damping)
    |> estoque.continous_collision_detection(config.ccd_enabled)
    |> estoque.enabled_rotations(!config.lock_x, !config.lock_y, !config.lock_z)
    |> estoque.new_rigidbody(world)

  // Build the collider
  let collider =
    estoque.collider(config.shape)
    |> estoque.friction(config.friction)
    |> estoque.restitution(config.restitution)
    |> estoque.density(config.density)
    |> estoque.sensor(config.is_sensor)
    // TODO: This should be optional
    |> estoque.set_collider_builder_active_events(True)
    |> apply_collision_groups(config.collision_group)
    |> estoque.new_collider(world, body)
  let collider_int_id = estoque.collider_id(collider)

  do_register_body(app, id, body, collider, collider_int_id, obj)
}

fn handle_remove(app: CacaoApp, id: String) -> Nil {
  case do_get_holder_world(app) {
    Ok(world) ->
      case do_get_body(app, id) {
        Ok(body) -> estoque.remove_rigid_body(world, body)
        Error(_) -> Nil
      }
    Error(Nil) -> Nil
  }
  // Compute the collider integer key so the registry map can be cleaned
  let collider_int_id =
    do_get_collider(app, id)
    |> result.map(estoque.collider_id)
    |> result.unwrap(-1)
  do_cleanup_body(app, id, collider_int_id)
}

pub fn step(physics_world: PhysicsWorld) -> PhysicsWorld {
  let PhysicsWorld(world, app, _) = physics_world
  estoque.step(world)
  let events = drain_collision_events(world, app)
  sync_transforms(app)
  PhysicsWorld(world, app, events)
}

// Convert raw estoque collision events to cacao CollisionEvent records,
// resolving collider objects to mesh IDs via the app's registry.
fn drain_collision_events(world: World, app: CacaoApp) -> List(CollisionEvent) {
  use event <- list.map(estoque.drain_collision_events(world))
  let mesh_id_a =
    do_get_collider_mesh_id(app, estoque.collider_id(event.collider_a))
    |> option.from_result
  let mesh_id_b =
    do_get_collider_mesh_id(app, estoque.collider_id(event.collider_b))
    |> option.from_result
  let collision_type = map_collision_type(event.event_type)
  CollisionEvent(
    mesh_id_a:,
    mesh_id_b:,
    collision_type:,
    is_sensor: event.is_sensor,
  )
}

fn sync_transforms(app: CacaoApp) -> Nil {
  use id <- list.each(do_get_all_body_ids(app))
  use body <- result.map(do_get_body(app, id))
  let position = estoque.get_translation(body)
  let rotation = estoque.get_rotation(body)
  use object <- result.map(do_get_object_ref(app, id))
  savoiardi.set_object_position(object, position)
  savoiardi.set_object_quaternion(object, rotation)
}

pub fn resolve_body(
  physics_world: PhysicsWorld,
  mesh_id: String,
) -> Result(estoque.RigidBody, Nil) {
  do_get_body(physics_world.app, mesh_id)
}

pub fn resolve_collider_to_mesh(
  physics_world: PhysicsWorld,
  collider: estoque.Collider,
) -> Result(String, Nil) {
  do_get_collider_mesh_id(physics_world.app, estoque.collider_id(collider))
}

fn with_body(
  physics_world: PhysicsWorld,
  mesh_id: String,
  run: fn(estoque.RigidBody) -> Nil,
) -> Effect(msg) {
  use _ <- effect.from
  case resolve_body(physics_world, mesh_id) {
    Ok(body) -> run(body)
    Error(_) -> Nil
  }
}

pub fn body_type(body_type_value: BodyType) -> Attribute(msg) {
  let value = case body_type_value {
    Dynamic -> "dynamic"
    Fixed -> "fixed"
    KinematicPosition -> "kinematic-position"
    KinematicVelocity -> "kinematic-velocity"
  }
  attribute.attribute("physics-body", value)
}

pub fn collider(shape: CollisionShape) -> Attribute(msg) {
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

pub fn friction(value: Float) -> Attribute(msg) {
  attribute.attribute("physics-friction", float.to_string(value))
}

pub fn restitution(value: Float) -> Attribute(msg) {
  attribute.attribute("physics-restitution", float.to_string(value))
}

pub fn density(value: Float) -> Attribute(msg) {
  attribute.attribute("physics-density", float.to_string(value))
}

pub fn sensor(is_sensor: Bool) -> Attribute(msg) {
  attribute.attribute("physics-sensor", bool.to_string(is_sensor))
}

pub fn continuous_collision_detection(enabled: Bool) -> Attribute(msg) {
  attribute.attribute("physics-ccd", bool.to_string(enabled))
}

pub fn linear_damping(value: Float) -> Attribute(msg) {
  attribute.attribute("physics-linear-damping", float.to_string(value))
}

pub fn angular_damping(value: Float) -> Attribute(msg) {
  attribute.attribute("physics-angular-damping", float.to_string(value))
}

pub fn lock_x(bool: Bool) -> Attribute(msg) {
  case bool {
    True -> attribute.attribute("physics-lock-rotation-x", "")
    False -> attribute.property("physics-lock-rotation-x", json.bool(False))
  }
}

pub fn lock_y(bool: Bool) -> Attribute(msg) {
  case bool {
    True -> attribute.attribute("physics-lock-rotation-y", "")
    False -> attribute.property("physics-lock-rotation-y", json.bool(False))
  }
}

pub fn lock_z(bool: Bool) -> Attribute(msg) {
  case bool {
    True -> attribute.attribute("physics-lock-rotation-z", "")
    False -> attribute.property("physics-lock-rotation-z", json.bool(False))
  }
}

pub fn collision_group(
  membership membership: Int,
  filter filter: Int,
) -> Attribute(msg) {
  attribute.attribute(
    "physics-collision-group",
    int.to_string(membership) <> "," <> int.to_string(filter),
  )
}

pub fn apply_force(
  physics_world: PhysicsWorld,
  mesh_id: String,
  force: vec3.Vec3(Float),
) -> Effect(msg) {
  with_body(physics_world, mesh_id, estoque.apply_force(_, force))
}

pub fn apply_impulse(
  physics_world: PhysicsWorld,
  mesh_id: String,
  impulse: vec3.Vec3(Float),
) -> Effect(msg) {
  estoque.apply_impulse(_, impulse)
  |> with_body(physics_world, mesh_id, _)
}

pub fn apply_torque(
  physics_world: PhysicsWorld,
  mesh_id: String,
  torque: vec3.Vec3(Float),
) -> Effect(msg) {
  estoque.apply_torque(_, torque)
  |> with_body(physics_world, mesh_id, _)
}

pub fn apply_torque_impulse(
  physics_world: PhysicsWorld,
  mesh_id: String,
  torque_impulse: vec3.Vec3(Float),
) -> Effect(msg) {
  estoque.apply_torque_impulse(_, torque_impulse)
  |> with_body(physics_world, mesh_id, _)
}

pub fn apply_force_at_point(
  physics_world: PhysicsWorld,
  mesh_id: String,
  force force: vec3.Vec3(Float),
  point point: vec3.Vec3(Float),
) -> Effect(msg) {
  estoque.apply_force_at_point(_, force, point)
  |> with_body(physics_world, mesh_id, _)
}

pub fn apply_impulse_at_point(
  physics_world: PhysicsWorld,
  mesh_id: String,
  impulse impulse: vec3.Vec3(Float),
  point point: vec3.Vec3(Float),
) -> Effect(msg) {
  estoque.apply_impulse_at_point(_, impulse, point)
  |> with_body(physics_world, mesh_id, _)
}

pub fn set_linear_velocity(
  physics_world: PhysicsWorld,
  mesh_id: String,
  linear_velocity: vec3.Vec3(Float),
) -> Effect(msg) {
  estoque.set_linear_velocity(_, linear_velocity)
  |> with_body(physics_world, mesh_id, _)
}

pub fn set_angular_velocity(
  physics_world: PhysicsWorld,
  mesh_id: String,
  angular_velocity: vec3.Vec3(Float),
) -> Effect(msg) {
  estoque.set_angular_velocity(_, angular_velocity)
  |> with_body(physics_world, mesh_id, _)
}

pub fn get_position(
  physics_world: PhysicsWorld,
  mesh_id: String,
  to_msg: fn(vec3.Vec3(Float)) -> msg,
) -> Effect(msg) {
  use dispatch <- effect.from
  case resolve_body(physics_world, mesh_id) {
    Ok(body) ->
      body
      |> estoque.get_translation
      |> to_msg
      |> dispatch
    Error(_) -> Nil
  }
}

pub fn get_rotation(
  physics_world: PhysicsWorld,
  mesh_id: String,
  to_msg: fn(quaternion.Quaternion) -> msg,
) -> Effect(msg) {
  use dispatch <- effect.from
  case resolve_body(physics_world, mesh_id) {
    Ok(body) ->
      body
      |> estoque.get_rotation
      |> to_msg
      |> dispatch
    Error(_) -> Nil
  }
}

/// Get the linear velocity of a body.
///
pub fn get_linear_velocity(
  physics_world: PhysicsWorld,
  mesh_id: String,
  to_msg: fn(vec3.Vec3(Float)) -> msg,
) -> Effect(msg) {
  use dispatch <- effect.from
  case resolve_body(physics_world, mesh_id) {
    Ok(body) ->
      body
      |> estoque.get_linear_velocity
      |> to_msg
      |> dispatch
    Error(_) -> Nil
  }
}

/// Get the angular velocity of a body.
///
pub fn get_angular_velocity(
  physics_world: PhysicsWorld,
  mesh_id: String,
  to_msg: fn(vec3.Vec3(Float)) -> msg,
) -> Effect(msg) {
  use dispatch <- effect.from
  case resolve_body(physics_world, mesh_id) {
    Ok(body) ->
      body
      |> estoque.get_angular_velocity
      |> to_msg
      |> dispatch
    Error(_) -> Nil
  }
}

/// Check if a body is sleeping (not being simulated).
///
pub fn is_sleeping(
  physics_world: PhysicsWorld,
  mesh_id: String,
  to_msg: fn(Bool) -> msg,
) -> Effect(msg) {
  use dispatch <- effect.from
  case resolve_body(physics_world, mesh_id) {
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
pub fn teleport(
  physics_world: PhysicsWorld,
  mesh_id: String,
  position: vec3.Vec3(Float),
) -> Effect(msg) {
  estoque.set_translation(_, position)
  |> with_body(physics_world, mesh_id, _)
}

/// Set the rotation of a body as a quaternion.
///
pub fn set_rotation(
  physics_world: PhysicsWorld,
  mesh_id: String,
  rotation: quaternion.Quaternion,
) -> Effect(msg) {
  estoque.set_rotation(_, rotation)
  |> with_body(physics_world, mesh_id, _)
}

/// Wake up a sleeping body so it resumes simulation.
///
pub fn wake_up(physics_world: PhysicsWorld, mesh_id: String) -> Effect(msg) {
  with_body(physics_world, mesh_id, estoque.wake_up)
}

/// Put a body to sleep, pausing its simulation.
///
pub fn sleep(physics_world: PhysicsWorld, mesh_id: String) -> Effect(msg) {
  with_body(physics_world, mesh_id, estoque.sleep)
}

// COLLISION HELPERS -----------------------------------------------------------

/// Get collision events involving a specific mesh.
///
pub fn get_collisions_for(
  physics_world: PhysicsWorld,
  mesh_id: String,
) -> List(CollisionEvent) {
  physics_world.collision_events
  |> list.filter(involves(_, mesh_id))
}

fn involves(info: CollisionEvent, mesh_id: String) -> Bool {
  info.mesh_id_a == option.Some(mesh_id)
  || info.mesh_id_b == option.Some(mesh_id)
}

// RAYCASTING ------------------------------------------------------------------

/// Cast a ray into the physics world and return the closest hit.
///
/// The ray starts at `origin` and travels in `direction` up to
/// `max_distance` units.
///
pub fn cast_ray(
  physics_world: PhysicsWorld,
  to_msg: fn(Result(RayHit, Nil)) -> msg,
  origin origin: vec3.Vec3(Float),
  direction direction: vec3.Vec3(Float),
  max_distance max_distance: Float,
) -> Effect(msg) {
  use dispatch <- effect.from
  do_cast_ray(physics_world, origin, direction, max_distance)
  |> to_msg
  |> dispatch
}

fn do_cast_ray(
  physics_world: PhysicsWorld,
  origin: vec3.Vec3(Float),
  direction: vec3.Vec3(Float),
  max_distance: Float,
) -> Result(RayHit, Nil) {
  case
    estoque.cast_ray(
      physics_world.world,
      origin,
      direction,
      max_distance,
      False,
    )
  {
    Error(_) -> Error(Nil)
    Ok(hit) -> {
      // We should return 
      let mesh_id =
        option.from_result(do_get_collider_mesh_id(
          physics_world.app,
          estoque.collider_id(hit.collider),
        ))
      Ok(RayHit(hit.point, hit.time_of_impact, mesh_id))
    }
  }
}

@external(javascript, "./cacao.ffi.mjs", "setWorld")
fn do_set_world(app: CacaoApp, world: World) -> Nil

@external(javascript, "./cacao.ffi.mjs", "queueBodyCreate")
fn do_queue_body_create(
  app: CacaoApp,
  id: String,
  object: Option(Object3D),
  create_fn: fn(World, String, Object3D) -> Nil,
) -> Nil

@external(javascript, "./cacao.ffi.mjs", "onObjectResolved")
fn do_on_object_resolved(app: CacaoApp, id: String, object: Object3D) -> Nil

@external(javascript, "./cacao.ffi.mjs", "getHolderWorld")
fn do_get_holder_world(app: CacaoApp) -> Result(World, Nil)

@external(javascript, "./cacao.ffi.mjs", "registerBody")
fn do_register_body(
  app: CacaoApp,
  id: String,
  body: estoque.RigidBody,
  collider: estoque.Collider,
  collider_id: Int,
  obj: Object3D,
) -> Nil

@external(javascript, "./cacao.ffi.mjs", "cleanupBody")
fn do_cleanup_body(app: CacaoApp, id: String, collider_id: Int) -> Nil

@external(javascript, "./cacao.ffi.mjs", "bodyExists")
fn do_body_exists(app: CacaoApp, id: String) -> Bool

@external(javascript, "./cacao.ffi.mjs", "getBody")
fn do_get_body(app: CacaoApp, id: String) -> Result(estoque.RigidBody, Nil)

@external(javascript, "./cacao.ffi.mjs", "getCollider")
fn do_get_collider(app: CacaoApp, id: String) -> Result(estoque.Collider, Nil)

@external(javascript, "./cacao.ffi.mjs", "getColliderMeshId")
fn do_get_collider_mesh_id(
  app: CacaoApp,
  collider_id: Int,
) -> Result(String, Nil)

@external(javascript, "./cacao.ffi.mjs", "getAllBodyIds")
fn do_get_all_body_ids(app: CacaoApp) -> List(String)

@external(javascript, "./cacao.ffi.mjs", "getObjectRef")
fn do_get_object_ref(app: CacaoApp, id: String) -> Result(Object3D, Nil)

fn apply_collision_groups(
  builder: estoque.ColliderBuilder,
  collision_group: Option(#(Int, Int)),
) -> estoque.ColliderBuilder {
  case collision_group {
    option.None -> builder
    option.Some(#(membership, filter)) ->
      estoque.collision_groups(
        builder,
        estoque.new_collision_group(membership, filter),
      )
  }
}

/// Parse body config from the attribute string and the full attrs dict.
fn parse_body_config(
  body_type_str: String,
  attrs: dict.Dict(String, String),
) -> BodyConfig {
  let body_type = case body_type_str {
    "fixed" -> estoque.Fixed
    "kinematic-position" -> estoque.KinematicPosition
    "kinematic-velocity" -> estoque.KinematicVelocity
    _ -> estoque.Dynamic
  }

  let shape =
    dict.get(attrs, "physics-collider")
    |> result.unwrap("cuboid:0.5,0.5,0.5")
    |> parse_collider_shape

  let lock_x =
    dict.get(attrs, "physics-lock-rotation-x")
    |> result.map(fn(_) { True })
    |> result.unwrap(False)
  let lock_y =
    dict.get(attrs, "physics-lock-rotation-y")
    |> result.map(fn(_) { True })
    |> result.unwrap(False)
  let lock_z =
    dict.get(attrs, "physics-lock-rotation-z")
    |> result.map(fn(_) { True })
    |> result.unwrap(False)

  let collision_group =
    dict.get(attrs, "physics-collision-group")
    |> result.try(parse_collision_group)
    |> option.from_result

  BodyConfig(
    body_type:,
    shape:,
    friction: dict.get(attrs, "physics-friction")
      |> result.try(float.parse)
      |> result.unwrap(0.5),
    restitution: dict.get(attrs, "physics-restitution")
      |> result.try(float.parse)
      |> result.unwrap(0.0),
    density: dict.get(attrs, "physics-density")
      |> result.try(float.parse)
      |> result.unwrap(1.0),
    is_sensor: dict.get(attrs, "physics-sensor")
    |> result.unwrap("False")
      == "True",
    ccd_enabled: dict.get(attrs, "physics-ccd")
    |> result.unwrap("False")
      == "True",
    linear_damping: dict.get(attrs, "physics-linear-damping")
      |> result.try(float.parse)
      |> result.unwrap(0.0),
    angular_damping: dict.get(attrs, "physics-angular-damping")
      |> result.try(float.parse)
      |> result.unwrap(0.0),
    lock_x:,
    lock_y:,
    lock_z:,
    collision_group:,
  )
}

/// Parse a collider string like `"cuboid:0.5,0.5,0.5"` or `"ball:0.5"`.
fn parse_collider_shape(s: String) -> estoque.ColliderShape {
  case string.split(s, ":") {
    [kind, params] ->
      case string.split(params, ",") {
        [p0] -> {
          let p0 = float.parse(p0) |> result.unwrap(0.5)
          case kind {
            "ball" -> estoque.Ball(p0)
            _ -> estoque.Cuboid(p0, p0, p0)
          }
        }
        [p0, p1] -> {
          let p0 = float.parse(p0) |> result.unwrap(0.5)
          let p1 = float.parse(p1) |> result.unwrap(0.5)
          case kind {
            "capsule" -> estoque.Capsule(p0, p1)
            "cylinder" -> estoque.Cylinder(p0, p1)
            "cone" -> estoque.Cone(p0, p1)
            _ -> estoque.Cuboid(p0, p1, 0.5)
          }
        }
        [p0, p1, p2, ..] -> {
          let p0 = float.parse(p0) |> result.unwrap(0.5)
          let p1 = float.parse(p1) |> result.unwrap(0.5)
          let p2 = float.parse(p2) |> result.unwrap(0.5)
          estoque.Cuboid(p0, p1, p2)
        }
        _ -> estoque.Cuboid(0.5, 0.5, 0.5)
      }
    _ -> estoque.Cuboid(0.5, 0.5, 0.5)
  }
}

fn parse_collision_group(s: String) -> Result(#(Int, Int), Nil) {
  case string.split(s, ",") {
    [membership_str, filter_str] ->
      case int.parse(membership_str), int.parse(filter_str) {
        Ok(membership), Ok(filter) -> Ok(#(membership, filter))
        _, _ -> Error(Nil)
      }
    _ -> Error(Nil)
  }
}

fn map_collision_type(event: estoque.CollisionEventType) -> CollisionEventType {
  case event {
    estoque.CollisionStarted -> CollisionStarted
    estoque.CollisionStopped -> CollisionStopped
  }
}

pub fn get_translation(body: estoque.RigidBody) -> vec3.Vec3(Float) {
  estoque.get_translation(body)
}
