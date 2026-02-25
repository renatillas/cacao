// Cacao FFI — Physics world management, extension hooks, and raycasting.
//
// Bodies are created/destroyed in response to tiramisu extension lifecycle
// hooks (handleCreate, handleRemove, handleResolved) rather than per-frame
// DOM scanning. Three.js object references are stored in _objectRegistry on
// the world for direct transform sync during step() without DOM access.
//
// The WorldHolder acts as a bridge between the tiramisu extension (registered
// at startup) and the Rapier world (initialized asynchronously). Pending
// configs are queued in the holder until the world is ready.

import { Result$Ok, Result$Error, toList } from "./gleam.mjs";
import {
  Option$Some,
  Option$None,
  Option$isSome,
  Option$Some$0
} from "../gleam_stdlib/gleam/option.mjs";
import { Vec3$Vec3 } from "../vec/vec/vec3.mjs";

// Import Gleam constructors using the 1.13+ API
import {
  PhysicsWorld$PhysicsWorld,
  PhysicsWorld$PhysicsWorld$world,
  CollisionEvent$CollisionEvent,
  CollisionEventType$CollisionStarted,
  CollisionEventType$CollisionStopped,
  RayHit$RayHit,
} from "./cacao.mjs";

import RAPIER from "@dimforge/rapier3d-compat";

// === WORLD HOLDER ===

/**
 * Create a new world holder.
 *
 * The holder bridges the extension hooks (registered at startup) and the
 * Rapier world (initialized asynchronously via init()).
 *
 * _pendingConfigs: meshId → physics config, queued when world or object
 *                  is not yet available.
 * _resolvedObjects: meshId → Object3D, stored when the object resolves
 *                   before the body can be created.
 */
export function newWorldHolder() {
  return {
    world: null,
    _pendingConfigs: new Map(),
    _resolvedObjects: new Map(),
  };
}

/**
 * Connect the initialized Rapier world to the holder.
 *
 * Called by cacao.init() once Rapier WASM loads. Flushes any pending
 * body configs that arrived via extension hooks before init() completed.
 */
export function setWorld(holder, pw) {
  const world = PhysicsWorld$PhysicsWorld$world(pw);

  // Initialize the Object3D reference registry for direct transform sync
  world._objectRegistry = world._objectRegistry || new Map();
  holder.world = world;

  // Flush pending creates: entries where both config and resolved object exist
  for (const [meshId, config] of holder._pendingConfigs) {
    if (world._bodyRegistry.has(meshId)) continue;

    const obj = holder._resolvedObjects.get(meshId);
    if (obj) {
      // Both config and object available — create body now
      createBodyFromConfig(world, meshId, config, obj);
      world._objectRegistry.set(meshId, obj);
      holder._resolvedObjects.delete(meshId);
      holder._pendingConfigs.delete(meshId);
    }
    // If no object yet (async mesh): leave in _pendingConfigs for handleResolved
  }
}

// === INITIALIZATION ===

/**
 * Initialize Rapier WASM and create a physics world.
 * Called from Gleam inside effect.from — callback receives the PhysicsWorld.
 */
export async function init(gravity, callback) {
  await RAPIER.init();

  const [gx, gy, gz] = gravity;
  const world = new RAPIER.World({ x: gx, y: gy, z: gz });

  // Attach registries and event queue to the world object
  world._eventQueue = new RAPIER.EventQueue(true);
  world._bodyRegistry = new Map();
  world._colliderRegistry = new Map();
  world._colliderToMeshId = new Map();
  // Object3D references for direct transform sync (populated via extension hooks)
  world._objectRegistry = new Map();

  const pw = PhysicsWorld$PhysicsWorld(world, toList([]));
  callback(pw);
}

// === STEP ===

/**
 * Step the physics simulation:
 * 1. Step Rapier
 * 2. Drain collision events
 * 3. Sync body transforms to Three.js via stored object references
 *
 * Bodies are tracked via extension hooks — no DOM scanning needed.
 */
export function step(pw) {
  const world = PhysicsWorld$PhysicsWorld$world(pw);

  // Step Rapier simulation
  world.step(world._eventQueue);

  // Drain collision events
  const events = drainEvents(world);

  // Sync transforms from Rapier → Three.js using stored object references
  for (const [meshId, handle] of world._bodyRegistry) {
    const body = world.getRigidBody(handle);
    if (!body) continue;

    // Use stored Object3D reference — avoids DOM lookup per frame
    const obj = world._objectRegistry?.get(meshId);
    if (!obj) continue; // Object not yet resolved (async mesh still loading)

    const t = body.translation();
    const r = body.rotation();
    obj.position.set(t.x, t.y, t.z);
    obj.quaternion.set(r.x, r.y, r.z, r.w);
  }

  return PhysicsWorld$PhysicsWorld(world, events);
}

// === BODY LIFECYCLE (extension hooks) ===

/**
 * Called by the tiramisu on_create hook when a scene node with
 * data-physics-body appears. Physics config is fully parsed on the
 * Gleam side — only raw values arrive here.
 *
 * Creates the Rapier body immediately if the world is ready and the
 * Object3D is available. Otherwise queues for later.
 */
export function handleCreate(
  holder, id, objectOrNone,
  bodyType, colliderType, p0, p1, p2,
  friction, restitution, density,
  isSensor, ccd, linearDamping, angularDamping,
  lockX, lockY, lockZ,
  hasCg, cgMembership, cgFilter
) {
  const config = {
    bodyType,
    collider: { type: colliderType, params: [p0, p1, p2] },
    friction, restitution, density,
    isSensor, ccdEnabled: ccd,
    linearDamping, angularDamping,
    lockRotations: [lockX, lockY, lockZ],
    collisionGroup: hasCg ? [cgMembership, cgFilter] : null,
  };

  const obj = fromGleamOption(objectOrNone);
  const world = holder.world;

  if (world && obj) {
    // Best case: world ready and object available
    if (!world._bodyRegistry.has(id)) {
      createBodyFromConfig(world, id, config, obj);
      world._objectRegistry.set(id, obj);
    }
  } else if (world && !obj) {
    // World ready but mesh is async — queue config for handleResolved
    holder._pendingConfigs.set(id, config);
  } else if (!world && obj) {
    // Object available but world not ready yet — queue for setWorld
    holder._pendingConfigs.set(id, config);
    holder._resolvedObjects.set(id, obj);
  } else {
    // Neither world nor object ready — queue config for both
    holder._pendingConfigs.set(id, config);
  }
}

/**
 * Called by the tiramisu on_remove hook when a scene node is removed.
 * Destroys the Rapier body and cleans up all registries.
 */
export function handleRemove(holder, id) {
  // Clean up pending state regardless of world readiness
  holder._pendingConfigs.delete(id);
  holder._resolvedObjects.delete(id);

  const world = holder.world;
  if (!world || !world._bodyRegistry.has(id)) return;

  const handle = world._bodyRegistry.get(id);
  destroyBody(world, id, handle);
  world._objectRegistry?.delete(id);
}

/**
 * Called by the tiramisu on_object_resolved hook when an async GLB/FBX/OBJ
 * mesh finishes loading. Stores the Object3D reference and creates the
 * pending physics body if the world is ready.
 */
export function handleResolved(holder, id, object) {
  const world = holder.world;

  if (world) {
    // World is ready — store object reference for transform sync
    world._objectRegistry = world._objectRegistry || new Map();
    world._objectRegistry.set(id, object);

    // Create body if config was queued waiting for this object
    if (holder._pendingConfigs.has(id) && !world._bodyRegistry.has(id)) {
      const config = holder._pendingConfigs.get(id);
      holder._pendingConfigs.delete(id);
      createBodyFromConfig(world, id, config, object);
    }
  } else {
    // World not ready — store resolved object so setWorld can use it
    holder._resolvedObjects.set(id, object);
  }
}

/**
 * Check whether a physics body exists for the given mesh ID.
 * Used in on_update to avoid re-creating existing bodies.
 */
export function bodyExists(holder, id) {
  if (!holder.world) return false;
  return holder.world._bodyRegistry.has(id);
}

// === BODY RESOLUTION ===

/**
 * Resolve a mesh ID to a Rapier RigidBody.
 * Returns Result$Ok(body) or Result$Error(undefined).
 */
export function resolveBody(pw, meshId) {
  const world = PhysicsWorld$PhysicsWorld$world(pw);
  const handle = world._bodyRegistry.get(meshId);
  if (handle === undefined) return Result$Error(undefined);

  const body = world.getRigidBody(handle);
  if (!body) return Result$Error(undefined);

  return Result$Ok(body);
}

/**
 * Resolve a collider handle (int) to a mesh ID.
 * Returns Result$Ok(meshId) or Result$Error(undefined).
 */
export function resolveColliderToMesh(pw, colliderHandle) {
  const world = PhysicsWorld$PhysicsWorld$world(pw);
  const meshId = world._colliderToMeshId.get(colliderHandle);
  if (meshId === undefined) return Result$Error(undefined);
  return Result$Ok(meshId);
}

// === RAYCASTING ===

/**
 * Cast a ray and return the closest hit with mesh ID resolved.
 */
export function castRay(pw, originX, originY, originZ, dirX, dirY, dirZ, maxDistance) {
  const world = PhysicsWorld$PhysicsWorld$world(pw);
  const ray = new RAPIER.Ray(
    { x: originX, y: originY, z: originZ },
    { x: dirX, y: dirY, z: dirZ }
  );

  const hit = world.castRay(ray, maxDistance, true);
  if (hit === null || hit === undefined) return Result$Error();

  const toi = hit.timeOfImpact;
  if (toi === null || toi === undefined) return Result$Error();

  // Compute hit point
  const point = ray.pointAt(toi);

  // Compute Euclidean distance from origin to hit point
  const dx = point.x - originX;
  const dy = point.y - originY;
  const dz = point.z - originZ;
  const distance = Math.sqrt(dx * dx + dy * dy + dz * dz);

  // Get collider — hit.collider is already a Collider object from World.castRay
  const collider = hit.collider;
  let normal = { x: 0, y: 1, z: 0 }; // default up
  if (collider) {
    const normalResult = collider.castRayAndGetNormal(ray, toi, true);
    if (normalResult && normalResult.normal) {
      normal = normalResult.normal;
    }
  }

  // Resolve collider → mesh ID using collider handle (integer key)
  let meshId = Option$None();
  if (world._colliderToMeshId && collider) {
    const handle = collider.handle;
    const id = world._colliderToMeshId.get(handle);
    if (id) {
      meshId = Option$Some(id);
    }
  }

  return Result$Ok(RayHit$RayHit(
    Vec3$Vec3(point.x, point.y, point.z),
    Vec3$Vec3(normal.x, normal.y, normal.z),
    distance,
    meshId
  ));
}

// === INTERNAL HELPERS ===

/**
 * Extract a value from a Gleam Option.
 * Returns the contained value for Some, or null for None.
 *
 */
function fromGleamOption(opt) {
  return Option$isSome(opt) ? Option$Some$0(opt) : null;
}

/**
 * Create a Rapier rigid body + collider from a parsed config object.
 * Takes the Three.js object for initial position/rotation.
 * Mutates the world's registry Maps in place.
 */
function createBodyFromConfig(world, meshId, config, obj) {
  // Create body descriptor
  let bodyDesc;
  switch (config.bodyType) {
    case "fixed":
      bodyDesc = RAPIER.RigidBodyDesc.fixed();
      break;
    case "kinematic-position":
      bodyDesc = RAPIER.RigidBodyDesc.kinematicPositionBased();
      break;
    case "kinematic-velocity":
      bodyDesc = RAPIER.RigidBodyDesc.kinematicVelocityBased();
      break;
    default:
      bodyDesc = RAPIER.RigidBodyDesc.dynamic();
  }

  // Set initial position/rotation from the Three.js object
  if (obj) {
    bodyDesc.setTranslation(obj.position.x, obj.position.y, obj.position.z);
    bodyDesc.setRotation(obj.quaternion);
  }

  // Apply body properties
  bodyDesc.setLinearDamping(config.linearDamping);
  bodyDesc.setAngularDamping(config.angularDamping);
  if (config.ccdEnabled) bodyDesc.setCcdEnabled(true);

  // Lock rotations
  if (config.lockRotations[0] || config.lockRotations[1] || config.lockRotations[2]) {
    bodyDesc.enabledRotations(
      !config.lockRotations[0],
      !config.lockRotations[1],
      !config.lockRotations[2]
    );
  }

  const body = world.createRigidBody(bodyDesc);
  const handle = body.handle;
  world._bodyRegistry.set(meshId, handle);

  // Create collider descriptor
  let colliderDesc;
  const p = config.collider.params;
  switch (config.collider.type) {
    case "ball":
      colliderDesc = RAPIER.ColliderDesc.ball(p[0]);
      break;
    case "capsule":
      colliderDesc = RAPIER.ColliderDesc.capsule(p[0], p[1]);
      break;
    case "cylinder":
      colliderDesc = RAPIER.ColliderDesc.cylinder(p[0], p[1]);
      break;
    case "cone":
      colliderDesc = RAPIER.ColliderDesc.cone(p[0], p[1]);
      break;
    default:
      colliderDesc = RAPIER.ColliderDesc.cuboid(p[0], p[1], p[2]);
  }

  // Apply collider properties
  colliderDesc.setFriction(config.friction);
  colliderDesc.setRestitution(config.restitution);
  colliderDesc.setDensity(config.density);
  if (config.isSensor) colliderDesc.setSensor(true);

  // Collision groups
  if (config.collisionGroup) {
    const [membership, filter] = config.collisionGroup;
    const groups = ((membership & 0xffff) << 16) | (filter & 0xffff);
    colliderDesc.setCollisionGroups(groups);
  }

  // Enable active events for collision detection
  colliderDesc.setActiveEvents(1); // COLLISION_EVENTS

  const collider = world.createCollider(colliderDesc, body);
  const colliderHandle = collider.handle;
  world._colliderRegistry.set(meshId, colliderHandle);
  world._colliderToMeshId.set(colliderHandle, meshId);
}

/**
 * Destroy a Rapier body and its collider.
 * Mutates the world's registry Maps in place.
 */
function destroyBody(world, meshId, handle) {
  // Remove collider from colliderToMeshId
  const colliderHandle = world._colliderRegistry.get(meshId);
  if (colliderHandle !== undefined) {
    world._colliderToMeshId.delete(colliderHandle);
  }

  // Remove the body (also removes attached colliders)
  const body = world.getRigidBody(handle);
  if (body) {
    world.removeRigidBody(body);
  }

  world._bodyRegistry.delete(meshId);
  world._colliderRegistry.delete(meshId);
}

/**
 * Drain collision events and resolve collider handles to mesh IDs.
 * Returns a Gleam List of CollisionEvent.
 */
function drainEvents(world) {
  const events = [];

  world._eventQueue.drainCollisionEvents((handle1, handle2, started) => {
    const meshIdA = world._colliderToMeshId.get(handle1);
    const meshIdB = world._colliderToMeshId.get(handle2);

    // Determine if either collider is a sensor
    let isSensor = false;
    const c1 = world.getCollider(handle1);
    const c2 = world.getCollider(handle2);
    if ((c1 && c1.isSensor()) || (c2 && c2.isSensor())) {
      isSensor = true;
    }

    events.push(
      CollisionEvent$CollisionEvent(
        meshIdA != null ? Option$Some(meshIdA) : Option$None(),
        meshIdB != null ? Option$Some(meshIdB) : Option$None(),
        started
          ? CollisionEventType$CollisionStarted()
          : CollisionEventType$CollisionStopped(),
        isSensor
      )
    );
  });

  return toList(events);
}
