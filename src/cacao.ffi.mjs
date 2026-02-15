// Cacao FFI — Physics world management, auto-discovery, Three.js sync, and raycasting.
//
// Registries (_bodyRegistry, _colliderRegistry, _colliderToMeshId) are stored
// as JS Maps directly on the Rapier world object for zero-cost per-frame access.
// This mirrors how estoque stores _eventQueue on the world.

import { Result$Ok, Result$Error, toList } from "./gleam.mjs";
import { Option$Some, Option$None, Option$isSome, Option$Some$0 } from "../gleam_stdlib/gleam/option.mjs";
import {
  setPosition,
  setQuaternion,
  getObject,
} from "../tiramisu/tiramisu/internal/runtime.ffi.mjs";

// Import Gleam constructors using the 1.13+ API
import {
  PhysicsWorld$PhysicsWorld,
  PhysicsWorld$PhysicsWorld$world,
  CollisionEvent$CollisionEvent,
  CollisionEventType$CollisionStarted,
  CollisionEventType$CollisionStopped,
  RayHit$RayHit,
} from "./cacao.mjs";
// Rapier module — loaded dynamically
let RAPIER = null;

// === INITIALIZATION ===

/**
 * Initialize Rapier WASM and create a physics world.
 * Called from Gleam inside effect.from — callback receives the PhysicsWorld.
 */
export function init(gravity, callback) {
  (async () => {
    try {
      const rapierModule = await import("@dimforge/rapier3d-compat");
      await rapierModule.default.init();
      RAPIER = rapierModule.default;

      const [gx, gy, gz] = gravity;
      const world = new RAPIER.World({ x: gx, y: gy, z: gz });

      // Attach registries and event queue to the world object
      world._eventQueue = new RAPIER.EventQueue(true);
      world._bodyRegistry = new Map(); // meshId -> RigidBodyHandle
      world._colliderRegistry = new Map(); // meshId -> ColliderHandle
      world._colliderToMeshId = new Map(); // colliderHandle (int) -> meshId

      const pw = PhysicsWorld$PhysicsWorld(world, toList([]));
      callback(pw);
    } catch (e) {
      console.error("Cacao: Failed to initialize Rapier:", e);
    }
  })();
}

// === STEP ===

/**
 * Step the physics simulation:
 * 1. Auto-discover bodies from DOM
 * 2. Create/remove Rapier bodies
 * 3. Step Rapier
 * 4. Drain collision events
 * 5. Sync transforms to Three.js
 */
export function step(pw) {
  const world = PhysicsWorld$PhysicsWorld$world(pw);

  // 1. Auto-discover bodies from DOM
  const domBodies = document.querySelectorAll("[data-physics-body]");
  const domMeshIds = new Set();

  for (const el of domBodies) {
    const meshId = el.id;
    if (!meshId) continue;
    domMeshIds.add(meshId);

    // New mesh — create body
    if (!world._bodyRegistry.has(meshId)) {
      const config = parsePhysicsAttributes(el);
      createBodyFromConfig(world, meshId, config);
    }
  }

  // Remove bodies for meshes no longer in DOM
  for (const [meshId, handle] of world._bodyRegistry) {
    if (!domMeshIds.has(meshId)) {
      destroyBody(world, meshId, handle);
    }
  }

  // 2. Step Rapier
  world.step(world._eventQueue);

  // 3. Drain collision events
  const events = drainEvents(world);

  // 4. Sync transforms to Three.js
  for (const [meshId, handle] of world._bodyRegistry) {
    const body = world.getRigidBody(handle);
    if (!body) continue;

    const t = body.translation();
    const r = body.rotation();
    setPosition(meshId, t.x, t.y, t.z);
    setQuaternion(meshId, r.x, r.y, r.z, r.w);
  }

  // 5. Return updated PhysicsWorld
  return PhysicsWorld$PhysicsWorld(world, events);
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
  if (!RAPIER) return Result$Error();

  const world = PhysicsWorld$PhysicsWorld$world(pw);
  const ray = new RAPIER.Ray(
    { x: originX, y: originY, z: originZ },
    { x: dirX, y: dirY, z: dirZ }
  );

  const hit = world.castRay(ray, maxDistance, true);
  if (hit === null || hit === undefined) return Result$Error();

  // Compute hit point
  const point = ray.pointAt(hit.toi);

  // Compute hit normal
  const collider = world.getCollider(hit.collider);
  let normal = { x: 0, y: 1, z: 0 }; // default up
  if (collider) {
    const normalResult = collider.castRayAndGetNormal(ray, hit.toi, true);
    if (normalResult && normalResult.normal) {
      normal = normalResult.normal;
    }
  }

  // Resolve collider → mesh ID using the registry on the world object
  let meshId = Option$None();
  if (world._colliderToMeshId) {
    const id = world._colliderToMeshId.get(hit.collider);
    if (id) {
      meshId = Option$Some(id);
    }
  }

  return Result$Ok(RayHit$RayHit(
    [point.x, point.y, point.z],
    [normal.x, normal.y, normal.z],
    hit.toi,
    meshId
  ));
}

// === INTERNAL HELPERS ===

/**
 * Parse data-physics-* attributes from a DOM element.
 */
function parsePhysicsAttributes(el) {
  const bodyType = el.getAttribute("data-physics-body") || "dynamic";
  const colliderStr = el.getAttribute("data-physics-collider") || "cuboid:0.5,0.5,0.5";
  const collider = parseColliderString(colliderStr);

  const friction = parseFloat(el.getAttribute("data-physics-friction")) || 0.5;
  const restitution = parseFloat(el.getAttribute("data-physics-restitution")) || 0.0;
  const density = parseFloat(el.getAttribute("data-physics-density")) || 1.0;
  const isSensor = el.getAttribute("data-physics-sensor") === "True";
  const ccdEnabled = el.getAttribute("data-physics-ccd") === "True";
  const linearDamping = parseFloat(el.getAttribute("data-physics-linear-damping")) || 0.0;
  const angularDamping = parseFloat(el.getAttribute("data-physics-angular-damping")) || 0.0;

  // Parse lock rotations: "True,False,True"
  let lockRotations = [false, false, false];
  const lockStr = el.getAttribute("data-physics-lock-rotations");
  if (lockStr) {
    lockRotations = lockStr.split(",").map((p) => p.trim() === "True");
  }

  // Parse collision group: "membership,filter"
  let collisionGroup = null;
  const cgStr = el.getAttribute("data-physics-collision-group");
  if (cgStr) {
    const parts = cgStr.split(",");
    if (parts.length === 2) {
      collisionGroup = [parseInt(parts[0]), parseInt(parts[1])];
    }
  }

  return {
    bodyType,
    collider,
    friction,
    restitution,
    density,
    isSensor,
    ccdEnabled,
    linearDamping,
    angularDamping,
    lockRotations,
    collisionGroup,
  };
}

/**
 * Parse a collider string like "cuboid:0.5,0.5,0.5" or "ball:0.5"
 */
function parseColliderString(str) {
  const colonIdx = str.indexOf(":");
  if (colonIdx === -1) return { type: "cuboid", params: [0.5, 0.5, 0.5] };

  const type = str.substring(0, colonIdx);
  const params = str.substring(colonIdx + 1).split(",").map(Number);

  return { type, params };
}

/**
 * Create a Rapier body + collider from a parsed config object.
 * Mutates the world's registry Maps in place.
 */
function createBodyFromConfig(world, meshId, config) {
  if (!RAPIER) return;

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

  // Read initial position from Three.js
  const objOpt = getObject(meshId);
  if (Option$isSome(objOpt)) {
    const obj = Option$Some$0(objOpt);
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

  // Set physics-controlled attribute on the DOM element
  const el = document.getElementById(meshId);
  if (el) el.setAttribute("physics-controlled", "");
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

  // Unset physics-controlled on the DOM element
  const el = document.getElementById(meshId);
  if (el) el.removeAttribute("physics-controlled");
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
