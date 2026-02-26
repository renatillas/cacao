// Cacao FFI — CacaoApp holder (timing coordination + registry management).
//
// This module manages ONLY lifecycle state needed to bridge the tiramisu
// extension hooks (registered at startup) with the Rapier physics world
// (initialized asynchronously). It contains NO direct Rapier usage.
//
// All physics operations (body creation, step, raycasting, collision events)
// are implemented in cacao.gleam using the estoque typed API.
//
// The CacaoApp holder stores:
//   _pendingCreates: meshId → { fn: createFn, obj: Object3D | null }
//     createFn is a Gleam closure fn(World, String, Object3D) -> Nil
//     queued when world or Object3D is not yet available
//   _bodies:         meshId → RigidBody (from estoque)
//   _colliders:      meshId → Collider (from estoque)
//   _colliderToMesh: colliderId (Int) → meshId (String)
//   _objects:        meshId → Object3D (for transform sync in step)
//   world:           null | estoque.World (set by setWorld after init)

import { Result$Ok, Result$Error, toList } from "./gleam.mjs";
import {
  Option$isSome,
  Option$Some$0,
} from "../gleam_stdlib/gleam/option.mjs";

// Use estoque's FFI directly for WASM init and world creation.
// This ensures RAPIER is initialized inside estoque's module scope
// (where all subsequent estoque calls will use it).
import { init as estoqueInit, createWorld } from "../estoque/estoque.ffi.mjs";

// === CACAO APP ===

/**
 * Create a new CacaoApp holder.
 */
export function newWorldHolder() {
  return {
    world: null,
    _pendingCreates: new Map(),
    _bodies: new Map(),
    _colliders: new Map(),
    _colliderToMesh: new Map(),
    _objects: new Map(),
  };
}

// === INITIALIZATION ===

/**
 * Initialize Rapier WASM via estoque and create a physics world.
 *
 * Typed as (gravity, callback) -> Nil in Gleam. The callback is invoked
 * asynchronously with the new estoque.World once WASM loads.
 */
export async function doInitWorld(gravity, callback) {
  await estoqueInit();
  const [gx, gy, gz] = gravity;
  const world = createWorld(gx, gy, gz);
  callback(world);
}

/**
 * Connect the initialized estoque.World to the holder.
 *
 * Flushes any pending body creation closures that have both a stored
 * closure and a resolved Object3D.
 */
export function setWorld(holder, world) {
  holder.world = world;

  for (const [meshId, entry] of holder._pendingCreates) {
    if (holder._bodies.has(meshId)) continue;
    if (entry.obj) {
      entry.fn(world, meshId, entry.obj);
      holder._pendingCreates.delete(meshId);
    }
    // No obj yet — leave for onObjectResolved
  }
}

// === BODY LIFECYCLE ===

/**
 * Queue a body creation closure.
 *
 * createFn: Gleam fn(estoque.World, String, Object3D) -> Nil
 *
 * If both world and object are already available, calls createFn immediately.
 * Otherwise stores the closure until conditions are met.
 */
export function queueBodyCreate(holder, id, objectOrNone, createFn) {
  // Guard: skip if body already exists or creation is already pending
  if (holder._bodies.has(id) || holder._pendingCreates.has(id)) return;

  const obj = fromOption(objectOrNone);
  const world = holder.world;

  if (world && obj) {
    createFn(world, id, obj);
  } else if (world) {
    // World ready, waiting for async mesh to resolve
    holder._pendingCreates.set(id, { fn: createFn, obj: null });
  } else if (obj) {
    // Object ready, waiting for world init
    holder._pendingCreates.set(id, { fn: createFn, obj });
  } else {
    // Neither ready yet
    holder._pendingCreates.set(id, { fn: createFn, obj: null });
  }
}

/**
 * Called by the tiramisu on_object_resolved hook when an async mesh loads.
 *
 * Stores the Object3D and fires any pending create closure if the world
 * is also ready.
 */
export function onObjectResolved(holder, id, object) {
  const world = holder.world;

  if (world) {
    holder._objects.set(id, object);
    const entry = holder._pendingCreates.get(id);
    if (entry && !holder._bodies.has(id)) {
      entry.fn(world, id, object);
      holder._pendingCreates.delete(id);
    }
  } else {
    // World not ready — update the pending entry's obj
    const entry = holder._pendingCreates.get(id);
    if (entry) {
      entry.obj = object;
    } else {
      holder._objects.set(id, object);
    }
  }
}

/**
 * Get the estoque.World stored in the holder, or None if not yet initialized.
 * Used by handle_remove so Gleam can call estoque.remove_rigid_body.
 */
export function getHolderWorld(holder) {
  return holder.world != null ? Result$Ok(holder.world) : Result$Error();
}

/**
 * Register a newly created body in the holder's registries.
 * Called from Gleam after estoque body/collider creation.
 *
 * colliderId: integer key for _colliderToMesh lookup during
 *             collision events and raycasting.
 */
export function registerBody(holder, id, body, collider, colliderId, obj) {
  holder._bodies.set(id, body);
  holder._colliders.set(id, collider);
  holder._colliderToMesh.set(colliderId, id);
  if (obj) holder._objects.set(id, obj);
}

/**
 * Clean up all registry entries for a mesh ID after body removal.
 *
 * colliderId: integer key to remove from _colliderToMesh.
 *             Pass -1 if no collider was registered.
 */
export function cleanupBody(holder, id, colliderId) {
  holder._pendingCreates.delete(id);
  holder._objects.delete(id);
  holder._bodies.delete(id);
  holder._colliders.delete(id);
  if (colliderId >= 0) {
    holder._colliderToMesh.delete(colliderId);
  }
}

// === REGISTRY ACCESSORS ===

/**
 * Check whether a physics body (or pending creation) exists for this ID.
 */
export function bodyExists(holder, id) {
  return holder._bodies.has(id) || holder._pendingCreates.has(id);
}

export function getBody(holder, id) {
  const body = holder._bodies.get(id);
  return body !== undefined ? Result$Ok(body) : Result$Error();
}

export function getCollider(holder, id) {
  const collider = holder._colliders.get(id);
  return collider !== undefined ? Result$Ok(collider) : Result$Error();
}

export function getColliderMeshId(holder, colliderId) {
  const meshId = holder._colliderToMesh.get(colliderId);
  return meshId !== undefined ? Result$Ok(meshId) : Result$Error();
}

export function getAllBodyIds(holder) {
  return toList([...holder._bodies.keys()]);
}

export function getObjectRef(holder, id) {
  const obj = holder._objects.get(id);
  return obj != null ? Result$Ok(obj) : Result$Error();
}

// === HELPERS ===

function fromOption(opt) {
  return Option$isSome(opt) ? Option$Some$0(opt) : null;
}
