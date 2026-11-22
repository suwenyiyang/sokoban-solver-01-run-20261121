// Sokoban solver: push-optimal per room, then stitched together.
//
// Structure of every level (from the competition rules): a 1-wide hallway
// across the board, with 1-4 rooms (6x6 slots) off it. A box pushed through
// a door into the hallway can never come back, so every box stays in its
// room and the push-optimal count of a level is the sum of its rooms'
// optima. This solver therefore:
//
//   1. splits the level into rooms by removing the hallway row,
//   2. solves each room exactly with push-space A* (macro pushes, flood-fill
//      player reachability, min-cost box->goal assignment heuristic, and
//      deadlock pruning),
//   3. stitches the room solutions with hallway walks between them.
//
// Deterministic: every budget is a count of node expansions, never the wall
// clock; no clock, randomness or environment input is read anywhere.

import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const INPUT_DIR = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const OUTPUT_DIR = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Expansion budgets (counts, not time) so runs are reproducible.
const ROOM_EXPANSION_CAP = 1_000_000;
const GLOBAL_EXPANSION_CAP = 1_000_000;

interface Dir { dr: number; dc: number; ch: string; }
const DIRS: Dir[] = [
  { dr: -1, dc: 0, ch: 'u' },
  { dr: 1, dc: 0, ch: 'd' },
  { dr: 0, dc: -1, ch: 'l' },
  { dr: 0, dc: 1, ch: 'r' },
];
const OPP = [1, 0, 3, 2];

// ---------------------------------------------------------------------------
// Level parsing
// ---------------------------------------------------------------------------

interface Level {
  w: number;
  h: number;
  floor: boolean[]; // cell index r*w+c is not a wall
  boxes: number[]; // cell indices
  goals: Set<number>;
  player: number; // cell index
}

function parseLevel(text: string): Level {
  const rows = text.replace(/\r/g, '').split('\n');
  while (rows.length > 0 && rows[rows.length - 1] === '') rows.pop();
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const floor = new Array<boolean>(w * h).fill(false);
  const boxes: number[] = [];
  const goals = new Set<number>();
  let player = -1;
  for (let r = 0; r < h; r++) {
    for (let c = 0; c < w; c++) {
      const ch = c < rows[r].length ? rows[r][c] : '#';
      const idx = r * w + c;
      if (ch !== '#') floor[idx] = true;
      if (ch === '$' || ch === '*') boxes.push(idx);
      if (ch === '.' || ch === '*') goals.add(idx);
      if (ch === '@' || ch === '+') player = idx;
    }
  }
  boxes.sort((a, b) => a - b);
  return { w, h, floor, boxes, goals, player };
}

// ---------------------------------------------------------------------------
// Small deterministic binary heap (min-heap on [f, h, seq])
// ---------------------------------------------------------------------------

interface HeapItem { f: number; h: number; seq: number; node: number; }

class Heap {
  private a: HeapItem[] = [];
  get size(): number { return this.a.length; }
  push(it: HeapItem): void {
    const a = this.a;
    a.push(it);
    let i = a.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (cmp(a[i], a[p]) < 0) { const t = a[i]; a[i] = a[p]; a[p] = t; i = p; } else break;
    }
  }
  pop(): HeapItem {
    const a = this.a;
    const top = a[0];
    const last = a.pop()!;
    if (a.length > 0) {
      a[0] = last;
      let i = 0;
      for (;;) {
        const l = i * 2 + 1, r = l + 1;
        let m = i;
        if (l < a.length && cmp(a[l], a[m]) < 0) m = l;
        if (r < a.length && cmp(a[r], a[m]) < 0) m = r;
        if (m === i) break;
        const t = a[i]; a[i] = a[m]; a[m] = t; i = m;
      }
    }
    return top;
  }
}

function cmp(x: HeapItem, y: HeapItem): number {
  if (x.f !== y.f) return x.f - y.f;
  if (x.h !== y.h) return x.h - y.h;
  return x.seq - y.seq;
}

// ---------------------------------------------------------------------------
// Push-space A* over one connected floor region ("room").
// cells: global cell indices of the region. boxes/goals/player/exit are
// given as global cell indices. Returns the move string, or null.
// ---------------------------------------------------------------------------

interface PushProblem {
  w: number;
  cells: number[]; // global cell indices, local index = position here
  boxes: number[]; // global cell indices
  goals: number[]; // global cell indices
  startPlayer: number; // global cell index
  exit: number; // global cell index the player must be able to reach at the
  // end, or -1 for none
  noBox: number[]; // global cell indices a box may never rest on (e.g. the
  // hallway cell in front of a door: the player may stand there, and may
  // push a box off the door from there, but a box never goes there)
  cap: number;
}

function solvePush(p: PushProblem): string | null {
  const n = p.cells.length;
  const loc = new Map<number, number>();
  for (let i = 0; i < n; i++) loc.set(p.cells[i], i);
  // adj[i][d] = local index of the neighbour in direction d, or -1.
  const adj: number[][] = new Array(n);
  for (let i = 0; i < n; i++) {
    const g = p.cells[i];
    const r = Math.floor(g / p.w), c = g % p.w;
    adj[i] = DIRS.map((d) => {
      const nr = r + d.dr, nc = c + d.dc;
      if (nr < 0 || nc < 0 || nc >= p.w) return -1;
      const v = loc.get(nr * p.w + nc);
      return v === undefined ? -1 : v;
    });
  }
  const goalSet = new Set<number>(p.goals.map((g) => loc.get(g)!));
  const goalList = [...goalSet];
  const noBoxSet = new Set<number>(p.noBox.map((g) => loc.get(g)!));
  const startBoxes = p.boxes.map((g) => loc.get(g)!).sort((a, b) => a - b);
  const startPlayer = loc.get(p.startPlayer)!;
  const exit = p.exit >= 0 ? loc.get(p.exit) : undefined;

  // --- push distances: dist[goal][cell] = min pushes to move a box from
  // cell to goal with a freely teleporting player (a relaxation, hence an
  // admissible heuristic). BFS backwards from each goal.
  const INF = 1e9;
  const dist: number[][] = goalList.map(() => new Array<number>(n).fill(INF));
  for (let gi = 0; gi < goalList.length; gi++) {
    const d = dist[gi];
    const g0 = goalList[gi];
    d[g0] = 0;
    const q: number[] = [g0];
    let qi = 0;
    while (qi < q.length) {
      const x = q[qi++];
      for (let dir = 0; dir < 4; dir++) {
        const y = adj[x][OPP[dir]]; // box was at y, pushed in dir to land on x
        if (y < 0 || d[y] !== INF) continue;
        const behind = adj[y][OPP[dir]]; // player must stand here
        if (behind < 0) continue;
        d[y] = d[x] + 1;
        q.push(y);
      }
    }
  }
  // A cell from which no goal is reachable can never hold a box.
  const deadCell = new Array<boolean>(n).fill(false);
  for (let i = 0; i < n; i++) {
    let ok = false;
    for (let gi = 0; gi < goalList.length; gi++) if (dist[gi][i] < INF) { ok = true; break; }
    deadCell[i] = !ok;
  }
  for (const b of startBoxes) if (deadCell[b] || noBoxSet.has(b)) return null;

  const nb = startBoxes.length;

  // Flood fill the player's reachable region from `from`, treating boxes as
  // walls. Returns a bitmask over local cells (BigInt: rooms have up to 36
  // cells, past JS's 32-bit bitwise range). The mask is the canonical
  // identity of the region: two regions with the same lowest cell can still
  // differ, so the full mask (not a representative cell) keys the state.
  function regionMask(from: number, boxBits: bigint): bigint {
    let mask = 0n;
    const q: number[] = [from];
    mask |= 1n << BigInt(from);
    let qi = 0;
    while (qi < q.length) {
      const x = q[qi++];
      for (let dir = 0; dir < 4; dir++) {
        const y = adj[x][dir];
        if (y < 0 || (boxBits & (1n << BigInt(y)))) continue;
        const bit = 1n << BigInt(y);
        if (mask & bit) continue;
        mask |= bit;
        q.push(y);
      }
    }
    return mask;
  }

  const bitsOf = (boxes: number[]): bigint => {
    let b = 0n;
    for (const x of boxes) b |= 1n << BigInt(x);
    return b;
  };

  // 2x2 freeze check, only for the blocks containing cell `t`: four blocked
  // (box) cells with no goal inside can never be unblocked.
  function frozenAt(t: number, boxBits: bigint): boolean {
    // top-left corners of the 2x2 blocks that contain t
    const up = adj[t][0];
    const left = adj[t][2];
    const upLeft = up >= 0 ? adj[up][2] : -1;
    const tls: number[] = [t, left, up, upLeft];
    for (const tl of tls) {
      if (tl < 0) continue;
      const right = adj[tl][3];
      const down = adj[tl][1];
      if (right < 0 || down < 0) continue;
      const dr = adj[down][3];
      if (dr < 0) continue;
      const cells = [tl, right, down, dr];
      let allBlocked = true;
      let hasGoal = false;
      for (const cell of cells) {
        if (goalSet.has(cell)) hasGoal = true;
        if (!(boxBits & (1n << BigInt(cell)))) allBlocked = false;
      }
      if (allBlocked && !hasGoal) return true;
    }
    return false;
  }

  // Heuristic: min-cost assignment of boxes to goals by push distance.
  // Exact permutations for small box counts, else sum of per-box minima
  // (still admissible: each push moves one box). Cached per box layout.
  const hCache = new Map<string, number>();
  function heuristic(boxes: number[], boxesKey: string): number {
    const cached = hCache.get(boxesKey);
    if (cached !== undefined) return cached;
    let result: number;
    if (nb <= 6) {
      const cost: number[][] = [];
      for (let i = 0; i < nb; i++) {
        cost.push(goalList.map((_, j) => dist[j][boxes[i]]));
      }
      const perm: number[] = goalList.map((_, j) => j);
      let best = INF;
      const rec = (k: number, sum: number): void => {
        if (sum >= best) return;
        if (k === nb) { if (sum < best) best = sum; return; }
        for (let i = k; i < perm.length; i++) {
          const t = perm[k]; perm[k] = perm[i]; perm[i] = t;
          const c = cost[k][perm[k]];
          if (c < INF) rec(k + 1, sum + c);
          const t2 = perm[k]; perm[k] = perm[i]; perm[i] = t2;
        }
      };
      rec(0, 0);
      result = best;
    } else {
      let sum = 0;
      for (let i = 0; i < nb; i++) {
        let m = INF;
        for (let gi = 0; gi < goalList.length; gi++) if (dist[gi][boxes[i]] < m) m = dist[gi][boxes[i]];
        if (m >= INF) { sum = INF; break; }
        sum += m;
      }
      result = sum;
    }
    hCache.set(boxesKey, result);
    return result;
  }

  const isGoal = (boxes: number[]): boolean => {
    for (const b of boxes) if (!goalSet.has(b)) return false;
    return true;
  };

  // --- A* over pushes. Node arrays, parent-linked for reconstruction.
  // A state is (sorted box cells, full player-region mask).
  const nodeBoxes: number[][] = [startBoxes];
  const nodeBoxesKey: string[] = [startBoxes.join(',')];
  const nodePlayer: number[] = [startPlayer];
  const nodeMask: bigint[] = [regionMask(startPlayer, bitsOf(startBoxes))];
  const nodeParent: number[] = [-1];
  const nodePushBox: number[] = [-1]; // box cell before the push
  const nodePushDir: number[] = [-1];
  const nodeG: number[] = [0];
  const keyOf = (boxesKey: string, mask: bigint): string => boxesKey + '|' + mask.toString(36);

  const open = new Heap();
  const h0 = heuristic(startBoxes, nodeBoxesKey[0]);
  if (h0 >= INF) return null;
  let seq = 0;
  open.push({ f: h0, h: h0, seq: seq++, node: 0 });
  const best = new Map<string, number>();
  best.set(keyOf(nodeBoxesKey[0], nodeMask[0]), 0);

  let expansions = 0;
  let goalNode = -1;
  while (open.size > 0) {
    const cur = open.pop();
    const ni = cur.node;
    const g = nodeG[ni];
    const settled = best.get(keyOf(nodeBoxesKey[ni], nodeMask[ni]));
    if (settled !== undefined && settled < g) continue;
    const boxes = nodeBoxes[ni];
    if (isGoal(boxes)) {
      if (exit === undefined || (nodeMask[ni] & (1n << BigInt(exit)))) {
        goalNode = ni;
        break;
      }
    }
    if (++expansions > p.cap) return null;
    const region = nodeMask[ni];
    const boxBits = bitsOf(boxes);
    for (let bi = 0; bi < nb; bi++) {
      const b = boxes[bi];
      for (let dir = 0; dir < 4; dir++) {
        const t = adj[b][dir];
        if (t < 0 || noBoxSet.has(t) || (boxBits & (1n << BigInt(t)))) continue;
        const behind = adj[b][OPP[dir]];
        if (behind < 0 || !(region & (1n << BigInt(behind)))) continue;
        if (deadCell[t]) continue;
        const nb2 = boxes.slice();
        nb2[bi] = t;
        nb2.sort((a, b2) => a - b2);
        const bits2 = bitsOf(nb2);
        if (frozenAt(t, bits2)) continue;
        const boxesKey2 = nb2.join(',');
        const h = heuristic(nb2, boxesKey2);
        if (h >= INF) continue;
        const ng = g + 1;
        const mask2 = regionMask(b, bits2);
        const key = keyOf(boxesKey2, mask2);
        const prev = best.get(key);
        if (prev !== undefined && prev <= ng) continue;
        best.set(key, ng);
        nodeBoxes.push(nb2);
        nodeBoxesKey.push(boxesKey2);
        nodePlayer.push(b);
        nodeMask.push(mask2);
        nodeParent.push(ni);
        nodePushBox.push(b);
        nodePushDir.push(dir);
        nodeG.push(ng);
        open.push({ f: ng + h, h, seq: seq++, node: nodeBoxes.length - 1 });
      }
    }
  }
  if (goalNode < 0) return null;

  // Reconstruct the push list, then replay it with BFS walks.
  const pushes: { box: number; dir: number }[] = [];
  let cur = goalNode;
  while (nodeParent[cur] >= 0) {
    pushes.push({ box: nodePushBox[cur], dir: nodePushDir[cur] });
    cur = nodeParent[cur];
  }
  pushes.reverse();

  // BFS walk inside the room, boxes as walls; returns the move string.
  function walk(from: number, to: number, boxes: Set<number>): string | null {
    if (from === to) return '';
    const prev = new Array<number>(n).fill(-2);
    const prevDir = new Array<number>(n).fill(-1);
    const q: number[] = [from];
    prev[from] = -1;
    let qi = 0;
    while (qi < q.length) {
      const x = q[qi++];
      for (let dir = 0; dir < 4; dir++) {
        const y = adj[x][dir];
        if (y < 0 || boxes.has(y) || prev[y] !== -2) continue;
        prev[y] = x;
        prevDir[y] = dir;
        if (y === to) {
          let s = '';
          let c = y;
          while (c !== from) { s = DIRS[prevDir[c]].ch + s; c = prev[c]; }
          return s;
        }
        q.push(y);
      }
    }
    return null;
  }

  let moves = '';
  let curPlayer = startPlayer;
  let curBoxes = startBoxes.slice();
  for (const push of pushes) {
    const behind = adj[push.box][OPP[push.dir]];
    const w = walk(curPlayer, behind, new Set<number>(curBoxes));
    if (w === null) return null; // should not happen: A* verified reachability
    moves += w;
    moves += DIRS[push.dir].ch;
    curBoxes = curBoxes.map((b) => (b === push.box ? adj[push.box][push.dir] : b)).sort((a, b) => a - b);
    curPlayer = push.box;
  }
  return moves;
}

// ---------------------------------------------------------------------------
// Whole-level solve: split into rooms, solve each, stitch with hallway walks.
// ---------------------------------------------------------------------------

function solveLevel(level: Level): string {
  const { w, h } = level;
  const pr = Math.floor(level.player / w); // hallway row (player starts there)
  // Connected components of floor cells excluding the hallway row.
  const seen = new Array<boolean>(w * h).fill(false);
  interface Room { cells: number[]; boxes: number[]; goals: number[]; door: number; }
  const rooms: Room[] = [];
  let ok = true;
  for (let r = 0; r < h && ok; r++) {
    for (let c = 0; c < w && ok; c++) {
      const idx = r * w + c;
      if (r === pr || !level.floor[idx] || seen[idx]) continue;
      const q: number[] = [idx];
      seen[idx] = true;
      const cells: number[] = [];
      let qi = 0;
      while (qi < q.length) {
        const x = q[qi++];
        cells.push(x);
        const xr = Math.floor(x / w), xc = x % w;
        for (const d of DIRS) {
          const nr = xr + d.dr, nc = xc + d.dc;
          if (nr < 0 || nr >= h || nc < 0 || nc >= w || nr === pr) continue;
          const y = nr * w + nc;
          if (!level.floor[y] || seen[y]) continue;
          seen[y] = true;
          q.push(y);
        }
      }
      const boxes = cells.filter((x) => level.boxes.includes(x));
      const goals = cells.filter((x) => level.goals.has(x));
      if (boxes.length === 0) continue; // empty pocket, not a room
      const doors = cells.filter((x) => {
        const xr = Math.floor(x / w), xc = x % w;
        return Math.abs(xr - pr) === 1 && level.floor[pr * w + xc];
      });
      if (boxes.length !== goals.length || doors.length !== 1) { ok = false; break; }
      rooms.push({ cells, boxes, goals, door: doors[0] });
    }
  }
  if (ok && rooms.length > 0) {
    // Solve each room from the hallway cell in front of its door. That cell
    // is part of the room's floor for the player (a box may sit on the door
    // itself and must be pushed in from the hallway), but a box may never
    // rest on it. The player must be able to walk back out to it at the end.
    const roomMoves: { room: Room; moves: string }[] = [];
    for (const room of rooms) {
      const hallCell = pr * w + (room.door % w);
      const moves = solvePush({
        w,
        cells: [...room.cells, hallCell],
        boxes: room.boxes,
        goals: room.goals,
        startPlayer: hallCell,
        exit: hallCell,
        noBox: [hallCell],
        cap: ROOM_EXPANSION_CAP,
      });
      if (moves === null) { ok = false; break; }
      roomMoves.push({ room, moves });
    }
    if (ok) {
      const stitched = stitch(level, pr, roomMoves);
      if (stitched !== null) return stitched;
    }
  }
  // Fallback: one global push search (may be too big; capped).
  const allCells: number[] = [];
  for (let i = 0; i < w * h; i++) if (level.floor[i]) allCells.push(i);
  const global = solvePush({
    w,
    cells: allCells,
    boxes: level.boxes,
    goals: [...level.goals],
    startPlayer: level.player,
    exit: -1,
    noBox: [],
    cap: GLOBAL_EXPANSION_CAP,
  });
  return global ?? '';
}

// Walk the player on the whole level (boxes are walls) from `from` to `to`.
// Returns the move string, or null if unreachable.
function walkGlobal(level: Level, boxes: Set<number>, from: number, to: number): string | null {
  if (from === to) return '';
  const { w, h } = level;
  const prev = new Array<number>(w * h).fill(-2);
  const prevDir = new Array<number>(w * h).fill(-1);
  const q: number[] = [from];
  prev[from] = -1;
  let qi = 0;
  while (qi < q.length) {
    const x = q[qi++];
    const xr = Math.floor(x / w), xc = x % w;
    for (let dir = 0; dir < 4; dir++) {
      const nr = xr + DIRS[dir].dr, nc = xc + DIRS[dir].dc;
      if (nr < 0 || nr >= h || nc < 0 || nc >= w) continue;
      const y = nr * w + nc;
      if (!level.floor[y] || boxes.has(y) || prev[y] !== -2) continue;
      prev[y] = x;
      prevDir[y] = dir;
      if (y === to) {
        let s = '';
        let c = y;
        while (c !== from) { s = DIRS[prevDir[c]].ch + s; c = prev[c]; }
        return s;
      }
      q.push(y);
    }
  }
  return null;
}

// Concatenate room solutions: walk the hallway between doors, enter each
// room, run its moves, walk back out. Rooms are visited nearest-first
// (greedy by hallway distance from the player's current door).
function stitch(
  level: Level,
  pr: number,
  roomMoves: { room: { cells: number[]; boxes: number[]; goals: number[]; door: number }; moves: string }[],
): string | null {
  const { w } = level;
  const remaining = roomMoves.slice();
  let moves = '';
  let cur = level.player; // hallway cell
  const liveBoxes = new Set<number>(level.boxes);
  while (remaining.length > 0) {
    // nearest remaining door along the hallway
    let bestIdx = -1;
    let bestWalk: string | null = null;
    for (let i = 0; i < remaining.length; i++) {
      const door = remaining[i].room.door;
      const hallCell = pr * w + (door % w);
      const wk = walkGlobal(level, liveBoxes, cur, hallCell);
      if (wk === null) continue;
      if (bestWalk === null || wk.length < bestWalk.length) {
        bestWalk = wk;
        bestIdx = i;
      }
    }
    if (bestIdx < 0 || bestWalk === null) return null;
    const { room, moves: rm } = remaining.splice(bestIdx, 1)[0];
    const door = room.door;
    const hallCell = pr * w + (door % w);
    moves += bestWalk;
    // The room's moves start with the player on the hallway cell and end
    // with the hallway cell reachable; no separate step in or out.
    moves += rm;
    // Walk back out to the hallway cell: replay the room's end state to find
    // where the player stands after the room's moves.
    const endPlayer = replayRoomEnd(level, room, hallCell, rm);
    if (endPlayer < 0) return null;
    const outWalk = walkGlobal(level, new Set<number>(room.goals), endPlayer, hallCell);
    if (outWalk === null) return null;
    moves += outWalk;
    cur = hallCell;
    // this room's boxes are now on its goals; keep them as obstacles
    for (const g of room.goals) liveBoxes.add(g);
    for (const b of room.boxes) liveBoxes.delete(b);
  }
  return moves;
}

// Simulate a room's move string to find where the player ends up.
function replayRoomEnd(
  level: Level,
  room: { cells: number[]; boxes: number[]; goals: number[]; door: number },
  startPlayer: number,
  moves: string,
): number {
  const { w } = level;
  const boxSet = new Set<number>(room.boxes);
  let player = startPlayer;
  for (const ch of moves) {
    const dir = DIRS.findIndex((d) => d.ch === ch);
    if (dir < 0) return -1;
    const r = Math.floor(player / w), c = player % w;
    const nr = r + DIRS[dir].dr, nc = c + DIRS[dir].dc;
    const nxt = nr * w + nc;
    if (!level.floor[nxt]) return -1;
    if (boxSet.has(nxt)) {
      const beyond = (nr + DIRS[dir].dr) * w + (nc + DIRS[dir].dc);
      if (!level.floor[beyond] || boxSet.has(beyond)) return -1;
      boxSet.delete(nxt);
      boxSet.add(beyond);
    }
    player = nxt;
  }
  return player;
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

for (const id of readdirSync(INPUT_DIR).sort()) {
  const levelText = readFileSync(join(INPUT_DIR, id, 'level.txt'), 'utf8');
  const level = parseLevel(levelText);
  const moves = solveLevel(level);
  mkdirSync(join(OUTPUT_DIR, id), { recursive: true });
  writeFileSync(join(OUTPUT_DIR, id, 'solution.txt'), `${moves}\n`);
}

