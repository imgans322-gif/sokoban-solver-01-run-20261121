// Sokoban solver for make.fun's generated levels.
//
// The generator's levels share one shape: a one-square-wide hallway runs
// across the board with the player in it, and each room hangs off the
// hallway through a single door, holding 3-4 boxes and as many goals.
// Boxes never need to leave their room, and a level's push optimum is the
// sum of its rooms' optima. So this solver splits the board at the
// player's row into room components and solves each room on its own with
// a push-based A* whose cost is pushes alone:
//
//   - states are (sorted box cells, canonical player region), where the
//     player region is flood-filled over the whole board and named by its
//     smallest cell, so positions the player can walk between are one
//     state;
//   - the heuristic is the cheapest box-to-goal matching under true
//     single-box push distances (computed by reverse BFS per goal), which
//     is admissible, so each room's solution is push-optimal;
//   - squares no box can ever be pushed from to any of the room's goals
//     (dead squares, from the same BFS) are pruned as push targets;
//   - other rooms' boxes are fixed obstacles, and pushes are confined to
//     the room's own cells plus the hallway cells touching it.
//
// Everything is budgeted in expanded nodes, never by the clock, so two
// runs write byte-identical answers. If the split ever fails (a level
// that is not room-shaped), the same A* runs on the whole board instead.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from "node:fs";
import { join } from "node:path";

const INPUT_DIR = process.env.MAKEFUN_INPUT_DIR ?? "/input";
const OUTPUT_DIR = process.env.MAKEFUN_OUTPUT_DIR ?? "/output";
const DIRS = [
  { dx: 0, dy: -1, ch: "u" },
  { dx: 0, dy: 1, ch: "d" },
  { dx: -1, dy: 0, ch: "l" },
  { dx: 1, dy: 0, ch: "r" },
];
const INF = 1e9;

interface Level {
  w: number;
  h: number;
  wall: Uint8Array;
  floor: Uint8Array;
  goals: number[];
  goalSet: Set<number>;
  boxes: number[];
  player: number;
  playerRow: number;
}

function parseLevel(text: string): Level {
  const lines = text.replace(/\r/g, "").split("\n");
  while (lines.length && lines[lines.length - 1] === "") lines.pop();
  const h = lines.length;
  const w = Math.max(...lines.map((l) => l.length));
  const wall = new Uint8Array(w * h).fill(1);
  const floor = new Uint8Array(w * h);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = -1;
  let playerRow = 0;
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      const ch = lines[y][x] ?? "#";
      const c = y * w + x;
      if (ch === "#") continue;
      wall[c] = 0;
      floor[c] = 1;
      if (ch === "." || ch === "*" || ch === "+") goals.push(c);
      if (ch === "$" || ch === "*") boxes.push(c);
      if (ch === "@" || ch === "+") { player = c; playerRow = y; }
    }
  }
  return { w, h, wall, floor, goals, goalSet: new Set(goals), boxes, player, playerRow };
}

// Single-box push distances to one goal over the given allowed box cells:
// dist[cell] = pushes to bring one box from cell to the goal, ignoring the
// other boxes (the player must be able to stand behind the box, on any
// floor cell). Reverse BFS from the goal.
function pushDistances(lv: Level, goal: number, allowed: Uint8Array): Int32Array {
  const { w, floor } = lv;
  const dist = new Int32Array(lv.w * lv.h).fill(INF);
  dist[goal] = 0;
  const queue = [goal];
  for (let head = 0; head < queue.length; head++) {
    const c = queue[head];
    const cx = c % w, cy = (c / w) | 0;
    for (const d of DIRS) {
      // A push in direction d moved a box from prev to c; the player stood
      // at prev - d. Reverse step: box at prev = c - d, stand at c - 2d.
      const px = cx - d.dx, py = cy - d.dy;
      const sx = cx - 2 * d.dx, sy = cy - 2 * d.dy;
      if (px < 0 || py < 0 || px >= w || py >= lv.h || sx < 0 || sy < 0 || sx >= w || sy >= lv.h) continue;
      const prev = py * w + px, stand = sy * w + sx;
      if (!allowed[prev] || !floor[stand] || dist[prev] !== INF) continue;
      dist[prev] = dist[c] + 1;
      queue.push(prev);
    }
  }
  return dist;
}

// Cheapest matching between boxes and goals under a per-goal distance
// table: exact DP over subsets for small counts, greedy-free sum of
// per-box minima (still admissible) for large ones.
function matchingCost(boxes: number[], dists: Int32Array[]): number {
  const n = boxes.length;
  if (n === 0) return 0;
  if (n > 10) {
    let sum = 0;
    for (const b of boxes) {
      let best = INF;
      for (const d of dists) best = Math.min(best, d[b]);
      if (best >= INF) return INF;
      sum += best;
    }
    return sum;
  }
  const full = (1 << n) - 1;
  const dp = new Float64Array(1 << n).fill(Infinity);
  dp[0] = 0;
  for (let mask = 0; mask <= full; mask++) {
    const assigned = popcount(mask); // boxes already matched under this mask
    if (assigned >= n || dp[mask] === Infinity) continue;
    const b = boxes[assigned];
    for (let g = 0; g < n; g++) {
      if (mask & (1 << g)) continue;
      const c = dists[g][b];
      if (c >= INF) continue;
      const nm = mask | (1 << g);
      if (dp[mask] + c < dp[nm]) dp[nm] = dp[mask] + c;
    }
  }
  return dp[full];
}
function popcount(x: number): number {
  let n = 0;
  while (x) { x &= x - 1; n++; }
  return n;
}

interface SolveResult { moves: string; pushes: number; boxes: number[]; player: number; }

// Push-A* for one group of boxes (a room, or the whole board as fallback).
// `fixedBoxes` are other groups' boxes: obstacles that never move.
// Push targets are confined to `allowed`. Returns null when the node
// budget runs out or the group cannot be solved.
function solveGroup(
  lv: Level,
  startBoxes: number[],
  goals: number[],
  fixedBoxes: Set<number>,
  allowed: Uint8Array,
  playerStart: number,
  nodeBudget: number,
): SolveResult | null {
  const { w, h, floor } = lv;
  const goalSet = new Set(goals);
  const dists = goals.map((g) => pushDistances(lv, g, allowed));
  // Dead squares: no goal of this group is reachable from them.
  const dead = new Uint8Array(w * h);
  for (let c = 0; c < w * h; c++) {
    if (!allowed[c] || goalSet.has(c)) continue;
    let live = false;
    for (const d of dists) if (d[c] < INF) { live = true; break; }
    if (!live) dead[c] = 1;
  }
  for (const b of startBoxes) if (dead[b]) return null;

  const occupied = (boxes: number[], extra?: Set<number>) => {
    const s = new Set(boxes);
    for (const f of fixedBoxes) s.add(f);
    if (extra) for (const e of extra) s.add(e);
    return s;
  };

  // Flood the player's reachable region; returns the visited set and its
  // canonical name (smallest cell), plus a parent map for path building.
  function region(from: number, boxSet: Set<number>) {
    const seen = new Set<number>([from]);
    const parent = new Map<number, number>();
    const stack = [from];
    let rep = from;
    while (stack.length) {
      const c = stack.pop()!;
      if (c < rep) rep = c;
      const cx = c % w, cy = (c / w) | 0;
      for (const d of DIRS) {
        const nx = cx + d.dx, ny = cy + d.dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const ncell = ny * w + nx;
        if (!floor[ncell] || boxSet.has(ncell) || seen.has(ncell)) continue;
        seen.add(ncell);
        parent.set(ncell, c);
        stack.push(ncell);
      }
    }
    return { seen, rep, parent };
  }
  function walkPath(parent: Map<number, number>, from: number, to: number): string {
    const cells = [to];
    let cur = to;
    while (cur !== from) {
      cur = parent.get(cur)!;
      cells.push(cur);
    }
    cells.reverse();
    let s = "";
    for (let i = 1; i < cells.length; i++) {
      const diff = cells[i] - cells[i - 1];
      s += diff === -w ? "u" : diff === w ? "d" : diff === -1 ? "l" : "r";
    }
    return s;
  }

  interface Node {
    boxes: number[];
    player: number; // exact cell, just after this node's push
    g: number;
    f: number;
    parent: number; // index into nodes, -1 for root
    segment: string; // walk + push letters from the parent to this node
  }
  const nodes: Node[] = [];
  const best = new Map<string, number>();
  const heap: number[] = []; // node indices, by f then by lower h
  const hOf = (idx: number) => nodes[idx].f - nodes[idx].g;
  function heapPush(idx: number) {
    heap.push(idx);
    let i = heap.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (nodes[heap[i]].f < nodes[heap[p]].f || (nodes[heap[i]].f === nodes[heap[p]].f && hOf(heap[i]) < hOf(heap[p]))) {
        [heap[i], heap[p]] = [heap[p], heap[i]];
        i = p;
      } else break;
    }
  }
  function heapPop(): number {
    const top = heap[0];
    const last = heap.pop()!;
    if (heap.length) {
      heap[0] = last;
      let i = 0;
      for (;;) {
        const l = 2 * i + 1, r = l + 1;
        let m = i;
        if (l < heap.length && (nodes[heap[l]].f < nodes[heap[m]].f || (nodes[heap[l]].f === nodes[heap[m]].f && hOf(heap[l]) < hOf(heap[m])))) m = l;
        if (r < heap.length && (nodes[heap[r]].f < nodes[heap[m]].f || (nodes[heap[r]].f === nodes[heap[m]].f && hOf(heap[r]) < hOf(heap[m])))) m = r;
        if (m === i) break;
        [heap[i], heap[m]] = [heap[m], heap[i]];
        i = m;
      }
    }
    return top;
  }

  const startSorted = [...startBoxes].sort((a, b) => a - b);
  const startH = matchingCost(startSorted, dists);
  if (startH >= INF) return null;
  nodes.push({ boxes: startSorted, player: playerStart, g: 0, f: startH, parent: -1, segment: "" });
  const startRegion = region(playerStart, occupied(startSorted));
  best.set(startSorted.join(",") + "|" + startRegion.rep, 0);
  heapPush(0);

  let expansions = 0;
  while (heap.length) {
    const idx = heapPop();
    const node = nodes[idx];
    if (node.boxes.every((b) => goalSet.has(b))) {
      // Reconstruct the move string.
      const segments: string[] = [];
      let cur = idx;
      while (nodes[cur].parent !== -1) {
        segments.push(nodes[cur].segment);
        cur = nodes[cur].parent;
      }
      segments.reverse();
      return { moves: segments.join(""), pushes: node.g, boxes: node.boxes, player: node.player };
    }
    if (++expansions > nodeBudget) return null;
    const boxSet = occupied(node.boxes);
    const reg = region(node.player, boxSet);
    for (let bi = 0; bi < node.boxes.length; bi++) {
      const b = node.boxes[bi];
      const bx = b % w, by = (b / w) | 0;
      for (const d of DIRS) {
        const stand = (by - d.dy) * w + (bx - d.dx);
        const target = (by + d.dy) * w + (bx + d.dx);
        if (!reg.seen.has(stand)) continue;
        if (!floor[target] || boxSet.has(target) || !allowed[target] || dead[target]) continue;
        const nextBoxes = [...node.boxes];
        nextBoxes[bi] = target;
        nextBoxes.sort((a, b2) => a - b2);
        const ng = node.g + 1;
        const nReg = region(b, occupied(nextBoxes));
        const key = nextBoxes.join(",") + "|" + nReg.rep;
        if (best.has(key) && best.get(key)! <= ng) continue;
        const hh = matchingCost(nextBoxes, dists);
        if (hh >= INF) continue;
        best.set(key, ng);
        nodes.push({
          boxes: nextBoxes,
          player: b, // the player ends where the box was
          g: ng,
          f: ng + hh,
          parent: idx,
          segment: walkPath(reg.parent, node.player, stand) + d.ch,
        });
        heapPush(nodes.length - 1);
      }
    }
  }
  return null;
}

function solveLevel(text: string): string {
  const lv = parseLevel(text);
  const { w, h, floor, playerRow } = lv;

  // Split the board at the player's row: components of floor on either
  // side. A component holding boxes is a room.
  const comp = new Int32Array(w * h).fill(-1);
  const comps: { cells: number[]; boxes: number[]; goals: number[] }[] = [];
  for (let c = 0; c < w * h; c++) {
    if (!floor[c] || ((c / w) | 0) === playerRow || comp[c] !== -1) continue;
    const id = comps.length;
    const cells: number[] = [];
    const stack = [c];
    comp[c] = id;
    while (stack.length) {
      const cur = stack.pop()!;
      cells.push(cur);
      const cx = cur % w, cy = (cur / w) | 0;
      for (const d of DIRS) {
        const nx = cx + d.dx, ny = cy + d.dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h || ny === playerRow) continue;
        const ncell = ny * w + nx;
        if (!floor[ncell] || comp[ncell] !== -1) continue;
        comp[ncell] = id;
        stack.push(ncell);
      }
    }
    comps.push({
      cells,
      boxes: cells.filter((x) => lv.boxes.includes(x)),
      goals: cells.filter((x) => lv.goalSet.has(x)),
    });
  }
  const rooms = comps.filter((c) => c.boxes.length > 0);
  const roomOfBox = new Map<number, number>();
  rooms.forEach((room, i) => room.boxes.forEach((b) => roomOfBox.set(b, i)));
  const splitOk =
    rooms.length > 0 &&
    rooms.every((r) => r.goals.length === r.boxes.length) &&
    lv.boxes.every((b) => roomOfBox.has(b));

  if (!splitOk) {
    const allowed = new Uint8Array(w * h);
    for (let c = 0; c < w * h; c++) if (floor[c]) allowed[c] = 1;
    const res = solveGroup(lv, lv.boxes, lv.goals, new Set(), allowed, lv.player, 2_000_000);
    return res ? res.moves : "";
  }

  // Solve room by room on the live board. The order does not matter for
  // reachability: solveGroup floods the player's region over the whole
  // board, so the player can always walk from wherever the previous room
  // left them to the next room's first push — only boxes are confined,
  // each to its own room.
  let boxesNow = [...lv.boxes];
  let playerNow = lv.player;
  let moves = "";
  for (const room of rooms) {
    const allowed = new Uint8Array(w * h);
    for (const c of room.cells) allowed[c] = 1;
    // The hallway cells touching this room are fair push targets too
    // (the door square); the dead-square test keeps boxes out of any
    // hallway cell they could not come back from.
    for (const c of room.cells) {
      const cx = c % w, cy = (c / w) | 0;
      for (const d of DIRS) {
        const nx = cx + d.dx, ny = cy + d.dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        if (ny === playerRow && floor[ny * w + nx]) allowed[ny * w + nx] = 1;
      }
    }
    const mine = room.boxes;
    const fixed = new Set(boxesNow.filter((b) => !mine.includes(b)));
    const res = solveGroup(lv, mine, room.goals, fixed, allowed, playerNow, 600_000);
    if (!res) return "";
    moves += res.moves;
    const moved = new Map(mine.map((b, i) => [b, res.boxes[i]]));
    boxesNow = boxesNow.map((b) => (moved.has(b) ? moved.get(b)! : b));
    playerNow = res.player;
  }
  return moves;
}

for (const id of readdirSync(INPUT_DIR).sort()) {
  const levelPath = join(INPUT_DIR, id, "level.txt");
  let moves = "";
  try {
    moves = solveLevel(readFileSync(levelPath, "utf8"));
  } catch {
    moves = "";
  }
  mkdirSync(join(OUTPUT_DIR, id), { recursive: true });
  writeFileSync(join(OUTPUT_DIR, id, "solution.txt"), `${moves}\n`);
}
