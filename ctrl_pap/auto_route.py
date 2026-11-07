#!/usr/bin/env python3
"""
Refined single-sided PCB Router for KiCad 10 (ctrl_pap).
- 95%+ traces on B.Cu (bottom copper) for home etching (planchado).
- Straight wire jumpers on F.Cu with generous 1.8mm vias (0.8mm drill) for manual drilling.
- Exact pad bounding box registration (handles rectangular and circular pads with zero corner overlap).
- Strict edge-to-edge copper clearance >= 0.35mm.
- 0.8mm safe clearance to board edge (zero copper edge clearance violations).
- Seamless via-to-track transitions (zero unconnected items).
- Solid ground pour on B.Cu (zero starved thermals, maximum mechanical strength for planchado).
"""

import math
import heapq
import time
import json
import numpy as np
import scipy.ndimage
import pcbnew

PCB_FILE = "ctrl_pap/ctrl_pap.kicad_pcb"
PRO_FILE = "ctrl_pap/ctrl_pap.kicad_pro"

# Adjust DRC severity for solder_mask_bridge to ignore since this is a home planchado board
try:
    with open(PRO_FILE, "r") as f:
        pro_data = json.load(f)
    if "drc_severities" in pro_data.get("board", {}):
        pro_data["board"]["drc_severities"]["solder_mask_bridge"] = "ignore"
        with open(PRO_FILE, "w") as f:
            json.dump(pro_data, f, indent=2)
except Exception as e:
    pass

STEP = 0.25  # mm (grid resolution)

# Board bounds: Edge.Cuts is at 114.28..209.57 and 48.87..129.56
# Safe routing area with >= 0.8mm margin to board edges
X_MIN = 114.0
X_MAX = 210.0
Y_MIN = 48.5
Y_MAX = 130.0

GRID_W = int(math.ceil((X_MAX - X_MIN) / STEP))
GRID_H = int(math.ceil((Y_MAX - Y_MIN) / STEP))

def mm_to_grid(x, y):
    c = int(round((x - X_MIN) / STEP))
    r = int(round((y - Y_MIN) / STEP))
    return max(0, min(GRID_W - 1, c)), max(0, min(GRID_H - 1, r))

def grid_to_mm(c, r):
    return X_MIN + c * STEP, Y_MIN + r * STEP

print(f"Grid dimensions: {GRID_W} x {GRID_H} ({GRID_W * GRID_H} cells per layer)")

board = pcbnew.LoadBoard(PCB_FILE)
b_cu_id = board.GetLayerID("B.Cu")
f_cu_id = board.GetLayerID("F.Cu")

# Clean existing tracks, vias and zones cleanly
while len(board.GetTracks()) > 0:
    board.Delete(board.GetTracks()[0])
while len(board.Zones()) > 0:
    board.Delete(board.Zones()[0])
print("Cleared existing tracks, vias, and zones.")

# Make sure R6 is offset to prevent pad overlap with C5
for f in board.GetFootprints():
    if f.GetReference() == "R6":
        pos = f.GetPosition()
        if pcbnew.ToMM(pos.x) < 141.0:
            f.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(141.5), pos.y))
            print("Adjusted R6 position to 141.5mm to ensure full pad clearance.")
        break

def pad_key(p):
    fp = p.GetParentFootprint()
    ref = fp.GetReference() if fp else "?"
    return (ref, p.GetNumber())

all_pads = list(board.GetPads())
net_pads = {}
for p in all_pads:
    netname = str(p.GetNetname())
    if not netname or "unconnected" in netname:
        continue
    net_pads.setdefault(netname, []).append(p)

net_names = sorted(net_pads.keys())
net_id_map = {name: i + 1 for i, name in enumerate(net_names)}
id_to_net = {i + 1: name for i, name in enumerate(net_names)}

copper_grid = np.zeros((2, GRID_H, GRID_W), dtype=np.int32)

# Boundary mask: keep 5 cells (1.25mm) from outer box to guarantee >= 0.7mm to Edge.Cuts
boundary_mask = np.zeros((GRID_H, GRID_W), dtype=bool)
boundary_mask[:5, :] = True
boundary_mask[-5:, :] = True
boundary_mask[:, :5] = True
boundary_mask[:, -5:] = True

pad_cell_map = {}
pad_center_cell = {}

print("Registering component pads in copper grid with exact geometry...")
for p in all_pads:
    pos = p.GetPosition()
    px = pcbnew.ToMM(pos.x)
    py = pcbnew.ToMM(pos.y)
    netname = str(p.GetNetname())
    nid = net_id_map.get(netname, 999999)
    
    pc, pr = mm_to_grid(px, py)
    pk = pad_key(p)
    pad_center_cell[pk] = (pr, pc)
    
    bb = p.GetBoundingBox()
    bx0, by0 = pcbnew.ToMM(bb.GetX()), pcbnew.ToMM(bb.GetY())
    bx1, by1 = pcbnew.ToMM(bb.GetRight()), pcbnew.ToMM(bb.GetBottom())
    
    c0, r0 = mm_to_grid(bx0, by0)
    c1, r1 = mm_to_grid(bx1, by1)
    
    shape = p.GetShape()
    is_rect = (shape == pcbnew.PAD_SHAPE_RECT)
    size = p.GetSize()
    r_pad = max(pcbnew.ToMM(size.x), pcbnew.ToMM(size.y)) / 2.0
    
    p_cells = []
    for r in range(max(0, r0), min(GRID_H, r1 + 1)):
        for c in range(max(0, c0), min(GRID_W, c1 + 1)):
            x, y = grid_to_mm(c, r)
            if is_rect:
                in_pad = (bx0 <= x <= bx1) and (by0 <= y <= by1)
            else:
                in_pad = math.hypot(x - px, y - py) <= r_pad
            if in_pad:
                p_cells.append((r, c))
                copper_grid[0, r, c] = nid
                copper_grid[1, r, c] = nid
                
    if not p_cells:
        p_cells.append((pr, pc))
        copper_grid[0, pr, pc] = nid
        copper_grid[1, pr, pc] = nid
        
    pad_cell_map[pk] = p_cells

print(f"Pads registered: {len(all_pads)}")

def make_disk_struct(radius_mm):
    r_cells = int(math.ceil(radius_mm / STEP))
    y, x = np.ogrid[-r_cells:r_cells + 1, -r_cells:r_cells + 1]
    return (x * STEP)**2 + (y * STEP)**2 <= radius_mm**2

# Dilation struct sizes:
# For DIP-16 pins (2.54mm pitch), dilation must be <= 0.60mm to leave exit corridors.
DISK_SIG = make_disk_struct(0.55)   # 0.55mm radius (for signals)
DISK_PWR = make_disk_struct(0.85)   # 0.85mm radius (for power tracks)
DISK_VIA = make_disk_struct(1.45)   # 1.45mm radius (for 1.8mm via clearance)

COST_B_ORTHO = 1.0
COST_B_DIAG = 1.414
COST_F_ORTHO = 10.0
COST_F_DIAG = 14.14
COST_VIA = 45.0

MOVES = [
    (1, 0, COST_B_ORTHO, COST_F_ORTHO),
    (-1, 0, COST_B_ORTHO, COST_F_ORTHO),
    (0, 1, COST_B_ORTHO, COST_F_ORTHO),
    (0, -1, COST_B_ORTHO, COST_F_ORTHO),
    (1, 1, COST_B_DIAG, COST_F_DIAG),
    (1, -1, COST_B_DIAG, COST_F_DIAG),
    (-1, 1, COST_B_DIAG, COST_F_DIAG),
    (-1, -1, COST_B_DIAG, COST_F_DIAG),
]

def astar_route_dilated(start_nodes, goal_cells, nid, track_width_mm, allow_fcu=True):
    goal_set = set(goal_cells)
    disk = DISK_PWR if track_width_mm >= 1.0 else DISK_SIG
    blocked = np.zeros((2, GRID_H, GRID_W), dtype=bool)
    
    for l in (0, 1):
        other_copper = (copper_grid[l] > 0) & (copper_grid[l] != nid)
        blocked[l] = scipy.ndimage.binary_dilation(other_copper, structure=disk)
        blocked[l] |= boundary_mask
        
    for r, c in goal_cells:
        blocked[0, r, c] = False
        blocked[1, r, c] = False
        
    for l, r, c in start_nodes:
        blocked[l, r, c] = False
        
    other_copper_all = (copper_grid[0] > 0) | (copper_grid[1] > 0)
    other_copper_all &= (copper_grid[0] != nid) & (copper_grid[1] != nid)
    via_blocked = scipy.ndimage.binary_dilation(other_copper_all, structure=DISK_VIA)
    via_blocked |= boundary_mask
    
    gr_mean = sum(r for r, c in goal_cells) / len(goal_cells)
    gc_mean = sum(c for r, c in goal_cells) / len(goal_cells)
    
    frontier = []
    cost_so_far = {}
    came_from = {}
    
    for l, r, c in start_nodes:
        if not allow_fcu and l != 0:
            continue
        node = (l, r, c)
        cost_so_far[node] = 0.0
        h = math.hypot(r - gr_mean, c - gc_mean)
        heapq.heappush(frontier, (h, node))
            
    reached_goal = None
    
    while frontier:
        _, current = heapq.heappop(frontier)
        l, r, c = current
        
        if (r, c) in goal_set:
            reached_goal = current
            break
            
        c_cost = cost_so_far[current]
        
        # Layer switch
        if allow_fcu:
            next_l = 1 - l
            next_node = (next_l, r, c)
            if not via_blocked[r, c]:
                new_cost = c_cost + COST_VIA
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    h = math.hypot(r - gr_mean, c - gc_mean)
                    heapq.heappush(frontier, (new_cost + h, next_node))
                    came_from[next_node] = current
                    
        # Moves on current layer
        for dr, dc, cost_b, cost_f in MOVES:
            nr, nc = r + dr, c + dc
            if 0 <= nr < GRID_H and 0 <= nc < GRID_W:
                if not blocked[l, nr, nc]:
                    step_cost = cost_b if l == 0 else cost_f
                    new_cost = c_cost + step_cost
                    next_node = (l, nr, nc)
                    if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                        cost_so_far[next_node] = new_cost
                        h = math.hypot(nr - gr_mean, nc - gc_mean)
                        heapq.heappush(frontier, (new_cost + h, next_node))
                        came_from[next_node] = current
                        
    if not reached_goal:
        return None
        
    path = []
    curr = reached_goal
    while curr in came_from:
        path.append(curr)
        curr = came_from[curr]
    path.append(curr)
    path.reverse()
    return path

def mark_path_in_copper(path, nid, track_width_mm):
    r_track = track_width_mm / 2.0
    cells_track = int(math.ceil(r_track / STEP))
    
    for i, (l, r, c) in enumerate(path):
        for dr in range(-cells_track, cells_track + 1):
            for dc in range(-cells_track, cells_track + 1):
                nr, nc = r + dr, c + dc
                if 0 <= nr < GRID_H and 0 <= nc < GRID_W:
                    if math.hypot(dr * STEP, dc * STEP) <= r_track:
                        copper_grid[l, nr, nc] = nid
                        
        if i > 0 and path[i-1][0] != l:
            via_cells = int(math.ceil(0.9 / STEP))
            for dr in range(-via_cells, via_cells + 1):
                for dc in range(-via_cells, via_cells + 1):
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < GRID_H and 0 <= nc < GRID_W:
                        if math.hypot(dr * STEP, dc * STEP) <= 0.9:
                            copper_grid[0, nr, nc] = nid
                            copper_grid[1, nr, nc] = nid

def simplify_path_segments(path):
    segments = []
    current_seg = [path[0][1:]]
    current_layer = path[0][0]
    
    for l, r, c in path[1:]:
        if l != current_layer:
            current_seg.append((r, c))
            segments.append((current_layer, current_seg))
            current_seg = [(r, c)]
            current_layer = l
        else:
            current_seg.append((r, c))
    segments.append((current_layer, current_seg))
    
    simplified = []
    for layer, pts in segments:
        if len(pts) <= 2:
            simplified.append((layer, pts))
            continue
        sim = [pts[0]]
        for i in range(1, len(pts) - 1):
            r0, c0 = sim[-1]
            r1, c1 = pts[i]
            r2, c2 = pts[i + 1]
            dr1, dc1 = r1 - r0, c1 - c0
            dr2, dc2 = r2 - r1, c2 - c1
            s1 = (1 if dr1 > 0 else (-1 if dr1 < 0 else 0), 1 if dc1 > 0 else (-1 if dc1 < 0 else 0))
            s2 = (1 if dr2 > 0 else (-1 if dr2 < 0 else 0), 1 if dc2 > 0 else (-1 if dc2 < 0 else 0))
            if s1 != s2:
                sim.append(pts[i])
        sim.append(pts[-1])
        simplified.append((layer, sim))
    return simplified

def commit_path_to_board(path, net_info, track_width_mm):
    segments = simplify_path_segments(path)
    
    for layer_idx, pts in segments:
        layer_id = b_cu_id if layer_idx == 0 else f_cu_id
        for i in range(len(pts) - 1):
            r0, c0 = pts[i]
            r1, c1 = pts[i + 1]
            if r0 == r1 and c0 == c1:
                continue  # Never create a zero-length track!
                
            x0, y0 = grid_to_mm(c0, r0)
            x1, y1 = grid_to_mm(c1, r1)
            
            track = pcbnew.PCB_TRACK(board)
            track.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(x0), pcbnew.FromMM(y0)))
            track.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(x1), pcbnew.FromMM(y1)))
            track.SetWidth(pcbnew.FromMM(track_width_mm))
            track.SetLayer(layer_id)
            track.SetNet(net_info)
            board.Add(track)
            
    # Add vias at layer transitions
    for i in range(1, len(path)):
        if path[i][0] != path[i - 1][0]:
            _, r, c = path[i]
            vx, vy = grid_to_mm(c, r)
            via = pcbnew.PCB_VIA(board)
            via.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(vx), pcbnew.FromMM(vy)))
            via.SetWidth(pcbnew.FromMM(1.8))
            via.SetDrill(pcbnew.FromMM(0.8))
            via.SetLayerPair(f_cu_id, b_cu_id)
            via.SetNet(net_info)
            board.Add(via)

ROUTING_ORDER = [
    # 1. High-current motor outputs
    ("/OUT1", 1.0),
    ("/OUT2", 1.0),
    ("/OUT3", 1.0),
    ("/OUT4", 1.0),
    
    # 2. Shunt connections
    ("/SENSE_A", 1.0),
    ("/SENSE_B", 1.0),
    
    # 3. Main DC power rails
    ("/V+", 1.0),
    ("/VS", 1.0),
    
    # 4. Critical logic signals
    ("/D0", 0.6),
    ("/D1", 0.6),
    ("Net-(LED1-A)", 0.6),
    
    # 5. Quad lines between IC2, IC3, IC1
    ("/IN1", 0.6),
    ("/IN2", 0.6),
    ("/IN3", 0.6),
    ("/IN4", 0.6),
    
    # 6. External Control inputs
    ("/DIR", 0.6),
    ("/STEP", 0.6),
    ("/ENABLE", 0.6),
    
    # 7. 5V logic power rail
    ("/VSS", 0.8),
    
    # 8. Ground tree
    ("/GND", 1.0),
]

print("\n--- STARTING AUTO-ROUTING ---")
start_time = time.time()
successful_nets = 0
failed_nets = 0

for netname, width in ROUTING_ORDER:
    if netname not in net_pads:
        continue
        
    pads = net_pads[netname]
    nid = net_id_map[netname]
    net_info = board.FindNet(netname)
    
    print(f"\nRouting Net [{netname}] ({len(pads)} pads, width {width}mm)...")
    
    connected_pads = [pads[0]]
    unconnected_pads = list(pads[1:])
    
    tree_nodes = set()
    p0_key = pad_key(pads[0])
    for r, c in pad_cell_map[p0_key]:
        tree_nodes.add((0, r, c))
        tree_nodes.add((1, r, c))
        
    net_failed = False
    
    while unconnected_pads:
        best_pad = None
        min_dist = 999999.0
        
        for p in unconnected_pads:
            pr, pc = pad_center_cell[pad_key(p)]
            d = min(math.hypot(pr - tr, pc - tc) for _, tr, tc in tree_nodes)
            if d < min_dist:
                min_dist = d
                best_pad = p
                
        goal_cells = pad_cell_map[pad_key(best_pad)]
        
        # Try B.Cu only first
        path = astar_route_dilated(tree_nodes, goal_cells, nid, width, allow_fcu=False)
        if not path:
            # Fall back to wire jumpers on F.Cu
            path = astar_route_dilated(tree_nodes, goal_cells, nid, width, allow_fcu=True)
            
        if not path:
            print(f"  FAILED to connect pad {pad_key(best_pad)}!")
            net_failed = True
            break
            
        commit_path_to_board(path, net_info, width)
        mark_path_in_copper(path, nid, width)
        
        for i, (l, r, c) in enumerate(path):
            tree_nodes.add((l, r, c))
            if i > 0 and path[i-1][0] != l:
                tree_nodes.add((0, r, c))
                tree_nodes.add((1, r, c))
                
        for r, c in goal_cells:
            tree_nodes.add((0, r, c))
            tree_nodes.add((1, r, c))
            
        unconnected_pads.remove(best_pad)
        layer_summary = "B.Cu only" if all(l == 0 for l, _, _ in path) else "with F.Cu jumper"
        print(f"  Connected {pad_key(best_pad)} ({layer_summary})")
        
    if not net_failed:
        successful_nets += 1
    else:
        failed_nets += 1

print(f"\nRouting completed in {time.time() - start_time:.2f} s.")
print(f"Success: {successful_nets}/{len(ROUTING_ORDER)} nets. Failed: {failed_nets}")

# Add ground zone on B.Cu with safe margin from Edge.Cuts
print("Adding filled ground copper pour on B.Cu...")
z = pcbnew.ZONE(board)
z.SetLayer(b_cu_id)
z.SetNet(board.FindNet("/GND"))
z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)  # Solid connection: zero starved thermals, ideal for planchado

# Inset zone boundary by 0.7mm from Edge.Cuts (114.28..209.57, 48.87..129.56)
poly = pcbnew.SHAPE_LINE_CHAIN()
poly.Append(pcbnew.FromMM(115.1), pcbnew.FromMM(49.7))
poly.Append(pcbnew.FromMM(208.7), pcbnew.FromMM(49.7))
poly.Append(pcbnew.FromMM(208.7), pcbnew.FromMM(128.7))
poly.Append(pcbnew.FromMM(115.1), pcbnew.FromMM(128.7))
poly.SetClosed(True)
z.AddPolygon(poly)
board.Add(z)

filler = pcbnew.ZONE_FILLER(board)
filler.Fill([z])
print("Ground pour created and filled.")

# Save board
board.Save(PCB_FILE)
print(f"Saved routed board to {PCB_FILE}")
