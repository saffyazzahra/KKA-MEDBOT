import heapq
import math
import random
from collections import deque

START = "pharmacy" # robot home base
UNIT = "min" # time unit
UNIT_LONG = "minutes" # full time unit
SHOW_ROUTES = True # print the route taken on every leg
SHOW_EXPANDED = True # print the nodes expanded, in order, on every leg

EDGES = [ # hospital hallways walking times
    ("emergency room", "radiology", 3),
    ("radiology", "laboratory", 3),
    ("laboratory", "physiotherapy", 3),
    ("laboratory", "lobby", 4),
    ("physiotherapy", "lobby", 2),
    ("physiotherapy", "ward a", 3),
    ("ward a", "lobby", 4),
    ("ward a", "ward b", 3),
    ("ward b", "ward c", 3),
    ("ward a", "pediatrics", 5),
    ("ward c", "internal medicine", 4),
    ("pediatrics", "internal medicine", 3),
    ("internal medicine", "nurse station", 4),
    ("emergency room", "receptionist", 6),
    ("receptionist", "lobby", 7),
    ("receptionist", "pharmacy", 4),
    ("pharmacy", "lobby", 10),
    ("lobby", "dentistry", 6),
    ("dentistry", "pulmonology", 3),
    ("pulmonology", "nurse station", 3),
]

GRAPH = {} # adjacency list network graph
for a, b, w in EDGES:
    GRAPH.setdefault(a, {})[b] = w
    GRAPH.setdefault(b, {})[a] = w

COORDS = { # room map layout coordinates
    "emergency room": (92, 115), "radiology": (275, 113), "laboratory": (475, 113),
    "physiotherapy": (695, 163), "ward a": (897, 163), "ward b": (1088, 163),
    "ward c": (1303, 163), "receptionist": (275, 327), "lobby": (695, 327),
    "pediatrics": (1088, 327), "internal medicine": (1303, 327), "pharmacy": (275, 532),
    "dentistry": (897, 490), "pulmonology": (1088, 490), "nurse station": (1303, 490),
}


def straight_line(a, b): # calculate straight line distance
    return math.dist(COORDS[a], COORDS[b])


SCALE = min(w / straight_line(a, b) for a in GRAPH for b, w in GRAPH[a].items()) # heuristic scale factor


def heuristic(node, goal): # estimated remaining distance compass
    return SCALE * straight_line(node, goal)


def build_path(parent, goal): # retrace path nodes backward
    path = []
    node = goal
    while node is not None:
        path.append(node)
        node = parent[node]
    return path[::-1]


def path_cost(path): # total walking time along path
    return sum(GRAPH[a][b] for a, b in zip(path, path[1:]))


def bfs(start, goal): # breadth first search hop count
    frontier = deque([start])
    parent = {start: None}
    expanded = []
    while frontier:
        node = frontier.popleft()
        expanded.append(node)
        if node == goal:
            return build_path(parent, goal), expanded
        for nbr in sorted(GRAPH[node]):
            if nbr not in parent:
                parent[nbr] = node
                frontier.append(nbr)
    return None, expanded


def ucs(start, goal): # uniform cost search walking time
    frontier = [(0, start)]
    best = {start: 0}
    parent = {start: None}
    explored = set()
    expanded = []
    while frontier:
        g, node = heapq.heappop(frontier)
        if node in explored:
            continue
        explored.add(node)
        expanded.append(node)
        if node == goal:
            return build_path(parent, goal), expanded
        for nbr, w in GRAPH[node].items():
            new_g = g + w
            if nbr not in explored and new_g < best.get(nbr, math.inf):
                best[nbr] = new_g
                parent[nbr] = node
                heapq.heappush(frontier, (new_g, nbr))
    return None, expanded


def astar(start, goal): # a-star search heuristic navigation
    frontier = [(heuristic(start, goal), 0, start)]
    best = {start: 0}
    parent = {start: None}
    explored = set()
    expanded = []
    while frontier:
        f, g, node = heapq.heappop(frontier)
        if node in explored:
            continue
        explored.add(node)
        expanded.append(node)
        if node == goal:
            return build_path(parent, goal), expanded
        for nbr, w in GRAPH[node].items():
            new_g = g + w
            if nbr not in explored and new_g < best.get(nbr, math.inf):
                best[nbr] = new_g
                parent[nbr] = node
                heapq.heappush(frontier, (new_g + heuristic(nbr, goal), new_g, nbr))
    return None, expanded


def fmt(x): # format display numbers
    return f"{x:g}"


def all_pairs_shortest(): # fastest possible walking time between any two rooms (floyd-warshall)
    rooms = list(GRAPH)
    dist = {a: {b: (0 if a == b else GRAPH[a].get(b, math.inf)) for b in rooms} for a in rooms}
    for k in rooms:
        for i in rooms:
            for j in rooms:
                if dist[i][k] + dist[k][j] < dist[i][j]:
                    dist[i][j] = dist[i][k] + dist[k][j]
    return dist


DIST = all_pairs_shortest() # only used to plan the visiting order and set deadlines, never by bfs / ucs / a*


def plan_order(patients): # static visiting order, planned once and shared by all three algorithms
    queue = [(deadline, pid, room) for pid, room, deadline in patients]
    heapq.heapify(queue) # priority queue, earliest deadline first

    order = []
    notes = []
    here = START
    clock = 0
    while queue:
        u_deadline, u_pid, u_room = queue[0] # the most urgent waiting patient
        pick = queue[0]

        if clock + DIST[here][u_room] <= u_deadline: # urgent patient can still make it, so a detour is worth checking
            best = None
            for item in queue[1:]:
                c_deadline, c_pid, c_room = item
                to_c = DIST[here][c_room]
                if to_c >= DIST[here][u_room]:
                    continue # not nearer than the urgent patient
                if clock + to_c > c_deadline:
                    continue # the nearer patient would be late anyway
                if clock + to_c + DIST[c_room][u_room] > u_deadline:
                    continue # the detour would make the urgent patient late
                key = (to_c, c_deadline, c_pid)
                if best is None or key < best[0]:
                    best = (key, item)
            if best is not None:
                pick = best[1]
                notes.append(f"patient {pick[1]} ({pick[2]}) goes before patient {u_pid} ({u_room}): "
                             f"nearer, and {u_room} still arrives by {fmt(u_deadline)} {UNIT}")

        queue.remove(pick)
        heapq.heapify(queue)
        deadline, pid, room = pick
        order.append((pid, room, deadline))
        clock += DIST[here][room]
        here = room
    return order, notes


def run_mission(name, search, order): # follow the planned order, one search per leg
    here = START
    clock = 0
    on_time = 0
    lateness = 0
    nodes = 0
    legs = []

    for pid, room, deadline in order:
        path, expanded = search(here, room)
        nodes += len(expanded)
        clock += path_cost(path)
        late_by = max(0, clock - deadline) # minutes past the deadline, 0 if on time
        if late_by == 0:
            on_time += 1
        lateness += late_by
        legs.append({"pid": pid, "room": room, "deadline": deadline, "arrival": clock,
                     "late_by": late_by, "path": path, "expanded": expanded})
        here = room

    delivery_cost = clock
    back_path, back_expanded = search(here, START)
    nodes += len(back_expanded)
    return_cost = path_cost(back_path)

    return {
        "name": name,
        "legs": legs,
        "back_from": here,
        "back_path": back_path,
        "back_expanded": back_expanded,
        "on_time": on_time,
        "total": len(order),
        "lateness": lateness,
        "delivery_cost": delivery_cost,
        "return_cost": return_cost,
        "total_cost": delivery_cost + return_cost,
        "nodes": nodes,
    }


def print_leg(path, expanded): # route and expanded nodes of one search
    if SHOW_ROUTES:
        print(f"     route   : {' -> '.join(path)}")
    if SHOW_EXPANDED:
        print(f"     expanded: {', '.join(expanded)} ({len(expanded)} nodes)")


def print_detail(res): # what the robot did, leg by leg (events only, no totals)
    print("=" * 60)
    print(f"{res['name']}: delivery run log")
    print("=" * 60)
    for leg in res["legs"]:
        status = "on time" if leg["late_by"] == 0 else f"late by {fmt(leg['late_by'])} {UNIT}"
        print(f" - patient {leg['pid']} ({leg['room']}) [deadline: {fmt(leg['deadline'])} {UNIT}]: "
              f"arrived at {fmt(leg['arrival'])} {UNIT} --> {status}")
        print_leg(leg["path"], leg["expanded"])
    print(f" - return to {START} (from {res['back_from']})")
    print_leg(res["back_path"], res["back_expanded"])
    print()


def print_comparison(results): # one side-by-side table, the only place the metrics appear
    late_nodes = []
    for r in results:
        nodes_for_r = [leg['room'] for leg in r['legs'] if leg['late_by'] > 0]
        late_nodes.append(", ".join(nodes_for_r) if nodes_for_r else "none")
    
    combined_lateness_str = " || ".join(late_nodes)

    rows = [
        ("on-time deliveries", [f"{r['on_time']} / {r['total']} ({100 * r['on_time'] / r['total']:.0f}%)" for r in results]),
        ("total lateness", [f"{fmt(r['lateness'])} {UNIT}" for r in results]),
        ("total delivery cost", [f"{fmt(r['delivery_cost'])} {UNIT}" for r in results]),
        ("return path cost", [f"{fmt(r['return_cost'])} {UNIT}" for r in results]),
        ("total trip cost", [f"{fmt(r['total_cost'])} {UNIT}" for r in results]),
        ("nodes expanded", [f"{r['nodes']} nodes" for r in results]),
    ]
    print("=" * 60)
    print("algorithm comparison (same patients, same visiting order)")
    print("=" * 60)
    print(f"{'metric':<21}" + "".join(f"{r['name']:<13}" for r in results))
    print("-" * 60)
    for label, values in rows:
        print(f"{label:<21}" + "".join(f"{v:<13}" for v in values))
    print("-" * 60)
    print(f"lateness locations : {combined_lateness_str}")
    print("=" * 60)


URGENCY_MULTIPLIERS = { # priority level deadline slack factors
    1: 1.15,
    2: 1.35,
    3: 1.60,
    4: 1.85,
    5: 2.20
}


def read_patients(): # interactive console patient prompt
    lookup = {room.lower(): room for room in GRAPH if room != START}

    while True:
        text = input("how many patients need medication? ").strip()
        if text.isdigit() and int(text) >= 1:
            count = int(text)
            break
        print("  please enter whole number at least 1.")

    print("\navailable rooms: " + ", ".join(sorted(lookup.values())))
    while True:
        text = input(f"enter {count} room(s) needing medication, separated by commas: ")
        names = [" ".join(part.split()).lower() for part in text.split(",") if part.strip()]
        unknown = [n for n in names if n not in lookup]
        if len(names) != count:
            print(f"  entered {len(names)} room(s), expected {count}.")
        elif unknown:
            print("  unknown room(s): " + ", ".join(unknown))
        else:
            rooms = [lookup[n] for n in names]
            break

    print("\n--- urgency levels (1 = critical, 5 = routine) ---")
    while True:
        text = input(f"enter urgency level (1-5) for {count} room(s), separated by commas\n"
                     "(or press enter for default level 3): ").strip()
        if text == "":
            levels = [3] * count
            break
        try:
            levels = [int(x) for x in text.split(",")]
        except ValueError:
            print("  urgency levels must be whole numbers between 1 and 5.")
            continue
        if len(levels) != count or any(lvl < 1 or lvl > 5 for lvl in levels):
            print(f"  please enter exactly {count} levels between 1 and 5.")
        else:
            break

    # assign fixed independent deadlines based on the fastest trip from start
    patients = []
    for i in range(count):
        r = rooms[i]
        lvl = levels[i]
        direct_dist = DIST[START][r] # fastest possible walking time from the pharmacy
        deadline = round(direct_dist * URGENCY_MULTIPLIERS[lvl], 1)
        patients.append((101 + i, r, deadline))

    return patients


def main(): # execution entry point
    patients = read_patients()
    order, notes = plan_order(patients)

    print("\n" + "=" * 60)
    print("planned visiting order (same for all 3 algorithms)")
    print("=" * 60)
    for n, (pid, room, deadline) in enumerate(order, 1):
        print(f" {n}. patient {pid} ({room}) [deadline: {fmt(deadline)} {UNIT}]")
    for note in notes:
        print(f"    * {note}")
    print()

    results = []
    for name, search in [("bfs", bfs), ("ucs", ucs), ("a*", astar)]:
        res = run_mission(name, search, order)
        print_detail(res)
        results.append(res)

    print_comparison(results)

    print("=" * 60)
    print("full paths taken (pharmacy -> deliveries -> return)")
    print("=" * 60)
    for r in results:
        nodes_seq = []
        for leg in r["legs"]:
            if not nodes_seq:
                nodes_seq.extend(leg["path"])
            else:
                nodes_seq.extend(leg["path"][1:])
        if r["back_path"]:
            nodes_seq.extend(r["back_path"][1:])
        print(f" {r['name']:<5}: {' -> '.join(nodes_seq)}")
    print("=" * 60)


if __name__ == "__main__":
    main()