extends CharacterBody2D

# movement property-k
@export var movement_speed : float = 100.0
@export var target_path : NodePath
@export var path_update_interval : float = 0.15
@export var arrival_distance : float = 10.0
@export var debug_draw : bool = true

# navigacios es utvonalkeresos valtozok
var target : Node2D
var path : Array = []
var current_path_index : int = 0
var nav_map_rid : RID
var path_timer : float = 0.0

# A* algoritmus implementacio
class AStarNode:
	var position : Vector2
	var parent : AStarNode
	var g_cost : float = 0.0 # cost a start-tol
	var h_cost : float = 0.0 # heurisztika cost
	var f_cost : float = 0.0 # teljes cost (g + h erteke)

	func _init(pos: Vector2, parent_node = null):
		position = pos
		parent = parent_node

	func calculate_costs(start_pos: Vector2, end_pos: Vector2):
		# cost szamolas a kiindulasi ponttol az aktualis node-ig
		if parent:
			g_cost = parent.g_cost + position.distance_to(parent.position)
		else:
			g_cost = 0.0
		
		# heurisztikus cost (becsult tavolsag a celig)
		h_cost = position.distance_to(end_pos)
		
		# teljes cost
		f_cost = g_cost + h_cost


func _ready():
	# player node lekeres
	if not target_path.is_empty():
		target = get_node(target_path)
	else:
		target = get_node("/root/Main/Player")
	
	# navigacios terulet lekerese
	var nav_regions = get_tree().get_nodes_in_group("navigation_region")
	if nav_regions.size() > 0:
		nav_map_rid = nav_regions[0].get_world_2d().get_navigation_map()
	else:
		# ha a groupban nincs nav terulet, akkor lekeres direktben
		var nav_region = get_node("/root/Main/NavigationRegion2D")
		if nav_region:
			nav_map_rid = nav_region.get_world_2d().get_navigation_map()
	
	# Wait for navigation to be ready
	call_deferred("_initialize_navigation")


func _initialize_navigation():
	if nav_map_rid:
		NavigationServer2D.map_force_update(nav_map_rid)
		# Wait one more frame for the navigation to be ready
		await get_tree().process_frame
		_update_path()


func _process(delta):
	# utvonal frissitese idonkent
	path_timer += delta
	if path_timer >= path_update_interval:
		path_timer = 0.0
		_update_path()
	
	# talalt utvonal kovetese
	if path.size() > 0:
		_follow_path()
	
	if debug_draw:
		queue_redraw()

func _update_path():
	if !is_instance_valid(target) or !nav_map_rid:
		return
	
	# utvonal keresese A* algoval
	var new_path = _find_path_astar(global_position, target.global_position)
	if new_path.size() > 0:
		path = new_path
		current_path_index = min(1, path.size() -1) # elso waypoint-tol indulas


func _follow_path():
	# ha elertuk az utvonal veget, stop
	if current_path_index >= path.size():
		return
	
	# jelenlegi cel poziciojanak lekerdezese
	var target_pos = path[current_path_index]
	
	# utvonal szamitas a target_pos-hoz
	var direction = (target_pos - global_position).normalized()
	
	# mozgas a target_pos fele
	velocity = direction * movement_speed
	move_and_slide()
	
	# falnak utkozeskor ujraszamitas
	if get_slide_collision_count() > 0:
		print("Enemy hit wall - recalculating path")
		_update_path()
	
	# megnezzuk, hogy elertuk e az aktualis waypointot
	if global_position.distance_to(target_pos) < arrival_distance:
		current_path_index += 1


# A* algoritmus
func _find_path_astar(start_pos: Vector2, end_pos: Vector2) -> Array:
	if !nav_map_rid:
		return []
		
	var start_point = NavigationServer2D.map_get_closest_point(nav_map_rid, start_pos)
	var end_point = NavigationServer2D.map_get_closest_point(nav_map_rid, end_pos)
	
	# open es closed set-ek
	var open_set : Array = []
	var closed_set : Array = []
	
	# kiindulasi (start) node letrehozasa
	var start_node = AStarNode.new(start_point)
	start_node.calculate_costs(start_point, end_point)
	open_set.append(start_node)
	
	# Fő A* ciklus
	while open_set.size() > 0:
		# legkisebb f_cost node megkeresese az open_set-ben
		var current_node = _get_lowest_f_cost_node(open_set)
		
		# jelenlegi node atrakasa closed set-be open-bol
		open_set.erase(current_node)
		closed_set.append(current_node)
		
		# ellenorzes, hogy elertuk-e a celt
		if current_node.position.distance_to(end_point) < arrival_distance:
			return _reconstruct_path(current_node)
		
		# szomszedos node-ok lekerese
		var neighbors = _get_neighbors(current_node, end_point)
		
		# szomszededos node-ok kiertekelese
		for neighbor in neighbors:
			# skip ha a closed set-ben van
			if _is_in_set(neighbor.position, closed_set):
				continue
			
			var in_open_node = _find_in_set(neighbor.position, open_set)
			
			if in_open_node:
				# ha van jobb utvonal a jelenlegi node-hoz, update
				if neighbor.g_cost < in_open_node.g_cost:
					in_open_node.g_cost = neighbor.g_cost
					in_open_node.f_cost = neighbor.g_cost + in_open_node.h_cost
					in_open_node.parent = current_node
			
			else:
				open_set.append(neighbor)
		
	return [] # ha idaig eljutunk, akkor elvileg nem talaltunk utvonalat


# helper func -> legkisebb f_cost node keresese
func _get_lowest_f_cost_node(nodes: Array) -> AStarNode:
	var lowest = nodes[0]
	for node in nodes:
		if node.f_cost < lowest.f_cost:
			lowest = node
	
	return lowest


# helper func -> szomszedos node generalas A* algohoz
func _get_neighbors(node: AStarNode, goal_pos: Vector2) -> Array:
	var neighbors : Array = []
	
	# sampling tavolsag
	var sample_distance = 16.0
	
	# cardinal + diagonal iranyok definialasa
	var directions = [
		Vector2(1, 0), Vector2(-1, 0),
		Vector2(0, 1), Vector2(0, -1),
		Vector2(1, 1), Vector2(-1, 1),
		Vector2(1, -1), Vector2(-1, -1)
	]
	
	# point sample-oles minden iranyba
	for dir in directions:
		var sample_pos = node.position + dir.normalized() * sample_distance
		
		# legkozelebbi pont megkeresese navmesh-en
		var closest_point = NavigationServer2D.map_get_closest_point(nav_map_rid, sample_pos)
		
		# csak olyan point-okat hasznaljunk, amik validak (kozel vannak a sample pos-hoz)
		if closest_point.distance_to(sample_pos) < sample_distance * 0.5:
			# ellenorzes, hogy tiszta e az ut
			var path_to_point = NavigationServer2D.map_get_path(
				nav_map_rid,
				node.position,
				closest_point,
				true
			)
			
			# ha talaltunk valid utvonalat
			if path_to_point.size() > 1:
				var neighbor = AStarNode.new(closest_point, node)
				neighbor.calculate_costs(node.position, goal_pos)
				neighbors.append(neighbor)
	
	return neighbors


# ellenorizzuk, hogy a position a set-ben van-e
func _is_in_set(position: Vector2, node_set: Array, threshold: float = 8.0) -> bool:
	for node in node_set:
		if node.position.distance_to(position) < threshold:
			return true
		
	return false


# node megkeresese a set-ben
func _find_in_set(position: Vector2, node_set: Array, threshold: float = 8.0) -> AStarNode:
	for node in node_set:
		if node.position.distance_to(position) < threshold:
			return node
	
	return null


# utvonal reconstruct az end node-bol start node-ba
func _reconstruct_path(end_node: AStarNode) -> Array:
	var path = []
	var current = end_node
	
	while current != null:
		path.append(current.position)
		current = current.parent
	
	# megforditas (hogy az elejetol a vegeig legyen)
	path.reverse()
	return path

func _draw():
	if !debug_draw or path.size() <= 1:
		return
	
	var local_points = []
	var node_scale = scale.x
	for point in path:
		local_points.append((point - global_position) / node_scale)
	
	for i in range(local_points.size() - 1):
		draw_line(
			local_points[i],
			local_points[i + 1],
			Color.RED,
			1.0 / node_scale
		)
	
	for i in range(local_points.size()):
		var color : Color
		var radius : float
		
		if i == current_path_index:
			color = Color.YELLOW
			radius = 2.0 / node_scale
		else:
			color = Color.WHITE
			radius = 1.5 / node_scale
		
		draw_circle(local_points[i], radius, color)
