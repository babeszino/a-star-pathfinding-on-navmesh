extends CharacterBody2D

# movement property-k
@export var movement_speed : float = 100.0
@export var target_path : NodePath
@export var path_update_interval : float = 0.5
@export var arrival_distance : float = 16.0
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
		# ---
		# cost szamolas a kiindulasi ponttol az aktualis node-ig
		# ---
		
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
	
	NavigationServer2D.map_force_update(nav_map_rid)
	
	_update_path()


func _process(delta):
	# utvonal frissitese idonkent
	path_timer += delta
	if path_timer >= path_update_interval:
		path_timer = 0.0
		_update_path()
	
	# talalt utvonal kovetese
	if path.size > 0:
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
	
	# megnezzuk, hogy elertuk e az aktualis waypointot
	if global_position.distance_to(target_pos) < arrival_distance:
		current_path_index += 1


# A* algoritmus
func _find_path_astar(start_pos: Vector2, end_pos: Vector2) -> Array:
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
