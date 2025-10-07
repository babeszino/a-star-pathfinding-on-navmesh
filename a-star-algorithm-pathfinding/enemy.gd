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
