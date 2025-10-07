extends CharacterBody2D

var movement_speed : int = 150
var screen_size


func _ready():
	screen_size = get_viewport_rect().size


func _physics_process(_delta: float) -> void:
	velocity = Vector2(0,0)
	
	if Input.is_action_pressed("right"):
		velocity.x += 1
	if Input.is_action_pressed("left"):
		velocity.x -= 1
	if Input.is_action_pressed("down"):
		velocity.y += 1
	if Input.is_action_pressed("up"):
		velocity.y -= 1

	if velocity.length() > 0:
		velocity = velocity.normalized() * movement_speed
	else:
		velocity = Vector2.ZERO
	
	move_and_slide()
	
	position = position.clamp(Vector2.ZERO, screen_size)
