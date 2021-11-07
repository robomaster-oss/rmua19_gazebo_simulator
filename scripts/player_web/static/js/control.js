
function key_down_controller(keyCode){
	active_map[key_map[keyCode]] = true
	$("#"+key_map[keyCode]).addClass("key_active");
}

function key_up_controller(keyCode){
	active_map[key_map[keyCode]] = false
	$("#"+key_map[keyCode]).removeClass("key_active");
}

function mouse_down_controller(){
	$("#shoot").addClass("key_active");
	active_map['shoot'] = true
}

function mouse_up_controller(){
	active_map['shoot'] = false
	$("#shoot").removeClass("key_active");
}

// function mousemoveX_controller(value){
// 	active_map['movementX'] = value
// }
//
// function mousemoveY_controller(value){
// 	active_map['movementY'] = value
// }