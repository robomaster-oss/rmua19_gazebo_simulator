
//------------------------------------
// keycode映射按键
//------------------------------------
var key_map = {
	87:"w",
	83:"s",
	65:"a",
	68:"d",
	81:"q",
	69:"e",
	27:"esc"
	
}

//------------------------------------
// 按键是否被按下
//------------------------------------
var active_map = {
	"w":false,
	"s":false,
	"a":false,
	"d":false,
	"q":false,
	"e":false,
	"shoot":false,
	"movementX":0,
	"movementY":0
}

//------------------------------------
// 发送间隔
//------------------------------------
var send_freq = 30

//------------------------------------
// 用于调整yaw的灵敏度， 越大越不灵敏
//------------------------------------
var yaw_factor = 800

//------------------------------------
// 越大越不灵敏
//------------------------------------
var pitch_factor = 800

//------------------------------------
// 发送指令的定时器
//------------------------------------
// var timer 

//------------------------------------
// 发送指令的定时器
// 0设置 1游戏
//------------------------------------
var interface_status = 0

// var socket