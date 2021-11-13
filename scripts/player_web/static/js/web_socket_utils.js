//------------------------------------
// 初始化连接
//------------------------------------
function init_socket() {
    window.socket = io();
    socket.on('connect', function () {
        console.log('connect')
        // $('#red_hp_text').text(300)
        $('#start_btn').removeAttr("disabled");
        $('#start_btn').text("开始游戏");

    });

    socket.on('disconnect', function () {
        console.log('disconnect')
    });

    

}

//------------------------------------
// 开始监听事件
//------------------------------------
function start_socket_transfer(socket){
    var start_time;
    var i = 0
    var ping_arr = []
    var my_camera = document.getElementById("camera")
    var blue_bar = $('#hp_bar_blue')
    var red_bar = $('#hp_bar_red')
    socket = window.socket
    socket.on('image', function (message) {
        i++
        my_camera.setAttribute('src', 'data:image/png;base64,' + message.img)
    });
    socket.on('blue_hp', function (message) {
        console.log('blue_hp',message.value )
        var hp = message.value 
        // console.log(`${hp/5}%`)
        blue_bar.css("width", `${hp/5}%`)
        // blue_bar.val(hp+'')
        $('#blue_hp_text').text(hp+'')
    });
    socket.on('red_hp', function (message) {
        console.log('red_hp',message.value)
        var hp = message.value 
        red_bar.css("width", `${hp/5}%`)
        $('#red_hp_text').text(hp+'')
    });
    window.fp_timer = setInterval(() => {
        start_time = (new Date).getTime();
        // console.log("帧率:" + i)
        $('#fps').text(i);
        i = 0
        socket.emit('ping');
    }, 1000)
    socket.on('pong', function () {
        // console.log('pong')
        var latency = (new Date).getTime() - start_time;
        ping_arr.push(latency);
        ping_arr = ping_arr.slice(-30);
        var sum = 0;
        for (var i = 0; i < ping_arr.length; i++)
            sum += ping_arr[i];
        ping = Math.round(10 * sum / ping_arr.length) / 10
        // console.log(ping)
        $('#ping').text(ping + 'ms');
    });
    window.control_timer = setInterval(() => {
        socket.emit('control', active_map);
        // console.log(active_map)
    }, send_freq)

}

//------------------------------------
// 关闭连接
//------------------------------------





