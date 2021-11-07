//------------------------------------
// 初始化连接
//------------------------------------
function init_socket() {
    let i = 0
    var my_camera = document.getElementById("camera")

    var socket = io();
    socket.on('connect', function () {
        console.log('connect')
        
    });
    
    socket.on('disconnect', function () {
        console.log('disconnect')
        clearInterval(timer)
    });

    socket.on('image', function (message) {
        i++
        setInterval(() => {
            console.log("帧率:"+i)
            i = 0
        }, 1000)
        my_camera.setAttribute('src', 'data:image/png;base64,' + message.img)
    });
    timer = setInterval(() => {
        socket.emit('control', active_map);
        // console.log(active_map)
    }, send_freq)
    
}

//------------------------------------
// 开始监听事件
//------------------------------------
// function start_socket_transfer(socket){
//     let i = 0
//     var my_camera = document.getElementById("camera")
//     socket.on('image', function (message) {
//         i++
//         setInterval(() => {
//             console.log("帧率:"+i)
//             i = 0
//         }, 1000)
//         my_camera.setAttribute('src', 'data:image/png;base64,' + message.img)
//     });
//     timer = setInterval(() => {
//         socket.emit('control', active_map);
//         // console.log(active_map)
//     }, send_freq)

// }





