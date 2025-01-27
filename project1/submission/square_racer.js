// Last edited by Dietrich Geisler 2025

const VSHADER_SOURCE = `
    attribute vec3 a_Position;
    uniform mat4 u_Model;
    uniform mat4 u_World;
    uniform mat4 u_Camera;
    attribute vec3 a_Color;
    varying vec3 v_Color;
    void main() {
        gl_Position = u_Camera * u_World * u_Model * vec4(a_Position, 1.0);
        v_Color = a_Color;
    }
`

const FSHADER_SOURCE = `
    varying mediump vec3 v_Color;
    void main() {
        gl_FragColor = vec4(v_Color, 1.0);
    }
`

// references to general information
var g_canvas
var gl
var g_lastFrameMS
var firstBuffer, secondBuffer

// GLSL uniform references
var g_u_model_ref
var g_u_world_ref
var g_u_camera_ref

// usual model/world matrices
var g_modelMatrix
var g_second_modelMatrix
var g_third_modelMatrix
var g_worldMatrix

// the current axis of rotation
var g_rotationAxis


// camera projection values
var g_camera_x
var g_camera_y
var g_camera_z
var g_near

// Unit cube mesh, size 1, oriented around zero
// TODO: replace with your mesh :)
class Square{
    constructor(VERTICES){
        this.VERTICES = VERTICES;
        this.VERTEX_COUNT = VERTICES.length / 3;
        this.TRIANGLE_SIZE = 3;
        this.COLOR_BUFFER = null;
        this.BUFFER = null
        this.SPEED = 0
        this.COLORS = null

    }

    bind(buffer, colorBuffer, colors){
        this.COLOR_BUFFER = colorBuffer;
        this.BUFFER = buffer;
        this.COLORS = colors

        gl.bindBuffer(gl.ARRAY_BUFFER, this.BUFFER);
        gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(this.VERTICES), gl.STATIC_DRAW);

        gl.bindBuffer(gl.ARRAY_BUFFER, this.COLOR_BUFFER);
        gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(this.COLORS), gl.STATIC_DRAW);
    }

    changeColor(colors){
        this.COLORS = colors
        gl.bindBuffer(gl.ARRAY_BUFFER, this.BUFFER);
        gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(this.VERTICES), gl.STATIC_DRAW);

        gl.bindBuffer(gl.ARRAY_BUFFER, this.COLOR_BUFFER);
        gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(this.COLORS), gl.STATIC_DRAW);
    }
}


const TRIANGLE_SIZE = 3
const FLOAT_SIZE = 4


const movement = Object.freeze({
    HORIZONTAL: "horizontal",
    VERTICAL: "vertical",
    TEST: "test",
    ROTATE: "rotate",
    RESET: "reset"
})

firstSquare = new Square(generateSquareFromVertex(0.5))
secondSquare = new Square(generateSquareFromVertex(0.5))
thirdSquare = new Square(generateSquareFromVertex(0.5))

var translate_time;

var data = {}
function main() {
    g_canvas = document.getElementById('canvas')
    // Get the rendering context for WebGL
    gl = getWebGLContext(g_canvas, true)
    if (!gl) {
        console.log('Failed to get the rendering context for WebGL')
        return
    }

    // Initialize GPU's vertex and fragment shaders programs
    if (!initShaders(gl, VSHADER_SOURCE, FSHADER_SOURCE)) {
        console.log('Failed to intialize shaders.')
        return
    }
    // Get references to GLSL uniforms
    g_u_model_ref = gl.getUniformLocation(gl.program, 'u_Model')
    g_u_world_ref = gl.getUniformLocation(gl.program, 'u_World')
    g_u_camera_ref = gl.getUniformLocation(gl.program, 'u_Camera')
    g_u_projection_ref = gl.getUniformLocation(gl.program, 'u_Projection')

    // initialize the VBO
    initBuffers();
    reset()
    
    // Enable Culling
    gl.enable(gl.CULL_FACE)

    // Setup for ticks
    g_lastFrameMS = Date.now()
    g_rotationAxis = [0, 0, 0]
    setupCamera()
    updateRotation()
    tick()
}

function initBuffers() {
    let vertexBuffer1 = gl.createBuffer();
    var colors = buildColorAttributes(firstSquare.VERTEX_COUNT);
    let colorBuffer1 = gl.createBuffer();
    firstSquare.bind(vertexBuffer1, colorBuffer1, colors)

    let vertexBuffer2 = gl.createBuffer();
    var colors = buildColorAttributes(secondSquare.VERTEX_COUNT);
    let colorBuffer2 = gl.createBuffer();
    secondSquare.bind(vertexBuffer2, colorBuffer2, colors)

    let vertexBuffer3 = gl.createBuffer();
    var colors = buildColorAttributes(thirdSquare.VERTEX_COUNT);
    let colorBuffer3 = gl.createBuffer();
    thirdSquare.bind(vertexBuffer3, colorBuffer3, colors)

}


// extra constants for cleanliness
var ROTATION_SPEED = .05


// function to apply all the logic for a single frame tick
function tick() {
    // time since the last frame
    var deltaTime

    // calculate deltaTime
    var current_time = Date.now()
    deltaTime = current_time - g_lastFrameMS
    g_lastFrameMS = current_time

    // rotate the arm constantly around the given axis (of the model)
    angle = ROTATION_SPEED * deltaTime 
    g_modelMatrix.concat(new Matrix4().setRotate(angle, ...g_rotationAxis))
    g_second_modelMatrix.concat(new Matrix4().setRotate(secondSquare.SPEED * 100, ...g_rotationAxis))
    g_third_modelMatrix.concat(new Matrix4().setRotate(thirdSquare.SPEED * 100, ...g_rotationAxis))

    translate_time += deltaTime
    if (translate_time >= 3000){
        translate_time = 0
        reset_moving_shapes()

        setSpeed()
        changeColor()
    }
    
    // console.log(g_second_modelMatrix)



    g_second_modelMatrix = move3DShape(g_second_modelMatrix, movement.HORIZONTAL, -secondSquare.SPEED)
    g_third_modelMatrix = move3DShape(g_third_modelMatrix, movement.HORIZONTAL, -thirdSquare.SPEED)
    

    draw()
    requestAnimationFrame(tick, g_canvas)
}

function changeColor(){
    var colors = buildColorAttributes(firstSquare.VERTEX_COUNT);
    firstSquare.changeColor(colors)

    var colors = buildColorAttributes(secondSquare.VERTEX_COUNT);
    secondSquare.changeColor(colors)

    var colors = buildColorAttributes(thirdSquare.VERTEX_COUNT);
    thirdSquare.changeColor(colors)
}
function setSpeed(){
    let lower = 0.005
    let upper = 0.01
    speed1 = lower + Math.random() * (upper - lower)
    speed2 = lower + Math.random() * (upper - lower)

    
    secondSquare.SPEED = speed2
    thirdSquare.SPEED = speed1
}

// draw to the screen on the next frame
function draw() {
    var cameraMatrix = new Matrix4().translate(-g_camera_x, -g_camera_y, -g_camera_z)
    var projection_matrix = new Matrix4().setOrtho(-1, 1, -1, 1, g_near, 2)

    




    gl.clearColor(0.0, 0.0, 0.0, 1.0)
    gl.clear(gl.COLOR_BUFFER_BIT)

    drawShape(g_modelMatrix.elements, g_worldMatrix.elements, firstSquare);
    drawShape(g_second_modelMatrix.elements, g_worldMatrix.elements, secondSquare);
    drawShape(g_third_modelMatrix.elements, g_worldMatrix.elements, thirdSquare);

    gl.uniformMatrix4fv(g_u_projection_ref, false, projection_matrix.elements)
    gl.uniformMatrix4fv(g_u_camera_ref, false, cameraMatrix.elements)
}

function setupCamera(){
    slider_input = document.getElementById('sliderX')
    slider_input.addEventListener('input', (event) => {
        updateCameraX(event.target.value)
    })
    slider_input = document.getElementById('sliderY')
    slider_input.addEventListener('input', (event) => {
        updateCameraY(event.target.value)
    })
    slider_input = document.getElementById('sliderZ')
    slider_input.addEventListener('input', (event) => {
        updateCameraZ(event.target.value)
    })

    slider_input = document.getElementById('sliderNear')
    slider_input.addEventListener('input', (event) => {
        updateNear(event.target.value)
    })

    updateCameraX(0)
    updateCameraY(0)
    updateCameraZ(0)

    updateNear(1)
}

function drawShape(model, world, shape) {
    // Set transformation matrices
    gl.uniformMatrix4fv(g_u_model_ref, false, new Float32Array(model));
    gl.uniformMatrix4fv(g_u_world_ref, false, new Float32Array(world));

    // Bind the vertex buffer and set up the a_Position attribute
    gl.bindBuffer(gl.ARRAY_BUFFER, shape.BUFFER);
    setupVec3('a_Position', 0, 0);

    // Bind the color buffer and set up the a_color attribute
    gl.bindBuffer(gl.ARRAY_BUFFER, shape.COLOR_BUFFER);
    setupVec3('a_Color', 0, 0);

    // Draw the shape
    gl.drawArrays(gl.TRIANGLES, 0, shape.VERTEX_COUNT);
}

// Helper to construct colors
// makes every triangle a slightly different shade of blue
function buildColorAttributes(vertex_count) {

    const color1 = [Math.random(), Math.random(), Math.random()];
    const color2 = [0.6, 0.4, 0.2];
    let colors = [];
    for (let i = 0; i < vertex_count; i++) {
        randomNumber = Math.floor(Math.random() * 10) + 1;
        // Alternate between color1 and color2 for each vertex
        if (randomNumber % 2 === 0) {
            colors.push(...color1);
        } else {
            colors.push(...color2); 
        }
    }
    return colors;
}


// Event to change which rotation is selected
function updateRotation() {
    var rotateX = document.getElementById('rotateX')
    var rotateY = document.getElementById('rotateY')
    var rotateZ = document.getElementById('rotateZ')

    g_rotationAxis[0] = Number(rotateX.checked)
    g_rotationAxis[1] = Number(rotateY.checked)
    g_rotationAxis[2] = Number(rotateZ.checked)
}

function updateNear(amount) {
    label = document.getElementById('near')
    label.textContent = `Near: ${Number(amount).toFixed(2)}`
    g_near = Number(amount)
}


function updateCameraX(amount) {
    label = document.getElementById('cameraX')
    label.textContent = `Camera X: ${Number(amount).toFixed(2)}`
    g_camera_x = Number(amount)
}
function updateCameraY(amount) {
    label = document.getElementById('cameraY')
    label.textContent = `Camera Y: ${Number(amount).toFixed(2)}`
    g_camera_y = Number(amount)
}
function updateCameraZ(amount) {
    label = document.getElementById('cameraZ')
    label.textContent = `Camera Z: ${Number(amount).toFixed(2)}`
    g_camera_z = Number(amount)
}


function rotate45(){
    g_modelMatrix = move3DShape(g_modelMatrix, movement.ROTATE,(45 * (Math.PI / 180)))
}

function translateLeft(){
    g_modelMatrix = move3DShape(g_modelMatrix, movement.HORIZONTAL, -0.5)
}

function translateRight(){
    g_modelMatrix = move3DShape(g_modelMatrix, movement.HORIZONTAL, 0.5)
}

function moveUp(){
    g_modelMatrix = move3DShape(g_modelMatrix, movement.VERTICAL, 0.5)
}

function moveDown(){
    g_modelMatrix = move3DShape(g_modelMatrix, movement.VERTICAL, -0.5)
}

function reset(){
    translate_time = 0
    g_modelMatrix = new Matrix4()
    g_worldMatrix = new Matrix4()
    g_modelMatrix = move3DShape(g_modelMatrix, movement.RESET, ID_MATRIX)
    g_modelMatrix = g_modelMatrix.setScale(.15, .15, .15)
    setSpeed()


    reset_moving_shapes()

}

function reset_moving_shapes(){
    g_second_modelMatrix = new Matrix4()
    g_second_modelMatrix = g_second_modelMatrix.setScale(0.25, 0.25, 0.25)
    g_second_modelMatrix = move3DShape(g_second_modelMatrix, movement.HORIZONTAL, 1)
    g_second_modelMatrix = move3DShape(g_second_modelMatrix, movement.VERTICAL, 0.5)

    g_third_modelMatrix = new Matrix4()
    g_third_modelMatrix = g_third_modelMatrix.setScale(0.25, 0.25, 0.25)
    g_third_modelMatrix = move3DShape(g_third_modelMatrix, movement.HORIZONTAL, 1)
    g_third_modelMatrix = move3DShape(g_third_modelMatrix, movement.VERTICAL, -0.5)
}