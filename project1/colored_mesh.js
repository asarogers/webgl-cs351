// Last edited by Dietrich Geisler 2025

const VSHADER_SOURCE = `
    attribute vec3 a_Position;
    uniform mat4 u_Model;
    uniform mat4 u_World;
    attribute vec3 a_Color;
    varying vec3 v_Color;
    void main() {
        gl_Position = u_World * u_Model * vec4(a_Position, 1.0);
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

// usual model/world matrices
var g_modelMatrix
var g_multi_modelMatrix
var g_worldMatrix

// the current axis of rotation
var g_rotationAxis


// Unit cube mesh, size 1, oriented around zero
// TODO: replace with your mesh :)
class Square{
    constructor(VERTICES){
        this.VERTICES = VERTICES
        this.VERTEX_COUNT = VERTICES.length / 3
        this.TRIANGLE_SIZE = 3
    }
}

firstSquare = new Square(generateSquareFromVertex(0.5))
secondSquare = new Square(generateSquareFromVertex(1.0))
const TRIANGLE_SIZE = 3
const FLOAT_SIZE = 4


const movement = Object.freeze({
    HORIZONTAL: "horizontal",
    VERTICAL: "vertical",
    TEST: "test",
    ROTATE: "rotate",
    RESET: "reset"
})

var data = {}
let vertexBuffer1, vertexBuffer2, colorBuffer1, colorBuffer2;
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

    // initialize the VBO
    initBuffers();
    reset()
    
    // Enable Culling
    gl.enable(gl.CULL_FACE)

    // Setup for ticks
    g_lastFrameMS = Date.now()
    g_rotationAxis = [0, 0, 0]

    // Make sure we have a defined rotation
    updateRotation()
    tick()
}


function initBuffers() {
    vertexBuffer1 = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, vertexBuffer1);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(firstSquare.VERTICES), gl.STATIC_DRAW);

    var colors = buildColorAttributes(firstSquare.VERTEX_COUNT);
    colorBuffer1 = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, colorBuffer1);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(colors), gl.STATIC_DRAW);
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
    

    draw()

    requestAnimationFrame(tick, g_canvas)
}

// draw to the screen on the next frame
function draw() {
    gl.clearColor(0.0, 0.0, 0.0, 1.0)
    gl.clear(gl.COLOR_BUFFER_BIT)

    drawShape(g_modelMatrix.elements, g_worldMatrix.elements, vertexBuffer1, colorBuffer1, firstSquare.VERTEX_COUNT);
}

function drawShape(model, world, vertexBuffer, colorBuffer, vertexCount) {
    // Set transformation matrices
    gl.uniformMatrix4fv(g_u_model_ref, false, new Float32Array(model));
    gl.uniformMatrix4fv(g_u_world_ref, false, new Float32Array(world));

    // Bind the vertex buffer and set up the a_Position attribute
    gl.bindBuffer(gl.ARRAY_BUFFER, vertexBuffer);
    setupVec3('a_Position', 0, 0);

    // Bind the color buffer and set up the a_color attribute
    gl.bindBuffer(gl.ARRAY_BUFFER, colorBuffer);
    setupVec3('a_Color', 0, 0);

    // Draw the shape
    gl.drawArrays(gl.TRIANGLES, 0, vertexCount);
}

// Helper to construct colors
// makes every triangle a slightly different shade of blue
function buildColorAttributes(vertex_count) {
    const color1 = [.5, 0.9, 0.];
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

function setupVec3(name, stride, offset) {
    // Get the attribute by name
    var attributeID = gl.getAttribLocation(gl.program, `${name}`)
    if (attributeID < 0) {
        console.log(`Failed to get the storage location of ${name}`)
        return false
    }

    // Set how the GPU fills the a_Position variable with data from the GPU 
    gl.vertexAttribPointer(attributeID, 3, gl.FLOAT, false, stride, offset)
    gl.enableVertexAttribArray(attributeID)

    return true
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
    g_modelMatrix = new Matrix4()
    g_worldMatrix = new Matrix4()
    g_modelMatrix = move3DShape(g_modelMatrix, movement.RESET, ID_MATRIX)
    g_modelMatrix = g_modelMatrix.setScale(.5, .5, .5)

    g_multi_modelMatrix = new Matrix4()
    g_multi_modelMatrix = move3DShape(g_multi_modelMatrix, movement.VERTICAL, 0.5)
}