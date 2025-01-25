// Last edited by Dietrich Geisler 2025

const VSHADER_SOURCE = `
    attribute vec3 a_Position;
    uniform mat4 u_Model;
    uniform mat4 u_World;
    void main() {
        gl_Position = u_World * u_Model * vec4(a_Position, 1.0);
    }
`

const FSHADER_SOURCE = `
    void main() {
        gl_FragColor = vec4(0.5, 0.2, 1.0, 1.0);
    }
`

// references to general information
var g_canvas
var gl
var g_lastFrameMS

// GLSL uniform references
var g_u_model_ref
var g_u_world_ref

// usual model/world matrices
var g_modelMatrix
var g_worldMatrix

// the current axis of rotation
var g_rotationAxis



// Unit cube mesh, size 1, oriented around zero
// TODO: replace with your mesh :)
const VERTICES = [
    // front face
   generateFront(1),

    1.0, 1.0, 1.0,
    -1.0, -1.0, 1.0,
    1.0, -1.0, 1.0,

    // back face
    1.0, 1.0, -1.0,
    -1.0, -1.0, -1.0,
    -1.0, 1.0, -1.0,

    1.0, 1.0, -1.0,
    1.0, -1.0, -1.0,
    -1.0, -1.0, -1.0,

    // right face
    1.0, 1.0, 1.0,
    1.0, -1.0, -1.0,
    1.0, 1.0, -1.0,

    1.0, 1.0, 1.0,
    1.0, -1.0, 1.0,
    1.0, -1.0, -1.0,

    // left face
    -1.0, 1.0, 1.0,
    -1.0, 1.0, -1.0,
    -1.0, -1.0, -1.0,

    -1.0, 1.0, 1.0,
    -1.0, -1.0, -1.0,
    -1.0, -1.0, 1.0,

    // top face
    1.0, 1.0, 1.0,
    1.0, 1.0, -1.0,
    -1.0, 1.0, -1.0,

    1.0, 1.0, 1.0,
    -1.0, 1.0, -1.0,
    -1.0, 1.0, 1.0,

    // bottom face
    1.0, -1.0, 1.0,
    -1.0, -1.0, -1.0,
    1.0, -1.0, -1.0,

    1.0, -1.0, 1.0,
    -1.0, -1.0, 1.0,
    -1.0, -1.0, -1.0,
]
const VERTEX_COUNT = VERTICES.length / 3

// We're using triangles, so our vertices each have 3 elements
const TRIANGLE_SIZE = 3

// The size in bytes of a floating point
const FLOAT_SIZE = 4

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

    // initialize the VBO
    var data = VERTICES
    if (!initVBO(new Float32Array(data))) {
        return
    }

    // Send our vertex data to the GPU
    if (!setupVec3('a_Position', 0, 0)) {
        return
    }

    // Get references to GLSL uniforms
    g_u_model_ref = gl.getUniformLocation(gl.program, 'u_Model')
    g_u_world_ref = gl.getUniformLocation(gl.program, 'u_World')

    // Setup our model by scaling
    g_modelMatrix = new Matrix4()
    g_modelMatrix = g_modelMatrix.setScale(.5, .5, .5)

    // Reposition our mesh (in this case as an identity operation)
    g_worldMatrix = new Matrix4()

    // Enable Culling
    gl.enable(gl.CULL_FACE)

    // Setup for ticks
    g_lastFrameMS = Date.now()
    g_rotationAxis = [0, 0, 0]

    // Make sure we have a defined rotation
    updateRotation()

    tick()
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

    // Update with our global transformation matrices
    gl.uniformMatrix4fv(g_u_model_ref, false, g_modelMatrix.elements)
    gl.uniformMatrix4fv(g_u_world_ref, false, g_worldMatrix.elements)

    // Clear the canvas with a black background
    gl.clearColor(0.0, 0.0, 0.0, 1.0)
    gl.clear(gl.COLOR_BUFFER_BIT)
    gl.drawArrays(gl.TRIANGLES, 0, VERTEX_COUNT)
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

/*
 * Initialize the VBO with the provided data
 * Assumes we are going to have "static" (unchanging) data
 */
function initVBO(data) {
    // get the VBO handle
    var VBOloc = gl.createBuffer()
    if (!VBOloc) {
        console.log('Failed to create the vertex buffer object')
        return false
    }

    // Bind the VBO to the GPU array and copy `data` into that VBO
    gl.bindBuffer(gl.ARRAY_BUFFER, VBOloc)
    gl.bufferData(gl.ARRAY_BUFFER, data, gl.STATIC_DRAW)

    return true
}

/*
 * Helper function to load the given vec3 data chunk onto the VBO
 * Requires that the VBO already be setup and assigned to the GPU
 */
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