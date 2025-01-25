// Last edited by Dietrich Geisler 2025

const VSHADER_SOURCE = `
    attribute vec3 a_Position;
    uniform mat4 u_Model;
    uniform mat4 u_World;
    uniform mat4 u_Camera;
    uniform mat4 u_Projection;
    attribute vec3 a_Color;
    varying vec3 v_Color;
    void main() {
        gl_Position = u_Projection * u_Camera * u_World * u_Model * vec4(a_Position, 1.0);
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

// GLSL uniform references
var g_u_model_ref
var g_u_world_ref

// usual model/world matrices
var g_modelMatrix
var g_worldMatrix
var g_cameraMatrix

// camera projection values
var g_near
var g_far
var g_left
var g_right
var g_top
var g_bottom

// the current axis of rotation
var g_rotationAxis

// Whether or not we have culling and depth enabled
var g_cull
var g_depth

// Mesh definitions
var g_teapotMesh

// We're using triangles, so our vertices each have 3 elements
const TRIANGLE_SIZE = 3

// The size in bytes of a floating point
const FLOAT_SIZE = 4

function main() {
    // Setup our 6 sliders
    slider_input = document.getElementById('sliderNear')
    slider_input.addEventListener('input', (event) => {
        updateNear(event.target.value)
    })

    slider_input = document.getElementById('sliderFar')
    slider_input.addEventListener('input', (event) => {
        updateFar(event.target.value)
    })

    slider_input = document.getElementById('sliderLeft')
    slider_input.addEventListener('input', (event) => {
        updateLeft(event.target.value)
    })

    slider_input = document.getElementById('sliderRight')
    slider_input.addEventListener('input', (event) => {
        updateRight(event.target.value)
    })

    slider_input = document.getElementById('sliderTop')
    slider_input.addEventListener('input', (event) => {
        updateTop(event.target.value)
    })

    slider_input = document.getElementById('sliderBottom')
    slider_input.addEventListener('input', (event) => {
        updateBottom(event.target.value)
    })

    g_canvas = document.getElementById('canvas')

    // Get the rendering context for WebGL
    gl = getWebGLContext(g_canvas, true)
    if (!gl) {
        console.log('Failed to get the rendering context for WebGL')
        return
    }

    // We will call this at the end of most main functions from now on
    loadOBJFiles()
}

/*
 * Helper function to load OBJ files in sequence
 * For much larger files, you may are welcome to make this more parallel
 * I made everything sequential for this class to make the logic easier to follow
 */
async function loadOBJFiles() {
    // open our OBJ file(s)
    data = await fetch('/resources/teapot.obj').then(response => response.text()).then((x) => x)
    g_teapotMesh = []
    readObjFile(data, g_teapotMesh)

    // Wait to load our models before starting to render
    startRendering()
}

function startRendering() {
    // Initialize GPU's vertex and fragment shaders programs
    if (!initShaders(gl, VSHADER_SOURCE, FSHADER_SOURCE)) {
        console.log('Failed to intialize shaders.')
        return
    }

    // initialize the VBO
    var colors = buildColorAttributes(g_teapotMesh.length / 3)
    var data = g_teapotMesh.concat(colors)
    if (!initVBO(new Float32Array(data))) {
        return
    }

    // Send our vertex data to the GPU
    if (!setupVec3('a_Position', 0, 0)) {
        return
    }
    if (!setupVec3('a_Color', 0, g_teapotMesh.length * FLOAT_SIZE)) {
        return
    }

    // Get references to GLSL uniforms
    g_u_model_ref = gl.getUniformLocation(gl.program, 'u_Model')
    g_u_world_ref = gl.getUniformLocation(gl.program, 'u_World')
    g_u_camera_ref = gl.getUniformLocation(gl.program, 'u_Camera')
    g_u_projection_ref = gl.getUniformLocation(gl.program, 'u_Projection')

    // Setup our model by scaling
    g_modelMatrix = new Matrix4()
    g_modelMatrix = g_modelMatrix.setScale(.01, .01, -.01)

    // Reposition our mesh to fit in the projection
    g_worldMatrix = new Matrix4().translate(0, 0, -2)

    // Initially set our camera to be in a default position
    g_cameraMatrix = new Matrix4()

    // Enable culling and depth
    gl.enable(gl.CULL_FACE)
    gl.enable(gl.DEPTH_TEST)

    // Setup for ticks
    g_lastFrameMS = Date.now()

    // Make sure we have a defined rotation
    updateNear(1)
    updateFar(2)
    updateLeft(-1)
    updateRight(1)
    updateBottom(-1)
    updateTop(1)

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

    // rotate the teapot constantly around the Y axis of the model
    angle = ROTATION_SPEED * deltaTime
    g_modelMatrix.concat(new Matrix4().setRotate(angle, 0, 1, 0))

    draw()

    requestAnimationFrame(tick, g_canvas)
}

// draw to the screen on the next frame
function draw() {
    // Construct our orthographic projection matrix
    var projection_matrix = new Matrix4().setOrtho(g_left, g_right, g_bottom, g_top, g_near, g_far)

    // Update with our global transformation matrices
    gl.uniformMatrix4fv(g_u_model_ref, false, g_modelMatrix.elements)
    gl.uniformMatrix4fv(g_u_world_ref, false, g_worldMatrix.elements)
    gl.uniformMatrix4fv(g_u_camera_ref, false, g_cameraMatrix.elements)
    gl.uniformMatrix4fv(g_u_projection_ref, false, projection_matrix.elements)

    // Clear the canvas with a black background
    gl.clearColor(0.0, 0.0, 0.0, 1.0)
    gl.clear(gl.COLOR_BUFFER_BIT)
    gl.drawArrays(gl.TRIANGLES, 0, g_teapotMesh.length / 3)
}

// Helper to construct colors
// makes every triangle a slightly different shade of blue
function buildColorAttributes(vertex_count) {
    var colors = []
    for (var i = 0; i < vertex_count / 3; i++) {
        // three vertices per triangle
        for (var vert = 0; vert < 3; vert++) {
            var shade = (i * 3) / vertex_count
            colors.push(shade, shade, 1.0)
        }
    }

    return colors
}

function updateNear(amount) {
    label = document.getElementById('near')
    label.textContent = `Near: ${Number(amount).toFixed(2)}`
    g_near = Number(amount)
}

function updateFar(amount) {
    label = document.getElementById('far')
    label.textContent = `Far: ${Number(amount).toFixed(2)}`
    g_far = Number(amount)
}

function updateLeft(amount) {
    label = document.getElementById('left')
    label.textContent = `Left: ${Number(amount).toFixed(2)}`
    g_left = Number(amount)
}

function updateRight(amount) {
    label = document.getElementById('right')
    label.textContent = `Right: ${Number(amount).toFixed(2)}`
    g_right = Number(amount)
}

function updateBottom(amount) {
    label = document.getElementById('bottom')
    label.textContent = `Bottom: ${Number(amount).toFixed(2)}`
    g_bottom = Number(amount)
}

function updateTop(amount) {
    label = document.getElementById('top')
    label.textContent = `Top: ${Number(amount).toFixed(2)}`
    g_top = Number(amount)
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