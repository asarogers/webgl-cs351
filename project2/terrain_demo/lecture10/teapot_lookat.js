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

// GLSL uniform references
var g_u_model_ref
var g_u_world_ref

// usual model matrix
var g_modelMatrix

// lookat information
var g_eye_x
var g_eye_y
var g_eye_z
var g_center_x
var g_center_y
var g_center_z

// Mesh definitions
var g_teapotMesh

// We're using triangles, so our vertices each have 3 elements
const TRIANGLE_SIZE = 3

// The size in bytes of a floating point
const FLOAT_SIZE = 4

function main() {
    // Setup our camera movement sliders
    slider_input = document.getElementById('sliderEyeX')
    slider_input.addEventListener('input', (event) => {
        updateEyeX(event.target.value)
    })
    slider_input = document.getElementById('sliderEyeY')
    slider_input.addEventListener('input', (event) => {
        updateEyeY(event.target.value)
    })
    slider_input = document.getElementById('sliderEyeZ')
    slider_input.addEventListener('input', (event) => {
        updateEyeZ(event.target.value)
    })
    slider_input = document.getElementById('sliderCenterX')
    slider_input.addEventListener('input', (event) => {
        updateCenterX(event.target.value)
    })
    slider_input = document.getElementById('sliderCenterY')
    slider_input.addEventListener('input', (event) => {
        updateCenterY(event.target.value)
    })
    slider_input = document.getElementById('sliderCenterZ')
    slider_input.addEventListener('input', (event) => {
        updateCenterZ(event.target.value)
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
    data = await fetch('./resources/teapot.obj').then(response => response.text()).then((x) => x)
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
    g_modelMatrix = g_modelMatrix.setScale(.005, .005, .005)

    // rotate the teapot so the spout is facing "forward"
    g_modelMatrix = g_modelMatrix.rotate(90, 0, 1, 0)

    // Enable culling and depth
    gl.enable(gl.CULL_FACE)
    gl.enable(gl.DEPTH_TEST)

    // Setup for ticks
    g_lastFrameMS = Date.now()

    // Initially set our camera to be at the origin, facing right
    updateEyeX(0)
    updateEyeY(0)
    updateEyeZ(0)
    updateCenterX(0)
    updateCenterY(0)
    updateCenterZ(1)

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

    draw()

    requestAnimationFrame(tick, g_canvas)
}

// draw to the screen on the next frame
function draw() {
    // directly set our world matrix to be the lookat matrix we want
    var lookatMatrix = new Matrix4()
    .setLookAt(
        g_eye_x, g_eye_y, g_eye_z,
        // negate Z for right-handed coordinates here
        g_center_x, g_center_y, -g_center_z,
        0, 1, 0
    )

    // Clear the canvas with a black background
    gl.clearColor(0.0, 0.0, 0.0, 1.0)
    gl.clear(gl.COLOR_BUFFER_BIT)

    // Update with our global transformation matrices
    gl.uniformMatrix4fv(g_u_model_ref, false, g_modelMatrix.elements)
    // confusingly, we need to invert the (usual) lookat matrix
    // this is because we want to apply the lookat transformation to the teapot rather than putting the teapot in the lookat space
    gl.uniformMatrix4fv(g_u_world_ref, false, lookatMatrix.invert().elements)

    // Draw our teapot
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

function updateEyeX(amount) {
    label = document.getElementById('eyeX')
    label.textContent = `Eye X: ${Number(amount).toFixed(2)}`
    g_eye_x = Number(amount)
}
function updateEyeY(amount) {
    label = document.getElementById('eyeY')
    label.textContent = `Eye Y: ${Number(amount).toFixed(2)}`
    g_eye_y = Number(amount)
}
function updateEyeZ(amount) {
    label = document.getElementById('eyeZ')
    label.textContent = `Eye Z: ${Number(amount).toFixed(2)}`
    g_eye_z = Number(amount)
}
function updateCenterX(amount) {
    label = document.getElementById('centerX')
    label.textContent = `Center X: ${Number(amount).toFixed(2)}`
    g_center_x = Number(amount)
}
function updateCenterY(amount) {
    label = document.getElementById('centerY')
    label.textContent = `Center Y: ${Number(amount).toFixed(2)}`
    g_center_y = Number(amount)
}
function updateCenterZ(amount) {
    label = document.getElementById('centerZ')
    label.textContent = `Center Z: ${Number(amount).toFixed(2)}`
    g_center_z = Number(amount)
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