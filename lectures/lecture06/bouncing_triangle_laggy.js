// Last edited by Dietrich Geisler 2024

const VSHADER_SOURCE = `
    attribute vec3 a_Position;
    uniform mat4 u_Model;
    uniform mat4 u_World;
    void main() {
        gl_Position = u_World * u_Model * vec4(a_Position, 1.);
    }
`

const FSHADER_SOURCE = `
    void main() {
        // Every pixel gets the same color
        gl_FragColor = vec4(0.2, 1.0, 0.5, 1.0);
    }
`

// the transformation we'll be updating
var g_model
var g_world

// references to general information
var g_canvas
var gl
var g_vertex_count
var g_u_model_ref
var g_u_world_ref

// global to keep track of the time of the _previous_ frame
var g_last_frame_ms

// the amount of time we've been moving one direction
var g_flip_time

// whether we are moving up or down
var g_moving_up

// identity matrix
const ID_MATRIX = [
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1
]

// Setup our vertices for a simple triangle
const VERTICES = [
    0.4, 0.2, 0.0,
    0.4, -0.5, 0.0,
    -0.4, -0.5, 0.0
]
const VERTEX_COUNT = VERTICES.length / 3

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
    if (!initVBO(new Float32Array(VERTICES))) {
        return
    }

    // Send our vertex data to the GPU
    if (!setupVec3('a_Position', 0, 0)) {
        return
    }

    // reference to the world and model matrices
    g_u_model_ref = gl.getUniformLocation(gl.program, 'u_Model')
    g_u_world_ref = gl.getUniformLocation(gl.program, 'u_World')
    
    // Keep track of time each frame by starting with our current time
    g_last_frame_ms = Date.now()

    // Keep track of our current Y component (initially 0)
    g_up_time = 0
    g_moving_up = true

    // start with identity matrices
    g_model = ID_MATRIX
    g_world = ID_MATRIX

    tick()
}

const MOVE_SPEED = .05

async function tick() {
    // calculate time since the last frame
    var current_time = Date.now()
    var delta_time = current_time - g_last_frame_ms
    g_last_frame_ms = current_time

    var frame_movement = MOVE_SPEED
    if (!g_moving_up) {
        frame_movement = -MOVE_SPEED
    }

    var translation_matrix = move_up(frame_movement)
    g_world = mat4multiply(translation_matrix, g_world)

    g_up_time += delta_time
    if (g_up_time > 1000) {
        g_moving_up = !g_moving_up
        g_up_time = 0
    }

    // Hacky trick to force a random delay each frame
    // https://stackoverflow.com/questions/951021/what-is-the-javascript-version-of-sleep
    await new Promise(resolve => setTimeout(resolve, Math.random() * 100))

    draw()

    requestAnimationFrame(tick, g_canvas)
}

// draw to the screen on the next frame
function draw() {
    
    // Update with our global transformation matrix
    gl.uniformMatrix4fv(g_u_model_ref, false, new Float32Array(g_model))
    gl.uniformMatrix4fv(g_u_world_ref, false, new Float32Array(g_world))

    // Clear the canvas with a gray background
    gl.clearColor(0.1, 0.1, 0.1, 1.0)
    gl.clear(gl.COLOR_BUFFER_BIT)
    gl.drawArrays(gl.TRIANGLES, 0, VERTEX_COUNT)
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

/*
 * rotation matrix for moving up X units
 */ 
function move_up(amount) {
    return [
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, amount, 0, 1
    ]
}

/*
 * Helper function for mat4-multiplication
 * Does no potential error checking
 */
function mat4multiply(m1, m2) {
    // column-first 4D matrix multiply
    return [
        m1[0] * m2[0] + m1[4] * m2[1] + m1[8] * m2[2] + m1[12] * m2[3],
        m1[1] * m2[0] + m1[5] * m2[1] + m1[9] * m2[2] + m1[13] * m2[3],
        m1[2] * m2[0] + m1[6] * m2[1] + m1[10] * m2[2] + m1[14] * m2[3],
        m1[3] * m2[0] + m1[7] * m2[1] + m1[11] * m2[2] + m1[15] * m2[3],
        m1[0] * m2[4] + m1[4] * m2[5] + m1[8] * m2[6] + m1[12] * m2[7],
        m1[1] * m2[4] + m1[5] * m2[5] + m1[9] * m2[6] + m1[13] * m2[7],
        m1[2] * m2[4] + m1[6] * m2[5] + m1[10] * m2[6] + m1[14] * m2[7],
        m1[3] * m2[4] + m1[7] * m2[5] + m1[11] * m2[6] + m1[15] * m2[7],
        m1[0] * m2[8] + m1[4] * m2[9] + m1[8] * m2[10] + m1[12] * m2[11],
        m1[1] * m2[8] + m1[5] * m2[9] + m1[9] * m2[10] + m1[13] * m2[11],
        m1[2] * m2[8] + m1[6] * m2[9] + m1[10] * m2[10] + m1[14] * m2[11],
        m1[3] * m2[8] + m1[7] * m2[9] + m1[11] * m2[10] + m1[15] * m2[11],
        m1[0] * m2[12] + m1[4] * m2[13] + m1[8] * m2[14] + m1[12] * m2[15],
        m1[1] * m2[12] + m1[5] * m2[13] + m1[9] * m2[14] + m1[13] * m2[15],
        m1[2] * m2[12] + m1[6] * m2[13] + m1[10] * m2[14] + m1[14] * m2[15],
        m1[3] * m2[12] + m1[7] * m2[13] + m1[11] * m2[14] + m1[15] * m2[15],
    ]
}