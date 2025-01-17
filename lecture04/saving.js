// Last edited by Dietrich Geisler 2024

const VSHADER_SOURCE = `
    attribute vec3 a_Position;
    uniform mat3 u_Transform;
    void main() {
        // We just need one matrix multiply here!
        vec3 pos = u_Transform * a_Position;
        gl_Position = vec4(pos, 1.);
    }
`

const FSHADER_SOURCE = `
    precision mediump float;
    void main() {
        // Every fragment has the same color
        gl_FragColor = vec4(0.5, 0.0, 1.0, 1.0);
    }
`

// Global reference to the webGL context and the canvas
var g_canvas
var gl

// Global reference to our uniform location
var g_u_transform

// Global transform matrix we will continuously update
var g_transform_matrix

// Constant 3x3 Matrices
// id matrix
var ID_MATRIX = [
    1, 0, 0,
    0, 1, 0,
    0, 0, 1,
]
// scale X by 2
var SCALE_DOUBLE = [
    2, 0, 0,
    0, 1, 0,
    0, 0, 1
]
var SCALE_HALF = [
    0.5, 0, 0,
    0, 1, 0,
    0, 0, 1
]
// rotate counterclockwise 90 degrees
// note that we transpose this matrix to satisfy webGL
var ROT90_COUNTER = [
    0, 1, 0,
   -1, 0, 0,
    0, 0, 1
]

// Setup our vertices for a simple triangle
const VERTICES = [
    0.0, 0.5, 0.0,
    0.5, -0.5, 0.0,
    -0.5, -0.5, 0.0
]
const VERTEX_COUNT = VERTICES.length / 3

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
    if (!initVBO(new Float32Array(VERTICES))) {
        return
    }

    // Send our vertex data to the GPU
    if (!setupVec3('a_Position', 0, 0)) {
        return
    }

    // Get the u_Rotation reference from the shader
    g_u_transform = gl.getUniformLocation(gl.program, 'u_Transform')

    // Start with no transformation
    g_transform_matrix = ID_MATRIX

    // Start drawing
    draw()
}

function draw() {
    // use our current transformation matrix every frame
    gl.uniformMatrix3fv(g_u_transform, false, g_transform_matrix)

    gl.clearColor(0.2, 0.2, 0.2, 1.0)
    gl.clear(gl.COLOR_BUFFER_BIT)
    gl.drawArrays(gl.TRIANGLES, 0, VERTEX_COUNT)

    requestAnimationFrame(draw, g_canvas)
}

// add a new 2x width scaling
function scaleXDouble() {
    g_transform_matrix = mat3multiply(SCALE_DOUBLE, g_transform_matrix)
}

// add a new 0.5x width scaling
function scaleXHalf() {
    g_transform_matrix = mat3multiply(SCALE_HALF, g_transform_matrix)
}

// add a new 90 degree counterclockwise rotation
function rotate90Counter() {
    g_transform_matrix = mat3multiply(ROT90_COUNTER, g_transform_matrix)
}

// Helper function for mat3-multiplication
// Does no potential error checking
function mat3multiply(m1, m2) {
    // column-first 3D matrix multiply
    return [
        m1[0] * m2[0] + m1[3] * m2[1] + m1[6] * m2[2],
        m1[1] * m2[0] + m1[4] * m2[1] + m1[7] * m2[2],
        m1[2] * m2[0] + m1[5] * m2[1] + m1[8] * m2[2],
        m1[0] * m2[3] + m1[3] * m2[4] + m1[6] * m2[5],
        m1[1] * m2[3] + m1[4] * m2[4] + m1[7] * m2[5],
        m1[2] * m2[3] + m1[5] * m2[4] + m1[8] * m2[5],
        m1[0] * m2[6] + m1[3] * m2[7] + m1[6] * m2[8],
        m1[1] * m2[6] + m1[4] * m2[7] + m1[7] * m2[8],
        m1[2] * m2[6] + m1[5] * m2[7] + m1[8] * m2[8],
    ]
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