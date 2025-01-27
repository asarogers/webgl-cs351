// Last edited by Dietrich Geisler 2024

const VSHADER_SOURCE = `
    attribute vec3 a_Position;
    uniform float u_XShift;
    uniform float u_YShift;
    uniform float u_ZShift;
    void main() {
        // update our x, y, z with our uniform
        vec3 pos = vec3(a_Position.x + u_XShift,
                        a_Position.y + u_YShift,
                        a_Position.z + u_ZShift);
        gl_Position = vec4(pos, 1.0);	
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
var g_uRedShift_ref

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
    // Listen for slider change
    slider_input = document.getElementById('xshift')
    slider_input.addEventListener('input', (event) => {
        updateXShift(event.target.value)
    })
    slider_input = document.getElementById('yshift')
    slider_input.addEventListener('input', (event) => {
        updateYShift(event.target.value)
    })
    slider_input = document.getElementById('zshift')
    slider_input.addEventListener('input', (event) => {
        updateZShift(event.target.value)
    })

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
    g_uXShift_ref = gl.getUniformLocation(gl.program, 'u_XShift')
    g_uYShift_ref = gl.getUniformLocation(gl.program, 'u_YShift')
    g_uZShift_ref = gl.getUniformLocation(gl.program, 'u_ZShift')

    // Start with no color shift
    updateXShift(0)
    updateYShift(0)
    updateZShift(0)

    // Start drawing
    draw()
}

function draw() {
    gl.clearColor(0.2, 0.2, 0.2, 1.0)
    gl.clear(gl.COLOR_BUFFER_BIT)
    gl.drawArrays(gl.TRIANGLES, 0, VERTEX_COUNT)

    requestAnimationFrame(draw, g_canvas)
}

// calls for when we update the associated slider
function updateXShift(amount) {
    label = document.getElementById('xLabel')
    label.textContent = `XShift: ${amount}`
    gl.uniform1f(g_uXShift_ref, amount)
}
function updateYShift(amount) {
    label = document.getElementById('yLabel')
    label.textContent = `YShift: ${amount}`
    gl.uniform1f(g_uYShift_ref, amount)
}
function updateZShift(amount) {
    label = document.getElementById('zLabel')
    label.textContent = `ZShift: ${amount}`
    gl.uniform1f(g_uZShift_ref, amount)
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