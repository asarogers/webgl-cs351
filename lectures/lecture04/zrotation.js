// Last edited by Dietrich Geisler 2024

const VSHADER_SOURCE = `
    uniform float u_Rotation;
    attribute vec3 a_Position;
    void main() {
        // Recall that openGL matrices are transposed from "regular" matrices
        // So the negative is "across the diagonal" from our regular spot
        mat3 zRotation = mat3(
            cos(u_Rotation),  sin(u_Rotation), 0,
            -sin(u_Rotation), cos(u_Rotation), 0,
            0               , 0              , 1
        );
        // first calculate our transformation
        vec3 transformed = zRotation * a_Position;
        gl_Position = vec4(transformed, 1.);
    }
`

const FSHADER_SOURCE = `
    void main() {
        // Every pixel gets the same color
        gl_FragColor = vec4(1.0, 0.0, 0.5, 1.0);
    }
`

// global hooks for updating data
var g_canvas
var gl
var g_vertexCount
var g_uOffset_ref

// Setup our vertices for a simple triangle of lines
const VERTICES = [
    0.0, 0.4, 0.0,
    0.3, -0.4, 0.0,
    -0.3, -0.4, 0.0,
]
const VERTEX_COUNT = VERTICES.length / 3

// The size in bytes of a floating point
const FLOAT_SIZE = 4

function main() {
    // Listen for slider change
    slider_input = document.getElementById('rotation')
    slider_input.addEventListener('input', (event) => {
        updateRotation(event.target.value)
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
    g_uRotation_ref = gl.getUniformLocation(gl.program, 'u_Rotation')

    // Start with a rotation of 0
    updateRotation(0)

    // Start drawing
    draw()
}

function draw() {
    gl.clearColor(0.1, 0.1, 0.1, 1.0)
    gl.clear(gl.COLOR_BUFFER_BIT)
    gl.drawArrays(gl.TRIANGLES, 0, VERTEX_COUNT)

    requestAnimationFrame(draw, g_canvas)
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
 * Helper function to load the given vec3 onto the VBO
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

// called when we update the rotation slider
function updateRotation(amount) {
    label = document.getElementById('sliderLabel')
    var rotationDegrees = Number(amount * 180 / Math.PI)
    label.textContent = `ZRotation: ${rotationDegrees.toFixed(0)} degrees`
    gl.uniform1f(g_uRotation_ref, amount)
}