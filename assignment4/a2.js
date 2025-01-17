// Vertex shader definition
const VSHADER_SOURCE = `
    attribute vec3 a_Position;
    attribute vec3 a_color;
    varying vec3 u_Color;
    uniform float x_Offset;
    uniform float y_Offset;
    uniform mat4 u_Model;
    uniform mat4 u_World;


    void main() {
        vec3 offset = vec3(x_Offset, y_Offset, 0);
        vec3 transformed = (a_Position + offset);
        gl_Position = u_World * u_Model * vec4(transformed, 1.0);
        gl_PointSize = 5.0;
        u_Color = a_color;
    }
`

// Fragment shader definition
const FSHADER_SOURCE = `
    precision mediump float;
    // use the same color everywhere
    // HINT: consider making this non-uniform
    varying vec3 u_Color;
    void main() {
        gl_FragColor = vec4(u_Color, 1.0);
    }
`
// the transformation we'll be updating
var g_model
var g_world

// Global reference to the webGL context, canvas, and VBO
var g_canvas
var gl
var g_xOffset_ref
var g_yOffset_ref
var g_roation_ref
var g_u_model_ref


// Global reference to the uniform pointer to u_Color from our shaders
var g_uColor_ref

// The current color we are using
// You may consider modifying this to instead indicate which vertex is colored
var g_color_index = 0
const defaultColor = [1.0, 0.0, 0.25];
const customColor = [0.6, 0.4, 0.2];

const repeatedColors1 = [...Array(2).fill(defaultColor), ...customColor].flat();
const repeatedColors2 = [...Array(1).fill(defaultColor), ...customColor, ...defaultColor].flat();
// const repeatedColors3 = [(defaultColor), ...customColor, ...Array(1).fill(defaultColor)].flat();
const repeatedColors3 = [...customColor,...Array(2).fill(defaultColor)].flat();

const COLORS = [
    [...repeatedColors1],
    [...repeatedColors2],
    [...repeatedColors3],
]

const ROT45 = [
    Math.cos(Math.PI / 4), Math.sin(Math.PI / 4), 0, 0,
    -Math.sin(Math.PI / 4), Math.cos(Math.PI / 4), 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1
]

// move up .1
const MOVE_UP = [
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, .1, 0, 1
]

// move down .1
const MOVE_DOWN = [
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, -.1, 0, 1
]

const TRANSLATE_LEFT = [
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    -0.1, 0, 0, 1
]

const TRANSLATE_RIGHT = [
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0.1, 0, 0, 1
]

// identity matrix
const ID_MATRIX = [
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1
]

// Setup our vertex (points) for a simple triangle
const VERTICES = [
    -0.5, 0.5, 0.0, //top left
    0.5, 0.5, 0.0, //top right
    0.5, -0.5, 0.0,
]
const VERTEX_COUNT = VERTICES.length / 3

// The size in bytes of a floating point
const FLOAT_SIZE = 4

function main() {
    g_canvas = document.getElementById('canvas')

    // listen for the slider change
    // slider_input_x = document.getElementById('offset_x')
    // slider_input_x.addEventListener('input', (event)=>{
    //     updateOffset_x(event.target.value)
    // })

    // slider_input_y = document.getElementById('offset_y')
    // slider_input_y.addEventListener('input', (event)=>{
    //     updateOffset_y(event.target.value)
    // })

    // slider_rotation = document.getElementById("rotation")
    // slider_rotation.addEventListener("input", (event)=>{
    //     updateRotation(event.target.value)
    // })

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

    // Create our VBO and bind an ARRAY BUFFER to it
    // This only needs to be done once
    var VBOloc = gl.createBuffer()
    if (!VBOloc) {
        console.log('Failed to create the vertex buffer object')
        return false
    }
    gl.bindBuffer(gl.ARRAY_BUFFER, VBOloc)

    // get the u_offset reference from the shader
    g_xOffset_ref = gl.getUniformLocation(gl.program, 'x_Offset')
    g_yOffset_ref = gl.getUniformLocation(gl.program, 'y_Offset')
    g_roation_ref = gl.getUniformLocation(gl.program, 'z_Rotation')
    g_u_model_ref = gl.getUniformLocation(gl.program, 'u_Model')
    g_u_world_ref = gl.getUniformLocation(gl.program, 'u_World')

    // updateOffset_x(0)
    // updateOffset_y(0)
    // updateRotation(0)
    reset()
    // rotate45()


    draw()
}

function draw() {

    gl.uniformMatrix4fv(g_u_model_ref, false, new Float32Array(g_model))
    gl.uniformMatrix4fv(g_u_world_ref, false, new Float32Array(g_world))

    // Create and bind the vertex buffer
    var VBOloc = gl.createBuffer();
    if (!VBOloc) {
        console.log('Failed to create the vertex buffer object');
        return;
    }
    gl.bindBuffer(gl.ARRAY_BUFFER, VBOloc);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(VERTICES), gl.STREAM_DRAW);

    // Use setupVec3 to set up the a_Position attribute
    setupVec3('a_Position', 0, 0);

    // Create and bind the color buffer
    var colorVBO = gl.createBuffer();
    if (!colorVBO) {
        console.log('Failed to create the color buffer object');
        return;
    }

    gl.bindBuffer(gl.ARRAY_BUFFER, colorVBO);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(COLORS[g_color_index]), gl.STREAM_DRAW);

    // Use setupVec3 to set up the a_color attribute
    setupVec3('a_color', 0, 0);

    // Clear the canvas with a dark grey background
    gl.clearColor(0.1, 0.1, 0.1, 1.0);
    gl.clear(gl.COLOR_BUFFER_BIT);
    

    // Draw points and lines
    gl.drawArrays(gl.POINTS, 0, VERTEX_COUNT);
    gl.drawArrays(gl.TRIANGLES, 0, VERTEX_COUNT);

    requestAnimationFrame(draw, g_canvas)
}


// Handler for the nextColor button
// HINT: consider adjusting this function
function nextColor() {
    
    // note our use of === for comparison rather than ==
    g_color_index = (g_color_index + 1) % COLORS.length;
    console.log(g_color_index)

    draw()
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

function rotate45() {
    g_model = mat4multiply(ROT45, g_model)
}

// function updateOffset_x(amount) {
//     label = document.getElementById('sliderLabel_x')
//     label.textContent = `X Offset: ${amount}`
//     gl.uniform1f(g_xOffset_ref, amount)
// }



// function updateOffset_y(amount) {
//     label = document.getElementById('sliderLabel_y')
//     label.textContent = `Y Offset: ${amount}`
//     gl.uniform1f(g_yOffset_ref, amount)
// }

// function updateRotation(amount) {
//     label = document.getElementById('sliderLabelRotation')
//     label.textContent = `Rotation: ${amount}`
//     gl.uniform1f(g_roation_ref, amount)
// }

function reset() {
    g_model = ID_MATRIX
    g_world = ID_MATRIX
}

function translateLeft(){
    g_world = mat4multiply(TRANSLATE_LEFT, g_world)
}

function translateRight(){
    g_world = mat4multiply(TRANSLATE_RIGHT, g_world)
}


function moveUp(){
    g_world = mat4multiply(MOVE_UP, g_world)
}

function moveDown(){
    g_world = mat4multiply(MOVE_DOWN, g_world)
}

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