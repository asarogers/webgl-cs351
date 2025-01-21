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
var g_model_small
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
var g_color_small_index = 0
const defaultColor_small = [0.0, 0.2, 0.5];
const customColor_small = [0.0, 0.5, 0.2];

const defaultColor = [1.0, 0.0, 0.25];
const customColor = [0.6, 0.4, 0.2];


function createColors(primary, second){
    const color1 = [...Array(2).fill(primary), ...second].flat();
    const color2 = [...Array(1).fill(primary), ...second, ...primary].flat();
    const color3 = [...second,...Array(2).fill(primary)].flat();

    return [
        ...[color1],
        ...[color2],
        ...[color3]
    ]
}


const COLORS = createColors(defaultColor, customColor)
const COLORS_small = createColors(defaultColor_small, customColor_small)

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
    0.0, -0.5, 0.0,
]

const VERTICES_small = VERTICES.map(vertex => vertex/2)


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

    // get the u_offset reference from the shader
    g_xOffset_ref = gl.getUniformLocation(gl.program, 'x_Offset')
    g_yOffset_ref = gl.getUniformLocation(gl.program, 'y_Offset')
    g_roation_ref = gl.getUniformLocation(gl.program, 'z_Rotation')
    g_u_model_ref = gl.getUniformLocation(gl.program, 'u_Model')
    g_u_world_ref = gl.getUniformLocation(gl.program, 'u_World')

    // updateOffset_x(0)
    // updateOffset_y(0)
    // updateRotation(0)
    reset();
    initBuffers();
    draw();
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
    setupVec3('a_color', 0, 0);

    // Draw the shape
    gl.drawArrays(gl.TRIANGLES, 0, vertexCount);
}

let vertexBuffer1, vertexBuffer2, colorBuffer1, colorBuffer2;

function initBuffers() {

    vertexBuffer1 = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, vertexBuffer1);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(VERTICES), gl.STATIC_DRAW);

    colorBuffer1 = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, colorBuffer1);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(COLORS[g_color_index]), gl.STATIC_DRAW);

    //  second triangle
    vertexBuffer2 = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, vertexBuffer2);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(VERTICES_small), gl.STATIC_DRAW);

    colorBuffer2 = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, colorBuffer2);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(COLORS_small[g_color_index]), gl.STATIC_DRAW);
}


function draw() {
   gl.clearColor(0.1, 0.1, 0.1, 1.0);
   gl.clear(gl.COLOR_BUFFER_BIT);

   drawShape(g_model, g_world, vertexBuffer1, colorBuffer1, VERTEX_COUNT);
   drawShape(g_model_small, g_world, vertexBuffer2, colorBuffer2, VERTEX_COUNT);

    // Request the next frame
    requestAnimationFrame(draw);
}


// Handler for the nextColor button
// HINT: consider adjusting this function
function nextColor() {
    
    g_color_index = (g_color_index + 1) % COLORS.length;
    // console.log(g_color_index)
    gl.bindBuffer(gl.ARRAY_BUFFER, colorBuffer1);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(COLORS[g_color_index]), gl.STATIC_DRAW);

}

function nextColor_second(){
    g_color_small_index = (g_color_small_index + 1) % COLORS.length;
    // console.log(g_color_index)
    gl.bindBuffer(gl.ARRAY_BUFFER, colorBuffer2);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(COLORS_small[g_color_small_index]), gl.STATIC_DRAW);
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
    // (g_model * ROT45) * g_world
    g_model = mat4multiply(mat4multiply(g_model, ROT45), g_world);

}


function reset() {
    g_model = ID_MATRIX
    g_world = ID_MATRIX
    g_model_small = ID_MATRIX;
}

function translateLeft(){
    g_model = mat4multiply(TRANSLATE_LEFT, g_model)

}

function translateRight(){
    g_model = mat4multiply(TRANSLATE_RIGHT, g_model)

}


function moveUp(){
    g_model = mat4multiply(MOVE_UP, g_model)

}

function moveDown(){
    g_model = mat4multiply(MOVE_DOWN, g_model)

}

function translateLeft_second(){
    g_model_small = mat4multiply(TRANSLATE_LEFT, g_model_small)
}

function translateRight_second(){
    g_model_small = mat4multiply(TRANSLATE_RIGHT, g_model_small)
}



function rotate45_second(){
    g_model_small = mat4multiply(mat4multiply(g_model_small, ROT45), g_world);
}

function moveUp_second(){
    g_model_small = mat4multiply(MOVE_UP, g_model_small)
}

function moveDown_second(){
    g_model_small = mat4multiply(MOVE_DOWN, g_model_small)
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