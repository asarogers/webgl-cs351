// Last edited by Dietrich Geisler 2025

const VSHADER_SOURCE = `
    attribute vec3 a_Position;
    uniform mat4 u_Model;
    uniform mat4 u_World;
    uniform mat4 u_Camera;
    attribute vec3 a_Color;
    varying vec3 v_Color;
    uniform mat4 u_Projection;
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
var g_lastFrameMS
var gl
var bufferInfo

// GLSL uniform references
var g_u_model_ref
var g_u_world_ref
var g_u_camera_ref
var g_gridMesh

// usual model/world matrices
var g_modelMatrix
var g_second_modelMatrix
var g_third_modelMatrix
var g_worldMatrix

// camera projection values
var g_camera_x
var g_camera_y
var g_camera_z

// camera projection values
var g_near_orth
var g_far_orth
var g_near
var g_far
var g_left
var g_right
var g_top
var g_bottom
var first_square_scale = 0.15
var other_box_scale = 0.25
var isOrth = true


// Grid constants
const GRID_X_RANGE = 10  // Reduced from 1000 for better performance
const GRID_Z_RANGE = 10
const GRID_Y_OFFSET = -0.5

class Square {
    constructor(VERTICES) {
        this.VERTICES = VERTICES;
        this.VERTEX_COUNT = VERTICES.length / 3;
        this.TRIANGLE_SIZE = 3;
        this.SPEED = 0;
        this.COLORS = null;
    }
}

const TRIANGLE_SIZE = 3;
const FLOAT_SIZE = 4;

const movement = Object.freeze({
    HORIZONTAL: "horizontal",
    VERTICAL: "vertical",
    TEST: "test",
    ROTATE: "rotate",
    RESET: "reset"
});

let firstSquare = new Square(generateSpaceship(0.5));
let secondSquare = new Square(generateSpaceship(0.5));
let thirdSquare = new Square(generateSpaceship(0.5));

var translate_time;

var data = {}
var translate_time;
var g_rotationAxis;

function main() {
    g_canvas = document.getElementById('canvas');
    gl = getWebGLContext(g_canvas, true);

    if (!gl) {
        console.log('Failed to get the rendering context for WebGL');
        return;
    }

    if (!initShaders(gl, VSHADER_SOURCE, FSHADER_SOURCE)) {
        console.log('Failed to initialize shaders.');
        return;
    }

    g_u_model_ref = gl.getUniformLocation(gl.program, 'u_Model');
    g_u_world_ref = gl.getUniformLocation(gl.program, 'u_World');
    g_u_camera_ref = gl.getUniformLocation(gl.program, 'u_Camera');
    g_u_projection_ref = gl.getUniformLocation(gl.program, 'u_Projection');

    bufferInfo = initBuffers();
    if (!bufferInfo) {
        console.log('Failed to initialize buffers');
        return;
    }




    reset();
    gl.enable(gl.CULL_FACE);
    
    g_lastFrameMS = Date.now();
    g_rotationAxis = [0, 0, 0];
    setupCamera();
    updateRotation();
    tick();
}

function initBuffers() {
    // Create combined data arrays
    let vertices = [
        ...firstSquare.VERTICES,
        ...secondSquare.VERTICES,
        ...thirdSquare.VERTICES
    ];
    
    let gridInfo = buildGridAttributes(1, 1, [0.0, 1.0, 0.0]);
    g_gridMesh = gridInfo[0];
    vertices = [...vertices, ...g_gridMesh];

    // Create colors for all shapes
    let colors = [
        ...buildColorAttributes(firstSquare.VERTEX_COUNT),
        ...buildColorAttributes(secondSquare.VERTEX_COUNT),
        ...buildColorAttributes(thirdSquare.VERTEX_COUNT),
        ...gridInfo[1]
    ];

    // Create and bind single VBO
    let VBO = gl.createBuffer();
    if (!VBO) {
        console.log('Failed to create buffer');
        return null;
    }

    // Combine vertices and colors
    let combinedData = new Float32Array([...vertices, ...colors]);
    
    gl.bindBuffer(gl.ARRAY_BUFFER, VBO);
    gl.bufferData(gl.ARRAY_BUFFER, combinedData, gl.STATIC_DRAW);

    return {
        buffer: VBO,
        vertexOffset: 0,
        colorOffset: vertices.length * FLOAT_SIZE,
        firstSquareCount: firstSquare.VERTEX_COUNT,
        secondSquareCount: secondSquare.VERTEX_COUNT,
        thirdSquareCount: thirdSquare.VERTEX_COUNT,
        gridCount: g_gridMesh.length / 3
    };
}


// extra constants for cleanliness
var ROTATION_SPEED = .05



function tick() {
    var deltaTime = Date.now() - g_lastFrameMS;
    g_lastFrameMS = Date.now();

    let angle = ROTATION_SPEED * deltaTime;
    g_modelMatrix.concat(new Matrix4().setRotate(angle, ...g_rotationAxis));
    g_second_modelMatrix.concat(new Matrix4().setRotate(secondSquare.SPEED * 100, ...g_rotationAxis));
    g_third_modelMatrix.concat(new Matrix4().setRotate(thirdSquare.SPEED * 100, ...g_rotationAxis));

    translate_time += deltaTime;
    if (translate_time >= 3000) {
        translate_time = 0;
        reset_moving_shapes();
        setSpeed();
        updateColors();
    }

    g_second_modelMatrix = move3DShape(g_second_modelMatrix, movement.HORIZONTAL, -secondSquare.SPEED);
    g_third_modelMatrix = move3DShape(g_third_modelMatrix, movement.HORIZONTAL, -thirdSquare.SPEED);

    draw();
    requestAnimationFrame(tick);
}



function buildGridAttributes(grid_row_spacing, grid_column_spacing, grid_color) {
    var mesh = []
    var colors = []

    // Construct the rows
    for (var x = -GRID_X_RANGE; x < GRID_X_RANGE; x += grid_row_spacing) {
        // two vertices for each line
        // one at -Z and one at +Z
        mesh.push(x, 0, -GRID_Z_RANGE)
        mesh.push(x, 0, GRID_Z_RANGE)
    }

    // Construct the columns extending "outward" from the camera
    for (var z = -GRID_Z_RANGE; z < GRID_Z_RANGE; z += grid_column_spacing) {
        // two vertices for each line
        // one at -Z and one at +Z
        mesh.push(-GRID_X_RANGE, 0, z)
        mesh.push(GRID_X_RANGE, 0, z)
    }

    // We need one color per vertex
    // since we have 3 components for each vertex, this is length/3
    for (var i = 0; i < mesh.length / 3; i++) {
        colors.push(grid_color[0], grid_color[1], grid_color[2])
    }

    return [mesh, colors]
}


function changeColor(){
    var colors = buildColorAttributes(firstSquare.VERTEX_COUNT);
    firstSquare.changeColor(colors)

    var colors = buildColorAttributes(secondSquare.VERTEX_COUNT);
    secondSquare.changeColor(colors)

    var colors = buildColorAttributes(thirdSquare.VERTEX_COUNT);
    thirdSquare.changeColor(colors)
}
function setSpeed(){
    let lower = 0.005
    let upper = 0.01
    speed1 = lower + Math.random() * (upper - lower)
    speed2 = lower + Math.random() * (upper - lower)

    
    secondSquare.SPEED = speed2
    thirdSquare.SPEED = speed1
}

// draw to the screen on the next frame
function draw() {
    const cameraMatrix = new Matrix4().translate(-g_camera_x, -g_camera_y, -g_camera_z);
    var projection_matrix
    if (isOrth){
        projection_matrix = new Matrix4().setOrtho(g_left, g_right, g_bottom, g_top, g_near_orth, g_far_orth)
    }else{
        projection_matrix = new Matrix4().setPerspective(g_fovy, g_aspect, g_near, g_far)
    }

    gl.clearColor(0.0, 0.0, 0.0, 1.0);
    gl.clear(gl.COLOR_BUFFER_BIT);

    gl.bindBuffer(gl.ARRAY_BUFFER, bufferInfo.buffer);

    // Set up position attribute
    setupVec3('a_Position', 0, bufferInfo.vertexOffset);
    
    // Set up color attribute
    setupVec3('a_Color', 0, bufferInfo.colorOffset);

    // Draw first square
    gl.uniformMatrix4fv(g_u_model_ref, false, g_modelMatrix.elements);
    gl.uniformMatrix4fv(g_u_world_ref, false, g_worldMatrix.elements);
    gl.drawArrays(gl.TRIANGLES, 0, bufferInfo.firstSquareCount);

    // Draw second square
    gl.uniformMatrix4fv(g_u_model_ref, false, g_second_modelMatrix.elements);
    gl.drawArrays(gl.TRIANGLES, bufferInfo.firstSquareCount, 
                 bufferInfo.secondSquareCount);

    // Draw third square
    gl.uniformMatrix4fv(g_u_model_ref, false, g_third_modelMatrix.elements);
    gl.drawArrays(gl.TRIANGLES, 
                 bufferInfo.firstSquareCount + bufferInfo.secondSquareCount,
                 bufferInfo.thirdSquareCount);

    // Draw grid
    gl.uniformMatrix4fv(g_u_model_ref, false, new Matrix4().elements);
    gl.uniformMatrix4fv(g_u_world_ref, false, 
                       new Matrix4().translate(0, GRID_Y_OFFSET, 0).elements);
    gl.uniformMatrix4fv(g_u_camera_ref, false, cameraMatrix.elements);
    gl.uniformMatrix4fv(g_u_projection_ref, false, projection_matrix.elements);
    
    gl.drawArrays(gl.LINES, 
                 bufferInfo.firstSquareCount + bufferInfo.secondSquareCount + 
                 bufferInfo.thirdSquareCount,
                 bufferInfo.gridCount);
}

function updateColors() {
    const vertices = [
        ...firstSquare.VERTICES,
        ...secondSquare.VERTICES,
        ...thirdSquare.VERTICES,
        ...g_gridMesh
    ];

    const colors = [
        ...buildColorAttributes(firstSquare.VERTEX_COUNT),
        ...buildColorAttributes(secondSquare.VERTEX_COUNT),
        ...buildColorAttributes(thirdSquare.VERTEX_COUNT),
        ...buildGridAttributes(1, 1, [0.0, 1.0, 0.0])[1]
    ];

    gl.bindBuffer(gl.ARRAY_BUFFER, bufferInfo.buffer);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([...vertices, ...colors]), gl.STATIC_DRAW);
}

function setupCamera(){
    slider_input = document.getElementById('sliderX')
    slider_input.addEventListener('input', (event) => {
        updateCameraX(event.target.value)
    })
    slider_input = document.getElementById('sliderY')
    slider_input.addEventListener('input', (event) => {
        updateCameraY(event.target.value)
    })
    slider_input = document.getElementById('sliderZ')
    slider_input.addEventListener('input', (event) => {
        updateCameraZ(event.target.value)
    })

    slider_input = document.getElementById('orth_sliderNear')
    slider_input.addEventListener('input', (event) => {
        updateNear_orth(event.target.value)
    })

    slider_input = document.getElementById('orth_sliderFar')
    slider_input.addEventListener('input', (event) => {
        updateFar_orth(event.target.value)
    })

    slider_input = document.getElementById('orth_sliderLeft')
    slider_input.addEventListener('input', (event) => {
        updateLeft_orth(event.target.value)
    })

    slider_input = document.getElementById('orth_sliderRight')
    slider_input.addEventListener('input', (event) => {
        updateRight_orth(event.target.value)
    })

    slider_input = document.getElementById('orth_sliderTop')
    slider_input.addEventListener('input', (event) => {
        updateTop_orth(event.target.value)
    })

    slider_input = document.getElementById('orth_sliderBottom')
    slider_input.addEventListener('input', (event) => {
        updateBottom_orth(event.target.value)
    })


    slider_input = document.getElementById('sliderFOVY')
    slider_input.addEventListener('input', (event) => {
        updateFOVY(event.target.value)
    })

    slider_input = document.getElementById('sliderAspect')
    slider_input.addEventListener('input', (event) => {
        updateAspect(event.target.value)
    })

    slider_input = document.getElementById('sliderNear')
    slider_input.addEventListener('input', (event) => {
        updateNear(event.target.value)
    })

    slider_input = document.getElementById('sliderFar')
    slider_input.addEventListener('input', (event) => {
        updateFar(event.target.value)
    })

    updateCameraX(0)
    updateCameraY(0)
    updateCameraZ(0.25)


}

function drawShape(model, world, shape) {
    // Set transformation matrices
    gl.uniformMatrix4fv(g_u_model_ref, false, new Float32Array(model));
    gl.uniformMatrix4fv(g_u_world_ref, false, new Float32Array(world));

    // Bind the vertex buffer and set up the a_Position attribute
    gl.bindBuffer(gl.ARRAY_BUFFER, shape.BUFFER);
    setupVec3('a_Position', 0, 0);

    // Bind the color buffer and set up the a_color attribute
    gl.bindBuffer(gl.ARRAY_BUFFER, shape.COLOR_BUFFER);
    setupVec3('a_Color', 0, 0);

    // Draw the shape
    gl.drawArrays(gl.TRIANGLES, 0, shape.VERTEX_COUNT);
}

// Helper to construct colors
// makes every triangle a slightly different shade of blue
function buildColorAttributes(vertex_count) {

    const color1 = [Math.random(), Math.random(), Math.random()];
    const color2 = [Math.random(), Math.random(), Math.random()];
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

function updateNear_orth(amount) {
    label = document.getElementById('orth_near')
    label.textContent = `orth_Near: ${Number(amount).toFixed(2)}`
    g_near_orth = Number(amount)
}

function updateFar_orth(amount) {
    label = document.getElementById('orth_far')
    label.textContent = `orth_Far: ${Number(amount).toFixed(2)}`
    g_far_orth = Number(amount)
}

function updateLeft_orth(amount) {
    label = document.getElementById('orth_left')
    label.textContent = `orth_Left: ${Number(amount).toFixed(2)}`
    g_left = Number(amount)
}

function updateRight_orth(amount) {
    label = document.getElementById('orth_right')
    label.textContent = `orth_Right: ${Number(amount).toFixed(2)}`
    g_right = Number(amount)
}

function updateBottom_orth(amount) {
    label = document.getElementById('orth_bottom')
    label.textContent = `orth_Bottom: ${Number(amount).toFixed(2)}`
    g_bottom = Number(amount)
}

function updateTop_orth(amount) {
    label = document.getElementById('orth_top')
    label.textContent = `orth_Top: ${Number(amount).toFixed(2)}`
    g_top = Number(amount)
}


function updateCameraX(amount) {
    label = document.getElementById('cameraX')
    label.textContent = `Camera X: ${Number(amount).toFixed(2)}`
    g_camera_x = Number(amount)
}
function updateCameraY(amount) {
    label = document.getElementById('cameraY')
    label.textContent = `Camera Y: ${Number(amount).toFixed(2)}`
    g_camera_y = Number(amount)
}
function updateCameraZ(amount) {
    label = document.getElementById('cameraZ')
    label.textContent = `Camera Z: ${Number(amount).toFixed(2)}`
    g_camera_z = Number(amount)
}

function updateFOVY(amount) {
    label = document.getElementById('fovy')
    label.textContent = `FOVY: ${Number(amount).toFixed(2)}`
    g_fovy = Number(amount)
}

function updateAspect(amount) {
    label = document.getElementById('aspect')
    label.textContent = `Aspect: ${Number(amount).toFixed(2)}`
    g_aspect = Number(amount)
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
    translate_time = 0
    g_modelMatrix = new Matrix4()
    g_worldMatrix = new Matrix4()
    g_modelMatrix = move3DShape(g_modelMatrix, movement.RESET, ID_MATRIX)
    g_modelMatrix = g_modelMatrix.setScale(first_square_scale, first_square_scale, first_square_scale)

    setOrth()
    setPerspective()

    setSpeed()


    reset_moving_shapes()

}

function switchPerspective(){
    isOrth = !isOrth
    other_box_scale = 0.15
    first_square_scale = 0.25
}

function setPerspective(){
    updateFOVY(168)
    updateAspect(1)
    updateNear(1)
    updateFar(.11)
}

function setOrth(){
    updateNear_orth(1)
    updateFar_orth(-1)
    updateLeft_orth(-1)
    updateRight_orth(1)
    updateBottom_orth(-1)
    updateTop_orth(1)
}

function reset_moving_shapes(){
    g_second_modelMatrix = new Matrix4()
    g_second_modelMatrix = g_second_modelMatrix.setScale(other_box_scale, other_box_scale, other_box_scale)
    g_second_modelMatrix = move3DShape(g_second_modelMatrix, movement.HORIZONTAL, 1)
    g_second_modelMatrix = move3DShape(g_second_modelMatrix, movement.VERTICAL, 0.5)
    g_second_modelMatrix.rotate(90, 0, 0,1 )

    g_third_modelMatrix = new Matrix4()
    g_third_modelMatrix = g_third_modelMatrix.setScale(other_box_scale, other_box_scale, other_box_scale)
    g_third_modelMatrix = move3DShape(g_third_modelMatrix, movement.HORIZONTAL, 1)
    g_third_modelMatrix = move3DShape(g_third_modelMatrix, movement.VERTICAL, -0.5)
    g_third_modelMatrix.rotate(90, 0, 0,1 )
}