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
var bf_info

// GLSL uniform references
var g_u_model_ref
var g_u_world_ref
var g_u_camera_ref
var g_gridMesh

// usual model/world matrices
var g_modelMatrix
var g_second_modelMatrix
var g_third_modelMatrix
var g_ship_modelMatrix
var g_flame_modelMatrix1, g_flame_modelMatrix2
var g_worldMatrix
var g_shipMesh
var g_flameMesh1, g_flameMesh2

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
var isOrth = false




class Flame {
    constructor(xOffset = 0, yOffset = 0) {
        this.VERTICES = [];
        this.baseVertices = []; // Store the original vertices
        this.xOffset = xOffset;
        this.yOffset = yOffset;
        this.time = 0;
        this.flickerIntensity = 0.02;
        this.dissolveMap = new Map();
        this.dissolveRate = 0.05;
        this.regenerateRate = 0.025;
    }

    initializeVertices(objData) {
        // Make a deep copy of the vertices
        //shrink the shape
        this.baseVertices = [...objData];
        this.VERTICES = this.baseVertices.map((value, index) => {
            if (index % 3 === 0) { // X coordinate
                return value + this.xOffset;
            } else if (index % 3 === 1) { // Y coordinate
                return value + this.yOffset;
            }
            return value;
        });
        this.setLength();
    }

    setLength() {
        this.flameSize = this.VERTICES.length;
        for (let i = 0; i < this.flameSize; i += 3) {
            this.dissolveMap.set(i, 1.0);
        }
    }

    updateColors() {
        this.COLORS = [];
        const triangleCount = this.VERTICES.length / 9; 

        for (let i = 0; i < triangleCount; i++) {
            const vertexIndex = i * 9;
            const y = this.VERTICES[vertexIndex + 1]; 
            const maxY = Math.max(...this.VERTICES.filter((_, idx) => idx % 3 === 1));
            const heightFactor = y / maxY;
            

            const dissolveFactor = this.dissolveMap.get(vertexIndex) || 
                       this.dissolveMap.get(vertexIndex + 3) || 
                       this.dissolveMap.get(vertexIndex + 6) || 1.0;


            // Base flame colors
            const red = 1.0;
            const green = Math.max(0, 0.4 * (1 - heightFactor));
            const blue = Math.max(0, 0.1 * (1 - heightFactor * 2));
            
            // Add flicker
            const flickerAmount = (Math.random() - 0.5) * 0.1;
            
            // Blend with black based on dissolution
            const finalRed = red * dissolveFactor + (1 - dissolveFactor) * 0;
            const finalGreen = (green + flickerAmount) * dissolveFactor + (1 - dissolveFactor) * 0;
            const finalBlue = blue * dissolveFactor + (1 - dissolveFactor) * 0;
            

            // Apply to all three vertices of the triangle
            for (let vert = 0; vert < 3; vert++) {
                this.COLORS.push(finalRed, finalGreen, finalBlue);
            }
        }

        return this.COLORS;
    }

    dissolveFlame(deltaTime) {
        this.time += deltaTime * 0.01;
        const maxY = Math.max(...this.VERTICES.filter((_, i) => i % 3 === 1));

        // Update dissolution state for each vertex group
        for (let [index, value] of this.dissolveMap) {
            const y = this.VERTICES[index + 1];
            const heightFactor = y / maxY;

            if (heightFactor > 0.6) {
                // Higher parts are more likely to dissolve
                if (Math.random() < this.dissolveRate) {
                    this.dissolveMap.set(index, Math.max(0, value - 0.1));
                }
            } else {
                // Lower parts more likely to regenerate
                if (Math.random() < this.regenerateRate) {
                    this.dissolveMap.set(index, Math.min(1.0, value + 0.1));
                }
            }
        }


        return this.updateColors();
    }


    pulseAndFlicker(deltaTime, frequency = 3.0, amplitude = 0.15) {
        this.time += deltaTime * 0.001;
        this.scaleFactor = 1.0 + Math.sin(this.time * frequency) * amplitude;
        this.scaleFactor += (Math.random() - 0.5) * this.flickerIntensity;
        this.xFlicker = (Math.random() - 0.5) * this.flickerIntensity * 2;
        this.yFlicker = (Math.random() - 0.5) * this.flickerIntensity * 2;
    }


    applyTransformations() {
        return new Matrix4()
            .translate(this.xOffset + this.xFlicker, this.yOffset + this.yFlicker, 0)
            .scale(this.scaleFactor, this.scaleFactor, this.scaleFactor);
    }

    moveFlameTip(deltaTime, frequency = 6, amplitude = 0.2) {
        this.time += deltaTime * 0.001; // Convert time to seconds

        let maxY = Math.max(...this.VERTICES.filter((_, i) => i % 3 === 1)); // Find max Y

        this.VERTICES = this.VERTICES.map((value, index) => {
            let coordType = index % 3; // 0 = X, 1 = Y, 2 = Z
            let y = this.VERTICES[index + (coordType === 2 ? -1 : 0)]; // Get Y value

            // 🔹 Only move the tip (upper part of the flame)
            let heightFactor = y / maxY; // Normalize 0 at base, 1 at tip
            if (heightFactor < 0.6) return value; // Don't move lower vertices

            if (coordType === 0) { // X movement (left/right)
                return value + Math.sin(this.time * frequency) * amplitude * heightFactor;
            }
            if (coordType === 2) { // Z movement (forward/backward)
                return value + Math.cos(this.time * frequency) * amplitude * heightFactor;
            }
            return value;
        });
    }
    warpFlameTaper(factor = 0.5) {
        let maxY = Math.max(...this.VERTICES.filter((_, i) => i % 3 === 1)); // Find max Y
        this.VERTICES = this.VERTICES.map((value, index) => {
            if (index % 3 === 0 || index % 3 === 2) { // X or Z coordinates
                let y = this.VERTICES[index + 1]; // Get Y-coordinate
                let scale = 1 - (y / maxY) * factor; // Scale down higher up
                return value * scale;
            }
            return value;
        });
    }
    warpFlameNoise(intensity = 0.1) {
        this.VERTICES = this.VERTICES.map((value, index) => {
            if (index % 3 === 0 || index % 3 === 2) { // X or Z coordinates
                return value + (Math.random() * 2 - 1) * intensity; // Random shift
            }
            return value;
        });
    }
    scaleFlame(deltaTime, minScale = 0.8, maxScale = 1.2, speed = 2.0) {
        this.time += deltaTime * 0.001;
        let scale = minScale + (Math.sin(this.time * speed) * 0.5 + 0.5) * (maxScale - minScale);
        
        this.scaleFactor = scale;
    }
    
}



let flame1, flame2

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

class Mesh{
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

let firstSquare = new Square(generateSquareFromVertex(0.5));
let secondSquare = new Square(generateSpaceship(0.5));
let thirdSquare = new Square(generateSpaceship(0.5));
var shipMesh

var translate_time;

var data = {}
var translate_time;
var g_rotationAxis;

async function loadOBJFiles() {
    // Fetch the ship mesh
    const shipData = await fetch('./resources/ship.obj').then(response => response.text());
    const flameData = await fetch('./resources/cone.obj').then(response => response.text());

    // Parse the ship mesh
    g_shipMesh = [];
    readObjFile(shipData, g_shipMesh);
    shipMesh = new Mesh(g_shipMesh)


    // Parse the flame mesh
    g_flameMesh1 = [];
    g_flameMesh2 = [];
    readObjFile(flameData, g_flameMesh1);
    readObjFile(flameData, g_flameMesh2);


    flame1 = new Flame(0.25, 0)
    flame1.initializeVertices(g_flameMesh1);

    flame2 = new Flame(0.25, 0)
    flame2.initializeVertices(g_flameMesh2);


    // Start rendering after all files are loaded
    startRendering();
}


function main() {
    g_canvas = document.getElementById('canvas');
    gl = getWebGLContext(g_canvas, true);



    if (!gl) {
        console.log('Failed to get the rendering context for WebGL');
        return;
    }



    loadOBJFiles()
}

    

function startRendering(){
    if (!initShaders(gl, VSHADER_SOURCE, FSHADER_SOURCE)) {
        console.log('Failed to initialize shaders.');
        return;
    }


    g_u_model_ref = gl.getUniformLocation(gl.program, 'u_Model');
    g_u_world_ref = gl.getUniformLocation(gl.program, 'u_World');
    g_u_camera_ref = gl.getUniformLocation(gl.program, 'u_Camera');
    g_u_projection_ref = gl.getUniformLocation(gl.program, 'u_Projection');

    bf_info = initBuffers();
    if (!bf_info) {
        console.log('Failed to initialize buffers');
        return;
    }




    reset();
    gl.enable(gl.DEPTH_TEST);  // Enable depth testing
    // gl.depthFunc(gl.LESS);
    // gl.enable(gl.CULL_FACE);
    gl.cullFace(gl.BACK); 
        
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
        ...thirdSquare.VERTICES,
        ...g_shipMesh,
        ...flame1.VERTICES,
        ...flame2.VERTICES
    ];

    firstSquare.COLORS = buildColorAttributes(firstSquare.VERTEX_COUNT)
    secondSquare.COLORS = buildColorAttributes(secondSquare.VERTEX_COUNT)
    thirdSquare.COLORS = buildColorAttributes(thirdSquare.VERTEX_COUNT)
    shipMesh.COLORS = buildOriginalColorAttributes(g_shipMesh.length / 3)
    flame1.COLORS = flame1.updateColors()
    flame2.COLORS = flame2.updateColors()

    // Create colors for all shapes
    let colors = [
        ...firstSquare.COLORS,
        ...secondSquare.COLORS,
        ...thirdSquare.COLORS,
        ...shipMesh.COLORS,
        ...flame1.COLORS,
        ...flame2.COLORS
    ];



    let gridInfo = buildGridAttributes(1, 1, [1.0, 1.0, 1.0]);
    g_gridMesh = gridInfo[0];

    //add the grid last
    vertices = [...vertices, ...g_gridMesh];
    colors = [...colors, ...gridInfo[1]]

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
        shipMeshCount: g_shipMesh.length /3,
        gridCount: g_gridMesh.length / 3,
        //get the length of the flame
        flameMesh1Count: g_flameMesh1.length / 3, 
        flameMesh2Count: g_flameMesh2.length / 3,  
    };
}




// extra constants for cleanliness
var ROTATION_SPEED = .05

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

    updateCameraX(0.77)
    updateCameraY(0.38)
    updateCameraZ(1.71)


}

function setFlame(){
    var flame_size = 0.1

    g_flame_modelMatrix1 = g_flame_modelMatrix1.setScale(flame_size, flame_size, flame_size)
    g_flame_modelMatrix1 = g_flame_modelMatrix1.rotate(-90, 0, 0, 1)
    g_flame_modelMatrix1 = g_flame_modelMatrix1.translate(-0.55, 7.25, 1)
    

    g_flame_modelMatrix2 = g_flame_modelMatrix2.setScale(flame_size, flame_size, flame_size)
    g_flame_modelMatrix2 = g_flame_modelMatrix2.rotate(-90, 0, 0, 1 )
    g_flame_modelMatrix2 = g_flame_modelMatrix2.translate(-0.55, 7.25, -1)
    

    moveShip(movement.HORIZONTAL, 1.25)
}



function tick() {
    var deltaTime = Date.now() - g_lastFrameMS;
    g_lastFrameMS = Date.now();

    let angle = ROTATION_SPEED * deltaTime;

    g_modelMatrix.concat(new Matrix4().setRotate(angle, ...g_rotationAxis));
    g_second_modelMatrix.concat(new Matrix4().setRotate(secondSquare.SPEED * 100, ...g_rotationAxis));
    g_third_modelMatrix.concat(new Matrix4().setRotate(thirdSquare.SPEED * 100, ...g_rotationAxis));
    

    translate_time += deltaTime;

    if (translate_time >= 10500) {
        translate_time = 0;
        reset_moving_shapes();
        setSpeed();
        updateColors();
    }else{
        flickerFlame(deltaTime)
    }

    moveShip(movement.HORIZONTAL, -0.01)

    g_second_modelMatrix = move3DShape(g_second_modelMatrix, movement.HORIZONTAL, -secondSquare.SPEED);
    g_third_modelMatrix = move3DShape(g_third_modelMatrix, movement.HORIZONTAL, -thirdSquare.SPEED);

    draw();
    requestAnimationFrame(tick);
}

function moveShip(move, distance){
    g_ship_modelMatrix = move3DShape(g_ship_modelMatrix, move, distance)
    g_flame_modelMatrix1 = move3DShape(g_flame_modelMatrix1, move, distance)
    g_flame_modelMatrix2 = move3DShape(g_flame_modelMatrix2, move, distance)
}

function updateColors() {
    let gridInfo = buildGridAttributes(1, 1, [1.0, 1.0, 1.0]);
    let vertices = [
        ...firstSquare.VERTICES,
        ...secondSquare.VERTICES,
        ...thirdSquare.VERTICES,
        ...g_shipMesh,
        ...flame1.VERTICES,
        ...flame2.VERTICES,
        ...gridInfo[0]
    ];

    firstSquare.COLORS = buildColorAttributes(firstSquare.VERTEX_COUNT)
    secondSquare.COLORS = buildColorAttributes(secondSquare.VERTEX_COUNT)
    thirdSquare.COLORS = buildColorAttributes(thirdSquare.VERTEX_COUNT)
    flame1.COLORS = flame1.updateColors()
    flame2.COLORS = flame2.updateColors()

    let colors = [
        ...firstSquare.COLORS,
        ...secondSquare.COLORS,
        ...thirdSquare.COLORS,
        ...shipMesh.COLORS,
        ...flame1.COLORS,
        ...flame2.COLORS,
        ...gridInfo[1]
    ];


    gl.bindBuffer(gl.ARRAY_BUFFER, bf_info.buffer);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([...vertices, ...colors]), gl.STATIC_DRAW);
}

function flickerFlame(deltaTime) {

    let gridInfo = buildGridAttributes(1, 1, [1.0, 1.0, 1.0]);
    var amplitude, frequency, orth_freqency, orth_amplitude
    if (isOrth){
        orth_freqency = 6
        orth_amplitude = 20

        // important for orthographic flicker
        amplitude = 0.5
        frequency = 9
    }else{
        orth_freqency = 2
        orth_amplitude = 0.1

        // important for orthographic flicker
        amplitude = 0.5
        frequency = 9

    }

    flame1.pulseAndFlicker(deltaTime, orth_freqency, amplitude = orth_amplitude);
    flame1.moveFlameTip(deltaTime, frequency, amplitude);
    // flame1.dissolveFlame(deltaTime);

    flame2.pulseAndFlicker(deltaTime, orth_freqency-2, amplitude = orth_amplitude);
    flame2.moveFlameTip(deltaTime, frequency, amplitude);
    // flame2.scaleFlame(deltaTime, 2, 1.4, 2.5);

    let vertices = [
        ...firstSquare.VERTICES,
        ...secondSquare.VERTICES,
        ...thirdSquare.VERTICES,
        ...g_shipMesh,
        ...flame1.VERTICES,
        ...flame2.VERTICES,
        ...gridInfo[0]
    ];
    
    
    let colors = [
        ...firstSquare.COLORS,
        ...secondSquare.COLORS,
        ...thirdSquare.COLORS,
        ...shipMesh.COLORS,
        ...flame1.COLORS,
        ...flame2.COLORS,
        ...gridInfo[1]
    ];

    gl.bindBuffer(gl.ARRAY_BUFFER, bf_info.buffer);
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([...vertices, ...colors]), gl.STATIC_DRAW);
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

    gl.bindBuffer(gl.ARRAY_BUFFER, bf_info.buffer);

    // Set up position attribute
    setupVec3('a_Position', 0, bf_info.vertexOffset);
    
    // Set up color attribute
    setupVec3('a_Color', 0, bf_info.colorOffset);

    // Draw first square
    gl.uniformMatrix4fv(g_u_model_ref, false, g_modelMatrix.elements);
    gl.uniformMatrix4fv(g_u_world_ref, false, g_worldMatrix.elements);
    gl.drawArrays(gl.TRIANGLES, 0, bf_info.firstSquareCount);

    // Draw second square
    gl.uniformMatrix4fv(g_u_model_ref, false, g_second_modelMatrix.elements);
    gl.drawArrays(gl.TRIANGLES, bf_info.firstSquareCount, 
                 bf_info.secondSquareCount);

    // Draw third square
    gl.uniformMatrix4fv(g_u_model_ref, false, g_third_modelMatrix.elements);
    gl.drawArrays(gl.TRIANGLES, 
                 bf_info.firstSquareCount + bf_info.secondSquareCount,
                 bf_info.thirdSquareCount);

    // Draw ship mesh
    gl.uniformMatrix4fv(g_u_model_ref, false, g_ship_modelMatrix.elements);
    gl.drawArrays(gl.TRIANGLES, bf_info.firstSquareCount + bf_info.secondSquareCount + bf_info.thirdSquareCount, g_shipMesh.length / 3);
    
    // Draw flame mesh
    gl.uniformMatrix4fv(g_u_model_ref, false, g_flame_modelMatrix1.elements);
    gl.drawArrays(gl.TRIANGLES, bf_info.firstSquareCount + bf_info.secondSquareCount + bf_info.thirdSquareCount + bf_info.shipMeshCount, g_flameMesh1.length / 3);

    // Draw flame mesh
    gl.uniformMatrix4fv(g_u_model_ref, false, g_flame_modelMatrix2.elements);
    gl.drawArrays(gl.TRIANGLES, bf_info.firstSquareCount + bf_info.secondSquareCount + bf_info.thirdSquareCount + bf_info.shipMeshCount + bf_info.flameMesh1Count, g_flameMesh2.length / 3);
    
    // Draw grid
    gl.uniformMatrix4fv(g_u_model_ref, false, new Matrix4().elements);
    gl.uniformMatrix4fv(g_u_world_ref, false, new Matrix4().translate(0, GRID_Y_OFFSET, 0).elements);
    gl.uniformMatrix4fv(g_u_camera_ref, false, cameraMatrix.elements);
    gl.uniformMatrix4fv(g_u_projection_ref, false, projection_matrix.elements);



    gl.drawArrays(gl.LINES, 
                 bf_info.firstSquareCount + bf_info.secondSquareCount + 
                 bf_info.thirdSquareCount + bf_info.shipMeshCount + bf_info.flameMesh1Count+ bf_info.flameMesh2Count,
                 bf_info.gridCount);
}

function reset(){
    translate_time = 0
    g_modelMatrix = new Matrix4().translate(0, 0.5)
    g_worldMatrix = new Matrix4()

    g_modelMatrix = move3DShape(g_modelMatrix, movement.RESET, ID_MATRIX)

    
    //g_second_modelMatrix, movement.HORIZONTAL, -secondSquare.SPEED
    g_modelMatrix = g_modelMatrix.setScale(first_square_scale, first_square_scale, first_square_scale)
    g_modelMatrix = move3DShape(g_modelMatrix, movement.VERTICAL, 0.5)
    setPerspective()

    setSpeed()
    reset_moving_shapes()
    

}



function setPerspective(){
    updateFOVY(84)
    updateAspect(1.32)
    updateNear(8)
    updateFar(.11)
}



function reset_moving_shapes(){
    g_second_modelMatrix = new Matrix4()
    g_second_modelMatrix = g_second_modelMatrix.setScale(other_box_scale, other_box_scale, other_box_scale)
    g_second_modelMatrix = move3DShape(g_second_modelMatrix, movement.HORIZONTAL, 1.75)
    g_second_modelMatrix = move3DShape(g_second_modelMatrix, movement.VERTICAL, 0.5)
    g_second_modelMatrix.rotate(90, 0, 0,1 )

    g_third_modelMatrix = new Matrix4()
    g_third_modelMatrix = g_third_modelMatrix.setScale(other_box_scale, other_box_scale, other_box_scale)
    g_third_modelMatrix = move3DShape(g_third_modelMatrix, movement.HORIZONTAL, 1.75)
    g_third_modelMatrix = move3DShape(g_third_modelMatrix, movement.VERTICAL, -0.5)
    g_third_modelMatrix.rotate(90, 0, 0,1 )

    g_ship_modelMatrix = new Matrix4()
    var ship_size = 0.025
    g_ship_modelMatrix = g_ship_modelMatrix.setScale(ship_size, ship_size, ship_size)
    g_ship_modelMatrix = move3DShape(g_ship_modelMatrix, movement.HORIZONTAL, 0.5)
    g_ship_modelMatrix.rotate(-90, 0, 1,0)


    g_flame_modelMatrix1 = new Matrix4()
    g_flame_modelMatrix2 = new Matrix4()
    g_flame_modelMatrix1.set(g_ship_modelMatrix)
    g_flame_modelMatrix2.set(g_ship_modelMatrix)

    setFlame()
}