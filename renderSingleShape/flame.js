// Flame Vertex Shader
const VSHADER_SOURCE = `
    attribute vec3 a_Position;
    uniform mat4 u_Model;
    uniform mat4 u_World;
    uniform mat4 u_Camera;
    attribute vec3 a_Color;
    varying vec3 v_Color;
    void main() {
        gl_Position = u_Camera * u_World * u_Model * vec4(a_Position, 1.0);
        v_Color = a_Color;
    }
`;

// Flame Fragment Shader
const FSHADER_SOURCE = `
    varying mediump vec3 v_Color;
    void main() {
        gl_FragColor = vec4(v_Color, 1.0);
    }
`;

// references to general information
var g_canvas
var gl
var g_lastFrameMS

// GLSL uniform references
var g_u_model_ref
var g_u_world_ref
var g_u_camera_ref

// usual model/world matrices
var g_modelMatrix
var g_worldMatrix
var g_worldMatrix2

// camera projection values
var g_camera_x
var g_camera_y
var g_camera_z

// We're using triangles, so our vertices each have 3 elements
const TRIANGLE_SIZE = 3

// The size in bytes of a floating point
const FLOAT_SIZE = 4

class FlameSystem {
    constructor(flamePositions) {
        // flamePositions should be an array of objects like [{x: 0.25, y: 0}, {x: -0.25, y: 2}]
        this.flames = [];
        
        // Create a flame at each specified position
        flamePositions.forEach(pos => {
            this.flames.push(new Flame(pos.x, pos.y));
        });
    }
    addFlame(x, y) {
        this.flames.push(new Flame(x, y));
    }

    update(deltaTime) {
        this.flames.forEach(flame => {
            flame.pulseAndFlicker(deltaTime, 5.0, 0.1);
            flame.moveFlameTip(deltaTime, 6.0, 0.2);
            flame.dissolveFlame(deltaTime);
        });
    }

    draw() {
        this.flames.forEach(flame => {
            let modelMatrix = flame.applyTransformations();
            gl.uniformMatrix4fv(g_u_model_ref, false, modelMatrix.elements);
            gl.drawArrays(gl.TRIANGLES, 0, flame.VERTICES.length / 3);
        });
    }
}



class Flame {
    constructor(xOffset = 0, yOffset = 0) {
        this.VERTICES = [];
        this.baseVertices = []; // Store the original vertices
        this.xOffset = xOffset;
        this.yOffset = yOffset;
        this.time = 0;
        this.flickerIntensity = 0.02;
        this.dissolveMap = new Map();
        this.dissolveRate = 0.01;
        this.regenerateRate = 0.005;
    }

    initializeVertices(objData) {
        // Make a deep copy of the vertices
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
            this.dissolveMap.set(i, 1.0); // 1.0 = fully visible
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
            

            const dissolveFactor = this.dissolveMap.get(vertexIndex) || 1.0;

            // Base flame colors
            const red = 1.0;
            const green = Math.max(0, 0.4 * (1 - heightFactor));
            const blue = Math.max(0, 0.1 * (1 - heightFactor * 2));
            
            // Add flicker
            const flickerAmount = (Math.random() - 0.5) * 0.1;
            
            // Blend with black based on dissolution
            const finalRed = red * dissolveFactor;
            const finalGreen = (green + flickerAmount) * dissolveFactor;
            const finalBlue = blue * dissolveFactor;

            // Apply to all three vertices of the triangle
            for (let vert = 0; vert < 3; vert++) {
                this.COLORS.push(finalRed, finalGreen, finalBlue);
            }
        }

        return this.COLORS;
    }

    dissolveFlame(deltaTime) {
        this.time += deltaTime * 0.001;
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
        this.time += deltaTime * 0.001; // Convert to seconds


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
}


let flameSystem = new FlameSystem([]);  // Start with no flames
flameSystem.addFlame(0.25, 0);
flameSystem.addFlame(-0.25, 0);



function main() {
    // Setup our camera movement sliders
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
    data = await fetch('./resources/cone.obj').then(response => response.text());
    flameSystem.flames.forEach(flame => {
        readObjFile(data, flame.VERTICES);
        flame.setLength();
        flame.warpFlameTaper(0.7);
    });

    startRendering();
}


function startRendering() {
    // Initialize GPU's vertex and fragment shaders programs
    if (!initShaders(gl, VSHADER_SOURCE, FSHADER_SOURCE)) {
        console.log('Failed to initialize shaders.');
        return;
    }

    // Combine vertex and color data from all flames
    let allVertices = [];
    let allColors = [];

    flameSystem.flames.forEach(flame => {
        const colors = flame.updateColors();
        allVertices = allVertices.concat(flame.VERTICES);
        allColors = allColors.concat(colors);
    });

    const data = new Float32Array(allVertices.concat(allColors));

    // Initialize the VBO with the combined data
    if (!initVBO(data)) {
        return;
    }

    // Setup vertex attributes
    const vertexOffset = 0;
    const colorOffset = allVertices.length * FLOAT_SIZE;

    if (!setupVec3('a_Position', 0, vertexOffset)) {
        return;
    }
    if (!setupVec3('a_Color', 0, colorOffset)) {
        return;
    }

    // Get references to GLSL uniforms
    g_u_model_ref = gl.getUniformLocation(gl.program, 'u_Model');
    g_u_world_ref = gl.getUniformLocation(gl.program, 'u_World');
    g_u_camera_ref = gl.getUniformLocation(gl.program, 'u_Camera');

    // Set initial transformation matrices
    g_worldMatrix = new Matrix4();

    // Enable WebGL features
    gl.enable(gl.CULL_FACE);
    gl.enable(gl.DEPTH_TEST);

    // Track time for animation updates
    g_lastFrameMS = Date.now();

    // Initialize camera position
    updateCameraX(0);
    updateCameraY(0);
    updateCameraZ(0);

    // Start animation loop
    tick();
}


// extra constants for cleanliness
var ROTATION_SPEED = .05
let direction = 1; // 1 for right, -1 for left
let speed = 0.02; // Speed of movement
let yOffset = 0.0;

// function to apply all the logic for a single frame tick
function tick() {
    var deltaTime = Date.now() - g_lastFrameMS;
    g_lastFrameMS = Date.now();
    flameSystem.update(deltaTime);

    angle = ROTATION_SPEED * deltaTime
    g_worldMatrix.concat(new Matrix4().setRotate(angle, 0, 1, 0))
    draw();
    requestAnimationFrame(tick);
}



// draw to the screen on the next frame
function draw() {
    // Set up camera matrix
    var cameraMatrix = new Matrix4().translate(-g_camera_x, -g_camera_y, -g_camera_z);

    // Clear the canvas with a black background
    gl.clearColor(0.0, 0.0, 0.0, 1.0);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT); // Add depth buffer clear

    // Update with our global transformation matrices
    gl.uniformMatrix4fv(g_u_world_ref, false, g_worldMatrix.elements);
    gl.uniformMatrix4fv(g_u_camera_ref, false, cameraMatrix.elements);

    // Draw the flame system
    flameSystem.draw();
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

function buildFlameColor(vertex_count, time) {
    var colors = [];
    var triangle_count = vertex_count / 3;
    let flickerFactor = Math.sin(time * 2) * 0.1; // Create slight variation

    for (var i = 0; i < triangle_count; i++) {
        var shade = i / triangle_count; // Normalized gradient (0 to 1)
        
        // 🔥 Introduce "dissolve" probability
        let fadeFactor = Math.random() < (shade * 0.7 + 0.2) ? 0 : 1; // Higher = more likely to fade

        // More dynamic flame color transition
        var red = 1.0 * fadeFactor;
        var green = Math.max(0, 0.4 * (1 - shade) + flickerFactor) * fadeFactor;
        var blue = Math.max(0, 0.1 * (1 - shade * 2)) * fadeFactor;
        
        for (var vert = 0; vert < 3; vert++) {
            colors.push(red, green, blue);
        }
    }
    return colors;
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