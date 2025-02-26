// Last edited by Dietrich Geisler 2024

const VSHADER_SOURCE = `
    uniform float u_Projection;
    attribute vec3 a_Position;
    void main() {
        // reminder that this matrix is transposed!
        mat4 transformation = mat4(
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, -u_Projection,
            0, 0, 0, 1.0-u_Projection
        );
        gl_Position = transformation * vec4(a_Position, 1.0);
    }
`

const FSHADER_SOURCE = `
    void main() {
        // Every pixel gets the same color
        gl_FragColor = vec4(0.3, 0.2, 0.8, 1.0);
    }
`

// global hooks for updating data
var gl
var g_vertex_count
var g_uProjection_ref

function main() {

    // Listen for slider change
    slider_input = document.getElementById('slider')
    slider_input.addEventListener('input', (event) => {
        updateProjection(event.target.value);
    });

    canvas = document.getElementById('webgl');

    // Get the rendering context for WebGL
    gl = getWebGLContext(canvas, true);
    if (!gl) {
        console.log('Failed to get the rendering context for WebGL');
        return;
    }

    // Initialize GPU's vertex and fragment shaders programs
    if (!initShaders(gl, VSHADER_SOURCE, FSHADER_SOURCE)) {
        console.log('Failed to intialize shaders.');
        return;
    }
    
    // Setup our vertices for a simple triangle
    var vertices = new Float32Array([
        -0.4, -0.4, -2.0/3.0,
        -0.4, 0.4, -3.0/3.0,
        0.4, -0.4, -1.0/3.0,

        -0.4, 0.4, -3.0/3.0,
        0.4, 0.4, -2.0/3.0,
        0.4, -0.4, -1.0/3.0,
    ])
    g_vertex_count = 6 // The number of vertices

    // reference to the offset
    g_uProjection_ref = gl.getUniformLocation(gl.program, 'u_Projection');

    // Send our vertices to the GPU
    var error = initVertexBuffers(gl, vertices)
    if (error == 1) {
        console.log('Failed to load vertices into the GPU')
        return
    }

    updateProjection(0)
}

function initVertexBuffers(gl, vertices) {

    // get the VBO handle
    var VBOloc = gl.createBuffer();
    if (!VBOloc) {
        console.log('Failed to create the vertex buffer object');
        return -1;
    }

    // In the GPU, indicate the VBO as the location we are writing to
    gl.bindBuffer(gl.ARRAY_BUFFER, VBOloc);
    // COPY data from Javascript 'vertices' array into VBO in the GPU:
    gl.bufferData(gl.ARRAY_BUFFER, vertices, gl.STATIC_DRAW);

    // Set up 'attributes' -- the Vertex and Shader vars that get filled from VBO:
    var a_PositionID = gl.getAttribLocation(gl.program, 'a_Position');
    if (a_PositionID < 0) {
        console.log('Failed to get the storage location of a_Position');
        return -1;
    }

    // Set how the GPU fills the a_Position variable with data from the GPU 
    gl.vertexAttribPointer(a_PositionID, 3, gl.FLOAT, false, 0, 0);

    // Enable the GPU to fill the a_Position variable from the VBO.
    gl.enableVertexAttribArray(a_PositionID)

    return 0;
}

function updateProjection(amount) {
    label = document.getElementById('projection_amount')
    label.textContent = `"Projection Amount": ${amount}`
    gl.uniform1f(g_uProjection_ref, amount)

    // Clear the canvas with a gray background
    gl.clearColor(0.0, 0.0, 0.0, 1.0)
    gl.clear(gl.COLOR_BUFFER_BIT)
    gl.drawArrays(gl.TRIANGLES, 0, g_vertex_count)
}