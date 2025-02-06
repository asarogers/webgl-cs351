function generateFront(point){
    const top = [
     point, point, point,
    -point, point, point,
    -point, -point, point,
    ]

    const bottom = [
        point, point, point,
        -point, -point, point,
        point, -point, point,
    ]
    face = [...top, ...bottom]
    return face
}

function generateBottom(point){
    const top = [
        point, point, -point,
    -point, -point, -point,
    -point, point, -point,
       ]
   
       const bottom = [
        point, point, -point,
        point, -point, -point,
        -point, -point, -point,
       ]
       face = [...top, ...bottom]
       return face
}

function generateRight(point){
    const top = [
        point, point, point,
       point, -point, -point,
       point, point, -point,
       ]
       const bottom = [
        point, point, point,
       point, -point, point,
       point, -point, -point,
       ]
       face = [...top, ...bottom]
       return face
}
function generateLeft(point){
    const top = [
        -point, point, point,
        -point, point, -point,
        -point, -point, -point,
       ]

       const bottom = [
        -point, point, point,
        -point, -point, -point,
        -point, -point, point,
    
       ]
       face = [...top, ...bottom]
       return face
}
function generateTop(point){
    const top = [
        point, point, point,
        point, point, -point,
        -point, point, -point,
       ]

       const bottom = [
        point, point, point,
        -point, point, -point,
        -point, point, point,
       ]
       face = [...top, ...bottom]
       return face
}
function generateBottomFace(point){
    const top = [
        point, -point, point,
        -point, -point, -point,
        point, -point, -point,
       ]

       const bottom = [
        point, -point, point,
        -point, -point, point,
        -point, -point, -point,
       ]
       face = [...top, ...bottom]
       return face
}

function generateSquareFromVertex(point){
    vertices = [
        ...generateFront(point),
        ...generateBottom(point),
        ...generateRight(point),
        ...generateLeft(point),
        ...generateTop(point),
        ...generateBottomFace(point),
    ]
    return vertices
}



function generateSpaceship(point){
    return [
        ...generateShipFront(point),
        ...generateShipBottom(point),
        ...generateShipRight(point),
        ...generateShipLeft(point),
        ...generateShipBottomFace(point),
    ]
}

function generateShipFront(point){
    const top = [
     point, -point, point,
    0, point, 0,
    -point, -point, point,
    ]
    return top
}

function generateShipBottom(point){
    const top = [
        0, point, -0,
    point, -point, -point,
    -point, -point, -point,
       ]
       return top
}

function generateShipRight(point){
    const top = [
        0, point, 0,
       point, -point, point,
       point, -point, -point,
       ]
      
       return top
}
function generateShipLeft(point){
    const top = [
        -point, -point, point,
        -0, point, -0,
        -point, -point, -point,
       ]
       return top
}
function generateShipTop(point){
    const top = [
        point, point, point,
        point, point, -point,
        -point, point, -point,
       ]

       return top
}
function generateShipBottomFace(point){
    const top = [
        point, -point, point,
        -point, -point, -point,
        point, -point, -point,
       ]

       const bottom = [
        point, -point, point,
        -point, -point, point,
        -point, -point, -point,
       ]
       face = [...top, ...bottom]
       return face
}




function move3DShape(model, move, distance){
    // move the 3D shape one directional or another
    switch (move) {
        case movement.HORIZONTAL:
            model.elements = mat4multiply(HORIZONTAL(distance), model.elements)
            break;

        case movement.VERTICAL:
            model.elements = mat4multiply(VERTICAL(distance), model.elements)
            break;

        case movement.ROTATE:
            // 
            model.elements = mat4multiply(ROTATE(distance), model.elements)
            break;

        case movement.RESET:
            model.elements = ID_MATRIX
            break;
            
        case movement.TEST:
            model.elements = distance
            break;
            // 

        default:
            break;
    }
    
    return model
}

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


function ROTATE(q){
    const matrix = [
        Math.cos(q), Math.sin(q), 0, 0,
        -Math.sin(q), Math.cos(q), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    ]

    return matrix
}

function VERTICAL(q){
    const matrix = [
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, q, 0, 1
    ]

    return matrix
}

function HORIZONTAL(q){
    const matrix = [
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        q, 0, 0, 1
    ]

    return matrix
}

// identity matrix
const ID_MATRIX = [
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1
]

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

function buildOriginalColorAttributes(vertex_count) {
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