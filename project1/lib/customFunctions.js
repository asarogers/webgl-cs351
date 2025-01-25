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