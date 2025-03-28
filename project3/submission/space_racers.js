// Last edited by Dietrich Geisler 2025
const VSHADER_SOURCE = `
 attribute vec3 a_Position;
 attribute vec3 a_Normal;
 attribute vec3 a_Color;
 uniform mat4 u_Model;
 uniform mat4 u_World;
 uniform mat4 u_Camera;
 uniform mat4 u_Projection;
 varying vec3 v_Color;
 varying vec3 v_Normal;
 varying vec3 v_FragPos;

 void main() {
   gl_Position = u_Projection * u_Camera * u_World * u_Model * vec4(a_Position, 1.0);
   v_Normal = normalize(mat3(u_World * u_Model) * a_Normal);
   v_FragPos = vec3(u_World * u_Model * vec4(a_Position, 1.0));
   v_Color = a_Color;
 }
`;

const FSHADER_SOURCE = `
 precision mediump float;
 varying vec3 v_Color;
 varying vec3 v_Normal;
 varying vec3 v_FragPos;
 uniform vec3 u_LightPosition; // Sun's position
 uniform vec3 u_ViewPosition; // Camera position
 uniform vec3 u_LightColor; // Sunlight color
 uniform vec3 u_PlanetPosition; // Orbiting planet's position
 uniform float u_PlanetRadius; // Orbiting planet's radius
 uniform float u_BrightnessFactor;

 void main() {
   vec3 norm = normalize(v_Normal);
   
   // Light direction
   vec3 lightDir = normalize(u_LightPosition - v_FragPos);
   
   // Diffuse shading (Lambertian reflection)
   float diff = max(dot(norm, lightDir), 0.0);
   
   // Specular shading (Blinn-Phong reflection)
   vec3 viewDir = normalize(u_ViewPosition - v_FragPos);
   vec3 reflectDir = reflect(-lightDir, norm);
   float spec = pow(max(dot(viewDir, reflectDir), 0.0), 64.0); // Increased shininess

   // Shadow calculation for objects behind the planet
   float shadowFactor = 1.0;

   // Vector from light to planet
   vec3 lightToPlanet = u_PlanetPosition - u_LightPosition;
   float lightToPlanetDist = length(lightToPlanet);

   // Normalized direction from light to fragment
   vec3 lightToFragDir = normalize(v_FragPos - u_LightPosition);
   // Normalized direction from light to planet
   vec3 lightToPlanetDir = normalize(lightToPlanet);

   // Calculate the cosine of the angle between these directions
   float cosAngle = dot(lightToFragDir, lightToPlanetDir);

   // Calculate the distance from the light to the fragment
   float lightToFragDist = length(v_FragPos - u_LightPosition);

   // Check if fragment is farther from light than planet
   if (cosAngle > 0.99 && lightToFragDist > lightToPlanetDist) {
       // Calculate the closest approach of the fragment to the planet-light line
       // This is the distance from fragment to the shadow ray
       vec3 closestPoint = u_LightPosition + lightToPlanetDir * lightToPlanetDist;
       float distToShadowLine = distance(v_FragPos, closestPoint);
       
       // Check if this distance is less than the planet radius (with some tolerance)
       if (distToShadowLine < u_PlanetRadius * 1.1) {
           // Calculate a smooth shadow factor based on distance
           float shadowStrength = smoothstep(u_PlanetRadius * 1.1, u_PlanetRadius * 0.5, distToShadowLine);
           shadowFactor = mix(0.9, 0.1, shadowStrength);
       }
   }

   float eclipseFactor = 1.0;
   

   vec3 viewToSun = u_LightPosition - u_ViewPosition;
   float viewToSunDist = length(viewToSun);
   vec3 viewToSunDir = normalize(viewToSun);
   

   vec3 viewToPlanet = u_PlanetPosition - u_ViewPosition;
   float viewToPlanetDist = length(viewToPlanet);
   
   float viewAlignment = dot(normalize(viewToPlanet), viewToSunDir);
   
   if (viewToPlanetDist < viewToSunDist && viewAlignment > 0.99) {
       // Calculate closest approach to the view-sun line
       vec3 closestEclipsePoint = u_ViewPosition + viewToSunDir * viewToPlanetDist * viewAlignment;
       float distToEclipseLine = distance(u_PlanetPosition, closestEclipsePoint);
       
       // If planet is crossing in front of sun
       if (distToEclipseLine < u_PlanetRadius * 1.5) {
           // Partial eclipse effect based on how centered the planet is
           float eclipseStrength = smoothstep(u_PlanetRadius * 1.5, 0.0, distToEclipseLine);
           
           // Calculate eclipse coverage - how much of the sun is blocked
           float sunAngularRadius = 10.0 / viewToSunDist; // Adjust based on sun size
           float planetAngularRadius = u_PlanetRadius / viewToPlanetDist;
           
           // If planet's angular size is greater than the sun's, it can block more light
           float maxBlockage = min(1.0, pow(planetAngularRadius / sunAngularRadius, 2.0) * 0.95);
           
           // Mix between full brightness and eclipse brightness
           eclipseFactor = mix(1.0, 0.3, eclipseStrength * maxBlockage);
       }
   }

   float combinedLightFactor = shadowFactor * eclipseFactor;
   
   vec3 adjustedLightColor = combinedLightFactor * u_BrightnessFactor * u_LightColor;
   vec3 ambient = 0.2 * adjustedLightColor;
   vec3 diffuse = diff * adjustedLightColor;
   vec3 specular = spec * adjustedLightColor;
   vec3 result = (ambient + diffuse + specular) * v_Color;
   gl_FragColor = vec4(result, 1.0);
 }
`;

// references to general information
var g_canvas;
var g_lastFrameMS;
var gl;
var bf_info;

// GLSL uniform references
var g_u_terrain_ref;
var g_u_model_ref;
var g_u_world_ref;
var g_u_camera_ref;
var g_u_projection_ref;
var g_gridMesh;

// usual model/world matrices
var g_modelMatrix;
var g_first_modelMatrix;
var g_second_modelMatrix;
var g_third_modelMatrix;
var g_ship_modelMatrix;
var g_flame_modelMatrix1, g_flame_modelMatrix2;
var g_worldMatrix;
var g_shipMesh, shipSpeed;
var g_flameMesh1, g_flameMesh2;

// camera/projection
// note that we are using an identity matrix for our terrain for this demo
var g_terrainModelMatrix;
var g_terrainWorldMatrix;
var g_projectionMatrix;
var SUN_POSITION = [0, 0, -50];

// keep track of the camera position, always looking at (0, height, 0)
var g_cameraDistance;
var g_cameraAngle;
var g_cameraHeight;

// Mesh definition
var g_terrainMesh;
var isPaused = false; // Boolean to track the pause state

// Key states
var g_movingUp;
var g_movingDown;
var g_movingLeft;
var g_movingRight;
var g_movingForward;
var g_movingBackward;
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
      if (index % 3 === 0) {
        // X coordinate
        return value + this.xOffset;
      } else if (index % 3 === 1) {
        // Y coordinate
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

      const dissolveFactor =
        this.dissolveMap.get(vertexIndex) ||
        this.dissolveMap.get(vertexIndex + 3) ||
        this.dissolveMap.get(vertexIndex + 6) ||
        1.0;

      // Base flame colors
      const red = 1.0;
      const green = Math.max(0, 0.4 * (1 - heightFactor));
      const blue = Math.max(0, 0.1 * (1 - heightFactor * 2));

      // Add flicker
      const flickerAmount = (Math.random() - 0.5) * 0.1;

      // Blend with black based on dissolution
      const finalRed = red * dissolveFactor + (1 - dissolveFactor) * 0;
      const finalGreen =
        (green + flickerAmount) * dissolveFactor + (1 - dissolveFactor) * 0;
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

      if (coordType === 0) {
        // X movement (left/right)
        return (
          value + Math.sin(this.time * frequency) * amplitude * heightFactor
        );
      }
      if (coordType === 2) {
        // Z movement (forward/backward)
        return (
          value + Math.cos(this.time * frequency) * amplitude * heightFactor
        );
      }
      return value;
    });
  }
  warpFlameTaper(factor = 0.5) {
    let maxY = Math.max(...this.VERTICES.filter((_, i) => i % 3 === 1)); // Find max Y
    this.VERTICES = this.VERTICES.map((value, index) => {
      if (index % 3 === 0 || index % 3 === 2) {
        // X or Z coordinates
        let y = this.VERTICES[index + 1]; // Get Y-coordinate
        let scale = 1 - (y / maxY) * factor; // Scale down higher up
        return value * scale;
      }
      return value;
    });
  }
  warpFlameNoise(intensity = 0.1) {
    this.VERTICES = this.VERTICES.map((value, index) => {
      if (index % 3 === 0 || index % 3 === 2) {
        // X or Z coordinates
        return value + (Math.random() * 2 - 1) * intensity; // Random shift
      }
      return value;
    });
  }
  scaleFlame(deltaTime, minScale = 0.8, maxScale = 1.2, speed = 2.0) {
    this.time += deltaTime * 0.001;
    let scale =
      minScale +
      (Math.sin(this.time * speed) * 0.5 + 0.5) * (maxScale - minScale);

    this.scaleFactor = scale;
  }
}

let flame1, flame2;

// Grid constants
const GRID_X_RANGE = 10; // Reduced from 1000 for better performance
const GRID_Z_RANGE = 10;
const GRID_Y_OFFSET = -0.5;

class Square {
  constructor(VERTICES) {
    this.VERTICES = VERTICES;
    this.VERTEX_COUNT = VERTICES.length / 3;
    this.TRIANGLE_SIZE = 3;
    this.SPEED = 0;
    this.COLORS = null;
  }
}

class Mesh {
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
  DEPTH: "depth",
  TEST: "test",
  ROTATE: "rotate",
  RESET: "reset",
});

let firstSquare = new Square(generateSquareFromVertex(0.5));
let secondSquare = new Square(generateSpaceship(0.5));
let thirdSquare = new Square(generateSpaceship(0.5));
var shipMesh;

var data = {};
let translate_time;
var g_rotationAxis;
let allColors = [];

var distanceFirst = 0;
var distanceSecond = 0;
var distanceThird = 0;
var initialZFirst = 0;
var initialZSecond = 0;
var initialZThird = 0;

const spaceGenerator = new SpaceGenerator();

// generate planets
const rockyPlanet = spaceGenerator.generatePlanet([50, 10, -25], 3, "rocky");
const icePlanet = spaceGenerator.generatePlanet([0, 0, 50], 2.5, "ice");
const gasPlanet = spaceGenerator.generatePlanet([-50, 5, -30], 6, "gas");
const orbitingPlanet = spaceGenerator.generatePlanet(
  [SUN_POSITION[0] + 15, SUN_POSITION[1], SUN_POSITION[2]],
  2,
  "earth"
);

// Generate starfield (only stars)
const stars = spaceGenerator.generateStarfield(1000, 100);
const sun = spaceGenerator.generateSun(SUN_POSITION, 10.0);
//

const sunVertices = sun.generateSphere(150);
rockyPlanet.generateSphere(50);
gasPlanet.generateSphere(50);
icePlanet.generateSphere(50);
orbitingPlanet.generateSphere(75);

function main() {
  setupKeyBinds();

  g_canvas = document.getElementById("canvas");

  // Get the rendering context for WebGL
  gl = getWebGLContext(g_canvas, true);
  if (!gl) {
    console.log("Failed to get the rendering context for WebGL");
    return;
  }

  // We have no OBJ files to construct for this demo
  loadOBJFiles();
}

async function loadOBJFiles() {
  // Fetch the ship mesh
  const shipData = await fetch("./resources/ship.obj").then((response) =>
    response.text()
  );
  const flameData = await fetch("./resources/cone.obj").then((response) =>
    response.text()
  );

  // Parse the ship mesh
  g_shipMesh = [];
  readObjFile(shipData, g_shipMesh);
  shipMesh = new Mesh(g_shipMesh);

  // Parse the flame mesh
  g_flameMesh1 = [];
  g_flameMesh2 = [];
  readObjFile(flameData, g_flameMesh1);
  readObjFile(flameData, g_flameMesh2);

  flame1 = new Flame(0.25, 0);
  flame1.initializeVertices(g_flameMesh1);

  flame2 = new Flame(0.25, 0);
  flame2.initializeVertices(g_flameMesh2);

  // Start rendering after all files are loaded
  startRendering();
}

function startRendering() {
  // Initialize GPU's vertex and fragment shaders programs
  if (!initShaders(gl, VSHADER_SOURCE, FSHADER_SOURCE)) {
    console.log("Failed to intialize shaders.");
    return;
  }

  bf_info = initBuffers();
  if (!bf_info) {
    console.log("Failed to initialize buffers");
    return;
  }
  // Get references to GLSL uniforms
  g_u_model_ref = gl.getUniformLocation(gl.program, "u_Model");
  g_u_world_ref = gl.getUniformLocation(gl.program, "u_World");
  g_u_camera_ref = gl.getUniformLocation(gl.program, "u_Camera");
  g_u_projection_ref = gl.getUniformLocation(gl.program, "u_Projection");

  g_u_brightness_ref = gl.getUniformLocation(gl.program, "u_BrightnessFactor");

  reset();

  // Enable culling and depth
  gl.enable(gl.CULL_FACE);
  gl.enable(gl.DEPTH_TEST);

  g_lastFrameMS = Date.now();

  // Setup for ticks
  g_lastFrameMS = Date.now();
  setUpCamera();

  updateBrightness(5.0);
  tick();
}

function reset() {
  // Setup a model and world matrix for our terrain
  // Position can be given by our width/height,
  //   noting that we are centered initially at the "midpoint"
  // We want to be a bit above the terrain initially so we can see it
  // TODO: resize the terrain as needed to "fit" with your animation
  translate_time = 0;

  // Keep terrain model matrix as identity since it's our "world"
  g_terrainModelMatrix = new Matrix4();

  // Apply world transformations to terrain world matrix
  g_terrainWorldMatrix = new Matrix4();

  // g_terrainModelMatrix = new Matrix4()
  // // move in view of the initial camera
  // // TODO: you may want to move your terrain!  This is just placed for the demo
  // g_terrainWorldMatrix = new Matrix4().rotate(90, 0, 1, 0)//.translate(-bf_info.options.width / 2, -bf_info.options.height, -bf_info.options.depth / 2)
  setSpeed();
  reset_moving_shapes();
}

function setUpCamera() {
  // Initially set our camera to be at the origin, looking in the negative direction
  g_cameraMatrix = new Matrix4().setLookAt(0, 0, 0, 0, 0, -1, 0, 1, 0);

  // Setup a reasonable "basic" perspective projection
  g_projectionMatrix = new Matrix4().setPerspective(90, 1, 1, 1000);

  // Initially place the camera in "front" and above the teapot a bit
  g_cameraDistance = 1.5;
  g_cameraAngle = 0;
  g_cameraHeight = 0.2;

  // Initialize control values
  g_movingUp = false;
  g_movingDown = false;
  g_movingLeft = false;
  g_movingRight = false;
  g_movingForward = false;
  g_movingBackward = false;
}

function initBuffers() {
  let vertices = [
    ...firstSquare.VERTICES,
    ...secondSquare.VERTICES,
    ...thirdSquare.VERTICES,
    ...g_shipMesh,
    ...flame1.VERTICES,
    ...flame2.VERTICES,
    ...orbitingPlanet.vertices,
  ];

  // Generate the meshes
  firstSquare.COLORS = buildColorAttributes(firstSquare.VERTEX_COUNT);
  secondSquare.COLORS = buildColorAttributes(secondSquare.VERTEX_COUNT);
  thirdSquare.COLORS = buildColorAttributes(thirdSquare.VERTEX_COUNT);
  shipMesh.COLORS = buildOriginalColorAttributes(g_shipMesh.length / 3);
  flame1.COLORS = flame1.updateColors();
  flame2.COLORS = flame2.updateColors();

  let colors = [
    ...firstSquare.COLORS,
    ...secondSquare.COLORS,
    ...thirdSquare.COLORS,
    ...shipMesh.COLORS,
    ...flame1.COLORS,
    ...flame2.COLORS,
    ...orbitingPlanet.colors,
  ];

  // Combine all vertices and colors
  let allVertices = [];

  // Add stars
  for (const star of stars) {
    allVertices.push(...star.position);
    allColors.push(...star.color);
  }

  // Add Sun
  allVertices.push(...sunVertices);
  allColors.push(...sun.colors);

  // Add Planets
  allVertices.push(...rockyPlanet.vertices);
  allColors.push(...rockyPlanet.colors);

  allVertices.push(...gasPlanet.vertices);
  allColors.push(...gasPlanet.colors);

  allVertices.push(...icePlanet.vertices);
  allColors.push(...icePlanet.colors);

  // allVertices.push();
  // allColors.push();

  // Create and bind VBO
  let VBO = gl.createBuffer();
  if (!VBO) {
    console.log("Failed to create buffer");
    return null;
  }

  // Store vertices for later use
  g_terrainMesh = allVertices;

  vertices = [...vertices, ...allVertices];
  colors = [...colors, ...allColors];

  // Combine vertices and colors
  let combinedData = new Float32Array([...vertices, ...colors]);

  gl.bindBuffer(gl.ARRAY_BUFFER, VBO);
  gl.bufferData(gl.ARRAY_BUFFER, combinedData, gl.STATIC_DRAW);

  // console.log('Vertices length:', vertices.length);
  // console.log('Colors length:', colors.length);
  // console.log('Combined data length:', combinedData.length);

  return {
    buffer: VBO,
    options: {
      width: 200,
      height: 200,
      depth: 200,
    },
    colorOffset: vertices.length * FLOAT_SIZE,
    firstSquareCount: firstSquare.VERTEX_COUNT,
    secondSquareCount: secondSquare.VERTEX_COUNT,
    thirdSquareCount: thirdSquare.VERTEX_COUNT,
    shipMeshCount: g_shipMesh.length / 3,
    orbitingPlanetCount: orbitingPlanet.vertices.length / 3,
    totalCount:
      firstSquare.VERTEX_COUNT +
      secondSquare.VERTEX_COUNT +
      thirdSquare.VERTEX_COUNT +
      g_shipMesh.length / 3 +
      g_flameMesh1.length / 3 +
      g_flameMesh2.length / 3 +
      orbitingPlanet.vertices.length / 3,
    flameMesh1Count: g_flameMesh1.length / 3,
    flameMesh2Count: g_flameMesh2.length / 3,
  };
}

function randomizeScene() {
  const bounds = 50;

  // Generate random positions for planets
  const newRockyPosition = [
    Math.random() * bounds - bounds / 2,
    Math.random() * 20,
    Math.random() * bounds - bounds / 2,
  ];
  const newGasPosition = [
    Math.random() * bounds - bounds / 2,
    Math.random() * 10,
    Math.random() * bounds - bounds / 2,
  ];
  const newIcePosition = [
    Math.random() * bounds - bounds / 2,
    Math.random() * 15,
    Math.random() * bounds - bounds / 2,
  ];

  // Generate a new position for the Sun
  SUN_POSITION = [
    Math.random() * bounds - bounds / 2,
    Math.random() * 20,
    Math.random() * bounds - bounds / 2,
  ];
  // Update planet positions
  rockyPlanet.changeLocation(newRockyPosition);
  rockyPlanet.generateSphere(50);

  gasPlanet.changeLocation(newGasPosition);
  gasPlanet.generateSphere(50);

  icePlanet.changeLocation(newIcePosition);
  icePlanet.generateSphere(50);

  // Update Sun's position
  sun.changeLocation(SUN_POSITION);
  const sunVertices = sun.generateSphere(150);

  // Update the vertex buffer
  updateSpaceObjects(sunVertices);
}

function updateSpaceObjects(sunVertices) {
  let vertices = [
    ...firstSquare.VERTICES,
    ...secondSquare.VERTICES,
    ...thirdSquare.VERTICES,
    ...g_shipMesh,
    ...flame1.VERTICES,
    ...flame2.VERTICES,
  ];

  // Clear previous terrain and color buffers
  g_terrainMesh = [];
  allColors = [];

  // Add stars (assuming they don't change position)
  for (const star of stars) {
    g_terrainMesh.push(...star.position);
    allColors.push(...star.color);
  }

  // **Add new Sun and planets with updated sizes**
  g_terrainMesh.push(...sunVertices);
  allColors.push(...sun.colors);

  g_terrainMesh.push(...rockyPlanet.vertices);
  allColors.push(...rockyPlanet.colors);

  g_terrainMesh.push(...gasPlanet.vertices);
  allColors.push(...gasPlanet.colors);

  g_terrainMesh.push(...icePlanet.vertices);
  allColors.push(...icePlanet.colors);

  // Add inside updateSpaceObjects after the other planets
  // g_terrainMesh.push(...orbitingPlanet.vertices);
  // allColors.push(...orbitingPlanet.colors);

  // **Update vertex buffer**
  vertices = [...vertices, ...g_terrainMesh];

  let colors = [
    ...firstSquare.COLORS,
    ...secondSquare.COLORS,
    ...thirdSquare.COLORS,
    ...shipMesh.COLORS,
    ...flame1.COLORS,
    ...flame2.COLORS,
    ...allColors,
  ];

  // Re-upload data to GPU
  gl.bindBuffer(gl.ARRAY_BUFFER, bf_info.buffer);
  gl.bufferData(
    gl.ARRAY_BUFFER,
    new Float32Array([...vertices, ...colors]),
    gl.STATIC_DRAW
  );
}

// tick constants
const ROTATION_SPEED = 0.05;
const CAMERA_SPEED = 0.01;
const CAMERA_ROT_SPEED = 0.1;

let animationFrameId = null;
var g_lookAtPoint = [0, 0, 0];

// Camera state
var g_cameraPosition = new Vector3([0, 1, 5]);
var g_cameraFront = new Vector3([0, 0, -1]);
var g_cameraUp = new Vector3([0, 1, 0]);
var g_cameraSpeed = 0.02;

var yaw = -90.0;
var pitch = 0.0;
var g_followShip = false;

function setFollow() {
  g_followShip = !g_followShip;
}

// Store reference to brightness uniform
var g_u_brightness_ref;

function updateBrightness(amount) {
  let brightness = Number(amount);

  // Update UI label
  let label = document.getElementById("light");
  label.textContent = `Sun brightness: ${brightness.toFixed(2)}`;

  // Ensure WebGL has the latest brightness value
  if (gl && g_u_brightness_ref) {
    gl.uniform1f(g_u_brightness_ref, brightness);
  }
}

function followShip() {
  if (!g_followShip) return; // Exit if follow mode is OFF

  // Extract ship's position from its model matrix
  let shipX = g_ship_modelMatrix.elements[12];
  let shipY = g_ship_modelMatrix.elements[13];
  let shipZ = g_ship_modelMatrix.elements[14];

  // Extract the ship's forward direction (Z-axis of model matrix)
  let forwardX = -g_ship_modelMatrix.elements[8];
  let forwardY = -g_ship_modelMatrix.elements[9];
  let forwardZ = -g_ship_modelMatrix.elements[10];

  // Set a fixed distance behind the ship
  let distanceBehind = 5.0;
  let heightOffset = 2.0;

  g_cameraPosition = new Vector3([
    shipX + forwardX * distanceBehind,
    shipY + heightOffset,
    shipZ + forwardZ * distanceBehind,
  ]);

  // Update the camera front vector to look at the ship
  g_cameraFront = new Vector3([
    shipX - g_cameraPosition.elements[0],
    shipY - g_cameraPosition.elements[1],
    shipZ - g_cameraPosition.elements[2],
  ]);
  g_cameraFront.normalize();
}

function tick() {
  if (isPaused) {
    updateCameraPosition(16); // Assume a fixed frame time (16ms) for smooth movement
    draw();
    animationFrameId = requestAnimationFrame(tick, g_canvas);
    return;
  }

  // Add these with your other global variables

  updateOrbitingPlanet();

  g_rotationAxis = [1, 0, 0];
  mesh_rotation = [0, 1, 0];

  // Time and camera updates
  var deltaTime = Date.now() - g_lastFrameMS;
  g_lastFrameMS = Date.now();

  // Movement logic for squares
  g_first_modelMatrix.concat(
    new Matrix4().setRotate(ROTATION_SPEED * deltaTime, ...g_rotationAxis)
  );
  g_second_modelMatrix.concat(
    new Matrix4().setRotate(secondSquare.SPEED * 100, ...mesh_rotation)
  );
  g_third_modelMatrix.concat(
    new Matrix4().setRotate(thirdSquare.SPEED * 100, ...mesh_rotation)
  );

  // Update the distance traveled by each mesh based on their z position
  distanceFirst = Math.abs(g_first_modelMatrix.elements[14] - initialZFirst);
  distanceSecond = Math.abs(g_second_modelMatrix.elements[14] - initialZSecond);
  distanceThird = Math.abs(g_ship_modelMatrix.elements[14] - initialZThird);

  // Check which mesh has traveled the farthest

  // Continue with the rest of the game logic
  translate_time += deltaTime;
  moveShip(movement.DEPTH, shipSpeed);
  g_second_modelMatrix = move3DShape(
    g_second_modelMatrix,
    movement.DEPTH,
    secondSquare.SPEED
  );
  g_third_modelMatrix = move3DShape(
    g_third_modelMatrix,
    movement.DEPTH,
    thirdSquare.SPEED
  );

  if (translate_time >= 10500) {
    if (distanceFirst > distanceSecond && distanceFirst > distanceThird) {
      incrementScore("first");
    } else if (
      distanceSecond > distanceFirst &&
      distanceSecond > distanceThird
    ) {
      incrementScore("second");
    } else if (
      distanceThird > distanceFirst &&
      distanceThird > distanceSecond
    ) {
      incrementScore("third");
    }

    reset_moving_shapes();
    updateColors();
    setSpeed();
    translate_time = 0;
  } else {
    flickerFlame(deltaTime);
  }

  followShip();

  updateCameraPosition(deltaTime);
  draw();
  animationFrameId = requestAnimationFrame(tick, g_canvas);
}

let orbitAngle = 0;

function updateOrbitingPlanet() {
  const orbitSpeed = -0.010; // Adjust for faster/slower orbit
  const orbitRadius = 20; // Distance from the sun

  // Calculate new position
  orbitAngle += orbitSpeed;
  if (orbitAngle > Math.PI * 2) orbitAngle -= Math.PI * 2; // Keep angle in range

  // Calculate new position based on orbit
  const newX = SUN_POSITION[0] + Math.cos(orbitAngle) * orbitRadius;
  const newZ = SUN_POSITION[2] + Math.sin(orbitAngle) * orbitRadius;

  // Update planet position
  orbitingPlanet.changeLocation([newX, SUN_POSITION[1], newZ]);
  orbitingPlanet.generateSphere(75);

  // Update vertex buffer with new planet vertices
  // updateSpaceObjects(sunVertices);
}

function updateCameraPosition(deltaTime) {
  let speed = g_cameraSpeed * deltaTime;

  let forward = normalizeVector(g_cameraFront);
  let right = normalizeVector(crossProduct(forward, g_cameraUp));

  if (g_movingForward) {
    g_cameraPosition = addVectors(
      g_cameraPosition,
      multiplyScalar(forward, speed)
    );
  }
  if (g_movingBackward) {
    g_cameraPosition = subtractVectors(
      g_cameraPosition,
      multiplyScalar(forward, speed)
    );
  }
  if (g_movingUp) {
    g_cameraPosition = addVectors(
      g_cameraPosition,
      multiplyScalar(g_cameraUp, speed)
    );
  }
  if (g_movingDown) {
    g_cameraPosition = subtractVectors(
      g_cameraPosition,
      multiplyScalar(g_cameraUp, speed)
    );
  }

  if (g_movingLeft) {
    yaw += CAMERA_ROT_SPEED * deltaTime;
    updateCameraDirection();
    // g_cameraPosition = subtractVectors(g_cameraPosition, multiplyScalar(right, speed));
  }
  if (g_movingRight) {
    yaw -= CAMERA_ROT_SPEED * deltaTime;
    updateCameraDirection();
    // g_cameraPosition = addVectors(g_cameraPosition, multiplyScalar(right, speed));
  }
}

function normalizeVector(vector) {
  let length = Math.sqrt(
    vector.elements[0] * vector.elements[0] +
      vector.elements[1] * vector.elements[1] +
      vector.elements[2] * vector.elements[2]
  );
  if (length === 0) return new Vector3([0, 0, 0]); // Prevent division by zero
  return new Vector3([
    vector.elements[0] / length,
    vector.elements[1] / length,
    vector.elements[2] / length,
  ]);
}

function addVectors(v1, v2) {
  return new Vector3([
    v1.elements[0] + v2.elements[0],
    v1.elements[1] + v2.elements[1],
    v1.elements[2] + v2.elements[2],
  ]);
}

function subtractVectors(v1, v2) {
  return new Vector3([
    v1.elements[0] - v2.elements[0],
    v1.elements[1] - v2.elements[1],
    v1.elements[2] - v2.elements[2],
  ]);
}

function multiplyScalar(vector, scalar) {
  return new Vector3([
    vector.elements[0] * scalar,
    vector.elements[1] * scalar,
    vector.elements[2] * scalar,
  ]);
}

function crossProduct(a, b) {
  return new Vector3([
    a.elements[1] * b.elements[2] - a.elements[2] * b.elements[1],
    a.elements[2] * b.elements[0] - a.elements[0] * b.elements[2],
    a.elements[0] * b.elements[1] - a.elements[1] * b.elements[0],
  ]);
}

/*
 * Helper function to calculate camera position from the properties we update
 * Taken from the lecture 16 demos
 */
function calculateCameraPosition() {
  let lookAt = new Vector3([
    g_cameraPosition.elements[0] + g_cameraFront.elements[0],
    g_cameraPosition.elements[1] + g_cameraFront.elements[1],
    g_cameraPosition.elements[2] + g_cameraFront.elements[2],
  ]);

  return new Matrix4().setLookAt(
    g_cameraPosition.elements[0],
    g_cameraPosition.elements[1],
    g_cameraPosition.elements[2],
    lookAt.elements[0],
    lookAt.elements[1],
    lookAt.elements[2],
    g_cameraUp.elements[0],
    g_cameraUp.elements[1],
    g_cameraUp.elements[2]
  );
}

function updateCameraDirection() {
  let front = new Vector3([
    Math.cos(degToRad(yaw)), // X component (left/right movement)
    0, // Y component (ignore pitch for now)
    Math.sin(degToRad(yaw)), // Z component (forward/backward movement)
  ]);

  front.normalize();
  g_cameraFront = front;
}

function degToRad(degrees) {
  return degrees * (Math.PI / 180);
}

function togglePause() {
  isPaused = !isPaused;

  const pauseButton = document.getElementById("pauseButton");
  if (isPaused) {
    pauseButton.textContent = "Resume";
  } else {
    pauseButton.textContent = "Pause";
    g_lastFrameMS = Date.now(); // Reset the last frame time to avoid large delta
  }
}

var scores = {
  first: 0,
  second: 0,
  third: 0,
};

function incrementScore(mesh) {
  scores[mesh]++;
  updateScoreDisplay();
}

function updateScoreDisplay() {
  const scoreDisplay = document.getElementById("scoreDisplay");
  scoreDisplay.innerHTML = `First: ${scores.first} | Second: ${scores.second} | Ship: ${scores.third}`;
}

function updateColors() {
  let vertices = [
    ...firstSquare.VERTICES,
    ...secondSquare.VERTICES,
    ...thirdSquare.VERTICES,
    ...g_shipMesh,
    ...flame1.VERTICES,
    ...flame2.VERTICES,
    ...orbitingPlanet.vertices,
    ...g_terrainMesh,
  ];

  firstSquare.COLORS = buildColorAttributes(firstSquare.VERTEX_COUNT);
  secondSquare.COLORS = buildColorAttributes(secondSquare.VERTEX_COUNT);
  thirdSquare.COLORS = buildColorAttributes(thirdSquare.VERTEX_COUNT);
  flame1.COLORS = flame1.updateColors();
  flame2.COLORS = flame2.updateColors();

  let colors = [
    ...firstSquare.COLORS,
    ...secondSquare.COLORS,
    ...thirdSquare.COLORS,
    ...shipMesh.COLORS,
    ...flame1.COLORS,
    ...flame2.COLORS,
    ...orbitingPlanet.colors,
    ...allColors,
  ];

  gl.bindBuffer(gl.ARRAY_BUFFER, bf_info.buffer);
  gl.bufferData(
    gl.ARRAY_BUFFER,
    new Float32Array([...vertices, ...colors]),
    gl.STATIC_DRAW
  );
}

function flickerFlame(deltaTime) {
  var amplitude, frequency, orth_freqency, orth_amplitude;

  orth_freqency = 2;
  orth_amplitude = 0.1;

  // important for orthographic flicker
  amplitude = 0.5;
  frequency = 9;

  flame1.pulseAndFlicker(
    deltaTime,
    orth_freqency,
    (amplitude = orth_amplitude)
  );
  flame1.moveFlameTip(deltaTime, frequency, amplitude);
  // flame1.dissolveFlame(deltaTime);

  flame2.pulseAndFlicker(
    deltaTime,
    orth_freqency - 2,
    (amplitude = orth_amplitude)
  );
  flame2.moveFlameTip(deltaTime, frequency, amplitude);
  // flame2.scaleFlame(deltaTime, 2, 1.4, 2.5);
  let vertices = [
    ...firstSquare.VERTICES,
    ...secondSquare.VERTICES,
    ...thirdSquare.VERTICES,
    ...g_shipMesh,
    ...flame1.VERTICES,
    ...flame2.VERTICES,
    ...orbitingPlanet.vertices,
    ...g_terrainMesh,
  ];

  let colors = [
    ...firstSquare.COLORS,
    ...secondSquare.COLORS,
    ...thirdSquare.COLORS,
    ...shipMesh.COLORS,
    ...flame1.COLORS,
    ...flame2.COLORS,
    ...orbitingPlanet.colors,
    ...allColors,
  ];

  gl.bindBuffer(gl.ARRAY_BUFFER, bf_info.buffer);
  gl.bufferData(
    gl.ARRAY_BUFFER,
    new Float32Array([...vertices, ...colors]),
    gl.STATIC_DRAW
  );
}

// draw to the screen on the next frame
function draw() {
  var cameraMatrix = calculateCameraPosition();

  const sunPosition = new Float32Array(SUN_POSITION);
  const cameraPosition = [
    g_cameraPosition.elements[0],
    g_cameraPosition.elements[1],
    g_cameraPosition.elements[2],
  ];

  // Inside draw()
  const planetPosition = new Float32Array([
    orbitingPlanet.POSITIONS[0],
    orbitingPlanet.POSITIONS[1],
    orbitingPlanet.POSITIONS[2],
  ]);
  const planetRadius = orbitingPlanet.radius;

  gl.uniform3fv(
    gl.getUniformLocation(gl.program, "u_PlanetPosition"),
    planetPosition
  );
  gl.uniform1f(
    gl.getUniformLocation(gl.program, "u_PlanetRadius"),
    planetRadius
  );

  // Pass uniforms to shaders
  gl.uniform3fv(
    gl.getUniformLocation(gl.program, "u_LightPosition"),
    new Float32Array(sunPosition)
  );
  gl.uniform3fv(
    gl.getUniformLocation(gl.program, "u_ViewPosition"),
    new Float32Array(cameraPosition)
  );
  gl.uniform3fv(
    gl.getUniformLocation(gl.program, "u_LightColor"),
    new Float32Array([1.0, 0.9, 0.8])
  ); // Warm sun color

  let brightness = parseFloat(
    document.getElementById("light").textContent.split(": ")[1]
  );
  gl.uniform1f(g_u_brightness_ref, brightness);

  // Clear the canvas
  gl.clearColor(0.0, 0.0, 0.0, 1.0);
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

  gl.bindBuffer(gl.ARRAY_BUFFER, bf_info.buffer);

  // Setup vertex attributes
  setupVec3("a_Position", 0, 0);
  setupVec3("a_Color", 0, bf_info.colorOffset);

  // Set the shared matrices (world, camera, projection)
  gl.uniformMatrix4fv(g_u_world_ref, false, g_terrainWorldMatrix.elements);
  gl.uniformMatrix4fv(g_u_camera_ref, false, cameraMatrix.elements);
  gl.uniformMatrix4fv(g_u_projection_ref, false, g_projectionMatrix.elements);

  // Draw terrain with identity model matrix
  gl.uniformMatrix4fv(g_u_model_ref, false, g_terrainModelMatrix.elements);
  gl.drawArrays(gl.POINTS, bf_info.totalCount, g_terrainMesh.length / 3);

  // Draw each object with its own model matrix
  // First square
  gl.uniformMatrix4fv(g_u_model_ref, false, g_first_modelMatrix.elements);
  gl.drawArrays(gl.TRIANGLES, 0, bf_info.firstSquareCount);

  // Second square
  gl.uniformMatrix4fv(g_u_model_ref, false, g_second_modelMatrix.elements);
  gl.drawArrays(
    gl.TRIANGLES,
    bf_info.firstSquareCount,
    bf_info.secondSquareCount
  );

  // Third square
  gl.uniformMatrix4fv(g_u_model_ref, false, g_third_modelMatrix.elements);
  gl.drawArrays(
    gl.TRIANGLES,
    bf_info.firstSquareCount + bf_info.secondSquareCount,
    bf_info.thirdSquareCount
  );

  // Ship
  gl.uniformMatrix4fv(g_u_model_ref, false, g_ship_modelMatrix.elements);
  gl.drawArrays(
    gl.TRIANGLES,
    bf_info.firstSquareCount +
      bf_info.secondSquareCount +
      bf_info.thirdSquareCount,
    g_shipMesh.length / 3
  );

  // Draw flame mesh
  gl.uniformMatrix4fv(g_u_model_ref, false, g_flame_modelMatrix1.elements);
  gl.drawArrays(
    gl.TRIANGLES,
    bf_info.firstSquareCount +
      bf_info.secondSquareCount +
      bf_info.thirdSquareCount +
      bf_info.shipMeshCount,
    g_flameMesh1.length / 3
  );

  // Draw flame mesh
  gl.uniformMatrix4fv(g_u_model_ref, false, g_flame_modelMatrix2.elements);
  gl.drawArrays(
    gl.TRIANGLES,
    bf_info.firstSquareCount +
      bf_info.secondSquareCount +
      bf_info.thirdSquareCount +
      bf_info.shipMeshCount +
      bf_info.flameMesh1Count,
    g_flameMesh2.length / 3
  );

  gl.uniformMatrix4fv(g_u_model_ref, false, g_orbating_planetModel.elements);
  gl.drawArrays(
    gl.TRIANGLES,
    bf_info.firstSquareCount +
      bf_info.secondSquareCount +
      bf_info.thirdSquareCount +
      bf_info.shipMeshCount +
      bf_info.flameMesh1Count +
      bf_info.flameMesh2Count,
    orbitingPlanet.vertices.length / 3
  );
}

/*
 * Helper function to setup camera movement key binding logic
 * Taken from lecture 16 demos
 */
function setupKeyBinds() {
  // Start movement when the key starts being pressed
  document.addEventListener("keydown", function (event) {
    if (event.key == "r") {
      g_movingUp = true;
    } else if (event.key == "f") {
      g_movingDown = true;
    } else if (event.key == "d") {
      g_movingLeft = true;
    } else if (event.key == "a") {
      g_movingRight = true;
    } else if (event.key == "w") {
      g_movingForward = true;
    } else if (event.key == "s") {
      g_movingBackward = true;
    }
  });

  // End movement on key release
  document.addEventListener("keyup", function (event) {
    if (event.key == "r") {
      g_movingUp = false;
    } else if (event.key == "f") {
      g_movingDown = false;
    } else if (event.key == "d") {
      g_movingLeft = false;
    } else if (event.key == "a") {
      g_movingRight = false;
    } else if (event.key == "w") {
      g_movingForward = false;
    } else if (event.key == "s") {
      g_movingBackward = false;
    }
  });
}

function setSpeed() {
  let lower = 0.05;
  let upper = 0.15;
  speed1 = lower + Math.random() * (upper - lower);
  speed2 = lower + Math.random() * (upper - lower);
  shipSpeed = lower + Math.random() * (upper - lower);

  secondSquare.SPEED = speed2;
  thirdSquare.SPEED = speed1;
}

function reset_moving_shapes() {
  const scale_down = 0.25;
  const startingPoint = -10;

  // First square
  g_first_modelMatrix = new Matrix4();
  g_first_modelMatrix.setTranslate(0, 0, 0);
  g_first_modelMatrix.rotate(90, 0, 0, 1);
  g_first_modelMatrix.scale(scale_down, scale_down, scale_down);

  // Second square
  g_second_modelMatrix = new Matrix4();
  g_second_modelMatrix.setTranslate(1.75, 0.5, startingPoint);
  g_second_modelMatrix.rotate(90, 0, 0, 1);
  g_second_modelMatrix.rotate(90, 1, 0, 0);
  g_second_modelMatrix.scale(scale_down, scale_down, scale_down);

  // Third square
  g_third_modelMatrix = new Matrix4();
  g_third_modelMatrix.setTranslate(-1.75, 0.5, startingPoint);
  g_third_modelMatrix.rotate(90, 0, 0, 1);
  g_third_modelMatrix.rotate(90, 1, 0, 0);
  g_third_modelMatrix.scale(scale_down, scale_down, scale_down);

  // Ship
  g_ship_modelMatrix = new Matrix4();
  var ship_size = 0.025;
  g_ship_modelMatrix.setTranslate(0.0, 0.5, 0);
  g_ship_modelMatrix.scale(ship_size, ship_size, ship_size); // Scale

  g_flame_modelMatrix1 = new Matrix4();
  g_flame_modelMatrix2 = new Matrix4();
  g_flame_modelMatrix1.set(g_ship_modelMatrix);
  g_flame_modelMatrix2.set(g_ship_modelMatrix);

  g_orbating_planetModel = new Matrix4();

  setFlame();
}

function setFlame() {
  var flame_size = 0.1;

  g_flame_modelMatrix1 = g_flame_modelMatrix1.setScale(
    flame_size,
    flame_size,
    flame_size
  );
  g_flame_modelMatrix1 = g_flame_modelMatrix1.translate(-1.25, 5.25, -2.5);
  g_flame_modelMatrix1 = g_flame_modelMatrix1.rotate(270, 1, 0, 0);

  g_flame_modelMatrix2 = g_flame_modelMatrix2.setScale(
    flame_size,
    flame_size,
    flame_size
  );
  g_flame_modelMatrix2 = g_flame_modelMatrix2.translate(0.75, 5.25, -2.5);
  g_flame_modelMatrix2 = g_flame_modelMatrix2.rotate(270, 1, 0, 0);

  moveShip(movement.DEPTH, -10.25);
}

function moveShip(move, distance) {
  g_ship_modelMatrix = move3DShape(g_ship_modelMatrix, move, distance);
  g_flame_modelMatrix1 = move3DShape(g_flame_modelMatrix1, move, distance);
  g_flame_modelMatrix2 = move3DShape(g_flame_modelMatrix2, move, distance);
}

/*
 * Helper function to update the camera position each frame
 */

/*
 * Helper to construct _basic_ per-vertex terrain colors
 * We use the height of the terrain to select a color between white and blue
 * Requires that we pass in the height of the terrain (as a number), but feel free to change this
 * TODO: you should expect to modify this helper with custom (or more interesting) colors
 */
function buildTerrainColors(terrain, height) {
  var colors = [];
  for (var i = 0; i < terrain.length; i++) {
    // calculates the vertex color for each vertex independent of the triangle
    // the rasterizer can help make this look "smooth"

    // we use the y axis of each vertex alone for color
    // higher "peaks" have more shade
    var shade = terrain[i][1] / height + 1 / 2;
    var color = [shade, shade, 1.0];

    // give each triangle 3 colors
    colors.push(...color);
  }

  return colors;
}

/*
 * Initialize the VBO with the provided data
 * Assumes we are going to have "static" (unchanging) data
 */
function initVBO(data) {
  // get the VBO handle
  var VBOloc = gl.createBuffer();
  if (!VBOloc) {
    console.log("Failed to create the vertex buffer object");
    return false;
  }

  // Bind the VBO to the GPU array and copy `data` into that VBO
  gl.bindBuffer(gl.ARRAY_BUFFER, VBOloc);
  gl.bufferData(gl.ARRAY_BUFFER, data, gl.STATIC_DRAW);

  return true;
}

/*
 * Helper function to load the given vec3 data chunk onto the VBO
 * Requires that the VBO already be setup and assigned to the GPU
 */
function setupVec3(name, stride, offset) {
  // Get the attribute by name
  var attributeID = gl.getAttribLocation(gl.program, `${name}`);
  if (attributeID < 0) {
    console.log(`Failed to get the storage location of ${name}`);
    return false;
  }

  // Set how the GPU fills the a_Position variable with data from the GPU
  gl.vertexAttribPointer(attributeID, 3, gl.FLOAT, false, stride, offset);
  gl.enableVertexAttribArray(attributeID);

  return true;
}