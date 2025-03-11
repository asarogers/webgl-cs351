precision highp float;
uniform mat4 u_Model;
uniform mat4 u_World;
varying vec3 v_Position;

// The size (width) of the wave
uniform float u_WaveSize;

// The current time in milliseconds (mod 2000)
uniform float u_Time;

// tells us whether or not to use lighting at all
uniform bool u_FlatLighting;

void main() {
    if (u_FlatLighting) {
        // use a bright green "by default"
        gl_FragColor = vec4(0.0, 0.6, 0.2, 1.0);
    }
    else {
        gl_FragColor = vec4(0.4, 0.0, 0.8, 1.0);
    }
}