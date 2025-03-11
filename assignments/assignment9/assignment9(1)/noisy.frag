precision highp float;

// The current time in milliseconds (mod 2000)
uniform float u_Time;

// The size of each noisy "fragment tile"
uniform float u_TileSize;

// tells us whether or not to use lighting at all
uniform bool u_FlatLighting;

// https://thebookofshaders.com/10/
float random (vec2 st) {
    return fract(cos(dot(st.xy,vec2(39.,100.)))*502401.);
}

void main() {
    if (u_FlatLighting) {
        // use a bright green "by default"
        gl_FragColor = vec4(0.0, 0.6, 0.2, 1.0);
    }
    else {
        gl_FragColor = vec4(0.4, 0.0, 0.8, 1.0);
    }
}