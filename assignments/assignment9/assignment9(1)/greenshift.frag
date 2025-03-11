precision highp float;
uniform float u_GreenShift;

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