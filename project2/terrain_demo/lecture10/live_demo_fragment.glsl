precision mediump float;
varying vec3 v_Color;
void main() {
    vec3 color = v_Color;
    gl_FragColor = vec4(color, 1.0);
}