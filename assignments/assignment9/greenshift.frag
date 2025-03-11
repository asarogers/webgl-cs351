precision highp float;
uniform float u_GreenShift;
uniform bool u_FlatLighting;
varying vec3 v_Normal;

void main() {
    if (u_FlatLighting) {
        gl_FragColor = vec4(0.0, 0.6, 0.2, 1.0);
    } else {
        // Use normal for lighting or color calculation
        vec3 normal = normalize(v_Normal);
        // Simple lighting example
        float light = max(dot(normal, vec3(0.0, 1.0, 0.0)), 0.0);
        gl_FragColor = vec4(0.4, u_GreenShift, 0.8, 1.0) * (0.3 + 0.7 * light);
    }
}