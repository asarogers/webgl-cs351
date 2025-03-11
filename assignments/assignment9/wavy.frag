precision highp float;

uniform float u_WaveSize;  // Controls the intensity of the wave
uniform float u_Time;      // Current time in milliseconds (mod 2000)
uniform bool u_FlatLighting;
varying vec3 v_Position;   // Fragment world position
varying vec3 v_Normal;     // Normal for shading

void main() {
    if (u_FlatLighting) {
        // Use a bright green for flat lighting
        gl_FragColor = vec4(0.0, 0.6, 0.2, 1.0);
    } else {

        float radialOffset =  (u_Time / 2000.0);

        radialOffset = fract(radialOffset);

        float waveAmount = sin((radialOffset - 0.5) * 2.0 * 3.14159);


        float waveSize = pow(abs(waveAmount), u_WaveSize);


        vec3 normal = normalize(v_Normal);

        float light = max(dot(normal, vec3(0.0, 1.0, 0.0)), 0.0);

        gl_FragColor = vec4(0.0, waveSize * 0.5, 0.7, 1.0) * (0.3 + 0.7 * light);
    }
}
