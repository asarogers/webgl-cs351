precision highp float;

uniform mat4 u_CameraWorldModelInverseTranspose;  // Transforms normals to camera space
varying vec3 v_Normal;  // Interpolated normal from vertex shader
uniform float u_CrossIntensity;  // Intensity of the cross effect
uniform bool u_FlatLighting;  // Controls whether lighting is applied

void main() {
    if (u_FlatLighting) {
        // Default bright green for flat lighting mode
        gl_FragColor = vec4(0.0, 0.6, 0.2, 1.0);
    } else {
        vec3 cameraNormal = normalize((u_CameraWorldModelInverseTranspose * vec4(normalize(v_Normal), 0.0)).xyz);

        float crossColor = abs(cameraNormal.x) * abs(cameraNormal.y) * 50.0 * u_CrossIntensity;

        crossColor = clamp(crossColor, 0.0, 1.0);
        gl_FragColor = vec4(crossColor, 0.0, 0.5, 1.0);
    }
}
