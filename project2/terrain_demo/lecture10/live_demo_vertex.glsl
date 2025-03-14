attribute vec3 a_Position;
uniform mat4 u_Model;
uniform mat4 u_World;
uniform mat4 u_Camera;
uniform mat4 u_Projection;
attribute vec3 a_Color;
varying vec3 v_Color;
void main() {
    vec3 pos = a_Position;
    gl_Position = u_Projection * u_Camera * u_World * u_Model * vec4(pos, 1.0);
    v_Color = a_Color;
}