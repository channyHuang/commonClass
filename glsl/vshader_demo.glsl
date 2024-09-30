#version 300 es

#ifdef GL_ES
    precision mediump float;
    precision mediump int;
#endif

uniform mat4 mvpMat;

in vec3 o_vertex;
in vec3 o_normal;
in vec2 o_texcoord;

out vec3 vertex;
out vec3 normal;
out vec2 texcoord;

void main() {
    vertex = o_vertex;
    normal = o_normal;
    texcoord = o_texcoord;

    gl_Position = mvpMat * vec3(o_vertex, 1.f);
    gl_PointSize = 5.f;
}
