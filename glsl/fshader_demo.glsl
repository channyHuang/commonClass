#version 300 es

#ifdef GL_ES
    precision mediump float;
    precision mediump int;
#endif

in vec3 vertex;
in vec3 normal;
in vec2 texcoord;

out vec4 fragColor;

uniform sampler2D textureMap;
uniform sampler2D normalMap;

uniform vec3 lightPos;
uniform mat4 viewMatInv;

void main() {
    vec3 color = texture(textureMap, texcoord).xyz;
    vec3 lightDir = normalize(lightPos - vertex);
    vec4 cameraPos = viewMatInv * vec4(0, 0, 0, 1);
    vec3 viewDir = normalize(cameraPos.xyz - vertex);

    vec3 nn = vec3(1.0);
    nn = 2.0 * texture(normalMap, texcoord).xyz - vec3(1.0);

    vec3 shadow = vec3(1.f, 1.f, 1.f);

    float diffuse = max(0.2, dot(lightDir, normal));

    vec3 highlight = normalize(lightDir + viewDir);

    vec3 colorFin = diffuse * color * shadow;
    float specular = pow(max(0.0, dot(highlight, normal)), 40.0);

    fragColor = vec4(colorFin, 1.0);
}
