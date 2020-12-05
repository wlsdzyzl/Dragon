#version 130

in vec3 position;
in vec3 color;
out vec4 vColor;
void main()
{
gl_Position = vec4(position.x, position.y, position.z, 1.0);
vColor = vec4(color.xyz,1);
}