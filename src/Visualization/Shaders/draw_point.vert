#version 130

in vec3 position;
uniform mat4 MV;
uniform mat4 proj;
out vec4 vColor;

void main()
{
        vColor = vec4(0.0, 0.0, 0.0, 1.0);
	    gl_Position = proj * MV * vec4(position.xyz, 1.0);
        
}