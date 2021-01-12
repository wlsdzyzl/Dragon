#version 130

in vec3 position;
uniform mat4 MVP;
out vec4 vColor;

void main()
{
        vColor = vec4(0.0, 0.0, 0.0, 1.0);
	    gl_Position = MVP * vec4(position.xyz, 1.0);
        
}