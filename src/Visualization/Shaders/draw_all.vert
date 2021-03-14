#version 130

in vec3 position;

in vec3 normal;
in vec3 color;
uniform mat4 MV;
uniform mat4 proj;
uniform vec3 light_pos;
uniform int colorType;
uniform int phong;
uniform float materialShininess;
uniform vec4 materialAmbient;
uniform vec4 materialDiffuse;
uniform vec4 materialSpecular;
uniform vec4 lightAmbient;
uniform vec4 lightDiffuse;
uniform vec4 lightSpecular;


out vec4 vColor;

void main()
{
        gl_Position = proj * MV * vec4(position.xyz, 1.0);
        if(colorType == 1)
        {
            vColor = vec4(-normal.xyz, 1.0);
        }
        else if(colorType == 2)
        {
            vColor = vec4(color.xyz, 1.0);
        }
        else vColor = vec4(1.0, 1.0, 1.0, 1.0);

        if(phong == 1)
        {
            // use Phong Model
            vec4 material = materialDiffuse;
            vec4 transformed_pos = MV * vec4(position.xyz, 1.0);
            vec3 transformed_normal = mat3(MV) * normal;
            vec3 eye_dir = normalize(vec3(0, 0, 0) - transformed_pos.xyz);
            vec3 light_dir = normalize(light_pos - transformed_pos.xyz); 
            vec3 h = normalize(light_dir + eye_dir);
            vec4 res = vColor * lightAmbient  * materialAmbient                                                       // Ambient
                + vColor * lightDiffuse * materialDiffuse  * max(dot(light_dir, transformed_normal), 0.0)      // Diffuse
                + lightSpecular * materialSpecular * pow(max(dot(h, transformed_normal),0.0f), materialShininess) ; // Specular

            vColor = clamp(res, 0.0, 1.0);

        }
	    
        
}
