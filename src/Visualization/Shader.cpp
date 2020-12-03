#include "Shader.h"
namespace dragon
{
namespace visualization
{


    GLint Shader::Compile(const char* source, GLenum type)
    {
        // create shader
        GLint id = glCreateShader(type);
        if (!id)
        {
            std::cout<<RED<< "Shader: Cannot create shader object";
            return 0;
        }

        // compile vertex shader
        glShaderSource(id, 1, &source, nullptr);
        glCompileShader(id);

        // check compile status
        GLint status;
        glGetShaderiv(id, GL_COMPILE_STATUS, &status);
        if (status == GL_FALSE)
        {
            GLint length;
            glGetShaderiv(id, GL_INFO_LOG_LENGTH, &length);

            auto* info = new GLchar[length + 1];
            glGetShaderInfoLog(id, length, nullptr, info);
            std::cout <<RED<<"[ERROR]::[Shader]:: Cannot compile shader "<< info <<RESET << std::endl;
            delete[] info;
            glDeleteShader(id);

            return 0;
        }

        return id;
    }
    bool Shader::LoadSource(const std::string &filename, std::string& source)
    {
        std::ifstream ifs(filename);
        if (!ifs)
        {
            std::cout<<RED<< "[ERROR]::[Shader]::Cannot open file " << filename << RESET<< std::endl;
            return false;
        }

        std::stringstream ss;
        ss << ifs.rdbuf();
        source = ss.str();

        ifs.close();
        return true;        
    }
    bool Shader::Load(const std::string &vfile, const std::string &ffile)
    {
        GLint id = 0;

        // cleanup existing shaders first
        Clear();

        // create program
        pid = glCreateProgram();

        // vertex shader
        std::string source; 
        if(Load(vfile, source))
        id = Compile(source.c_str(), GL_VERTEX_SHADER);
        if (!id)
        {
            std::cout<<RED <<"[ERROR]::[Shader]::Cannot compile vertex shader!"<<RESET<<std::endl;
            return false;
        }
        glAttachShader(pid, id);
        shaders.push_back(id);

        // fragment shader
        id = 0;
        if(Load(ffile, source))
        id = Compile(source.c_str(), GL_FRAGMENT_SHADER);
        if (!id)
        {
            std::cout<<RED<< "[ERROR]::[Shader]::Cannot compile fragment shader!"<<RESET<< std::endl;
            return false;
        }
        glAttachShader(pid, id);
        shaders.push_back(id);

        // link program
        if (!Link())
        {
            std::cout<<RED<< "[ERROR]::[Shader]::Cannot link program!"<<RESET<< std::endl;
            return false;
        }
        return true;
    }
    bool Shader::Link()
    {
        glLinkProgram(pid);
        GLint status;
        glGetProgramiv(pid, GL_LINK_STATUS, &status);
        if (status == GL_FALSE)
        {
            GLint length;
            glGetProgramiv(pid, GL_INFO_LOG_LENGTH, &length);

            auto* info = new GLchar[length + 1];
            glGetProgramInfoLog(pid, length, nullptr, info);
            std::cout<<RED<< "[ERROR]::[Shader]::Cannot link program:" << info <<RESET<< std::endl;
            delete[] info;

            Clear();

            return false;
        }

        return true;
    } 
    void Shader::Enable()
    {
        if(pid)
            glUseProgram(pid);
    }   
    void Shader::Disable()
    {
        glUseProgram(0);
    }
    void Shader::Clear()
    {
        if (pid)
        {
            glDeleteProgram(pid);
            pid = 0;
        }

        for (GLint id : shaders)
        {
            glDeleteShader(id);
        }
        shaders.clear();
    }
}
}