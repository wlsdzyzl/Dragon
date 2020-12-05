#ifndef DRAGON_SHADER_H
#define DRAGON_SHADER_H
#include "Uniform.h"
#include <GL/glew.h>
#include "IO/ConsoleColor.h"
#include <iostream>
#include <fstream>
namespace dragon
{
namespace visualization
{
    class Shader
    {
        public:
        bool Load(const std::string &vfile, const std::string &ffile);
        //Load source code of shaders
        bool LoadSource(const std::string &vfile, std::string& source);
        void SetUniform(const Uniform & v)
        {
            
            GLuint loc = glGetUniformLocation(pid, v.id.c_str());
            
            switch(v.t)
            {
                case Uniform::INT:
                    glUniform1i(loc, v.i);
                    break;
                case Uniform::FLOAT:
                    glUniform1f(loc, v.f);
                    break;
                case Uniform::VEC2:
                    glUniform2f(loc, v.v2(0), v.v2(1));
                    break;
                case Uniform::VEC3:
                    glUniform3f(loc, v.v3(0), v.v3(1), v.v3(2));
                    break;
                case Uniform::VEC4:
                    glUniform4f(loc, v.v4(0), v.v4(1), v.v4(2), v.v4(3));
                    break;
                case Uniform::MAT4:
                    glUniformMatrix4fv(loc, 1, false, v.m4.data());
                    break;
                default:
                    assert(false && "Uniform type not implemented!");
                    break;
            } 
        }
        void Clear();
        void Enable();
        void Disable();
        GLint Compile(const char* source, GLenum type);
        bool Link();
        protected:
        GLuint pid = 0;
        //usually the size of shaders is 2
        std::vector<GLuint> shaders;
    };
}
}
#endif