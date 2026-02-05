#pragma once

#include <string>
#include <unordered_map>
#include <GL/glew.h>
#include <glm/glm.hpp>

namespace wowee {
namespace rendering {

class Shader {
public:
    Shader() = default;
    ~Shader();

    bool loadFromFile(const std::string& vertexPath, const std::string& fragmentPath);
    bool loadFromSource(const std::string& vertexSource, const std::string& fragmentSource);

    void use() const;
    void unuse() const;

    void setUniform(const std::string& name, int value);
    void setUniform(const std::string& name, float value);
    void setUniform(const std::string& name, const glm::vec2& value);
    void setUniform(const std::string& name, const glm::vec3& value);
    void setUniform(const std::string& name, const glm::vec4& value);
    void setUniform(const std::string& name, const glm::mat3& value);
    void setUniform(const std::string& name, const glm::mat4& value);
    void setUniformMatrixArray(const std::string& name, const glm::mat4* matrices, int count);

    GLuint getProgram() const { return program; }

    // Adopt an externally-created program (no ownership of individual shaders)
    void setProgram(GLuint prog) { program = prog; }
    // Release ownership without deleting (caller retains the GL program)
    void releaseProgram() { program = 0; vertexShader = 0; fragmentShader = 0; }

private:
    bool compile(const std::string& vertexSource, const std::string& fragmentSource);
    GLint getUniformLocation(const std::string& name) const;

    GLuint program = 0;
    GLuint vertexShader = 0;
    GLuint fragmentShader = 0;

    // Cache uniform locations to avoid expensive glGetUniformLocation calls
    mutable std::unordered_map<std::string, GLint> uniformLocationCache;
};

} // namespace rendering
} // namespace wowee
